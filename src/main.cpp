#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#ifdef PLATE_ENABLE_CAMERA
#include "gate_controller.hpp"
#include "serial_rfid_reader.hpp"
#ifdef PLATE_ENABLE_GPIO
#include "gate_gpio.hpp"
#include "status_leds.hpp"
#endif
#include <curl/curl.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#endif

namespace fs = std::filesystem;

constexpr int kInputSize = 640;
constexpr float kConfidence = 0.60F;
constexpr float kNmsThreshold = 0.50F;
std::string gControllerId = "legacy-plate-controller";

std::string controllerIdentity() {
    std::string identity;
    if (const char* configured = std::getenv("CONTROLLER_ID")) {
        identity = configured;
    }
    if (identity.empty()) {
        std::ifstream cpuInfo("/proc/cpuinfo");
        std::string line;
        while (std::getline(cpuInfo, line)) {
            if (line.rfind("Serial", 0) == 0) {
                const std::size_t separator = line.find(':');
                if (separator != std::string::npos) {
                    identity = line.substr(separator + 1);
                    break;
                }
            }
        }
    }
    if (identity.empty()) {
        std::ifstream machineId("/etc/machine-id");
        std::getline(machineId, identity);
    }
    identity.erase(
        std::remove_if(identity.begin(), identity.end(), [](unsigned char value) {
            return std::isspace(value) != 0;
        }),
        identity.end()
    );
    for (char& value : identity) {
        const unsigned char byte = static_cast<unsigned char>(value);
        if (!std::isalnum(byte) && value != '.' && value != '_' &&
            value != ':' && value != '-') {
            value = '-';
        }
    }
    if (identity.empty()) identity = "unknown";
    if (identity.size() > 58) identity.resize(58);
    return "plate-" + identity;
}

struct LetterboxResult {
    cv::Mat image;
    float scale;
    int padX;
    int padY;
};

struct Detection {
    cv::Rect box;
    float confidence;
};

LetterboxResult letterbox(const cv::Mat& source) {
    const float scale = std::min(
        static_cast<float>(kInputSize) / source.cols,
        static_cast<float>(kInputSize) / source.rows
    );
    const int resizedWidth = static_cast<int>(std::round(source.cols * scale));
    const int resizedHeight = static_cast<int>(std::round(source.rows * scale));
    const int padX = (kInputSize - resizedWidth) / 2;
    const int padY = (kInputSize - resizedHeight) / 2;

    cv::Mat resized;
    cv::resize(source, resized, cv::Size(resizedWidth, resizedHeight));
    cv::Mat padded(kInputSize, kInputSize, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(padded(cv::Rect(padX, padY, resizedWidth, resizedHeight)));
    return {padded, scale, padX, padY};
}

std::vector<Detection> detectPlates(cv::dnn::Net& network, const cv::Mat& source) {
    const LetterboxResult input = letterbox(source);
    cv::Mat blob = cv::dnn::blobFromImage(
        input.image,
        1.0 / 255.0,
        cv::Size(kInputSize, kInputSize),
        cv::Scalar(),
        true,
        false
    );
    network.setInput(blob);
    cv::Mat output = network.forward();

    cv::Mat rows;
    if (output.dims == 3 && output.size[1] < output.size[2]) {
        cv::Mat channels(output.size[1], output.size[2], CV_32F, output.ptr<float>());
        cv::transpose(channels, rows);
    } else if (output.dims == 3) {
        rows = cv::Mat(output.size[1], output.size[2], CV_32F, output.ptr<float>());
    } else {
        rows = output;
    }

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    for (int index = 0; index < rows.rows; ++index) {
        const float* values = rows.ptr<float>(index);
        const float score = values[4];
        if (score < kConfidence) {
            continue;
        }

        const float centerX = (values[0] - input.padX) / input.scale;
        const float centerY = (values[1] - input.padY) / input.scale;
        const float width = values[2] / input.scale;
        const float height = values[3] / input.scale;
        int left = static_cast<int>(std::round(centerX - width / 2));
        int top = static_cast<int>(std::round(centerY - height / 2));
        int right = static_cast<int>(std::round(centerX + width / 2));
        int bottom = static_cast<int>(std::round(centerY + height / 2));

        left = std::clamp(left, 0, source.cols - 1);
        top = std::clamp(top, 0, source.rows - 1);
        right = std::clamp(right, left + 1, source.cols);
        bottom = std::clamp(bottom, top + 1, source.rows);
        boxes.emplace_back(left, top, right - left, bottom - top);
        scores.push_back(score);
    }

    std::vector<int> kept;
    cv::dnn::NMSBoxes(boxes, scores, kConfidence, kNmsThreshold, kept);
    std::vector<Detection> detections;
    for (const int index : kept) {
        detections.push_back({boxes[index], scores[index]});
    }
    std::sort(detections.begin(), detections.end(), [](const Detection& a, const Detection& b) {
        if (a.box.x == b.box.x) {
            return a.box.y < b.box.y;
        }
        return a.box.x < b.box.x;
    });
    return detections;
}

cv::Mat zoomPlate(const cv::Mat& crop, int targetWidth = 800) {
    const double scale = std::clamp(
        static_cast<double>(targetWidth) / std::max(1, crop.cols),
        1.0,
        6.0
    );
    cv::Mat zoomed;
    cv::resize(crop, zoomed, cv::Size(), scale, scale, cv::INTER_CUBIC);
    return zoomed;
}

std::string cleanPlateText(const std::string& raw) {
    std::string cleaned;
    for (const unsigned char character : raw) {
        if (std::isalnum(character)) {
            cleaned.push_back(static_cast<char>(std::toupper(character)));
        }
    }
    return cleaned;
}

char paddleCharacter(int index) {
    // PP-OCRv5 English classes: blank, 0-9, A-Z, a-z, then punctuation.
    if (index >= 1 && index <= 10) {
        return static_cast<char>('0' + index - 1);
    }
    if (index >= 11 && index <= 36) {
        return static_cast<char>('A' + index - 11);
    }
    if (index >= 37 && index <= 62) {
        return static_cast<char>('a' + index - 37);
    }
    return '\0';
}

double estimatePlateSkew(const cv::Mat& crop) {
    cv::Mat gray;
    cv::cvtColor(crop, gray, cv::COLOR_BGR2GRAY);
    cv::Mat edges;
    cv::Canny(gray, edges, 50, 150);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(
        edges,
        lines,
        1.0,
        CV_PI / 180.0,
        std::max(25, crop.cols / 8),
        crop.cols * 0.30,
        crop.cols * 0.08
    );

    std::vector<std::pair<double, double>> angles;
    double totalWeight = 0.0;
    for (const cv::Vec4i& line : lines) {
        const double dx = line[2] - line[0];
        const double dy = line[3] - line[1];
        const double angle = std::atan2(dy, dx) * 180.0 / CV_PI;
        if (std::abs(angle) <= 30.0) {
            const double weight = std::hypot(dx, dy);
            angles.emplace_back(angle, weight);
            totalWeight += weight;
        }
    }
    if (angles.empty()) {
        return 0.0;
    }

    std::sort(angles.begin(), angles.end());
    double accumulated = 0.0;
    for (const auto& [angle, weight] : angles) {
        accumulated += weight;
        if (accumulated >= totalWeight / 2.0) {
            return angle;
        }
    }
    return angles.back().first;
}

cv::Mat allowedPlateInkMask(const cv::Mat& image) {
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::Mat dark;
    cv::Mat green;
    cv::Mat redLow;
    cv::Mat redHigh;
    cv::Mat yellow;
    cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(179, 255, 145), dark);
    cv::inRange(hsv, cv::Scalar(30, 38, 35), cv::Scalar(105, 255, 255), green);
    cv::inRange(hsv, cv::Scalar(0, 65, 45), cv::Scalar(13, 255, 255), redLow);
    cv::inRange(hsv, cv::Scalar(165, 65, 45), cv::Scalar(179, 255, 255), redHigh);
    cv::inRange(hsv, cv::Scalar(14, 55, 55), cv::Scalar(38, 255, 255), yellow);
    cv::Mat allowed = dark | green | redLow | redHigh | yellow;
    cv::morphologyEx(
        allowed,
        allowed,
        cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3))
    );
    return allowed;
}

cv::Mat isolateRegistrationCharacters(const cv::Mat& aligned, bool correctedSkew) {
    const cv::Mat supportedInk = allowedPlateInkMask(aligned);
    cv::Mat hsv;
    cv::Mat strictGreenMask;
    cv::cvtColor(aligned, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(40, 70, 40), cv::Scalar(90, 255, 255), strictGreenMask);
    const double topRatio = correctedSkew ? 0.10 : 0.05;
    const double strictGreenRatio = cv::countNonZero(strictGreenMask) /
        static_cast<double>(aligned.total());
    const double supportedInkRatio = cv::countNonZero(supportedInk) /
        static_cast<double>(aligned.total());
    const bool strongGreenRegistration = strictGreenRatio >= 0.10 &&
        supportedInkRatio >= 0.12;
    const double bottomRatio = strongGreenRegistration
        ? 0.80
        : (correctedSkew ? 0.80 : 0.85);
    const int top = static_cast<int>(aligned.rows * topRatio);
    const int bottom = std::max(top + 1, static_cast<int>(aligned.rows * bottomRatio));
    const cv::Rect bandBox(
        0,
        top,
        aligned.cols,
        std::min(bottom, aligned.rows) - top
    );

    // Color segmentation decides when the lower edge can be tightened to
    // discard a small same-color slogan. Preserve the natural antialiased
    // character edges because hard pixel masking reduces OCR accuracy.
    return aligned(bandBox).clone();
}

struct OcrResult {
    std::string text;
    double confidence = 0.0;
};

OcrResult readPlate(cv::dnn::Net& recognizer, const cv::Mat& zoomedCrop) {
    constexpr int inputHeight = 48;
    constexpr int inputWidth = 320;
    const double skew = estimatePlateSkew(zoomedCrop);
    cv::Mat aligned = zoomedCrop;
    bool correctedSkew = false;
    if (std::abs(skew) > 8.0) {
        const cv::Point2f center(zoomedCrop.cols / 2.0F, zoomedCrop.rows / 2.0F);
        const double radians = skew * 0.5 * CV_PI / 180.0;
        const double alpha = std::cos(radians);
        const double beta = std::sin(radians);
        cv::Mat transform(2, 3, CV_64F);
        transform.at<double>(0, 0) = alpha;
        transform.at<double>(0, 1) = beta;
        transform.at<double>(0, 2) = (1.0 - alpha) * center.x - beta * center.y;
        transform.at<double>(1, 0) = -beta;
        transform.at<double>(1, 1) = alpha;
        transform.at<double>(1, 2) = beta * center.x + (1.0 - alpha) * center.y;
        cv::warpAffine(
            zoomedCrop,
            aligned,
            transform,
            zoomedCrop.size(),
            cv::INTER_CUBIC,
            cv::BORDER_CONSTANT,
            cv::Scalar(0, 0, 0)
        );
        correctedSkew = true;
    }

    // Use accepted character colors to locate the large registration row,
    // then reject smaller slogans, logos, borders, and stickers by component
    // size. Geometry-only cropping remains as a fallback for poor lighting.
    const cv::Mat registrationBand = isolateRegistrationCharacters(aligned, correctedSkew);
    cv::Mat grayscaleBand;
    cv::cvtColor(registrationBand, grayscaleBand, cv::COLOR_BGR2GRAY);
    cv::Mat grayscaleOcrInput;
    cv::cvtColor(grayscaleBand, grayscaleOcrInput, cv::COLOR_GRAY2BGR);

    const double ratio = static_cast<double>(grayscaleOcrInput.cols) / grayscaleOcrInput.rows;
    const int resizedWidth = std::min(
        inputWidth,
        static_cast<int>(std::ceil(inputHeight * ratio))
    );

    cv::Mat resized;
    cv::resize(grayscaleOcrInput, resized, cv::Size(resizedWidth, inputHeight));
    resized.convertTo(resized, CV_32FC3, 1.0 / 127.5, -1.0);
    cv::Mat padded(inputHeight, inputWidth, CV_32FC3, cv::Scalar(0, 0, 0));
    resized.copyTo(padded(cv::Rect(0, 0, resizedWidth, inputHeight)));

    const cv::Mat blob = cv::dnn::blobFromImage(padded, 1.0, cv::Size(), cv::Scalar(), false, false);
    recognizer.setInput(blob);
    const cv::Mat output = recognizer.forward();
    if (output.dims != 3 || output.size[0] != 1) {
        return {"UNREADABLE", 0.0};
    }

    const int timeSteps = output.size[1];
    const int classCount = output.size[2];
    const float* probabilities = output.ptr<float>();
    std::string decoded;
    double characterConfidence = 0.0;
    int characterCount = 0;
    int previous = -1;
    for (int step = 0; step < timeSteps; ++step) {
        const float* row = probabilities + step * classCount;
        const int index = static_cast<int>(std::max_element(row, row + classCount) - row);
        if (index != previous && index != 0) {
            const char character = paddleCharacter(index);
            if (character != '\0') {
                decoded.push_back(character);
                characterConfidence += row[index];
                ++characterCount;
            }
        }
        previous = index;
    }

    const std::string cleaned = cleanPlateText(decoded);
    if (cleaned.empty() || characterCount == 0) {
        return {"UNREADABLE", 0.0};
    }
    return {
        cleaned,
        characterConfidence / static_cast<double>(characterCount)
    };
}

void drawLabel(
    cv::Mat& image,
    const cv::Rect& box,
    const std::string& text,
    const cv::Scalar& color = cv::Scalar(0, 255, 0)
) {
    cv::rectangle(image, box, color, 3);
    const double scale = std::max(0.6, std::min(image.cols, image.rows) / 900.0);
    const int thickness = std::max(1, static_cast<int>(std::round(scale * 2)));
    int baseline = 0;
    const cv::Size textSize = cv::getTextSize(
        text,
        cv::FONT_HERSHEY_SIMPLEX,
        scale,
        thickness,
        &baseline
    );
    const int labelY = std::max(textSize.height + baseline + 8, box.y);
    cv::rectangle(
        image,
        cv::Rect(box.x, labelY - textSize.height - baseline - 8, textSize.width + 12, textSize.height + baseline + 8),
        cv::Scalar(color[0] * 0.45, color[1] * 0.45, color[2] * 0.45),
        cv::FILLED
    );
    cv::putText(
        image,
        text,
        cv::Point(box.x + 6, labelY - baseline - 4),
        cv::FONT_HERSHEY_SIMPLEX,
        scale,
        cv::Scalar(255, 255, 255),
        thickness,
        cv::LINE_AA
    );
}

bool supportedImage(const fs::path& path) {
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
    return extension == ".jpg" || extension == ".jpeg" || extension == ".png" ||
           extension == ".webp" || extension == ".avif";
}

#ifdef PLATE_ENABLE_CAMERA
struct BurstCandidate {
    int frameIndex = 0;
    Detection detection;
    cv::Mat enhancedCrop;
    double sharpness = 0.0;
    double exposure = 0.0;
    double quality = 0.0;
};

struct OcrVote {
    std::string reading;
    double quality = 0.0;
    double ocrConfidence = 0.0;
    std::size_t candidateIndex = 0;
};

double plateSharpness(const cv::Mat& crop) {
    cv::Mat gray;
    cv::cvtColor(crop, gray, cv::COLOR_BGR2GRAY);
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar mean;
    cv::Scalar deviation;
    cv::meanStdDev(laplacian, mean, deviation);
    return deviation[0] * deviation[0];
}

double plateExposureScore(const cv::Mat& crop) {
    cv::Mat gray;
    cv::cvtColor(crop, gray, cv::COLOR_BGR2GRAY);
    const double brightness = cv::mean(gray)[0];
    const double brightnessScore = std::clamp(
        1.0 - std::abs(brightness - 135.0) / 135.0,
        0.0,
        1.0
    );
    cv::Mat shadows;
    cv::Mat highlights;
    cv::inRange(gray, cv::Scalar(0), cv::Scalar(15), shadows);
    cv::inRange(gray, cv::Scalar(240), cv::Scalar(255), highlights);
    const double clippedRatio = (
        cv::countNonZero(shadows) + cv::countNonZero(highlights)
    ) / static_cast<double>(gray.total());
    return brightnessScore * std::clamp(1.0 - clippedRatio * 2.0, 0.0, 1.0);
}

double plateCandidateQuality(
    const Detection& detection,
    const cv::Size& frameSize,
    double sharpness,
    double exposure
) {
    const double sharpnessScore = std::clamp(
        std::log1p(sharpness) / std::log(1001.0),
        0.0,
        1.0
    );
    const double areaRatio = detection.box.area() /
        static_cast<double>(frameSize.area());
    const double sizeScore = std::clamp(std::sqrt(areaRatio) / 0.20, 0.0, 1.0);
    return detection.confidence * 0.45 +
        sharpnessScore * 0.30 +
        exposure * 0.15 +
        sizeScore * 0.10;
}

int editDistance(const std::string& first, const std::string& second) {
    std::vector<int> previous(second.size() + 1);
    std::vector<int> current(second.size() + 1);
    for (std::size_t index = 0; index <= second.size(); ++index) {
        previous[index] = static_cast<int>(index);
    }
    for (std::size_t row = 1; row <= first.size(); ++row) {
        current[0] = static_cast<int>(row);
        for (std::size_t column = 1; column <= second.size(); ++column) {
            const int substitution = previous[column - 1] +
                (first[row - 1] == second[column - 1] ? 0 : 1);
            current[column] = std::min({
                previous[column] + 1,
                current[column - 1] + 1,
                substitution
            });
        }
        std::swap(previous, current);
    }
    return previous.back();
}

std::string consensusPlate(const std::vector<OcrVote>& votes) {
    const auto voteWeight = [](const OcrVote& vote) {
        return std::max(0.01, vote.quality) *
            std::max(0.01, vote.ocrConfidence);
    };
    std::vector<OcrVote> readable;
    for (const OcrVote& vote : votes) {
        if (!vote.reading.empty() && vote.reading != "UNREADABLE") {
            readable.push_back(vote);
        }
    }
    if (readable.empty()) {
        return "UNREADABLE";
    }
    if (readable.size() == 1) {
        return readable.front().reading;
    }

    struct VoteTotal {
        int count = 0;
        double weight = 0.0;
    };
    std::map<std::string, VoteTotal> exactVotes;
    for (const OcrVote& vote : readable) {
        VoteTotal& total = exactVotes[vote.reading];
        ++total.count;
        total.weight += voteWeight(vote);
    }
    std::string exactWinner;
    VoteTotal exactWinnerTotal;
    for (const auto& [reading, total] : exactVotes) {
        if (total.count > exactWinnerTotal.count ||
            (total.count == exactWinnerTotal.count && total.weight > exactWinnerTotal.weight)) {
            exactWinner = reading;
            exactWinnerTotal = total;
        }
    }
    if (exactWinnerTotal.count >= 2) {
        return exactWinner;
    }

    std::map<std::size_t, VoteTotal> lengthVotes;
    for (const OcrVote& vote : readable) {
        VoteTotal& total = lengthVotes[vote.reading.size()];
        ++total.count;
        total.weight += voteWeight(vote);
    }
    std::size_t consensusLength = 0;
    VoteTotal lengthWinnerTotal;
    for (const auto& [length, total] : lengthVotes) {
        if (total.count > lengthWinnerTotal.count ||
            (total.count == lengthWinnerTotal.count && total.weight > lengthWinnerTotal.weight)) {
            consensusLength = length;
            lengthWinnerTotal = total;
        }
    }
    if (lengthWinnerTotal.count >= 2) {
        std::string result;
        result.reserve(consensusLength);
        for (std::size_t position = 0; position < consensusLength; ++position) {
            std::map<char, double> characterVotes;
            for (const OcrVote& vote : readable) {
                if (vote.reading.size() == consensusLength) {
                    characterVotes[vote.reading[position]] += voteWeight(vote);
                }
            }
            const auto winner = std::max_element(
                characterVotes.begin(),
                characterVotes.end(),
                [](const auto& first, const auto& second) {
                    return first.second < second.second;
                }
            );
            result.push_back(winner->first);
        }
        return result;
    }

    // When all readings have different lengths, choose the medoid: the OCR
    // value with the smallest total edit distance to the other observations.
    std::string medoid = readable.front().reading;
    int bestDistance = std::numeric_limits<int>::max();
    double bestQuality = -1.0;
    for (const OcrVote& candidate : readable) {
        int distance = 0;
        for (const OcrVote& other : readable) {
            distance += editDistance(candidate.reading, other.reading);
        }
        if (distance < bestDistance ||
            (distance == bestDistance && candidate.quality > bestQuality)) {
            medoid = candidate.reading;
            bestDistance = distance;
            bestQuality = candidate.quality;
        }
    }
    return medoid;
}

size_t appendHttpResponse(char* data, size_t size, size_t count, void* target) {
    const size_t bytes = size * count;
    static_cast<std::string*>(target)->append(data, bytes);
    return bytes;
}

bool sendRecognition(
    const std::string& serverUrl,
    const std::string& plate,
    float detectorConfidence,
    const fs::path& cropPath,
    const cv::Mat& rawFrame,
    const cv::Mat& annotatedFrame,
    long commandId,
    const std::string& rfidTag,
    bool rfidRequired,
    std::string& responseBody
) {
    if (serverUrl.empty()) {
        responseBody = "PLATE_SERVER_URL is not configured";
        return false;
    }
    CURL* client = curl_easy_init();
    if (!client) {
        responseBody = "unable to initialize HTTP client";
        return false;
    }

    std::string endpoint = serverUrl;
    while (!endpoint.empty() && endpoint.back() == '/') {
        endpoint.pop_back();
    }
    endpoint += "/api/reader/recognitions";
    const std::string confidence = std::to_string(detectorConfidence);
    curl_mime* form = curl_mime_init(client);

    std::vector<unsigned char> rawFrameJpeg;
    std::vector<unsigned char> annotatedFrameJpeg;
    const std::vector<int> frameEncoding{cv::IMWRITE_JPEG_QUALITY, 88};
    if ((!rawFrame.empty() && !cv::imencode(
            ".jpg", rawFrame, rawFrameJpeg, frameEncoding
        )) ||
        (!annotatedFrame.empty() && !cv::imencode(
            ".jpg", annotatedFrame, annotatedFrameJpeg, frameEncoding
        ))) {
        curl_mime_free(form);
        curl_easy_cleanup(client);
        responseBody = "unable to encode dashboard camera frames";
        return false;
    }

    curl_mimepart* part = curl_mime_addpart(form);
    curl_mime_name(part, "plate");
    curl_mime_data(part, plate.c_str(), CURL_ZERO_TERMINATED);
    part = curl_mime_addpart(form);
    curl_mime_name(part, "detector_confidence");
    curl_mime_data(part, confidence.c_str(), CURL_ZERO_TERMINATED);
    part = curl_mime_addpart(form);
    curl_mime_name(part, "controller_id");
    curl_mime_data(part, gControllerId.c_str(), CURL_ZERO_TERMINATED);
    part = curl_mime_addpart(form);
    curl_mime_name(part, "rfid");
    curl_mime_data(part, rfidTag.c_str(), CURL_ZERO_TERMINATED);
    part = curl_mime_addpart(form);
    curl_mime_name(part, "rfid_required");
    curl_mime_data(part, rfidRequired ? "1" : "0", CURL_ZERO_TERMINATED);
    if (commandId > 0) {
        const std::string commandIdText = std::to_string(commandId);
        part = curl_mime_addpart(form);
        curl_mime_name(part, "command_id");
        curl_mime_data(part, commandIdText.c_str(), CURL_ZERO_TERMINATED);
    }
    if (!cropPath.empty()) {
        part = curl_mime_addpart(form);
        curl_mime_name(part, "image");
        curl_mime_type(part, "image/jpeg");
        curl_mime_filedata(part, cropPath.string().c_str());
    }
    const auto addFrame = [&form](
        const char* fieldName,
        const char* filename,
        const std::vector<unsigned char>& bytes
    ) {
        if (bytes.empty()) return;
        curl_mimepart* framePart = curl_mime_addpart(form);
        curl_mime_name(framePart, fieldName);
        curl_mime_filename(framePart, filename);
        curl_mime_type(framePart, "image/jpeg");
        curl_mime_data(
            framePart,
            reinterpret_cast<const char*>(bytes.data()),
            bytes.size()
        );
    };
    addFrame("raw_frame", "raw-frame.jpg", rawFrameJpeg);
    addFrame("annotated_frame", "annotated-frame.jpg", annotatedFrameJpeg);

    curl_easy_setopt(client, CURLOPT_URL, endpoint.c_str());
    curl_easy_setopt(client, CURLOPT_MIMEPOST, form);
    curl_easy_setopt(client, CURLOPT_CONNECTTIMEOUT_MS, 5000L);
    curl_easy_setopt(client, CURLOPT_TIMEOUT_MS, 30000L);
    curl_easy_setopt(client, CURLOPT_WRITEFUNCTION, appendHttpResponse);
    curl_easy_setopt(client, CURLOPT_WRITEDATA, &responseBody);

    const CURLcode result = curl_easy_perform(client);
    long status = 0;
    curl_easy_getinfo(client, CURLINFO_RESPONSE_CODE, &status);
    if (result != CURLE_OK) {
        responseBody = curl_easy_strerror(result);
    }
    curl_mime_free(form);
    curl_easy_cleanup(client);
    return result == CURLE_OK && status >= 200 && status < 300;
}

enum class RemoteCommandPoll {
    None,
    Capture,
    BarrierOpen,
    BarrierClose,
    TrafficRed,
    TrafficGreen,
    RfidSerial,
    Error
};

struct RemoteCommandResult {
    RemoteCommandPoll command = RemoteCommandPoll::None;
    long commandId = 0;
    std::string error;
    std::string serialTxHex;
    gate::SerialDebugSettings serialSettings;
    long serialTimeoutMs = 2000;
};

struct SerialDebugCompletion {
    long commandId = 0;
    std::string transmittedHex;
    gate::SerialDebugResult result;
};

std::string serverEndpoint(const std::string& serverUrl, const std::string& path) {
    std::string endpoint = serverUrl;
    while (!endpoint.empty() && endpoint.back() == '/') {
        endpoint.pop_back();
    }
    return endpoint + path;
}

bool sendControllerStatus(
    const std::string& serverUrl,
    bool cameraConnected,
    bool rfidConnected,
    bool loopActive,
    bool irBlocked,
    bool barrierOpen,
    bool trafficGreen,
    bool plateUnrecognized,
    bool detectorActive,
    const std::string& gateState
) {
    if (serverUrl.empty()) return false;
    CURL* client = curl_easy_init();
    if (!client) return false;
    const std::string endpoint = serverEndpoint(serverUrl, "/api/reader/status");
    curl_mime* form = curl_mime_init(client);
    const auto addField = [&form](const char* name, const std::string& value) {
        curl_mimepart* part = curl_mime_addpart(form);
        curl_mime_name(part, name);
        curl_mime_data(part, value.c_str(), CURL_ZERO_TERMINATED);
    };
    const auto booleanText = [](bool value) { return value ? "1" : "0"; };
    addField("camera_connected", booleanText(cameraConnected));
    addField("rfid_connected", booleanText(rfidConnected));
    addField("controller_id", gControllerId);
    addField("loop_active", booleanText(loopActive));
    addField("ir_blocked", booleanText(irBlocked));
    addField("barrier_open", booleanText(barrierOpen));
    addField("traffic_green", booleanText(trafficGreen));
    addField("plate_unrecognized", booleanText(plateUnrecognized));
    addField("detector_state", detectorActive ? "active" : "idle");
    addField("gate_state", gateState);
    std::string responseBody;
    curl_easy_setopt(client, CURLOPT_URL, endpoint.c_str());
    curl_easy_setopt(client, CURLOPT_MIMEPOST, form);
    // A Cloudflare Tunnel may need more than one second for DNS, TLS, tunnel
    // routing, and the origin response. Telemetry runs asynchronously, so a
    // larger connection window does not block gate control or camera capture.
    curl_easy_setopt(client, CURLOPT_CONNECTTIMEOUT_MS, 5000L);
    curl_easy_setopt(client, CURLOPT_TIMEOUT_MS, 10000L);
    curl_easy_setopt(client, CURLOPT_WRITEFUNCTION, appendHttpResponse);
    curl_easy_setopt(client, CURLOPT_WRITEDATA, &responseBody);
    const CURLcode result = curl_easy_perform(client);
    long status = 0;
    curl_easy_getinfo(client, CURLINFO_RESPONSE_CODE, &status);
    curl_mime_free(form);
    curl_easy_cleanup(client);
    return result == CURLE_OK && status >= 200 && status < 300;
}

bool serverHealthCheck(const std::string& serverUrl, std::string& errorMessage) {
    errorMessage.clear();
    if (serverUrl.empty()) {
        errorMessage = "PLATE_SERVER_URL is not configured";
        return false;
    }
    CURL* client = curl_easy_init();
    if (!client) {
        errorMessage = "unable to initialize HTTP client";
        return false;
    }
    std::string responseBody;
    const std::string endpoint = serverEndpoint(serverUrl, "/health");
    curl_easy_setopt(client, CURLOPT_URL, endpoint.c_str());
    curl_easy_setopt(client, CURLOPT_CONNECTTIMEOUT_MS, 5000L);
    curl_easy_setopt(client, CURLOPT_TIMEOUT_MS, 10000L);
    curl_easy_setopt(client, CURLOPT_WRITEFUNCTION, appendHttpResponse);
    curl_easy_setopt(client, CURLOPT_WRITEDATA, &responseBody);
    const CURLcode result = curl_easy_perform(client);
    long status = 0;
    curl_easy_getinfo(client, CURLINFO_RESPONSE_CODE, &status);
    curl_easy_cleanup(client);
    if (result != CURLE_OK) {
        errorMessage = curl_easy_strerror(result);
        return false;
    }
    if (status < 200 || status >= 300) {
        errorMessage = "website returned HTTP " + std::to_string(status);
        return false;
    }
    return true;
}

bool parseJsonLong(const std::string& body, const std::string& field, long& value) {
    const std::string key = "\"" + field + "\"";
    std::size_t position = body.find(key);
    if (position == std::string::npos) {
        return false;
    }
    position = body.find(':', position + key.size());
    if (position == std::string::npos) {
        return false;
    }
    ++position;
    while (position < body.size() && std::isspace(static_cast<unsigned char>(body[position]))) {
        ++position;
    }
    std::size_t end = position;
    while (end < body.size() && std::isdigit(static_cast<unsigned char>(body[end]))) {
        ++end;
    }
    if (end == position) {
        return false;
    }
    try {
        value = std::stol(body.substr(position, end - position));
        return value > 0;
    } catch (const std::exception&) {
        return false;
    }
}

bool parseJsonBool(const std::string& body, const std::string& field, bool& value) {
    const std::string key = "\"" + field + "\"";
    std::size_t position = body.find(key);
    if (position == std::string::npos) {
        return false;
    }
    position = body.find(':', position + key.size());
    if (position == std::string::npos) {
        return false;
    }
    ++position;
    while (position < body.size() && std::isspace(static_cast<unsigned char>(body[position]))) {
        ++position;
    }
    if (body.compare(position, 4, "true") == 0) {
        value = true;
        return true;
    }
    if (body.compare(position, 5, "false") == 0) {
        value = false;
        return true;
    }
    return false;
}

bool parseJsonString(
    const std::string& body,
    const std::string& field,
    std::string& value
) {
    const std::string key = "\"" + field + "\"";
    std::size_t position = body.find(key);
    if (position == std::string::npos) return false;
    position = body.find(':', position + key.size());
    if (position == std::string::npos) return false;
    position = body.find('"', position + 1);
    if (position == std::string::npos) return false;
    const std::size_t end = body.find('"', position + 1);
    if (end == std::string::npos) return false;
    value = body.substr(position + 1, end - position - 1);
    return true;
}

std::string decodeHexBytes(const std::string& hexadecimal) {
    if (hexadecimal.empty() || hexadecimal.size() % 2 != 0) return {};
    std::string bytes;
    bytes.reserve(hexadecimal.size() / 2);
    for (std::size_t index = 0; index < hexadecimal.size(); index += 2) {
        const std::string pair = hexadecimal.substr(index, 2);
        try {
            std::size_t consumed = 0;
            const unsigned long value = std::stoul(pair, &consumed, 16);
            if (consumed != 2 || value > 255) return {};
            bytes.push_back(static_cast<char>(value));
        } catch (const std::exception&) {
            return {};
        }
    }
    return bytes;
}

std::string printableSerialText(const std::string& bytes) {
    std::ostringstream output;
    for (const unsigned char byte : bytes) {
        if (byte == '\r') output << "\\r";
        else if (byte == '\n') output << "\\n\n";
        else if (byte == '\t') output << "\\t";
        else if (std::isprint(byte)) output << static_cast<char>(byte);
        else output << "<0x" << std::uppercase << std::hex
                    << std::setw(2) << std::setfill('0')
                    << static_cast<unsigned int>(byte) << std::dec << '>';
    }
    return output.str();
}

long environmentLong(const char* name, long fallback) {
    const char* raw = std::getenv(name);
    if (!raw || *raw == '\0') {
        return fallback;
    }
    try {
        return std::stol(raw);
    } catch (const std::exception&) {
        throw std::runtime_error(std::string("Invalid numeric value for ") + name);
    }
}

RemoteCommandResult pollRemoteCommand(const std::string& serverUrl) {
    RemoteCommandResult poll;
    if (serverUrl.empty()) {
        poll.command = RemoteCommandPoll::Error;
        poll.error = "PLATE_SERVER_URL is not configured";
        return poll;
    }

    CURL* client = curl_easy_init();
    if (!client) {
        poll.command = RemoteCommandPoll::Error;
        poll.error = "unable to initialize HTTP client";
        return poll;
    }
    std::string responseBody;
    const std::string endpoint = serverEndpoint(serverUrl, "/api/reader/commands/next");
    curl_easy_setopt(client, CURLOPT_URL, endpoint.c_str());
    curl_easy_setopt(client, CURLOPT_POST, 1L);
    const std::string requestBody = "controller_id=" + gControllerId;
    curl_easy_setopt(client, CURLOPT_POSTFIELDS, requestBody.c_str());
    curl_easy_setopt(client, CURLOPT_CONNECTTIMEOUT_MS, 5000L);
    curl_easy_setopt(client, CURLOPT_TIMEOUT_MS, 15000L);
    curl_easy_setopt(client, CURLOPT_WRITEFUNCTION, appendHttpResponse);
    curl_easy_setopt(client, CURLOPT_WRITEDATA, &responseBody);

    const CURLcode result = curl_easy_perform(client);
    long status = 0;
    curl_easy_getinfo(client, CURLINFO_RESPONSE_CODE, &status);
    curl_easy_cleanup(client);

    if (result != CURLE_OK) {
        poll.command = RemoteCommandPoll::Error;
        poll.error = curl_easy_strerror(result);
        return poll;
    }
    if (status == 204) {
        return poll;
    }
    if (status < 200 || status >= 300) {
        poll.command = RemoteCommandPoll::Error;
        poll.error = responseBody.empty()
            ? "website returned HTTP " + std::to_string(status)
            : responseBody;
        return poll;
    }
    std::string command;
    if (!parseJsonString(responseBody, "command", command) ||
        !parseJsonLong(responseBody, "command_id", poll.commandId)) {
        poll.command = RemoteCommandPoll::Error;
        poll.error = "website returned an invalid controller command";
        return poll;
    }
    if (command == "capture") poll.command = RemoteCommandPoll::Capture;
    else if (command == "barrier_open") poll.command = RemoteCommandPoll::BarrierOpen;
    else if (command == "barrier_close") poll.command = RemoteCommandPoll::BarrierClose;
    else if (command == "traffic_red") poll.command = RemoteCommandPoll::TrafficRed;
    else if (command == "traffic_green") poll.command = RemoteCommandPoll::TrafficGreen;
    else if (command == "rfid_serial") {
        long baud = 0;
        long dataBits = 0;
        long stopBits = 0;
        long timeoutMs = 0;
        std::string parity;
        if (!parseJsonString(responseBody, "tx_hex", poll.serialTxHex) ||
            !parseJsonLong(responseBody, "baud", baud) ||
            !parseJsonLong(responseBody, "data_bits", dataBits) ||
            !parseJsonString(responseBody, "parity", parity) ||
            !parseJsonLong(responseBody, "stop_bits", stopBits) ||
            !parseJsonLong(responseBody, "timeout_ms", timeoutMs) ||
            parity.size() != 1) {
            poll.command = RemoteCommandPoll::Error;
            poll.error = "website returned invalid RFID serial settings";
            return poll;
        }
        poll.serialSettings = {
            static_cast<int>(baud),
            static_cast<int>(dataBits),
            parity.front(),
            static_cast<int>(stopBits)
        };
        poll.serialTimeoutMs = timeoutMs;
        poll.command = RemoteCommandPoll::RfidSerial;
    }
    else {
        poll.command = RemoteCommandPoll::Error;
        poll.error = "website returned an unsupported controller command";
    }
    return poll;
}

bool reportRemoteCommand(
    const std::string& serverUrl,
    long commandId,
    bool success,
    const std::string& message,
    long long framesMilliseconds = -1,
    long long yoloMilliseconds = -1,
    long long ocrMilliseconds = -1,
    long long serverMilliseconds = -1,
    long long totalMilliseconds = -1,
    const std::string& responseData = ""
) {
    if (commandId <= 0) {
        return true;
    }
    CURL* client = curl_easy_init();
    if (!client) {
        return false;
    }
    const std::string endpoint = serverEndpoint(
        serverUrl,
        "/api/reader/commands/" + std::to_string(commandId) + "/complete"
    );
    curl_mime* form = curl_mime_init(client);
    curl_mimepart* part = curl_mime_addpart(form);
    curl_mime_name(part, "status");
    curl_mime_data(part, success ? "completed" : "failed", CURL_ZERO_TERMINATED);
    part = curl_mime_addpart(form);
    curl_mime_name(part, "message");
    curl_mime_data(part, message.c_str(), CURL_ZERO_TERMINATED);
    part = curl_mime_addpart(form);
    curl_mime_name(part, "controller_id");
    curl_mime_data(part, gControllerId.c_str(), CURL_ZERO_TERMINATED);
    if (!responseData.empty()) {
        part = curl_mime_addpart(form);
        curl_mime_name(part, "response_data");
        curl_mime_data(part, responseData.c_str(), CURL_ZERO_TERMINATED);
    }
    const auto addTiming = [&form](
        const char* name,
        long long value
    ) {
        if (value < 0) {
            return;
        }
        curl_mimepart* timingPart = curl_mime_addpart(form);
        curl_mime_name(timingPart, name);
        const std::string valueText = std::to_string(value);
        curl_mime_data(timingPart, valueText.c_str(), CURL_ZERO_TERMINATED);
    };
    addTiming("frames_ms", framesMilliseconds);
    addTiming("yolo_ms", yoloMilliseconds);
    addTiming("ocr_ms", ocrMilliseconds);
    addTiming("server_ms", serverMilliseconds);
    addTiming("total_ms", totalMilliseconds);

    std::string responseBody;
    curl_easy_setopt(client, CURLOPT_URL, endpoint.c_str());
    curl_easy_setopt(client, CURLOPT_MIMEPOST, form);
    curl_easy_setopt(client, CURLOPT_CONNECTTIMEOUT_MS, 5000L);
    curl_easy_setopt(client, CURLOPT_TIMEOUT_MS, 15000L);
    curl_easy_setopt(client, CURLOPT_WRITEFUNCTION, appendHttpResponse);
    curl_easy_setopt(client, CURLOPT_WRITEDATA, &responseBody);
    const CURLcode result = curl_easy_perform(client);
    long status = 0;
    curl_easy_getinfo(client, CURLINFO_RESPONSE_CODE, &status);
    curl_mime_free(form);
    curl_easy_cleanup(client);
    return result == CURLE_OK && status >= 200 && status < 300;
}

std::string eventSnapshotName(const std::string& plate, int frameNumber) {
    const auto now = std::chrono::system_clock::now();
    const std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
    std::tm localTime{};
#ifdef _WIN32
    localtime_s(&localTime, &timestamp);
#else
    localtime_r(&timestamp, &localTime);
#endif
    std::ostringstream name;
    name << std::put_time(&localTime, "%Y%m%d-%H%M%S")
         << '-' << plate << '-' << frameNumber << ".jpg";
    return name.str();
}

int runCamera(
    cv::dnn::Net& detector,
    cv::dnn::Net& recognizer,
    int cameraIndex,
    const fs::path& outputDirectory,
    const std::string& serverUrl,
    bool headless,
    const fs::path& commandFile,
    bool remoteCommands,
    bool gateMode
) {
    std::cout << std::unitbuf;
#ifdef PLATE_ENABLE_GPIO
    std::unique_ptr<gate::StatusLeds> statusLeds;
    gate::StatusLedPins statusLedPins;
    try {
        const auto configuredGpio = [](const char* name, long fallback) {
            const long value = environmentLong(name, fallback);
            if (value < 0 || value > 27) {
                throw std::runtime_error(
                    std::string(name) + " must be a BCM GPIO number from 0 to 27"
                );
            }
            return static_cast<unsigned int>(value);
        };
        statusLedPins.camera = configuredGpio("CAMERA_STATUS_LED_GPIO", 25);
        statusLedPins.server = configuredGpio("SERVER_STATUS_LED_GPIO", 5);
        statusLedPins.loop = configuredGpio("LOOP_STATUS_LED_GPIO", 6);
        statusLedPins.barrierOpen = configuredGpio(
            "BARRIER_OPEN_STATUS_LED_GPIO", 12
        );
        statusLedPins.plateUnrecognized = configuredGpio(
            "PLATE_UNRECOGNIZED_LED_GPIO", 13
        );
        const std::string chipPath = std::getenv("GATE_GPIO_CHIP")
            ? std::getenv("GATE_GPIO_CHIP")
            : "/dev/gpiochip0";
        statusLeds = std::make_unique<gate::StatusLeds>(chipPath, statusLedPins);
        std::cout
            << "Status LEDs ready: camera=" << statusLedPins.camera
            << " server=" << statusLedPins.server
            << " loop=" << statusLedPins.loop
            << " barrier-open=" << statusLedPins.barrierOpen
            << " plate-unrecognized=" << statusLedPins.plateUnrecognized
            << " (all currently off).\n";
    } catch (const std::exception& error) {
        std::cerr << "Unable to start status LEDs: " << error.what() << '\n';
        return 1;
    }
#endif
    int requestedWidth = 3840;
    int requestedHeight = 2160;
    int requestedFps = 30;
    std::string requestedFourcc = "MJPG";
    try {
        requestedWidth = static_cast<int>(environmentLong("CAMERA_WIDTH", requestedWidth));
        requestedHeight = static_cast<int>(environmentLong("CAMERA_HEIGHT", requestedHeight));
        requestedFps = static_cast<int>(environmentLong("CAMERA_FPS", requestedFps));
    } catch (const std::exception& error) {
        std::cerr << "Invalid camera configuration: " << error.what() << '\n';
        return 1;
    }
    if (const char* configuredFourcc = std::getenv("CAMERA_FOURCC")) {
        requestedFourcc = configuredFourcc;
    }
    std::transform(
        requestedFourcc.begin(),
        requestedFourcc.end(),
        requestedFourcc.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::toupper(character));
        }
    );
    if (requestedWidth <= 0 || requestedHeight <= 0 || requestedFps <= 0 ||
        requestedFourcc.size() != 4) {
        std::cerr
            << "Camera settings require positive width, height, and FPS values, "
            << "plus a four-character CAMERA_FOURCC value.\n";
        return 1;
    }

    cv::VideoCapture camera;
#ifdef __APPLE__
    camera.open(cameraIndex, cv::CAP_AVFOUNDATION);
#else
    camera.open(cameraIndex, cv::CAP_V4L2);
#endif
    if (!camera.isOpened()) {
        camera.open(cameraIndex);
    }
    if (!camera.isOpened()) {
        std::cerr << "Unable to open camera " << cameraIndex << ".\n";
        return 1;
    }

    const int requestedFourccCode = cv::VideoWriter::fourcc(
        requestedFourcc[0],
        requestedFourcc[1],
        requestedFourcc[2],
        requestedFourcc[3]
    );
#ifndef __APPLE__
    // The EMEET C950 4K supports 3840x2160 at 30 FPS and exposes compressed
    // MJPEG through UVC. Select the format before the dimensions so V4L2 does
    // not fall back to a lower uncompressed mode because of USB bandwidth.
    camera.set(cv::CAP_PROP_FOURCC, requestedFourccCode);
#endif
    camera.set(cv::CAP_PROP_FRAME_WIDTH, requestedWidth);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, requestedHeight);
    camera.set(cv::CAP_PROP_FPS, requestedFps);
    camera.set(cv::CAP_PROP_AUTOFOCUS, 1);
    camera.set(cv::CAP_PROP_BUFFERSIZE, 1);

    cv::Mat cameraProbe;
    if (!camera.read(cameraProbe) || cameraProbe.empty()) {
        std::cerr << "Camera opened, but its first frame could not be read.\n";
        camera.release();
        return 1;
    }
    const int actualWidth = cameraProbe.cols;
    const int actualHeight = cameraProbe.rows;
    const double actualFps = camera.get(cv::CAP_PROP_FPS);
    const int actualFourccCode = static_cast<int>(camera.get(cv::CAP_PROP_FOURCC));
    std::string actualFourcc(4, ' ');
    for (int index = 0; index < 4; ++index) {
        const unsigned char value = static_cast<unsigned char>(
            (actualFourccCode >> (index * 8)) & 0xff
        );
        actualFourcc[index] = std::isprint(value)
            ? static_cast<char>(value)
            : '?';
    }
    std::cout << "Camera mode requested: " << requestedWidth << 'x'
              << requestedHeight << " @ " << requestedFps << " FPS, "
              << requestedFourcc << ".\n";
    std::cout << "Camera mode active: " << actualWidth << 'x' << actualHeight;
    if (actualFps > 0.0) {
        std::cout << " @ " << std::fixed << std::setprecision(1)
                  << actualFps << " FPS";
    }
    if (actualFourccCode != 0) {
        std::cout << ", " << actualFourcc;
    }
    std::cout << ".\n";
    if (actualWidth != requestedWidth || actualHeight != requestedHeight) {
        std::cerr
            << "WARNING: the webcam did not provide the requested resolution; "
            << "processing will continue at " << actualWidth << 'x' << actualHeight
            << ". Check the selected /dev/video device and its supported modes "
            << "with v4l2-ctl --list-formats-ext.\n";
    }
#ifdef PLATE_ENABLE_GPIO
    try {
        statusLeds->setCamera(true);
        std::cout << "CAMERA RECOGNIZED: status LED is on.\n";
    } catch (const std::exception& error) {
        std::cerr << "Unable to turn on camera status LED: " << error.what() << '\n';
        camera.release();
        return 1;
    }
#endif
    const auto cameraConnectionIsPresent = [&]() {
#if defined(PLATE_ENABLE_GPIO) && !defined(__APPLE__)
        const fs::path cameraDevice = "/dev/video" + std::to_string(cameraIndex);
        if (!fs::exists(cameraDevice)) {
            statusLeds->setCamera(false);
            return false;
        }
#endif
        return true;
    };
#ifdef PLATE_ENABLE_GPIO
    auto nextServerHealthCheck = std::chrono::steady_clock::time_point{};
    const auto refreshServerIndicator = [&](bool force = false) {
        const auto now = std::chrono::steady_clock::now();
        if (!force && now < nextServerHealthCheck) return;
        std::string healthError;
        const bool healthy = serverHealthCheck(serverUrl, healthError);
        statusLeds->setServer(healthy);
        nextServerHealthCheck = now + std::chrono::seconds(5);
    };
    try {
        refreshServerIndicator(true);
    } catch (const std::exception& error) {
        std::cerr << "Unable to update server status LED: " << error.what() << '\n';
        camera.release();
        return 1;
    }
#endif

    fs::create_directories(outputDirectory);
    const fs::path cropDirectory = outputDirectory / "Plate-Crops";
    fs::create_directories(cropDirectory);
    if (!commandFile.empty()) {
        if (!commandFile.parent_path().empty()) {
            fs::create_directories(commandFile.parent_path());
        }
        std::error_code error;
        fs::remove(commandFile, error);
    }
    if (!headless) {
        cv::namedWindow("On-demand License Plate Recognition", cv::WINDOW_NORMAL);
    }

    std::unique_ptr<gate::Controller> gateController;
    bool rfidEnabled = false;
    bool rfidConnected = false;
#ifdef PLATE_ENABLE_GPIO
    std::unique_ptr<gate::RaspberryPiGpio> gateGpio;
    std::unique_ptr<gate::SerialRfidReader> serialRfidReader;
    std::chrono::milliseconds rfidReadTimeout{2000};
#endif
    gate::State previousGateState = gate::State::Startup;
    if (gateMode) {
#ifdef PLATE_ENABLE_GPIO
        try {
        gate::Config gateConfig;
        gateConfig.inputDebounce = gate::Milliseconds(environmentLong(
            "GATE_INPUT_DEBOUNCE_MS", gateConfig.inputDebounce.count()
        ));
        gateConfig.relayPulse = gate::Milliseconds(environmentLong(
            "GATE_RELAY_PULSE_MS", 1000
        ));
        gateConfig.recognitionTimeout = gate::Milliseconds(environmentLong(
            "GATE_RECOGNITION_TIMEOUT_MS", gateConfig.recognitionTimeout.count()
        ));
        gateConfig.openingTravelTime = gate::Milliseconds(environmentLong(
            "GATE_OPENING_TRAVEL_MS", gateConfig.openingTravelTime.count()
        ));
        gateConfig.passageTimeout = gate::Milliseconds(environmentLong(
            "GATE_PASSAGE_TIMEOUT_MS", gateConfig.passageTimeout.count()
        ));
        gateConfig.clearanceTime = gate::Milliseconds(environmentLong(
            "GATE_CLEARANCE_MS", gateConfig.clearanceTime.count()
        ));
        gateConfig.closingTravelTime = gate::Milliseconds(environmentLong(
            "GATE_CLOSING_TRAVEL_MS", gateConfig.closingTravelTime.count()
        ));
        const auto configuredGateGpio = [](const char* name, long fallback) {
            const long value = environmentLong(name, fallback);
            if (value < 0 || value > 27) {
                throw std::runtime_error(
                    std::string(name) + " must be a BCM GPIO number from 0 to 27"
                );
            }
            return static_cast<unsigned int>(value);
        };
        gate::GpioPins pins;
        pins.loop = configuredGateGpio("GATE_LOOP_GPIO", 17);
        pins.passage = configuredGateGpio("GATE_PASSAGE_GPIO", 27);
        pins.traffic = configuredGateGpio("GATE_TRAFFIC_GPIO", 22);
        pins.open = configuredGateGpio("GATE_OPEN_GPIO", 23);
        pins.close = configuredGateGpio("GATE_CLOSE_GPIO", 24);
        const long configuredRfidEnabled = environmentLong("RFID_ENABLED", 0);
        if (configuredRfidEnabled != 0 && configuredRfidEnabled != 1) {
            throw std::runtime_error("RFID_ENABLED must be 0 or 1");
        }
        rfidEnabled = configuredRfidEnabled == 1;
        const long configuredRfidTimeout = environmentLong(
            "RFID_READ_TIMEOUT_MS", 5000
        );
        const long configuredRfidBaud = environmentLong("RFID_BAUD_RATE", 9600);
        const long configuredRfidMinLength = environmentLong("RFID_MIN_LENGTH", 4);
        const long configuredRfidMaxLength = environmentLong("RFID_MAX_LENGTH", 64);
        const long configuredRfidTagBytes = environmentLong("RFID_TAG_BYTES", 0);
        const std::string configuredRfidProtocol = std::getenv("RFID_PROTOCOL")
            ? std::getenv("RFID_PROTOCOL")
            : "uhfreader18";
        gate::RfidProtocol rfidProtocol = gate::RfidProtocol::UhfReader18;
        if (configuredRfidProtocol == "passive") {
            rfidProtocol = gate::RfidProtocol::PassiveStream;
        } else if (configuredRfidProtocol != "uhfreader18") {
            throw std::runtime_error(
                "RFID_PROTOCOL must be uhfreader18 or passive"
            );
        }
        if (rfidEnabled && (configuredRfidTimeout <= 0 ||
            configuredRfidTimeout > 30000)) {
            throw std::runtime_error(
                "RFID_READ_TIMEOUT_MS must be from 1 to 30000"
            );
        }
        rfidReadTimeout = std::chrono::milliseconds(configuredRfidTimeout);
        if (rfidEnabled && (configuredRfidTagBytes < 0 ||
            configuredRfidTagBytes > 64)) {
            throw std::runtime_error("RFID_TAG_BYTES must be from 0 to 64");
        }
        const auto conflictsWithStatusLed = [&statusLedPins](unsigned int pin) {
            return pin == statusLedPins.camera ||
                pin == statusLedPins.server ||
                pin == statusLedPins.loop ||
                pin == statusLedPins.barrierOpen ||
                pin == statusLedPins.plateUnrecognized;
        };
        if (conflictsWithStatusLed(pins.loop) ||
            conflictsWithStatusLed(pins.passage) ||
            conflictsWithStatusLed(pins.traffic) ||
            conflictsWithStatusLed(pins.open) ||
            conflictsWithStatusLed(pins.close)) {
            throw std::runtime_error(
                "A status LED GPIO conflicts with a gate GPIO assignment"
            );
        }
        const std::string chipPath = std::getenv("GATE_GPIO_CHIP")
            ? std::getenv("GATE_GPIO_CHIP")
            : "/dev/gpiochip0";
        gateController = std::make_unique<gate::Controller>(gateConfig);
        gateGpio = std::make_unique<gate::RaspberryPiGpio>(chipPath, pins);
        if (rfidEnabled) {
            const std::string serialDevice = std::getenv("RFID_SERIAL_DEVICE")
                ? std::getenv("RFID_SERIAL_DEVICE")
                : "/dev/serial0";
            serialRfidReader = std::make_unique<gate::SerialRfidReader>(
                serialDevice,
                static_cast<int>(configuredRfidBaud),
                static_cast<std::size_t>(configuredRfidMinLength),
                static_cast<std::size_t>(configuredRfidMaxLength),
                static_cast<std::size_t>(configuredRfidTagBytes),
                rfidProtocol
            );
            const std::string initializationError =
                rfidProtocol == gate::RfidProtocol::UhfReader18
                    ? serialRfidReader->initialize(std::chrono::milliseconds(2000))
                    : serialRfidReader->discardPending();
            rfidConnected = initializationError.empty();
            if (!initializationError.empty()) {
                std::cerr << "RFID INITIALIZATION WARNING: "
                          << initializationError
                          << ". Plate authorization remains available.\n";
            } else if (rfidProtocol == gate::RfidProtocol::UhfReader18) {
                std::cout << "RFID INITIALIZED: UHFReader18 Answer Mode; "
                             "single inventory will be requested on each gate cycle.\n";
            }
        }
        const auto initialInputs = gateGpio->readInputs();
        const auto initialStatus = gateController->update(
            std::chrono::steady_clock::now(), initialInputs
        );
        gateGpio->applyOutputs(initialStatus.outputs);
        statusLeds->setLoop(initialInputs.loopPresent);
        statusLeds->setBarrierOpen(false);
        statusLeds->setPlateUnrecognized(false);
        previousGateState = initialStatus.state;
        std::cout << "Gate GPIO ready: loop=" << pins.loop
                  << " IR=" << pins.passage
                  << " traffic=" << pins.traffic
                  << " open=" << pins.open
                  << " close=" << pins.close << ".\n";
        if (rfidEnabled) {
            std::cout << "RFID enabled: serial inventory over UART "
                      << (std::getenv("RFID_SERIAL_DEVICE")
                            ? std::getenv("RFID_SERIAL_DEVICE")
                            : "/dev/serial0")
                      << ", " << configuredRfidBaud << " baud, protocol "
                      << configuredRfidProtocol << ".\n";
        } else {
            std::cout << "RFID disabled: plate-only authorization remains active.\n";
        }
        } catch (const std::exception& error) {
            std::cerr << "Unable to start gate GPIO: " << error.what() << '\n';
            camera.release();
            return 1;
        }
#else
        std::cerr << "Gate mode requested, but GPIO support was not built.\n";
        camera.release();
        return 1;
#endif
    }
    std::cout << "Camera ready in on-demand mode. YOLO and OCR are idle.\n";
    if (gateMode) {
        std::cout << "Waiting for a grounded inductive-loop input.\n";
    } else if (remoteCommands) {
        std::cout << "Waiting for Capture requests from the website.\n";
    } else {
        std::cout << "Commands: capture | status | help | quit\n";
    }
    if (!remoteCommands && !commandFile.empty()) {
        std::cout << "Waiting for website commands in " << commandFile.string() << ".\n";
    }

    bool telemetryCameraConnected = true;
    bool telemetryLoopActive = false;
    bool telemetryIrBlocked = false;
    bool telemetryBarrierOpen = false;
    bool telemetryTrafficGreen = false;
    bool telemetryPlateUnrecognized = false;
    bool telemetryDetectorActive = false;
    std::string telemetryGateState = gateMode
        ? gate::stateName(previousGateState)
        : "disabled";
    std::future<bool> telemetryFuture;
    auto nextTelemetryAt = std::chrono::steady_clock::time_point{};
    const auto queueTelemetry = [&](bool force = false) {
        if (serverUrl.empty()) return;
        if (telemetryFuture.valid()) {
            if (telemetryFuture.wait_for(std::chrono::milliseconds(0)) !=
                std::future_status::ready) {
                return;
            }
            telemetryFuture.get();
        }
        const auto now = std::chrono::steady_clock::now();
        if (!force && now < nextTelemetryAt) return;
        nextTelemetryAt = now + std::chrono::seconds(1);
        telemetryFuture = std::async(
            std::launch::async,
            sendControllerStatus,
            serverUrl,
            telemetryCameraConnected,
            rfidConnected,
            telemetryLoopActive,
            telemetryIrBlocked,
            telemetryBarrierOpen,
            telemetryTrafficGreen,
            telemetryPlateUnrecognized,
            telemetryDetectorActive,
            telemetryGateState
        );
    };
    queueTelemetry(true);
    std::future<RemoteCommandResult> gateCommandFuture;
    auto nextGateCommandPollAt = std::chrono::steady_clock::time_point{};
    std::vector<std::future<bool>> commandReportFutures;
    const auto reportCommandAsync = [&](long commandId, bool success,
                                         const std::string& message,
                                         const std::string& responseData = "") {
        commandReportFutures.erase(
            std::remove_if(
                commandReportFutures.begin(),
                commandReportFutures.end(),
                [](std::future<bool>& result) {
                    if (result.wait_for(std::chrono::milliseconds(0)) !=
                        std::future_status::ready) {
                        return false;
                    }
                    result.get();
                    return true;
                }
            ),
            commandReportFutures.end()
        );
        commandReportFutures.push_back(std::async(
            std::launch::async,
            [serverUrl, commandId, success, message, responseData] {
                return reportRemoteCommand(
                    serverUrl, commandId, success, message,
                    -1, -1, -1, -1, -1, responseData
                );
            }
        ));
    };
    std::future<SerialDebugCompletion> serialDebugFuture;

    int captureNumber = 0;
    std::string command;
    while (true) {
        command.clear();
        telemetryDetectorActive = false;
        bool loopTriggeredCapture = false;
        long activeCommandId = 0;
        if (gateMode) {
#ifdef PLATE_ENABLE_GPIO
            while (command.empty()) {
                telemetryCameraConnected = cameraConnectionIsPresent();
                try {
                    const gate::Inputs inputs = gateGpio->readInputs();
                    const gate::Snapshot status = gateController->update(
                        std::chrono::steady_clock::now(), inputs
                    );
                    gateGpio->applyOutputs(status.outputs);
                    telemetryLoopActive = inputs.loopPresent;
                    telemetryIrBlocked = inputs.passageBlocked;
                    if (status.state == gate::State::Opening) {
                        telemetryBarrierOpen = true;
                    } else if (
                        status.state == gate::State::Closing ||
                        status.state == gate::State::Rearming ||
                        status.state == gate::State::IdleClosed
                    ) {
                        telemetryBarrierOpen = false;
                    }
                    telemetryTrafficGreen = status.outputs.trafficGreen;
                    telemetryGateState = gate::stateName(status.state);
                    queueTelemetry();
                    const auto pollNow = std::chrono::steady_clock::now();
                    if (serialDebugFuture.valid() &&
                        serialDebugFuture.wait_for(std::chrono::milliseconds(0)) ==
                            std::future_status::ready) {
                        const SerialDebugCompletion completion =
                            serialDebugFuture.get();
                        std::ostringstream terminal;
                        terminal << "TX HEX: " << completion.transmittedHex << '\n';
                        if (!completion.result.received.empty()) {
                            terminal << "RX HEX: "
                                     << gate::encodeRfidBytes(
                                            completion.result.received
                                        ) << '\n'
                                     << "RX TEXT: "
                                     << printableSerialText(
                                            completion.result.received
                                        ) << '\n';
                        } else {
                            terminal << "RX HEX: (no data)\nRX TEXT: (no data)\n";
                        }
                        if (!completion.result.error.empty()) {
                            terminal << "ERROR: " << completion.result.error << '\n';
                        }
                        reportCommandAsync(
                            completion.commandId,
                            completion.result.error.empty(),
                            completion.result.error.empty()
                                ? "RFID serial transaction completed"
                                : "RFID serial transaction failed",
                            terminal.str()
                        );
                    }
                    if (gateCommandFuture.valid() &&
                        gateCommandFuture.wait_for(std::chrono::milliseconds(0)) ==
                            std::future_status::ready) {
                        const RemoteCommandResult remote = gateCommandFuture.get();
                        if (remote.command == RemoteCommandPoll::Capture) {
                            activeCommandId = remote.commandId;
                            telemetryDetectorActive = true;
                            command = "capture";
                            std::cout << "REMOTE CAPTURE " << activeCommandId
                                      << ": received from website.\n";
                        } else if (remote.command == RemoteCommandPoll::BarrierOpen) {
                            const bool accepted = gateController->manualOpen(
                                pollNow, inputs
                            );
                            reportCommandAsync(
                                remote.commandId,
                                accepted,
                                accepted
                                    ? "Manual boom-barrier OPEN accepted"
                                    : "Manual OPEN rejected because the IR beam is blocked"
                            );
                        } else if (remote.command == RemoteCommandPoll::BarrierClose) {
                            const bool accepted = gateController->manualClose(
                                pollNow, inputs
                            );
                            reportCommandAsync(
                                remote.commandId,
                                accepted,
                                accepted
                                    ? "Manual boom-barrier CLOSE accepted"
                                    : "Manual CLOSE rejected because the IR beam is blocked"
                            );
                        } else if (remote.command == RemoteCommandPoll::TrafficGreen ||
                                   remote.command == RemoteCommandPoll::TrafficRed) {
                            const bool green =
                                remote.command == RemoteCommandPoll::TrafficGreen;
                            gateController->testTrafficSignal(
                                pollNow, green, std::chrono::seconds(3)
                            );
                            reportCommandAsync(
                                remote.commandId,
                                true,
                                green
                                    ? "Traffic signal GREEN test active for 3 seconds"
                                    : "Traffic signal RED test active for 3 seconds"
                            );
                        } else if (remote.command == RemoteCommandPoll::RfidSerial) {
                            const std::string transmitted = decodeHexBytes(
                                remote.serialTxHex
                            );
                            if (!rfidEnabled || !serialRfidReader) {
                                reportCommandAsync(
                                    remote.commandId,
                                    false,
                                    "RFID serial debugging is unavailable because RFID is disabled"
                                );
                            } else if (transmitted.empty() ||
                                       remote.serialTimeoutMs < 50 ||
                                       remote.serialTimeoutMs > 10000) {
                                reportCommandAsync(
                                    remote.commandId,
                                    false,
                                    "RFID serial debug settings were rejected by the controller"
                                );
                            } else if (serialDebugFuture.valid()) {
                                reportCommandAsync(
                                    remote.commandId,
                                    false,
                                    "Another RFID serial transaction is still running"
                                );
                            } else {
                                const std::string serialDevice =
                                    std::getenv("RFID_SERIAL_DEVICE")
                                        ? std::getenv("RFID_SERIAL_DEVICE")
                                        : "/dev/serial0";
                                serialDebugFuture = std::async(
                                    std::launch::async,
                                    [serialDevice, remote, transmitted] {
                                        return SerialDebugCompletion{
                                            remote.commandId,
                                            remote.serialTxHex,
                                            gate::transactSerial(
                                                serialDevice,
                                                remote.serialSettings,
                                                transmitted,
                                                std::chrono::milliseconds(
                                                    remote.serialTimeoutMs
                                                )
                                            )
                                        };
                                    }
                                );
                            }
                        }
                        nextGateCommandPollAt = pollNow +
                            std::chrono::milliseconds(250);
                    }
                    if (command.empty() && !gateCommandFuture.valid() &&
                        pollNow >= nextGateCommandPollAt) {
                        gateCommandFuture = std::async(
                            std::launch::async,
                            pollRemoteCommand,
                            serverUrl
                        );
                        nextGateCommandPollAt = pollNow +
                            std::chrono::milliseconds(500);
                    }
                    if (status.state == gate::State::IdleClosed) {
                        // A health request may block briefly. Only perform it
                        // while the barrier is closed, never during movement.
                        refreshServerIndicator();
                    }
                    statusLeds->setLoop(inputs.loopPresent);
                    if (status.state == gate::State::Opening) {
                        statusLeds->setBarrierOpen(true);
                    } else if (
                        status.state == gate::State::Closing ||
                        status.state == gate::State::Rearming ||
                        status.state == gate::State::IdleClosed
                    ) {
                        statusLeds->setBarrierOpen(false);
                    }
                    if (
                        status.state == gate::State::Recognizing ||
                        status.state == gate::State::IdleClosed
                    ) {
                        statusLeds->setPlateUnrecognized(false);
                        telemetryPlateUnrecognized = false;
                    }
                    if (status.state != previousGateState) {
                        std::cout << "GATE STATE: " << gate::stateName(previousGateState)
                                  << " -> " << gate::stateName(status.state) << '\n';
                        if (!status.faultReason.empty()) {
                            std::cerr << "GATE FAULT: " << status.faultReason << '\n';
                        }
                        previousGateState = status.state;
                    }
                    if (command.empty() && status.state == gate::State::Recognizing) {
                        telemetryDetectorActive = true;
                        queueTelemetry(true);
                        loopTriggeredCapture = true;
                        command = "capture";
                    } else {
                        std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    }
                } catch (const std::exception& error) {
                    gateGpio->safeOutputs();
                    std::cerr << "GATE GPIO FAILURE: " << error.what() << '\n';
                    camera.release();
                    return 1;
                }
            }
#endif
        } else if (remoteCommands) {
            auto lastError = std::chrono::steady_clock::time_point{};
            while (command.empty()) {
                telemetryCameraConnected = cameraConnectionIsPresent();
                const RemoteCommandResult remote = pollRemoteCommand(serverUrl);
                const RemoteCommandPoll result = remote.command;
                activeCommandId = remote.commandId;
                queueTelemetry();
                if (result == RemoteCommandPoll::Capture) {
#ifdef PLATE_ENABLE_GPIO
                    statusLeds->setServer(true);
#endif
                    command = "capture";
                    telemetryDetectorActive = true;
                    queueTelemetry(true);
                    std::cout << "REMOTE CAPTURE " << activeCommandId << ": received from website.\n";
                } else if (result == RemoteCommandPoll::Error) {
#ifdef PLATE_ENABLE_GPIO
                    statusLeds->setServer(false);
#endif
                    const auto now = std::chrono::steady_clock::now();
                    if (lastError.time_since_epoch().count() == 0 ||
                        now - lastError >= std::chrono::seconds(10)) {
                        std::cerr << "WEBSITE POLL FAILED: " << remote.error << '\n';
                        lastError = now;
                    }
                } else if (result != RemoteCommandPoll::None) {
                    reportCommandAsync(
                        activeCommandId,
                        false,
                        "Hardware diagnostics require automatic GPIO gate mode"
                    );
                } else {
#ifdef PLATE_ENABLE_GPIO
                    statusLeds->setServer(true);
#endif
                }
                if (command.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            }
        } else if (commandFile.empty()) {
            std::cout << "plate-reader> " << std::flush;
            if (!std::getline(std::cin, command)) {
                break;
            }
        } else {
            while (command.empty()) {
                telemetryCameraConnected = cameraConnectionIsPresent();
                queueTelemetry();
#ifdef PLATE_ENABLE_GPIO
                refreshServerIndicator();
#endif
                std::ifstream input(commandFile);
                if (input) {
                    std::getline(input, command);
                    input.close();
                    std::error_code error;
                    fs::remove(commandFile, error);
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }
        command.erase(command.begin(), std::find_if(command.begin(), command.end(), [](unsigned char c) {
            return !std::isspace(c);
        }));
        command.erase(std::find_if(command.rbegin(), command.rend(), [](unsigned char c) {
            return !std::isspace(c);
        }).base(), command.end());
        std::transform(command.begin(), command.end(), command.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });

        if (command == "quit" || command == "exit" || command == "q") {
            break;
        }
        if (command == "help") {
            std::cout
                << "capture  Capture and evaluate two fresh 4K frames.\n"
                << "status   Show whether the camera and models are idle.\n"
                << "quit     Release the camera and stop the reader.\n";
            continue;
        }
        if (command == "status") {
            std::cout << "IDLE - camera open; waiting for the capture command.\n";
            continue;
        }
        if (command.empty()) {
            continue;
        }
        if (command != "capture") {
            std::cout << "Unknown command '" << command << "'. Type help for available commands.\n";
            continue;
        }

        ++captureNumber;
#ifdef PLATE_ENABLE_GPIO
        statusLeds->setPlateUnrecognized(false);
#endif
        telemetryPlateUnrecognized = false;
        const auto startedAt = std::chrono::steady_clock::now();
        std::string rfidTag;
        std::string rfidReadError;
        bool rfidRequired = false;
#ifdef PLATE_ENABLE_GPIO
        std::future<gate::RfidReadResult> rfidReadFuture;
        if (loopTriggeredCapture && rfidEnabled) {
            rfidRequired = true;
            try {
                rfidReadError = serialRfidReader->discardPending();
                if (rfidReadError.empty()) {
                    rfidReadFuture = std::async(
                        std::launch::async,
                        [&serialRfidReader, rfidReadTimeout] {
                            return serialRfidReader->readTag(rfidReadTimeout);
                        }
                    );
                }
                std::cout << "RFID INVENTORY: serial request and camera capture starting.\n";
            } catch (const std::exception& error) {
                gateGpio->safeOutputs();
                std::cerr << "RFID INVENTORY FAILURE: " << error.what() << '\n';
                camera.release();
                return 1;
            }
        }
#endif
        const auto millisecondsBetween = [](const auto& beginning, const auto& end) {
            return std::chrono::duration_cast<std::chrono::milliseconds>(end - beginning).count();
        };
        constexpr int captureFrameCount = 2;
        constexpr int ocrCandidateCount = 2;
        long long framesMilliseconds = 0;
        long long yoloMilliseconds = 0;
        long long ocrMilliseconds = 0;
        std::cout << "CAPTURE " << captureNumber
                  << ": acquiring two fresh 4K frames...\n";

        // Drop stale UVC buffers without retrieving and JPEG-decoding pixels
        // that will never be used. read() below decodes only retained frames.
        const auto flushStartedAt = std::chrono::steady_clock::now();
        for (int attempt = 0; attempt < 6; ++attempt) {
            if (!camera.grab()) {
                break;
            }
        }
        framesMilliseconds += millisecondsBetween(
            flushStartedAt,
            std::chrono::steady_clock::now()
        );

        std::vector<cv::Mat> frames;
        frames.reserve(captureFrameCount);
        std::vector<BurstCandidate> candidates;
        const std::string captureStem = fs::path(
            eventSnapshotName("CAPTURE", captureNumber)
        ).stem().string();

        const auto captureAndDetect = [&]() {
            const auto frameStartedAt = std::chrono::steady_clock::now();
            cv::Mat frame;
            if (!camera.read(frame) || frame.empty()) {
                framesMilliseconds += millisecondsBetween(
                    frameStartedAt,
                    std::chrono::steady_clock::now()
                );
                return false;
            }
            telemetryCameraConnected = true;
#ifdef PLATE_ENABLE_GPIO
            statusLeds->setCamera(true);
#endif
            frames.push_back(frame.clone());
            framesMilliseconds += millisecondsBetween(
                frameStartedAt,
                std::chrono::steady_clock::now()
            );

            const std::size_t frameIndex = frames.size() - 1;
            const auto yoloStartedAt = std::chrono::steady_clock::now();
            const cv::Mat& retainedFrame = frames[frameIndex];
            const std::vector<Detection> detections = detectPlates(
                detector,
                retainedFrame
            );
            yoloMilliseconds += millisecondsBetween(
                yoloStartedAt,
                std::chrono::steady_clock::now()
            );
            std::cout << "CAPTURE " << captureNumber << ": frame "
                      << frameIndex + 1 << '/' << captureFrameCount
                      << " - YOLO found "
                      << detections.size() << " plate region(s).\n";

            bool foundFrameCandidate = false;
            BurstCandidate bestFrameCandidate;
            for (const Detection& detection : detections) {
                const cv::Mat crop = retainedFrame(detection.box).clone();
                const double sharpness = plateSharpness(crop);
                const double exposure = plateExposureScore(crop);
                const double quality = plateCandidateQuality(
                    detection,
                    retainedFrame.size(),
                    sharpness,
                    exposure
                );
                if (!foundFrameCandidate || quality > bestFrameCandidate.quality) {
                    bestFrameCandidate = {
                        static_cast<int>(frameIndex),
                        detection,
                        zoomPlate(crop),
                        sharpness,
                        exposure,
                        quality
                    };
                    foundFrameCandidate = true;
                }
            }
            if (foundFrameCandidate) {
                candidates.push_back(std::move(bestFrameCandidate));
            }
            return true;
        };

        const auto rankCandidates = [&]() {
            std::sort(candidates.begin(), candidates.end(), [](
            const BurstCandidate& first,
            const BurstCandidate& second
            ) {
                return first.quality > second.quality;
            });
            if (candidates.size() > ocrCandidateCount) {
                candidates.resize(ocrCandidateCount);
            }
        };

        std::vector<OcrVote> votes;
        const auto readCandidates = [&]() {
            votes.clear();
            votes.reserve(candidates.size());
            for (std::size_t index = 0; index < candidates.size(); ++index) {
                const BurstCandidate& candidate = candidates[index];
                const auto ocrStartedAt = std::chrono::steady_clock::now();
                const OcrResult result = readPlate(recognizer, candidate.enhancedCrop);
                ocrMilliseconds += millisecondsBetween(
                    ocrStartedAt,
                    std::chrono::steady_clock::now()
                );
                votes.push_back({
                    result.text,
                    candidate.quality,
                    result.confidence,
                    index
                });
                std::cout << "CAPTURE " << captureNumber << ": OCR sample "
                          << index + 1 << '/' << candidates.size()
                          << " from frame " << candidate.frameIndex + 1
                          << " = " << result.text
                          << " (OCR confidence " << std::fixed << std::setprecision(2)
                          << result.confidence << ", quality " << candidate.quality
                          << ", sharpness " << std::setprecision(0)
                          << candidate.sharpness << ")\n";
            }
        };

        for (int attempt = 0; attempt < captureFrameCount; ++attempt) {
            captureAndDetect();
        }
        if (!candidates.empty()) {
            rankCandidates();
            readCandidates();
        }

#ifdef PLATE_ENABLE_GPIO
        if (rfidRequired) {
            if (rfidReadFuture.valid()) {
                const gate::RfidReadResult result = rfidReadFuture.get();
                rfidTag = result.tag;
                rfidReadError = result.error;
            }
            if (!rfidTag.empty()) {
                std::cout << "RFID READ: " << rfidTag << '\n';
            } else {
                std::cout << "RFID READ: NO VALID TAG";
                if (!rfidReadError.empty()) {
                    std::cout << " (" << rfidReadError << ')';
                }
                std::cout << '\n';
            }
        }
#endif

        if (frames.empty()) {
            telemetryCameraConnected = false;
            telemetryPlateUnrecognized = false;
            queueTelemetry(true);
#ifdef PLATE_ENABLE_GPIO
            statusLeds->setCamera(false);
            statusLeds->setPlateUnrecognized(false);
#endif
            const auto elapsed = millisecondsBetween(
                startedAt,
                std::chrono::steady_clock::now()
            );
            std::cerr << "CAPTURE " << captureNumber
                      << ": camera frames could not be read.\n";
            std::cerr << "CAPTURE " << captureNumber << " TIMING: frames="
                      << framesMilliseconds << " ms, total=" << elapsed << " ms.\n";
            reportRemoteCommand(
                serverUrl,
                activeCommandId,
                false,
                "Camera frames could not be read",
                framesMilliseconds,
                yoloMilliseconds,
                ocrMilliseconds,
                0,
                elapsed
            );
            if (gateMode) {
                gateController->recognitionCompleted(
                    std::chrono::steady_clock::now(),
                    false,
                    true,
                    "Camera frames could not be read"
                );
            }
            continue;
        }

        if (candidates.empty()) {
            telemetryPlateUnrecognized = true;
#ifdef PLATE_ENABLE_GPIO
            statusLeds->setPlateUnrecognized(true);
#endif
            bool rfidAuthorized = false;
            long long noPlateServerMilliseconds = 0;
            std::string noPlateServerResponse;
            cv::Mat noPlateAnnotatedFrame = frames.back().clone();
            const std::string noPlateLabel = "NO PLATE DETECTED";
            const double labelScale = std::max(
                1.0,
                static_cast<double>(noPlateAnnotatedFrame.cols) / 1600.0
            );
            const int labelThickness = std::max(2, cvRound(labelScale * 3.0));
            int labelBaseline = 0;
            const cv::Size labelSize = cv::getTextSize(
                noPlateLabel,
                cv::FONT_HERSHEY_SIMPLEX,
                labelScale,
                labelThickness,
                &labelBaseline
            );
            const int labelPadding = std::max(12, cvRound(labelScale * 12.0));
            const cv::Point labelOrigin(
                labelPadding * 2,
                labelPadding * 2 + labelSize.height
            );
            cv::rectangle(
                noPlateAnnotatedFrame,
                cv::Rect(
                    labelOrigin.x - labelPadding,
                    labelOrigin.y - labelSize.height - labelPadding,
                    labelSize.width + labelPadding * 2,
                    labelSize.height + labelBaseline + labelPadding * 2
                ),
                cv::Scalar(0, 0, 180),
                cv::FILLED
            );
            cv::putText(
                noPlateAnnotatedFrame,
                noPlateLabel,
                labelOrigin,
                cv::FONT_HERSHEY_SIMPLEX,
                labelScale,
                cv::Scalar(255, 255, 255),
                labelThickness,
                cv::LINE_AA
            );

            const auto serverStartedAt = std::chrono::steady_clock::now();
            const bool sent = sendRecognition(
                serverUrl,
                "UNREADABLE",
                0.0F,
                {},
                frames.back(),
                noPlateAnnotatedFrame,
                activeCommandId,
                rfidTag,
                rfidRequired,
                noPlateServerResponse
            );
            noPlateServerMilliseconds = millisecondsBetween(
                serverStartedAt,
                std::chrono::steady_clock::now()
            );
            bool serverAuthorized = false;
            bool serverRfidAuthorized = false;
            rfidAuthorized = sent && rfidRequired && !rfidTag.empty() &&
                parseJsonBool(
                    noPlateServerResponse,
                    "authorized",
                    serverAuthorized
                ) && serverAuthorized &&
                parseJsonBool(
                    noPlateServerResponse,
                    "rfid_authorized",
                    serverRfidAuthorized
                ) && serverRfidAuthorized;
#ifdef PLATE_ENABLE_GPIO
            statusLeds->setServer(sent);
#endif
            std::cout << (sent
                    ? "SERVER ACCEPTED NO-PLATE FRAME "
                    : "SERVER SEND FAILED NO-PLATE FRAME ")
                      << noPlateServerResponse << '\n';
            const auto elapsed = millisecondsBetween(
                startedAt,
                std::chrono::steady_clock::now()
            );
            std::cout << "CAPTURE " << captureNumber
                      << ": NO PLATE DETECTED IN THE CAPTURED FRAMES.\n";
            std::cout << "CAPTURE " << captureNumber << " TIMING: frames="
                      << framesMilliseconds << " ms, YOLO=" << yoloMilliseconds
                      << " ms, OCR=" << ocrMilliseconds
                      << " ms, server=" << noPlateServerMilliseconds
                      << " ms, total=" << elapsed << " ms.\n";
            std::cout << "CAPTURE " << captureNumber << ": complete in "
                      << elapsed << " ms; returning to IDLE.\n";
            reportRemoteCommand(
                serverUrl,
                activeCommandId,
                true,
                rfidAuthorized ? "RFID authorized; no plate detected" : "No plate detected",
                framesMilliseconds,
                yoloMilliseconds,
                ocrMilliseconds,
                noPlateServerMilliseconds,
                elapsed
            );
            if (gateMode) {
                gateController->recognitionCompleted(
                    std::chrono::steady_clock::now(), rfidAuthorized
                );
                std::cout << "GATE DECISION: "
                          << (rfidAuthorized
                                ? "authorized by registered RFID despite no plate detection."
                                : "denied because neither plate nor RFID was authorized.")
                          << '\n';
            }
            continue;
        }

        const std::string plate = consensusPlate(votes);
        std::size_t winnerIndex = 0;
        int winnerDistance = std::numeric_limits<int>::max();
        double winnerQuality = -1.0;
        for (const OcrVote& vote : votes) {
            const int distance = plate == "UNREADABLE" || vote.reading == "UNREADABLE"
                ? std::numeric_limits<int>::max() / 2
                : editDistance(vote.reading, plate);
            const double voteQuality = vote.quality * vote.ocrConfidence;
            if (distance < winnerDistance ||
                (distance == winnerDistance && voteQuality > winnerQuality)) {
                winnerIndex = vote.candidateIndex;
                winnerDistance = distance;
                winnerQuality = voteQuality;
            }
        }
        const BurstCandidate& winner = candidates[winnerIndex];
        cv::Mat annotatedFrame = frames[winner.frameIndex].clone();
        const fs::path cropPath = cropDirectory / (captureStem + "-plate.jpg");
        cv::imwrite(
            cropPath.string(),
            winner.enhancedCrop,
            {cv::IMWRITE_JPEG_QUALITY, 95}
        );

        cv::Scalar color(0, 200, 255);
        drawLabel(annotatedFrame, winner.detection.box, plate, color);
        std::cout << "CAPTURE " << captureNumber << ": CONSENSUS " << plate
                  << " from " << votes.size() << " OCR sample(s).\n";

        long long serverMilliseconds = 0;
        bool serverSent = false;
        bool authorizedByServer = false;
        std::string serverResultMessage = "OCR returned unreadable";
        if (plate == "UNREADABLE") {
            std::cout << "UNREADABLE - plate regions were detected but the OCR burst "
                      << "returned no value.\n";
        }
        if (plate != "UNREADABLE" || (rfidRequired && !rfidTag.empty())) {
            std::string serverResponse;
            const auto serverStartedAt = std::chrono::steady_clock::now();
            serverSent = sendRecognition(
                serverUrl,
                plate,
                winner.detection.confidence,
                cropPath,
                frames[winner.frameIndex],
                annotatedFrame,
                activeCommandId,
                rfidTag,
                rfidRequired,
                serverResponse
            );
            serverMilliseconds = millisecondsBetween(
                serverStartedAt,
                std::chrono::steady_clock::now()
            );
            if (serverSent) {
#ifdef PLATE_ENABLE_GPIO
                statusLeds->setServer(true);
#endif
                std::cout << "SERVER ACCEPTED " << plate << ' ' << serverResponse << '\n';
                serverResultMessage = "Recognized " + plate;
                if (!parseJsonBool(serverResponse, "authorized", authorizedByServer)) {
                    authorizedByServer = false;
                    std::cerr << "SERVER RESPONSE DID NOT INCLUDE AUTHORIZATION; access denied.\n";
                }
            } else {
#ifdef PLATE_ENABLE_GPIO
                statusLeds->setServer(false);
#endif
                std::cerr << "SERVER SEND FAILED " << plate << ": "
                          << serverResponse << '\n';
                serverResultMessage = "Recognition upload failed: " + serverResponse;
            }
        }
#ifdef PLATE_ENABLE_GPIO
        statusLeds->setPlateUnrecognized(
            plate == "UNREADABLE" || (serverSent && !authorizedByServer)
        );
#endif
        telemetryPlateUnrecognized =
            plate == "UNREADABLE" || (serverSent && !authorizedByServer);

        if (!headless) {
            cv::imshow("On-demand License Plate Recognition", annotatedFrame);
            cv::waitKey(1);
        }
        const auto completedAt = std::chrono::steady_clock::now();
        const auto elapsed = millisecondsBetween(startedAt, completedAt);
        std::cout << "CAPTURE " << captureNumber << " TIMING: frames="
                  << framesMilliseconds
                  << " ms, YOLO=" << yoloMilliseconds
                  << " ms, OCR=" << ocrMilliseconds
                  << " ms, server=" << serverMilliseconds
                  << " ms, total=" << elapsed << " ms.\n";
        std::cout << "CAPTURE " << captureNumber << ": complete in "
                  << elapsed << " ms; captured frames discarded, returning to IDLE.\n";
        reportRemoteCommand(
            serverUrl,
            activeCommandId,
            plate == "UNREADABLE" || serverSent,
            serverResultMessage,
            framesMilliseconds,
            yoloMilliseconds,
            ocrMilliseconds,
            serverMilliseconds,
            elapsed
        );
        if (gateMode) {
            gateController->recognitionCompleted(
                std::chrono::steady_clock::now(),
                plate != "UNREADABLE" && serverSent && authorizedByServer
            );
            std::cout << "GATE DECISION: "
                      << (plate != "UNREADABLE" && serverSent && authorizedByServer
                          ? "AUTHORIZED - switching green and pulsing OPEN."
                          : "DENIED - remaining red with no barrier pulse.")
                      << '\n';
        }
    }

    camera.release();
    if (!headless) {
        cv::destroyAllWindows();
    }
    return 0;
}
#endif

int main(int argc, char** argv) {
    gControllerId = controllerIdentity();
#ifdef PLATE_ENABLE_CAMERA
    if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) {
        std::cerr << "Unable to initialize the HTTP client runtime.\n";
        return 1;
    }
#endif
    const bool cameraMode = argc > 1 && std::string(argv[1]) == "--camera";
    const bool headless = std::find_if(
        argv + 1,
        argv + argc,
        [](const char* argument) { return std::string(argument) == "--headless"; }
    ) != argv + argc;
    const bool remoteCommands = std::find_if(
        argv + 1,
        argv + argc,
        [](const char* argument) { return std::string(argument) == "--remote-commands"; }
    ) != argv + argc;
    const bool gateMode = std::find_if(
        argv + 1,
        argv + argc,
        [](const char* argument) { return std::string(argument) == "--gate"; }
    ) != argv + argc;
    const int cameraIndex = cameraMode && argc > 2 ? std::stoi(argv[2]) : 0;
    const fs::path inputDirectory = cameraMode
        ? "raw-images"
        : (argc > 1 ? argv[1] : "raw-images");
    const fs::path outputDirectory = cameraMode
        ? (argc > 5 ? argv[5] : "Output")
        : (argc > 2 ? argv[2] : "Output");
    const fs::path modelPath = argc > 3
        ? argv[3]
        : "models/license_plate_detector.onnx";
    const fs::path ocrModelPath = argc > 4
        ? argv[4]
        : "models/en_PP-OCRv5_rec_mobile.onnx";
    fs::path commandFile;
    std::string serverUrl = std::getenv("PLATE_SERVER_URL")
        ? std::getenv("PLATE_SERVER_URL")
        : "";
    for (int index = 1; index + 1 < argc; ++index) {
        if (std::string(argv[index]) == "--command-file") {
            commandFile = argv[index + 1];
        } else if (std::string(argv[index]) == "--server-url") {
            serverUrl = argv[index + 1];
        }
    }
    const fs::path cropDirectory = outputDirectory / "Plate-Crops";
    fs::create_directories(cropDirectory);

    cv::dnn::Net detector = cv::dnn::readNetFromONNX(modelPath.string());
    detector.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);

    cv::dnn::Net recognizer = cv::dnn::readNetFromONNX(ocrModelPath.string());
    recognizer.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);

    if (cameraMode) {
#ifdef PLATE_ENABLE_CAMERA
        return runCamera(
            detector,
            recognizer,
            cameraIndex,
            outputDirectory,
            serverUrl,
            headless,
            commandFile,
            remoteCommands,
            gateMode
        );
#else
        std::cerr << "Camera support was disabled when this executable was built.\n";
        return 1;
#endif
    }

    std::vector<fs::path> inputs;
    for (const auto& entry : fs::directory_iterator(inputDirectory)) {
        if (entry.is_regular_file() && supportedImage(entry.path())) {
            inputs.push_back(entry.path());
        }
    }
    std::sort(inputs.begin(), inputs.end());

    for (const fs::path& path : inputs) {
        cv::Mat image = cv::imread(path.string(), cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cerr << path.filename().string() << ": unable to read image\n";
            continue;
        }

        const std::vector<Detection> detections = detectPlates(detector, image);
        std::vector<std::string> labels;
        for (std::size_t index = 0; index < detections.size(); ++index) {
            const cv::Mat crop = image(detections[index].box).clone();
            const cv::Mat zoomed = zoomPlate(crop);
            const fs::path cropPath = cropDirectory /
                (path.stem().string() + "_plate_" + std::to_string(index + 1) + ".jpg");
            cv::imwrite(cropPath.string(), zoomed, {cv::IMWRITE_JPEG_QUALITY, 95});

            const OcrResult result = readPlate(recognizer, zoomed);
            drawLabel(image, detections[index].box, result.text);
            labels.push_back(result.text);
        }

        const fs::path target = outputDirectory / path.filename();
        cv::imwrite(target.string(), image);
        std::cout << path.filename().string() << ": ";
        if (labels.empty()) {
            std::cout << "no plate detected";
        } else {
            for (std::size_t index = 0; index < labels.size(); ++index) {
                if (index > 0) std::cout << ", ";
                std::cout << labels[index];
            }
        }
        std::cout << '\n';
    }

    return 0;
}
