#include "serial_rfid_reader.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace gate {
namespace {

speed_t baudConstant(int baudRate) {
    switch (baudRate) {
        case 1200: return B1200;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default:
            throw std::invalid_argument(
                "RFID baud rate must be 1200, 2400, 4800, 9600, 19200, "
                "38400, 57600, or 115200"
            );
    }
}

class FileDescriptor {
public:
    explicit FileDescriptor(int descriptor) : descriptor_(descriptor) {}
    ~FileDescriptor() {
        if (descriptor_ >= 0) close(descriptor_);
    }
    FileDescriptor(const FileDescriptor&) = delete;
    FileDescriptor& operator=(const FileDescriptor&) = delete;
    int get() const noexcept { return descriptor_; }

private:
    int descriptor_;
};

std::string systemError(const std::string& operation) {
    return operation + ": " + std::strerror(errno);
}

std::string configureDescriptor(int descriptor, int baudRate) {
    termios settings{};
    if (tcgetattr(descriptor, &settings) < 0) {
        return systemError("Unable to read RFID serial settings");
    }
    cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~CSTOPB;
    settings.c_cflag &= ~PARENB;
    settings.c_cflag &= ~CSIZE;
    settings.c_cflag |= CS8;
    const speed_t speed = baudConstant(baudRate);
    if (cfsetispeed(&settings, speed) < 0 ||
        cfsetospeed(&settings, speed) < 0 ||
        tcsetattr(descriptor, TCSANOW, &settings) < 0) {
        return systemError("Unable to configure RFID serial port");
    }
    return {};
}

}  // namespace

std::string encodeRfidBytes(const std::string& bytes) {
    std::ostringstream encoded;
    encoded << std::uppercase << std::hex << std::setfill('0');
    for (unsigned char byte : bytes) {
        encoded << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return encoded.str();
}

std::string extractRfidTag(
    const std::string& bytes,
    std::size_t minimumLength,
    std::size_t maximumLength
) {
    if (minimumLength == 0 || maximumLength < minimumLength ||
        maximumLength > 128) {
        throw std::invalid_argument("Invalid RFID tag length limits");
    }
    std::string best;
    std::string current;
    const auto consider = [&] {
        if (current.size() >= minimumLength && current.size() <= maximumLength &&
            current.size() >= best.size()) {
            best = current;
        }
        current.clear();
    };
    for (unsigned char byte : bytes) {
        if (std::isalnum(byte)) {
            if (current.size() < maximumLength + 1) {
                current.push_back(static_cast<char>(std::toupper(byte)));
            }
        } else if (byte != '-') {
            consider();
        }
    }
    consider();
    return best;
}

SerialRfidReader::SerialRfidReader(
    std::string device,
    int baudRate,
    std::size_t minimumLength,
    std::size_t maximumLength,
    std::size_t expectedBytes
) : device_(std::move(device)),
    baudRate_(baudRate),
    minimumLength_(minimumLength),
    maximumLength_(maximumLength),
    expectedBytes_(expectedBytes) {
    if (device_.empty() || device_.front() != '/') {
        throw std::invalid_argument("RFID serial device must be an absolute path");
    }
    (void)baudConstant(baudRate_);
    if (minimumLength_ == 0 || maximumLength_ < minimumLength_ ||
        maximumLength_ > 128) {
        throw std::invalid_argument("Invalid RFID tag length limits");
    }
    if (expectedBytes_ > 64) {
        throw std::invalid_argument("RFID binary tag length cannot exceed 64 bytes");
    }
}

std::string SerialRfidReader::discardPending() const {
    const int descriptor = open(
        device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC
    );
    if (descriptor < 0) return systemError("Unable to open " + device_);
    FileDescriptor serial(descriptor);
    if (const std::string error = configureDescriptor(serial.get(), baudRate_);
        !error.empty()) {
        return error;
    }
    if (tcflush(serial.get(), TCIFLUSH) < 0) {
        return systemError("Unable to clear stale RFID serial data");
    }
    return {};
}

RfidReadResult SerialRfidReader::readTag(
    std::chrono::milliseconds timeout
) const {
    if (timeout.count() <= 0 || timeout > std::chrono::seconds(30)) {
        return {{}, "RFID read timeout is invalid"};
    }
    const int descriptor = open(
        device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC
    );
    if (descriptor < 0) return {{}, systemError("Unable to open " + device_)};
    FileDescriptor serial(descriptor);

    if (const std::string error = configureDescriptor(serial.get(), baudRate_);
        !error.empty()) {
        return {{}, error};
    }

    std::string received;
    received.reserve(256);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline && received.size() < 2048) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()
        );
        pollfd descriptorState{serial.get(), POLLIN, 0};
        const int result = poll(
            &descriptorState,
            1,
            static_cast<int>(std::max<long long>(1, remaining.count()))
        );
        if (result < 0) {
            if (errno == EINTR) continue;
            return {{}, systemError("RFID serial poll failed")};
        }
        if (result == 0) break;
        if (descriptorState.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            return {{}, "RFID serial port disconnected during read"};
        }
        std::array<char, 256> buffer{};
        const ssize_t count = read(serial.get(), buffer.data(), buffer.size());
        if (count > 0) {
            received.append(buffer.data(), static_cast<std::size_t>(count));
            if (expectedBytes_ > 0 && received.size() >= expectedBytes_) {
                return {
                    encodeRfidBytes(received.substr(0, expectedBytes_)),
                    {}
                };
            }
            const std::string tag = extractRfidTag(
                received, minimumLength_, maximumLength_
            );
            if (!tag.empty() && (
                    received.find('\r') != std::string::npos ||
                    received.find('\n') != std::string::npos ||
                    received.find('\x03') != std::string::npos
                )) {
                return {tag, {}};
            }
        } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
                   errno != EINTR) {
            return {{}, systemError("RFID serial read failed")};
        }
    }
    if (expectedBytes_ > 0) {
        return {{}, received.empty()
            ? "RFID reader timed out without sending a tag"
            : "RFID reader returned fewer bytes than the configured tag length"};
    }
    const std::string tag = extractRfidTag(
        received, minimumLength_, maximumLength_
    );
    if (!tag.empty()) return {tag, {}};
    return {{}, received.empty()
        ? "RFID reader timed out without sending a tag"
        : "RFID reader returned no valid alphanumeric tag"};
}

}  // namespace gate
