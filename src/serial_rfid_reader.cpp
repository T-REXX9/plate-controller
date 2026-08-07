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
#include <cstdint>
#include <iomanip>
#include <optional>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace gate {
namespace {

std::mutex serialPortMutex;

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

std::string configureDescriptor(
    int descriptor,
    const SerialDebugSettings& serial
) {
    termios settings{};
    if (tcgetattr(descriptor, &settings) < 0) {
        return systemError("Unable to read RFID serial settings");
    }
    cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
    if (serial.stopBits == 2) settings.c_cflag |= CSTOPB;
    else settings.c_cflag &= ~CSTOPB;
    settings.c_cflag &= ~(PARENB | PARODD);
    if (serial.parity == 'E') settings.c_cflag |= PARENB;
    else if (serial.parity == 'O') settings.c_cflag |= PARENB | PARODD;
    settings.c_cflag &= ~CSIZE;
    switch (serial.dataBits) {
        case 5: settings.c_cflag |= CS5; break;
        case 6: settings.c_cflag |= CS6; break;
        case 7: settings.c_cflag |= CS7; break;
        case 8: settings.c_cflag |= CS8; break;
        default: return "RFID serial data bits must be 5, 6, 7, or 8";
    }
    if (serial.stopBits != 1 && serial.stopBits != 2) {
        return "RFID serial stop bits must be 1 or 2";
    }
    if (serial.parity != 'N' && serial.parity != 'E' && serial.parity != 'O') {
        return "RFID serial parity must be N, E, or O";
    }
    const speed_t speed = baudConstant(serial.baudRate);
    if (cfsetispeed(&settings, speed) < 0 ||
        cfsetospeed(&settings, speed) < 0 ||
        tcsetattr(descriptor, TCSANOW, &settings) < 0) {
        return systemError("Unable to configure RFID serial port");
    }
    return {};
}

std::string configureDescriptor(int descriptor, int baudRate) {
    return configureDescriptor(
        descriptor, SerialDebugSettings{baudRate, 8, 'N', 1}
    );
}

std::string writeBytes(int descriptor, std::string_view bytes) {
    std::size_t offset = 0;
    while (offset < bytes.size()) {
        const ssize_t count = write(
            descriptor, bytes.data() + offset, bytes.size() - offset
        );
        if (count > 0) {
            offset += static_cast<std::size_t>(count);
            continue;
        }
        if (count < 0 &&
            (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            pollfd state{descriptor, POLLOUT, 0};
            if (poll(&state, 1, 500) > 0) continue;
        }
        return systemError("Unable to send RFID command");
    }
    if (tcdrain(descriptor) < 0) {
        return systemError("Unable to finish sending RFID command");
    }
    return {};
}

constexpr std::array<unsigned char, 11> kAnswerModeCommand{
    0x0A, 0x00, 0x35, 0x00, 0x02, 0x04, 0x02, 0x06, 0x00, 0xCD, 0x09
};
constexpr std::array<unsigned char, 5> kSingleInventoryCommand{
    0x04, 0x00, 0x0F, 0xA5, 0xA2
};

std::uint16_t frameCrc(std::string_view bytes) {
    std::uint16_t crc = 0xFFFF;
    for (const unsigned char byte : bytes) {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) != 0U
                ? static_cast<std::uint16_t>((crc >> 1U) ^ 0x8408U)
                : static_cast<std::uint16_t>(crc >> 1U);
        }
    }
    return crc;
}

bool validFrame(std::string_view frame) {
    if (frame.size() < 6U || frame.size() !=
        static_cast<std::size_t>(static_cast<unsigned char>(frame.front())) + 1U) {
        return false;
    }
    const std::uint16_t expected = frameCrc(frame.substr(0, frame.size() - 2U));
    const std::uint16_t actual =
        static_cast<unsigned char>(frame[frame.size() - 2U]) |
        (static_cast<std::uint16_t>(
            static_cast<unsigned char>(frame.back())) << 8U);
    return expected == actual;
}

std::vector<std::string_view> validFrames(std::string_view bytes) {
    std::vector<std::string_view> frames;
    std::size_t offset = 0;
    while (offset < bytes.size()) {
        const std::size_t length = static_cast<unsigned char>(bytes[offset]);
        const std::size_t total = length + 1U;
        if (length < 5U || length > 128U) {
            ++offset;
            continue;
        }
        if (offset + total > bytes.size()) break;
        const std::string_view candidate = bytes.substr(offset, total);
        if (validFrame(candidate)) {
            frames.push_back(candidate);
            offset += total;
        } else {
            ++offset;
        }
    }
    return frames;
}

std::string epcFromFrame(std::string_view frame) {
    if (!validFrame(frame) || frame.size() < 8U ||
        static_cast<unsigned char>(frame[3]) != 0x01U) {
        return {};
    }
    const unsigned char command = static_cast<unsigned char>(frame[2]);
    if (command != 0x0FU && command != 0x01U) return {};
    if (static_cast<unsigned char>(frame[4]) == 0U) return {};
    const std::size_t epcLength = static_cast<unsigned char>(frame[5]);
    if (epcLength == 0U || 6U + epcLength + 2U > frame.size()) return {};
    return encodeRfidBytes(std::string(frame.substr(6U, epcLength)));
}

template <std::size_t Size>
std::string writeCommand(
    int descriptor,
    const std::array<unsigned char, Size>& command
) {
    return writeBytes(
        descriptor,
        std::string_view(
            reinterpret_cast<const char*>(command.data()), command.size()
        )
    );
}

std::optional<std::string> readValidFrame(
    int descriptor,
    unsigned char expectedCommand,
    std::chrono::steady_clock::time_point deadline,
    std::string& error
) {
    std::string received;
    received.reserve(256);
    while (std::chrono::steady_clock::now() < deadline && received.size() < 4096U) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()
        );
        pollfd state{descriptor, POLLIN, 0};
        const int result = poll(
            &state, 1, static_cast<int>(std::max<long long>(1, remaining.count()))
        );
        if (result < 0) {
            if (errno == EINTR) continue;
            error = systemError("RFID serial poll failed");
            return std::nullopt;
        }
        if (result == 0) break;
        if (state.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            error = "RFID serial port disconnected during read";
            return std::nullopt;
        }
        std::array<char, 256> buffer{};
        const ssize_t count = read(descriptor, buffer.data(), buffer.size());
        if (count > 0) {
            received.append(buffer.data(), static_cast<std::size_t>(count));
            for (const std::string_view frame : validFrames(received)) {
                if (static_cast<unsigned char>(frame[2]) == expectedCommand) {
                    return std::string(frame);
                }
            }
        } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
                   errno != EINTR) {
            error = systemError("RFID serial read failed");
            return std::nullopt;
        }
    }
    error = "RFID reader did not return the expected command response";
    return std::nullopt;
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

std::string extractUhfReader18Epc(const std::string& bytes) {
    for (const std::string_view frame : validFrames(bytes)) {
        const std::string epc = epcFromFrame(frame);
        if (!epc.empty()) return epc;
    }
    return {};
}

SerialDebugResult transactSerial(
    const std::string& device,
    const SerialDebugSettings& settings,
    const std::string& transmitted,
    std::chrono::milliseconds timeout
) {
    if (device.empty() || device.front() != '/') {
        return {{}, "RFID serial device must be an absolute path"};
    }
    if (transmitted.empty() || transmitted.size() > 512U) {
        return {{}, "RFID debug transmission must contain 1 to 512 bytes"};
    }
    if (timeout.count() < 50 || timeout > std::chrono::seconds(10)) {
        return {{}, "RFID debug timeout must be from 50 to 10000 ms"};
    }
    std::lock_guard<std::mutex> lock(serialPortMutex);
    const int descriptor = open(
        device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC
    );
    if (descriptor < 0) return {{}, systemError("Unable to open " + device)};
    FileDescriptor serial(descriptor);
    std::string configurationError;
    try {
        configurationError = configureDescriptor(serial.get(), settings);
    } catch (const std::exception& error) {
        configurationError = error.what();
    }
    if (!configurationError.empty()) return {{}, configurationError};
    if (tcflush(serial.get(), TCIFLUSH) < 0) {
        return {{}, systemError("Unable to clear stale RFID serial data")};
    }
    if (const std::string error = writeBytes(serial.get(), transmitted);
        !error.empty()) {
        return {{}, error};
    }
    std::string received;
    received.reserve(512);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline && received.size() < 4096U) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()
        );
        pollfd state{serial.get(), POLLIN, 0};
        const int result = poll(
            &state, 1, static_cast<int>(std::max<long long>(1, remaining.count()))
        );
        if (result < 0) {
            if (errno == EINTR) continue;
            return {received, systemError("RFID serial poll failed")};
        }
        if (result == 0) break;
        if (state.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            return {received, "RFID serial port disconnected during debug read"};
        }
        std::array<char, 256> buffer{};
        const ssize_t count = read(serial.get(), buffer.data(), buffer.size());
        if (count > 0) {
            received.append(buffer.data(), static_cast<std::size_t>(count));
        } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
                   errno != EINTR) {
            return {received, systemError("RFID serial read failed")};
        }
    }
    return {received, {}};
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
    std::size_t expectedBytes,
    RfidProtocol protocol
) : device_(std::move(device)),
    baudRate_(baudRate),
    minimumLength_(minimumLength),
    maximumLength_(maximumLength),
    expectedBytes_(expectedBytes),
    protocol_(protocol) {
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

std::string SerialRfidReader::initialize(
    std::chrono::milliseconds timeout
) const {
    std::lock_guard<std::mutex> lock(serialPortMutex);
    if (protocol_ != RfidProtocol::UhfReader18) return {};
    if (timeout.count() <= 0 || timeout > std::chrono::seconds(10)) {
        return "RFID initialization timeout is invalid";
    }
    const int descriptor = open(
        device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC
    );
    if (descriptor < 0) return systemError("Unable to open " + device_);
    FileDescriptor serial(descriptor);
    if (const std::string error = configureDescriptor(serial.get(), baudRate_);
        !error.empty()) {
        return error;
    }
    if (tcflush(serial.get(), TCIOFLUSH) < 0) {
        return systemError("Unable to clear stale RFID serial data");
    }
    if (const std::string error = writeCommand(serial.get(), kAnswerModeCommand);
        !error.empty()) {
        return error;
    }
    std::string error;
    const auto response = readValidFrame(
        serial.get(), 0x35U, std::chrono::steady_clock::now() + timeout, error
    );
    if (!response) return error;
    if (response->size() < 6U || static_cast<unsigned char>((*response)[3]) != 0U) {
        std::ostringstream message;
        message << "RFID reader rejected Answer Mode with status 0x"
                << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<unsigned int>(
                    static_cast<unsigned char>((*response)[3])
                );
        return message.str();
    }
    return {};
}

std::string SerialRfidReader::discardPending() const {
    std::lock_guard<std::mutex> lock(serialPortMutex);
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
    std::lock_guard<std::mutex> lock(serialPortMutex);
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

    if (protocol_ == RfidProtocol::UhfReader18) {
        if (tcflush(serial.get(), TCIFLUSH) < 0) {
            return {{}, systemError("Unable to clear stale RFID serial data")};
        }
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (const std::string error = writeCommand(
                    serial.get(), kSingleInventoryCommand
                ); !error.empty()) {
                return {{}, error};
            }
            std::string error;
            const auto response = readValidFrame(
                serial.get(), 0x0FU, deadline, error
            );
            if (!response) return {{}, error};
            const std::string tag = epcFromFrame(*response);
            if (!tag.empty()) {
                if (tag.size() < minimumLength_ || tag.size() > maximumLength_) {
                    return {{}, "RFID EPC length is outside the configured limits"};
                }
                return {tag, {}};
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return {{}, "RFID reader timed out without detecting a tag"};
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
