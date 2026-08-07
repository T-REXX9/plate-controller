#pragma once

#include <chrono>
#include <cstddef>
#include <string>

namespace gate {

enum class RfidProtocol {
    UhfReader18,
    PassiveStream
};

struct RfidReadResult {
    std::string tag;
    std::string error;
};

std::string extractRfidTag(
    const std::string& bytes,
    std::size_t minimumLength = 4,
    std::size_t maximumLength = 64
);

std::string encodeRfidBytes(const std::string& bytes);
std::string extractUhfReader18Epc(const std::string& bytes);

class SerialRfidReader {
public:
    SerialRfidReader(
        std::string device = "/dev/serial0",
        int baudRate = 9600,
        std::size_t minimumLength = 4,
        std::size_t maximumLength = 64,
        std::size_t expectedBytes = 0,
        RfidProtocol protocol = RfidProtocol::UhfReader18
    );

    std::string initialize(std::chrono::milliseconds timeout) const;
    std::string discardPending() const;
    RfidReadResult readTag(std::chrono::milliseconds timeout) const;

private:
    std::string device_;
    int baudRate_;
    std::size_t minimumLength_;
    std::size_t maximumLength_;
    std::size_t expectedBytes_;
    RfidProtocol protocol_;
};

}  // namespace gate
