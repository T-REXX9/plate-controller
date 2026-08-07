#include "serial_rfid_reader.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

}  // namespace

int main() {
    const std::string binaryTag{
        static_cast<char>(0x30), static_cast<char>(0x45),
        static_cast<char>(0x67), static_cast<char>(0x30),
        static_cast<char>(0x30), static_cast<char>(0x55),
        static_cast<char>(0x3F), static_cast<char>(0x90),
        static_cast<char>(0x30), static_cast<char>(0x55),
        static_cast<char>(0x3F), static_cast<char>(0x90)
    };
    require(gate::encodeRfidBytes(binaryTag) == "3045673030553F9030553F90",
            "12-byte binary RFID is encoded in canonical database form");
    require(gate::extractRfidTag("1234567890\r\n") == "1234567890",
            "decimal line is parsed");
    require(gate::extractRfidTag("\x02 EPC: e20034120123456789ab \x03") ==
                "E20034120123456789AB",
            "framed hexadecimal EPC is parsed and normalized");
    require(gate::extractRfidTag("ID=AB-CD-EF-12\n") == "ABCDEF12",
            "hyphenated tag is returned in clean database form");
    require(gate::extractRfidTag("noise\n00001234\n") == "00001234",
            "the longest complete token wins and leading zeroes remain");
    require(gate::extractRfidTag("x\r\n").empty(),
            "short serial noise is rejected");

    const std::string firstFrame{
        "\x13\x00\x0F\x01\x01\x0C\xE2\x84\x36\x11"
        "\x00\x00\x10\x00\x09\x49\x44\xAA\xDA\xFF",
        20
    };
    require(
        gate::extractUhfReader18Epc(firstFrame) ==
            "E284361100001000094944AA",
        "confirmed single-inventory frame yields only its EPC"
    );
    const std::string secondFrame{
        "\x13\x00\x0F\x01\x01\x0C\x20\x26\x05\x27"
        "\x00\x00\x00\x00\x00\x01\x97\x05\x87\x2A",
        20
    };
    require(
        gate::extractUhfReader18Epc(secondFrame) ==
            "202605270000000000019705",
        "a second confirmed EPC keeps its leading zeroes"
    );
    const std::string noTagFrame{"\x05\x00\x0F\xFB\xE2\xA7", 6};
    require(gate::extractUhfReader18Epc(noTagFrame).empty(),
            "a CRC-valid no-tag response is not mistaken for an EPC");
    std::string corrupted = firstFrame;
    corrupted.back() ^= 0x01;
    require(gate::extractUhfReader18Epc(corrupted).empty(),
            "a response with an invalid CRC is rejected");
    require(
        gate::extractUhfReader18Epc(noTagFrame + secondFrame) ==
            "202605270000000000019705",
        "the parser skips a no-tag frame and finds the next valid EPC"
    );

    bool rejected = false;
    try {
        gate::SerialRfidReader invalid("relative-device", 9600);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "relative serial device path is rejected");

    const gate::SerialDebugResult relativeDebug = gate::transactSerial(
        "relative-device", {}, "\x05", std::chrono::milliseconds(500)
    );
    require(!relativeDebug.error.empty(),
            "serial debug rejects a relative device path");
    const gate::SerialDebugResult emptyDebug = gate::transactSerial(
        "/dev/null", {}, {}, std::chrono::milliseconds(500)
    );
    require(!emptyDebug.error.empty(),
            "serial debug rejects an empty transmission");
    const gate::SerialDebugResult longDebug = gate::transactSerial(
        "/dev/null", {}, std::string(513, 'A'), std::chrono::milliseconds(500)
    );
    require(!longDebug.error.empty(),
            "serial debug enforces its transmission limit");
    const gate::SerialDebugResult timeoutDebug = gate::transactSerial(
        "/dev/null", {}, "\x05", std::chrono::milliseconds(20)
    );
    require(!timeoutDebug.error.empty(),
            "serial debug rejects an unsafe timeout");

    std::cout << "All serial RFID parser tests passed.\n";
    return 0;
}
