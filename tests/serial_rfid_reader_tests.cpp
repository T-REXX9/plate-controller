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

    bool rejected = false;
    try {
        gate::SerialRfidReader invalid("relative-device", 9600);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "relative serial device path is rejected");

    std::cout << "All serial RFID parser tests passed.\n";
    return 0;
}
