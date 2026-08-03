#pragma once

#include <chrono>
#include <memory>
#include <string>

namespace gate {

// The RFID trigger is active-low: inactive is HIGH and a trigger drives LOW.
constexpr bool rfidTriggerLineLevel(bool triggerActive) noexcept {
    return !triggerActive;
}

class RfidTriggerOutput {
public:
    explicit RfidTriggerOutput(
        const std::string& chipPath = "/dev/gpiochip0",
        unsigned int gpio = 16
    );
    ~RfidTriggerOutput();

    RfidTriggerOutput(const RfidTriggerOutput&) = delete;
    RfidTriggerOutput& operator=(const RfidTriggerOutput&) = delete;

    // Starts immediately and returns without waiting for the pulse to finish.
    // Another trigger received while LOW extends the existing pulse deadline.
    void pulse(std::chrono::milliseconds duration);
    void inactive() noexcept;

private:
    struct Impl;
    static void writeLine(Impl& impl, bool high);
    std::unique_ptr<Impl> impl_;
};

}  // namespace gate
