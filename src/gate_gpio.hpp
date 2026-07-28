#pragma once

#include "gate_controller.hpp"

#include <memory>
#include <string>

namespace gate {

struct GpioPins {
    unsigned int loop = 17;
    unsigned int passage = 27;
    unsigned int traffic = 22;
    unsigned int open = 23;
    unsigned int close = 24;
};

// The barrier control interface is active-low: a logical request drives LOW.
constexpr bool barrierLineLevel(bool requestActive) noexcept {
    return !requestActive;
}

class RaspberryPiGpio {
public:
    explicit RaspberryPiGpio(
        const std::string& chipPath = "/dev/gpiochip0",
        GpioPins pins = {}
    );
    ~RaspberryPiGpio();

    RaspberryPiGpio(const RaspberryPiGpio&) = delete;
    RaspberryPiGpio& operator=(const RaspberryPiGpio&) = delete;

    Inputs readInputs() const;
    void applyOutputs(const Outputs& outputs);
    void safeOutputs() noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gate
