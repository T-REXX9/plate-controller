#pragma once

#include <memory>
#include <string>

namespace gate {

struct StatusLedPins {
    unsigned int camera = 25;
    unsigned int server = 5;
    unsigned int loop = 6;
    unsigned int barrierOpen = 12;
    unsigned int plateUnrecognized = 13;
};

class StatusLeds {
public:
    explicit StatusLeds(
        const std::string& chipPath = "/dev/gpiochip0",
        StatusLedPins pins = {}
    );
    ~StatusLeds();

    StatusLeds(const StatusLeds&) = delete;
    StatusLeds& operator=(const StatusLeds&) = delete;

    void setCamera(bool on);
    void setServer(bool on);
    void setLoop(bool on);
    void setBarrierOpen(bool on);
    void setPlateUnrecognized(bool on);
    void allOff() noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gate
