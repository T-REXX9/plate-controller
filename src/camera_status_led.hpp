#pragma once

#include <memory>
#include <string>

namespace gate {

class CameraStatusLed {
public:
    explicit CameraStatusLed(
        const std::string& chipPath = "/dev/gpiochip0",
        unsigned int gpio = 25
    );
    ~CameraStatusLed();

    CameraStatusLed(const CameraStatusLed&) = delete;
    CameraStatusLed& operator=(const CameraStatusLed&) = delete;

    void setRecognized(bool recognized);
    void off() noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gate
