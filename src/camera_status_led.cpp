#include "camera_status_led.hpp"

#include <gpiod.h>

#include <stdexcept>

namespace gate {

struct CameraStatusLed::Impl {
    unsigned int gpio = 25;
    bool value = false;
#ifdef PLATE_GPIOD_V1
    gpiod_chip* chip = nullptr;
    gpiod_line* line = nullptr;
#else
    gpiod_line_request* request = nullptr;
#endif
};

CameraStatusLed::CameraStatusLed(const std::string& chipPath, unsigned int gpio)
    : impl_(std::make_unique<Impl>()) {
    impl_->gpio = gpio;
#ifdef PLATE_GPIOD_V1
    impl_->chip = gpiod_chip_open(chipPath.c_str());
    if (!impl_->chip) {
        throw std::runtime_error("Unable to open GPIO chip " + chipPath);
    }
    impl_->line = gpiod_chip_get_line(impl_->chip, gpio);
    if (!impl_->line ||
        gpiod_line_request_output(impl_->line, "plate-camera-status", 0) < 0) {
        if (impl_->line) {
            gpiod_line_release(impl_->line);
            impl_->line = nullptr;
        }
        gpiod_chip_close(impl_->chip);
        impl_->chip = nullptr;
        throw std::runtime_error(
            "Unable to request camera status GPIO " + std::to_string(gpio)
        );
    }
#else
    gpiod_chip* chip = gpiod_chip_open(chipPath.c_str());
    gpiod_line_settings* settings = nullptr;
    gpiod_line_config* lineConfig = nullptr;
    gpiod_request_config* requestConfig = nullptr;
    if (!chip) {
        throw std::runtime_error("Unable to open GPIO chip " + chipPath);
    }
    try {
        settings = gpiod_line_settings_new();
        lineConfig = gpiod_line_config_new();
        requestConfig = gpiod_request_config_new();
        if (!settings || !lineConfig || !requestConfig ||
            gpiod_line_settings_set_direction(
                settings, GPIOD_LINE_DIRECTION_OUTPUT
            ) < 0 ||
            gpiod_line_settings_set_output_value(
                settings, GPIOD_LINE_VALUE_INACTIVE
            ) < 0 ||
            gpiod_line_config_add_line_settings(
                lineConfig, &gpio, 1, settings
            ) < 0) {
            throw std::runtime_error(
                "Unable to configure camera status GPIO " + std::to_string(gpio)
            );
        }
        gpiod_request_config_set_consumer(requestConfig, "plate-camera-status");
        impl_->request = gpiod_chip_request_lines(chip, requestConfig, lineConfig);
        if (!impl_->request) {
            throw std::runtime_error(
                "Unable to request camera status GPIO " + std::to_string(gpio)
            );
        }
    } catch (...) {
        gpiod_request_config_free(requestConfig);
        gpiod_line_config_free(lineConfig);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);
        throw;
    }
    gpiod_request_config_free(requestConfig);
    gpiod_line_config_free(lineConfig);
    gpiod_line_settings_free(settings);
    gpiod_chip_close(chip);
#endif
}

CameraStatusLed::~CameraStatusLed() {
    off();
#ifdef PLATE_GPIOD_V1
    if (impl_->line) gpiod_line_release(impl_->line);
    if (impl_->chip) gpiod_chip_close(impl_->chip);
#else
    if (impl_->request) gpiod_line_request_release(impl_->request);
#endif
}

void CameraStatusLed::setRecognized(bool recognized) {
    if (impl_->value == recognized) {
        return;
    }
#ifdef PLATE_GPIOD_V1
    if (gpiod_line_set_value(impl_->line, recognized ? 1 : 0) < 0) {
#else
    if (gpiod_line_request_set_value(
            impl_->request,
            impl_->gpio,
            recognized ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE
        ) < 0) {
#endif
        throw std::runtime_error("Unable to update camera status LED");
    }
    impl_->value = recognized;
}

void CameraStatusLed::off() noexcept {
    if (!impl_) return;
#ifdef PLATE_GPIOD_V1
    if (impl_->line) gpiod_line_set_value(impl_->line, 0);
#else
    if (impl_->request) {
        gpiod_line_request_set_value(
            impl_->request, impl_->gpio, GPIOD_LINE_VALUE_INACTIVE
        );
    }
#endif
    impl_->value = false;
}

}  // namespace gate
