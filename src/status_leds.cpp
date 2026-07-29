#include "status_leds.hpp"

#include <gpiod.h>

#include <set>
#include <stdexcept>

namespace gate {

struct StatusLeds::Impl {
    StatusLedPins pins;
    bool camera = false;
    bool server = false;
    bool loop = false;
    bool barrierOpen = false;
    bool plateUnrecognized = false;
#ifdef PLATE_GPIOD_V1
    gpiod_chip* chip = nullptr;
    gpiod_line* cameraLine = nullptr;
    gpiod_line* serverLine = nullptr;
    gpiod_line* loopLine = nullptr;
    gpiod_line* barrierOpenLine = nullptr;
    gpiod_line* plateUnrecognizedLine = nullptr;
#else
    gpiod_line_request* request = nullptr;
#endif
};

namespace {

#ifdef PLATE_GPIOD_V1
gpiod_line* requestLed(
    gpiod_chip* chip,
    unsigned int gpio,
    const char* consumer
) {
    gpiod_line* line = gpiod_chip_get_line(chip, gpio);
    if (!line || gpiod_line_request_output(line, consumer, 0) < 0) {
        if (line) gpiod_line_release(line);
        throw std::runtime_error(
            "Unable to request status LED GPIO " + std::to_string(gpio)
        );
    }
    return line;
}

void setLed(
    gpiod_line* line,
    bool on,
    bool& cached,
    const char* name
) {
    if (cached == on) return;
    if (gpiod_line_set_value(line, on ? 1 : 0) < 0) {
        throw std::runtime_error(std::string("Unable to update ") + name + " LED");
    }
    cached = on;
}
#else
void setLed(
    gpiod_line_request* request,
    unsigned int gpio,
    bool on,
    bool& cached,
    const char* name
) {
    if (cached == on) return;
    if (gpiod_line_request_set_value(
            request,
            gpio,
            on ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE
        ) < 0) {
        throw std::runtime_error(std::string("Unable to update ") + name + " LED");
    }
    cached = on;
}
#endif

}  // namespace

StatusLeds::StatusLeds(const std::string& chipPath, StatusLedPins pins)
    : impl_(std::make_unique<Impl>()) {
    const std::set<unsigned int> uniquePins{
        pins.camera,
        pins.server,
        pins.loop,
        pins.barrierOpen,
        pins.plateUnrecognized
    };
    if (uniquePins.size() != 5) {
        throw std::runtime_error("Status LED GPIO assignments must be unique");
    }
    impl_->pins = pins;
#ifdef PLATE_GPIOD_V1
    impl_->chip = gpiod_chip_open(chipPath.c_str());
    if (!impl_->chip) {
        throw std::runtime_error("Unable to open GPIO chip " + chipPath);
    }
    try {
        impl_->cameraLine = requestLed(
            impl_->chip, pins.camera, "plate-status-camera"
        );
        impl_->serverLine = requestLed(
            impl_->chip, pins.server, "plate-status-server"
        );
        impl_->loopLine = requestLed(
            impl_->chip, pins.loop, "plate-status-loop"
        );
        impl_->barrierOpenLine = requestLed(
            impl_->chip, pins.barrierOpen, "plate-status-barrier"
        );
        impl_->plateUnrecognizedLine = requestLed(
            impl_->chip, pins.plateUnrecognized, "plate-status-unrecognized"
        );
    } catch (...) {
        if (impl_->cameraLine) gpiod_line_release(impl_->cameraLine);
        if (impl_->serverLine) gpiod_line_release(impl_->serverLine);
        if (impl_->loopLine) gpiod_line_release(impl_->loopLine);
        if (impl_->barrierOpenLine) gpiod_line_release(impl_->barrierOpenLine);
        if (impl_->plateUnrecognizedLine) {
            gpiod_line_release(impl_->plateUnrecognizedLine);
        }
        gpiod_chip_close(impl_->chip);
        impl_->chip = nullptr;
        throw;
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
            ) < 0) {
            throw std::runtime_error("Unable to configure status LED GPIO outputs");
        }
        const unsigned int offsets[] = {
            pins.camera,
            pins.server,
            pins.loop,
            pins.barrierOpen,
            pins.plateUnrecognized
        };
        if (
            gpiod_line_config_add_line_settings(
                lineConfig, offsets, 5, settings
            ) < 0) {
            throw std::runtime_error("Unable to assign status LED GPIO outputs");
        }
        gpiod_request_config_set_consumer(requestConfig, "plate-status-leds");
        impl_->request = gpiod_chip_request_lines(chip, requestConfig, lineConfig);
        if (!impl_->request) {
            throw std::runtime_error("Unable to request status LED GPIO outputs");
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

StatusLeds::~StatusLeds() {
    allOff();
#ifdef PLATE_GPIOD_V1
    if (impl_->cameraLine) gpiod_line_release(impl_->cameraLine);
    if (impl_->serverLine) gpiod_line_release(impl_->serverLine);
    if (impl_->loopLine) gpiod_line_release(impl_->loopLine);
    if (impl_->barrierOpenLine) gpiod_line_release(impl_->barrierOpenLine);
    if (impl_->plateUnrecognizedLine) {
        gpiod_line_release(impl_->plateUnrecognizedLine);
    }
    if (impl_->chip) gpiod_chip_close(impl_->chip);
#else
    if (impl_->request) gpiod_line_request_release(impl_->request);
#endif
}

void StatusLeds::setCamera(bool on) {
#ifdef PLATE_GPIOD_V1
    setLed(impl_->cameraLine, on, impl_->camera, "camera");
#else
    setLed(impl_->request, impl_->pins.camera, on, impl_->camera, "camera");
#endif
}

void StatusLeds::setServer(bool on) {
#ifdef PLATE_GPIOD_V1
    setLed(impl_->serverLine, on, impl_->server, "server");
#else
    setLed(impl_->request, impl_->pins.server, on, impl_->server, "server");
#endif
}

void StatusLeds::setLoop(bool on) {
#ifdef PLATE_GPIOD_V1
    setLed(impl_->loopLine, on, impl_->loop, "loop");
#else
    setLed(impl_->request, impl_->pins.loop, on, impl_->loop, "loop");
#endif
}

void StatusLeds::setBarrierOpen(bool on) {
#ifdef PLATE_GPIOD_V1
    setLed(impl_->barrierOpenLine, on, impl_->barrierOpen, "barrier");
#else
    setLed(
        impl_->request,
        impl_->pins.barrierOpen,
        on,
        impl_->barrierOpen,
        "barrier"
    );
#endif
}

void StatusLeds::setPlateUnrecognized(bool on) {
#ifdef PLATE_GPIOD_V1
    setLed(
        impl_->plateUnrecognizedLine,
        on,
        impl_->plateUnrecognized,
        "plate-unrecognized"
    );
#else
    setLed(
        impl_->request,
        impl_->pins.plateUnrecognized,
        on,
        impl_->plateUnrecognized,
        "plate-unrecognized"
    );
#endif
}

void StatusLeds::allOff() noexcept {
    if (!impl_) return;
#ifdef PLATE_GPIOD_V1
    if (impl_->cameraLine) gpiod_line_set_value(impl_->cameraLine, 0);
    if (impl_->serverLine) gpiod_line_set_value(impl_->serverLine, 0);
    if (impl_->loopLine) gpiod_line_set_value(impl_->loopLine, 0);
    if (impl_->barrierOpenLine) gpiod_line_set_value(impl_->barrierOpenLine, 0);
    if (impl_->plateUnrecognizedLine) {
        gpiod_line_set_value(impl_->plateUnrecognizedLine, 0);
    }
#else
    if (impl_->request) {
        const unsigned int offsets[] = {
            impl_->pins.camera,
            impl_->pins.server,
            impl_->pins.loop,
            impl_->pins.barrierOpen,
            impl_->pins.plateUnrecognized
        };
        const gpiod_line_value values[] = {
            GPIOD_LINE_VALUE_INACTIVE,
            GPIOD_LINE_VALUE_INACTIVE,
            GPIOD_LINE_VALUE_INACTIVE,
            GPIOD_LINE_VALUE_INACTIVE,
            GPIOD_LINE_VALUE_INACTIVE
        };
        gpiod_line_request_set_values_subset(
            impl_->request, 5, offsets, values
        );
    }
#endif
    impl_->camera = false;
    impl_->server = false;
    impl_->loop = false;
    impl_->barrierOpen = false;
    impl_->plateUnrecognized = false;
}

}  // namespace gate
