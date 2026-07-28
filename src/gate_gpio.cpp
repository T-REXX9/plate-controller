#include "gate_gpio.hpp"

#include <gpiod.h>

#include <set>
#include <stdexcept>
#include <utility>

namespace gate {

struct RaspberryPiGpio::Impl {
    GpioPins pins;
    bool trafficValue = false;
    bool openValue = true;
    bool closeValue = true;

#ifdef PLATE_GPIOD_V1
    gpiod_chip* chip = nullptr;
    gpiod_line* loop = nullptr;
    gpiod_line* passage = nullptr;
    gpiod_line* traffic = nullptr;
    gpiod_line* open = nullptr;
    gpiod_line* close = nullptr;
#else
    gpiod_line_request* request = nullptr;
#endif
};

#ifdef PLATE_GPIOD_V1

namespace {

gpiod_line* chipLine(gpiod_chip* chip, unsigned int offset) {
    gpiod_line* line = gpiod_chip_get_line(chip, offset);
    if (!line) {
        throw std::runtime_error("Unable to access GPIO line " + std::to_string(offset));
    }
    return line;
}

void requestInput(gpiod_line* line, const char* name) {
    if (gpiod_line_request_input_flags(
        line, name, GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP
    ) < 0) {
        throw std::runtime_error(std::string("Unable to request input ") + name);
    }
}

void requestOutput(gpiod_line* line, const char* name, int initialValue) {
    if (gpiod_line_request_output(line, name, initialValue) < 0) {
        throw std::runtime_error(std::string("Unable to request output ") + name);
    }
}

bool readActiveLow(gpiod_line* line, const char* name) {
    const int value = gpiod_line_get_value(line);
    if (value < 0) {
        throw std::runtime_error(std::string("Unable to read input ") + name);
    }
    return value == 0;
}

void writeValue(gpiod_line* line, bool value, bool& cached, const char* name) {
    if (cached == value) {
        return;
    }
    if (gpiod_line_set_value(line, value ? 1 : 0) < 0) {
        throw std::runtime_error(std::string("Unable to write output ") + name);
    }
    cached = value;
}

}  // namespace

#else

namespace {

gpiod_line_request* requestLines(
    const std::string& chipPath,
    const GpioPins& pins
) {
    gpiod_chip* chip = gpiod_chip_open(chipPath.c_str());
    if (!chip) {
        throw std::runtime_error("Unable to open GPIO chip " + chipPath);
    }
    gpiod_line_settings* inputSettings = nullptr;
    gpiod_line_settings* trafficSettings = nullptr;
    gpiod_line_settings* movementSettings = nullptr;
    gpiod_line_config* lineConfig = nullptr;
    gpiod_request_config* requestConfig = nullptr;
    gpiod_line_request* request = nullptr;
    try {
        inputSettings = gpiod_line_settings_new();
        trafficSettings = gpiod_line_settings_new();
        movementSettings = gpiod_line_settings_new();
        lineConfig = gpiod_line_config_new();
        requestConfig = gpiod_request_config_new();
        if (!inputSettings || !trafficSettings || !movementSettings ||
            !lineConfig || !requestConfig) {
            throw std::runtime_error("Unable to allocate GPIO configuration");
        }
        if (gpiod_line_settings_set_direction(
                inputSettings, GPIOD_LINE_DIRECTION_INPUT
            ) < 0 ||
            gpiod_line_settings_set_bias(
                inputSettings, GPIOD_LINE_BIAS_PULL_UP
            ) < 0 ||
            gpiod_line_settings_set_direction(
                trafficSettings, GPIOD_LINE_DIRECTION_OUTPUT
            ) < 0 ||
            gpiod_line_settings_set_output_value(
                trafficSettings, GPIOD_LINE_VALUE_INACTIVE
            ) < 0 ||
            gpiod_line_settings_set_direction(
                movementSettings, GPIOD_LINE_DIRECTION_OUTPUT
            ) < 0 ||
            gpiod_line_settings_set_output_value(
                movementSettings, GPIOD_LINE_VALUE_ACTIVE
            ) < 0) {
            throw std::runtime_error("Unable to configure GPIO directions or defaults");
        }
        const unsigned int inputOffsets[] = {pins.loop, pins.passage};
        const unsigned int trafficOffsets[] = {pins.traffic};
        const unsigned int movementOffsets[] = {pins.open, pins.close};
        if (gpiod_line_config_add_line_settings(
                lineConfig, inputOffsets, 2, inputSettings
            ) < 0 ||
            gpiod_line_config_add_line_settings(
                lineConfig, trafficOffsets, 1, trafficSettings
            ) < 0 ||
            gpiod_line_config_add_line_settings(
                lineConfig, movementOffsets, 2, movementSettings
            ) < 0) {
            throw std::runtime_error("Unable to assign GPIO line settings");
        }
        gpiod_request_config_set_consumer(requestConfig, "plate-controller");
        request = gpiod_chip_request_lines(chip, requestConfig, lineConfig);
        if (!request) {
            throw std::runtime_error("Unable to request gate GPIO lines");
        }
    } catch (...) {
        gpiod_request_config_free(requestConfig);
        gpiod_line_config_free(lineConfig);
        gpiod_line_settings_free(movementSettings);
        gpiod_line_settings_free(trafficSettings);
        gpiod_line_settings_free(inputSettings);
        gpiod_chip_close(chip);
        throw;
    }
    gpiod_request_config_free(requestConfig);
    gpiod_line_config_free(lineConfig);
    gpiod_line_settings_free(movementSettings);
    gpiod_line_settings_free(trafficSettings);
    gpiod_line_settings_free(inputSettings);
    gpiod_chip_close(chip);
    return request;
}

bool readActiveLow(
    gpiod_line_request* request,
    unsigned int offset,
    const char* name
) {
    const gpiod_line_value value = gpiod_line_request_get_value(request, offset);
    if (value == GPIOD_LINE_VALUE_ERROR) {
        throw std::runtime_error(std::string("Unable to read input ") + name);
    }
    return value == GPIOD_LINE_VALUE_INACTIVE;
}

void writeValue(
    gpiod_line_request* request,
    unsigned int offset,
    bool value,
    bool& cached,
    const char* name
) {
    if (cached == value) {
        return;
    }
    const gpiod_line_value lineValue = value
        ? GPIOD_LINE_VALUE_ACTIVE
        : GPIOD_LINE_VALUE_INACTIVE;
    if (gpiod_line_request_set_value(request, offset, lineValue) < 0) {
        throw std::runtime_error(std::string("Unable to write output ") + name);
    }
    cached = value;
}

}  // namespace

#endif

RaspberryPiGpio::RaspberryPiGpio(const std::string& chipPath, GpioPins pins)
    : impl_(std::make_unique<Impl>()) {
    const std::set<unsigned int> uniquePins{
        pins.loop, pins.passage, pins.traffic, pins.open, pins.close
    };
    if (uniquePins.size() != 5) {
        throw std::runtime_error("Gate GPIO line assignments must be unique");
    }
    impl_->pins = pins;

#ifdef PLATE_GPIOD_V1
    impl_->chip = gpiod_chip_open(chipPath.c_str());
    if (!impl_->chip) {
        throw std::runtime_error("Unable to open GPIO chip " + chipPath);
    }
    try {
        impl_->loop = chipLine(impl_->chip, pins.loop);
        impl_->passage = chipLine(impl_->chip, pins.passage);
        impl_->traffic = chipLine(impl_->chip, pins.traffic);
        impl_->open = chipLine(impl_->chip, pins.open);
        impl_->close = chipLine(impl_->chip, pins.close);
        requestInput(impl_->loop, "plate-loop");
        requestInput(impl_->passage, "plate-ir-beam");
        requestOutput(impl_->traffic, "plate-traffic", 0);
        requestOutput(impl_->open, "plate-open", 1);
        requestOutput(impl_->close, "plate-close", 1);
    } catch (...) {
        if (impl_->loop) gpiod_line_release(impl_->loop);
        if (impl_->passage) gpiod_line_release(impl_->passage);
        if (impl_->traffic) gpiod_line_release(impl_->traffic);
        if (impl_->open) gpiod_line_release(impl_->open);
        if (impl_->close) gpiod_line_release(impl_->close);
        gpiod_chip_close(impl_->chip);
        impl_->chip = nullptr;
        throw;
    }
#else
    impl_->request = requestLines(chipPath, pins);
#endif
    safeOutputs();
}

RaspberryPiGpio::~RaspberryPiGpio() {
    safeOutputs();
#ifdef PLATE_GPIOD_V1
    if (impl_->loop) gpiod_line_release(impl_->loop);
    if (impl_->passage) gpiod_line_release(impl_->passage);
    if (impl_->traffic) gpiod_line_release(impl_->traffic);
    if (impl_->open) gpiod_line_release(impl_->open);
    if (impl_->close) gpiod_line_release(impl_->close);
    if (impl_->chip) gpiod_chip_close(impl_->chip);
#else
    if (impl_->request) gpiod_line_request_release(impl_->request);
#endif
}

Inputs RaspberryPiGpio::readInputs() const {
#ifdef PLATE_GPIOD_V1
    return {
        readActiveLow(impl_->loop, "plate-loop"),
        readActiveLow(impl_->passage, "plate-ir-beam")
    };
#else
    return {
        readActiveLow(impl_->request, impl_->pins.loop, "plate-loop"),
        readActiveLow(impl_->request, impl_->pins.passage, "plate-ir-beam")
    };
#endif
}

void RaspberryPiGpio::applyOutputs(const Outputs& outputs) {
    if (outputs.requestOpen && outputs.requestClose) {
        safeOutputs();
        throw std::runtime_error("Unsafe simultaneous open and close request rejected");
    }

    const auto setOpen = [this](bool value) {
#ifdef PLATE_GPIOD_V1
        writeValue(impl_->open, value, impl_->openValue, "plate-open");
#else
        writeValue(impl_->request, impl_->pins.open, value, impl_->openValue, "plate-open");
#endif
    };
    const auto setClose = [this](bool value) {
#ifdef PLATE_GPIOD_V1
        writeValue(impl_->close, value, impl_->closeValue, "plate-close");
#else
        writeValue(impl_->request, impl_->pins.close, value, impl_->closeValue, "plate-close");
#endif
    };
    if (outputs.requestOpen) {
        setClose(barrierLineLevel(false));
        setOpen(barrierLineLevel(true));
    } else if (outputs.requestClose) {
        setOpen(barrierLineLevel(false));
        setClose(barrierLineLevel(true));
    } else {
        setOpen(barrierLineLevel(false));
        setClose(barrierLineLevel(false));
    }
#ifdef PLATE_GPIOD_V1
    writeValue(
        impl_->traffic, outputs.trafficGreen, impl_->trafficValue, "plate-traffic"
    );
#else
    writeValue(
        impl_->request,
        impl_->pins.traffic,
        outputs.trafficGreen,
        impl_->trafficValue,
        "plate-traffic"
    );
#endif
}

void RaspberryPiGpio::safeOutputs() noexcept {
    if (!impl_) return;
#ifdef PLATE_GPIOD_V1
    if (impl_->open) gpiod_line_set_value(impl_->open, 1);
    if (impl_->close) gpiod_line_set_value(impl_->close, 1);
    if (impl_->traffic) gpiod_line_set_value(impl_->traffic, 0);
#else
    if (impl_->request) {
        gpiod_line_request_set_value(
            impl_->request, impl_->pins.open, GPIOD_LINE_VALUE_ACTIVE
        );
        gpiod_line_request_set_value(
            impl_->request, impl_->pins.close, GPIOD_LINE_VALUE_ACTIVE
        );
        gpiod_line_request_set_value(
            impl_->request, impl_->pins.traffic, GPIOD_LINE_VALUE_INACTIVE
        );
    }
#endif
    impl_->openValue = true;
    impl_->closeValue = true;
    impl_->trafficValue = false;
}

}  // namespace gate
