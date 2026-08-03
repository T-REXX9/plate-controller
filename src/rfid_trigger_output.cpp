#include "rfid_trigger_output.hpp"

#include <gpiod.h>

#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace gate {

struct RfidTriggerOutput::Impl {
    unsigned int gpio = 16;
    std::mutex mutex;
    std::condition_variable condition;
    std::thread worker;
    bool stopping = false;
    bool pulseActive = false;
    std::chrono::steady_clock::time_point pulseUntil{};
    std::string failure;
#ifdef PLATE_GPIOD_V1
    gpiod_chip* chip = nullptr;
    gpiod_line* line = nullptr;
#else
    gpiod_line_request* request = nullptr;
#endif
};

void RfidTriggerOutput::writeLine(Impl& impl, bool high) {
#ifdef PLATE_GPIOD_V1
    if (gpiod_line_set_value(impl.line, high ? 1 : 0) < 0) {
        throw std::runtime_error("Unable to update RFID trigger output");
    }
#else
    if (gpiod_line_request_set_value(
            impl.request,
            impl.gpio,
            high ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE
        ) < 0) {
        throw std::runtime_error("Unable to update RFID trigger output");
    }
#endif
}

RfidTriggerOutput::RfidTriggerOutput(
    const std::string& chipPath,
    unsigned int gpio
) : impl_(std::make_unique<Impl>()) {
    if (gpio > 27) {
        throw std::runtime_error("RFID output must be a BCM GPIO from 0 to 27");
    }
    // Physical header pins 8 and 10 are BCM14 and BCM15.
    if (gpio == 14 || gpio == 15) {
        throw std::runtime_error(
            "RFID output cannot use physical pin 8 or 10 (BCM14/BCM15)"
        );
    }
    impl_->gpio = gpio;

#ifdef PLATE_GPIOD_V1
    impl_->chip = gpiod_chip_open(chipPath.c_str());
    if (!impl_->chip) {
        throw std::runtime_error("Unable to open GPIO chip " + chipPath);
    }
    impl_->line = gpiod_chip_get_line(impl_->chip, gpio);
    if (!impl_->line ||
        gpiod_line_request_output(
            impl_->line, "plate-rfid-trigger", 1
        ) < 0) {
        if (impl_->line) gpiod_line_release(impl_->line);
        gpiod_chip_close(impl_->chip);
        impl_->line = nullptr;
        impl_->chip = nullptr;
        throw std::runtime_error(
            "Unable to request RFID trigger GPIO " + std::to_string(gpio)
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
                settings, GPIOD_LINE_VALUE_ACTIVE
            ) < 0 ||
            gpiod_line_config_add_line_settings(
                lineConfig, &gpio, 1, settings
            ) < 0) {
            throw std::runtime_error("Unable to configure RFID trigger output");
        }
        gpiod_request_config_set_consumer(
            requestConfig, "plate-rfid-trigger"
        );
        impl_->request = gpiod_chip_request_lines(
            chip, requestConfig, lineConfig
        );
        if (!impl_->request) {
            throw std::runtime_error(
                "Unable to request RFID trigger GPIO " + std::to_string(gpio)
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

    impl_->worker = std::thread([implementation = impl_.get()] {
        std::unique_lock<std::mutex> lock(implementation->mutex);
        while (!implementation->stopping) {
            if (!implementation->pulseActive) {
                implementation->condition.wait(lock, [implementation] {
                    return implementation->stopping ||
                        implementation->pulseActive;
                });
                continue;
            }
            const auto deadline = implementation->pulseUntil;
            const bool changed = implementation->condition.wait_until(
                lock,
                deadline,
                [implementation, deadline] {
                    return implementation->stopping ||
                        !implementation->pulseActive ||
                        implementation->pulseUntil != deadline;
                }
            );
            if (changed) continue;
            try {
                writeLine(
                    *implementation,
                    rfidTriggerLineLevel(false)
                );
            } catch (const std::exception& error) {
                implementation->failure = error.what();
            }
            implementation->pulseActive = false;
        }
    });
}

RfidTriggerOutput::~RfidTriggerOutput() {
    if (!impl_) return;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->stopping = true;
        impl_->pulseActive = false;
        try {
            writeLine(*impl_, rfidTriggerLineLevel(false));
        } catch (...) {
        }
    }
    impl_->condition.notify_all();
    if (impl_->worker.joinable()) impl_->worker.join();
#ifdef PLATE_GPIOD_V1
    if (impl_->line) gpiod_line_release(impl_->line);
    if (impl_->chip) gpiod_chip_close(impl_->chip);
#else
    if (impl_->request) gpiod_line_request_release(impl_->request);
#endif
}

void RfidTriggerOutput::pulse(std::chrono::milliseconds duration) {
    if (duration.count() <= 0) {
        throw std::runtime_error("RFID trigger duration must be positive");
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->failure.empty()) {
        throw std::runtime_error(impl_->failure);
    }
    if (impl_->stopping) {
        throw std::runtime_error("RFID trigger output is stopping");
    }
    writeLine(*impl_, rfidTriggerLineLevel(true));
    const auto requestedDeadline = std::chrono::steady_clock::now() + duration;
    impl_->pulseUntil = impl_->pulseActive
        ? std::max(impl_->pulseUntil, requestedDeadline)
        : requestedDeadline;
    impl_->pulseActive = true;
    impl_->condition.notify_all();
}

void RfidTriggerOutput::inactive() noexcept {
    if (!impl_) return;
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->pulseActive = false;
    try {
        writeLine(*impl_, rfidTriggerLineLevel(false));
    } catch (const std::exception& error) {
        impl_->failure = error.what();
    }
    impl_->condition.notify_all();
}

}  // namespace gate
