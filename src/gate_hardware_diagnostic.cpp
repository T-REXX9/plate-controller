#include "gate_gpio.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

long environmentLong(const char* name, long fallback) {
    const char* raw = std::getenv(name);
    if (!raw || *raw == '\0') return fallback;
    std::size_t consumed = 0;
    const long value = std::stol(raw, &consumed);
    if (raw[consumed] != '\0') {
        throw std::runtime_error(std::string(name) + " must be an integer");
    }
    return value;
}

unsigned int gpioEnvironment(const char* name, unsigned int fallback) {
    const long value = environmentLong(name, fallback);
    if (value < 0 || value > 27) {
        throw std::runtime_error(
            std::string(name) + " must be a BCM GPIO number from 0 to 27"
        );
    }
    return static_cast<unsigned int>(value);
}

bool sleepMonitoringBeam(
    gate::RaspberryPiGpio& gpio,
    std::chrono::milliseconds duration
) {
    if (gpio.readInputs().passageBlocked) return false;
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
        if (gpio.readInputs().passageBlocked) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return true;
}

void pulse(
    gate::RaspberryPiGpio& gpio,
    bool open,
    std::chrono::milliseconds duration
) {
    gate::Outputs outputs;
    outputs.requestOpen = open;
    outputs.requestClose = !open;
    outputs.trafficGreen = open;
    gpio.applyOutputs(outputs);
    std::this_thread::sleep_for(duration);
    gpio.safeOutputs();
}

int barrierTest(
    gate::RaspberryPiGpio& gpio,
    std::chrono::milliseconds pulseDuration,
    std::chrono::milliseconds openingDuration,
    std::chrono::milliseconds holdDuration,
    std::chrono::milliseconds clearanceDuration,
    std::chrono::milliseconds closingDuration
) {
    if (gpio.readInputs().passageBlocked) {
        std::cerr
            << "BARRIER TEST REFUSED: the IR beam is currently broken.\n";
        return 2;
    }

    std::cout << "[ACTION] Pulsing barrier OPEN for "
              << pulseDuration.count() << " ms................";
    pulse(gpio, true, pulseDuration);
    std::cout << "[DONE]\n";
    gate::Outputs openIndicators;
    openIndicators.trafficGreen = true;
    gpio.applyOutputs(openIndicators);
    std::this_thread::sleep_for(openingDuration);

    std::cout << "[ WAIT ] Barrier open; holding for " << holdDuration.count()
              << " ms.\n";
    std::this_thread::sleep_for(holdDuration);

    std::cout << "[ SCAN ] Waiting for IR beam clearance before closing.\n";
    while (true) {
        if (sleepMonitoringBeam(gpio, clearanceDuration)) break;
        std::cout << "[ WARN ] IR beam obstructed; barrier remains open.\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    gate::Outputs redIndicators;
    gpio.applyOutputs(redIndicators);
    std::cout << "[ACTION] Pulsing barrier CLOSE for "
              << pulseDuration.count() << " ms...............";
    gate::Outputs closing;
    closing.requestClose = true;
    gpio.applyOutputs(closing);

    const auto pulseEnds = std::chrono::steady_clock::now() + pulseDuration;
    const auto closingEnds = std::chrono::steady_clock::now() + closingDuration;
    while (std::chrono::steady_clock::now() < closingEnds) {
        if (gpio.readInputs().passageBlocked) {
            gpio.safeOutputs();
            std::cout << "[ABORT]\n";
            std::cerr
                << "[ FAIL ] IR beam broken during closing; reopening.\n";
            pulse(gpio, true, pulseDuration);
            std::cerr
                << "[!!] Barrier left OPEN. Clear the obstruction before "
                   "restarting the controller.\n";
            return 20;
        }
        if (std::chrono::steady_clock::now() >= pulseEnds) {
            gpio.safeOutputs();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    gpio.safeOutputs();
    std::cout << "[DONE]\n";
    std::cout << "[ PASS ] Barrier open/close cycle completed safely.\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        std::cout << std::unitbuf;
        gate::GpioPins pins;
        pins.loop = gpioEnvironment("GATE_LOOP_GPIO", 17);
        pins.passage = gpioEnvironment("GATE_PASSAGE_GPIO", 27);
        pins.traffic = gpioEnvironment("GATE_TRAFFIC_GPIO", 22);
        pins.open = gpioEnvironment("GATE_OPEN_GPIO", 23);
        pins.close = gpioEnvironment("GATE_CLOSE_GPIO", 24);
        const std::string chipPath = std::getenv("GATE_GPIO_CHIP")
            ? std::getenv("GATE_GPIO_CHIP")
            : "/dev/gpiochip0";

        gate::RaspberryPiGpio gpio(chipPath, pins);
        if (argc < 2 || std::string(argv[1]) == "status") {
            const gate::Inputs inputs = gpio.readInputs();
            std::cout << "LOOP_PRESENT=" << (inputs.loopPresent ? 1 : 0) << '\n'
                      << "IR_BEAM_BROKEN="
                      << (inputs.passageBlocked ? 1 : 0) << '\n';
            return 0;
        }
        if (std::string(argv[1]) != "barrier-test") {
            std::cerr << "Usage: gate_hardware_diagnostic [status|barrier-test]\n";
            return 2;
        }

        const auto positiveMilliseconds = [](const char* name, long fallback) {
            const long value = environmentLong(name, fallback);
            if (value <= 0) {
                throw std::runtime_error(std::string(name) + " must be positive");
            }
            return std::chrono::milliseconds(value);
        };
        const auto nonNegativeMilliseconds = [](
            const char* name,
            long fallback
        ) {
            const long value = environmentLong(name, fallback);
            if (value < 0) {
                throw std::runtime_error(
                    std::string(name) + " must not be negative"
                );
            }
            return std::chrono::milliseconds(value);
        };

        return barrierTest(
            gpio,
            positiveMilliseconds("GATE_RELAY_PULSE_MS", 1000),
            positiveMilliseconds("GATE_OPENING_TRAVEL_MS", 3000),
            positiveMilliseconds("GATE_DIAGNOSTIC_OPEN_HOLD_MS", 5000),
            nonNegativeMilliseconds("GATE_CLEARANCE_MS", 1500),
            positiveMilliseconds("GATE_CLOSING_TRAVEL_MS", 3000)
        );
    } catch (const std::exception& error) {
        std::cerr << "GPIO DIAGNOSTIC FAILED: " << error.what() << '\n';
        return 1;
    }
}
