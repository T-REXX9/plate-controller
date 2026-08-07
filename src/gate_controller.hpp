#pragma once

#include <chrono>
#include <optional>
#include <string>

namespace gate {

using Milliseconds = std::chrono::milliseconds;
using TimePoint = std::chrono::steady_clock::time_point;

enum class State {
    Startup,
    IdleClosed,
    Recognizing,
    Denied,
    Opening,
    GateOpen,
    VehiclePassing,
    ClearanceWait,
    Closing,
    Rearming,
    Fault
};

struct Config {
    Milliseconds inputDebounce{300};
    Milliseconds relayPulse{1000};
    Milliseconds recognitionTimeout{20000};
    Milliseconds openingTravelTime{3000};
    Milliseconds passageTimeout{30000};
    Milliseconds clearanceTime{1500};
    Milliseconds closingTravelTime{3000};
};

struct Inputs {
    bool loopPresent = false;
    bool passageBlocked = false;
};

struct Outputs {
    // Logical requests; the GPIO adapter drives the selected barrier pin LOW.
    bool requestOpen = false;
    bool requestClose = false;
    // LOW means red and HIGH means green on the single traffic selector pin.
    bool trafficGreen = false;
};

struct Snapshot {
    State state = State::Startup;
    Outputs outputs;
    bool cycleActive = false;
    std::string faultReason;
};

class Controller {
public:
    explicit Controller(Config config = {});

    Snapshot update(TimePoint now, const Inputs& inputs);
    bool recognitionCompleted(
        TimePoint now,
        bool authorized,
        bool systemError = false,
        const std::string& reason = ""
    );
    bool acknowledgeFault(TimePoint now, const Inputs& inputs);
    bool manualOpen(TimePoint now, const Inputs& inputs);
    bool manualClose(TimePoint now, const Inputs& inputs);
    void testTrafficSignal(TimePoint now, bool green, Milliseconds duration);
    Snapshot snapshot(TimePoint now, const Inputs& inputs) const;

    State state() const;
    const std::string& faultReason() const;

private:
    enum class FaultRecovery {
        None,
        CloseAfterPassageClears
    };

    Config config_;
    State state_ = State::Startup;
    TimePoint stateEnteredAt_{};
    TimePoint relayPulseEndsAt_{};
    bool started_ = false;
    std::string faultReason_;
    FaultRecovery faultRecovery_ = FaultRecovery::None;
    std::optional<TimePoint> loopOccupiedSince_;
    std::optional<TimePoint> loopClearSince_;
    std::optional<bool> trafficOverride_;
    TimePoint trafficOverrideEndsAt_{};

    void transition(State next, TimePoint now);
    void enterFault(
        const std::string& reason,
        TimePoint now,
        FaultRecovery recovery = FaultRecovery::None
    );
    bool inputStableFor(
        bool level,
        TimePoint now,
        std::optional<TimePoint>& since,
        Milliseconds duration
    );
};

const char* stateName(State state);

}  // namespace gate
