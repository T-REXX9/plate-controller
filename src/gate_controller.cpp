#include "gate_controller.hpp"

#include <stdexcept>
#include <utility>

namespace gate {

Controller::Controller(Config config) : config_(std::move(config)) {
    if (config_.inputDebounce.count() < 0 || config_.relayPulse.count() <= 0 ||
        config_.recognitionTimeout.count() <= 0 ||
        config_.openingTravelTime < config_.relayPulse ||
        config_.passageTimeout.count() <= 0 || config_.clearanceTime.count() < 0 ||
        config_.closingTravelTime < config_.relayPulse) {
        throw std::invalid_argument("Invalid gate timing configuration");
    }
}

void Controller::transition(State next, TimePoint now) {
    state_ = next;
    stateEnteredAt_ = now;
    loopOccupiedSince_.reset();
    loopClearSince_.reset();
    if (next == State::Opening || next == State::Closing) {
        relayPulseEndsAt_ = now + config_.relayPulse;
    }
}

void Controller::enterFault(const std::string& reason, TimePoint now) {
    faultReason_ = reason;
    transition(State::Fault, now);
}

bool Controller::inputStableFor(
    bool level,
    TimePoint now,
    std::optional<TimePoint>& since,
    Milliseconds duration
) {
    if (!level) {
        since.reset();
        return false;
    }
    if (!since) {
        since = now;
    }
    return now - *since >= duration;
}

Snapshot Controller::update(TimePoint now, const Inputs& inputs) {
    if (!started_) {
        started_ = true;
        stateEnteredAt_ = now;
    }
    if (trafficOverride_ && now >= trafficOverrideEndsAt_) {
        trafficOverride_.reset();
    }

    switch (state_) {
        case State::Startup:
            faultReason_.clear();
            transition(State::IdleClosed, now);
            break;

        case State::IdleClosed:
            if (inputStableFor(
                inputs.loopPresent,
                now,
                loopOccupiedSince_,
                config_.inputDebounce
            )) {
                transition(State::Recognizing, now);
            }
            break;

        case State::Recognizing:
            if (now - stateEnteredAt_ >= config_.recognitionTimeout) {
                enterFault("Plate recognition timed out", now);
            }
            break;

        case State::Denied:
            if (inputStableFor(
                !inputs.loopPresent,
                now,
                loopClearSince_,
                config_.inputDebounce
            )) {
                transition(State::IdleClosed, now);
            }
            break;

        case State::Opening:
            if (now - stateEnteredAt_ >= config_.openingTravelTime) {
                transition(State::GateOpen, now);
            }
            break;

        case State::GateOpen:
            if (inputs.passageBlocked) {
                transition(State::VehiclePassing, now);
            } else if (now - stateEnteredAt_ >= config_.passageTimeout) {
                enterFault("No passage was detected while the barrier was open", now);
            }
            break;

        case State::VehiclePassing:
            if (!inputs.passageBlocked) {
                transition(State::ClearanceWait, now);
            } else if (now - stateEnteredAt_ >= config_.passageTimeout) {
                enterFault("Passage sensor remained blocked too long", now);
            }
            break;

        case State::ClearanceWait:
            if (inputs.passageBlocked) {
                transition(State::VehiclePassing, now);
            } else if (now - stateEnteredAt_ >= config_.clearanceTime) {
                transition(State::Closing, now);
            }
            break;

        case State::Closing:
            if (inputs.passageBlocked) {
                transition(State::Opening, now);
            } else if (now - stateEnteredAt_ >= config_.closingTravelTime) {
                transition(State::Rearming, now);
            }
            break;

        case State::Rearming:
            if (inputStableFor(
                inputs.loopPresent,
                now,
                loopOccupiedSince_,
                config_.inputDebounce
            )) {
                transition(State::Recognizing, now);
            } else if (inputStableFor(
                !inputs.loopPresent,
                now,
                loopClearSince_,
                config_.inputDebounce
            )) {
                transition(State::IdleClosed, now);
            }
            break;

        case State::Fault:
            break;
    }

    return snapshot(now, inputs);
}

bool Controller::recognitionCompleted(
    TimePoint now,
    bool authorized,
    bool systemError,
    const std::string& reason
) {
    if (state_ != State::Recognizing) {
        return false;
    }
    if (systemError) {
        enterFault(reason.empty() ? "Plate recognition system failed" : reason, now);
    } else if (authorized) {
        transition(State::Opening, now);
    } else {
        transition(State::Denied, now);
    }
    return true;
}

bool Controller::acknowledgeFault(TimePoint now, const Inputs& inputs) {
    if (state_ != State::Fault || inputs.passageBlocked) {
        return false;
    }
    faultReason_.clear();
    transition(State::IdleClosed, now);
    return true;
}

bool Controller::manualOpen(TimePoint now, const Inputs&) {
    faultReason_.clear();
    trafficOverride_.reset();
    transition(State::Opening, now);
    return true;
}

bool Controller::manualClose(TimePoint now, const Inputs& inputs) {
    if (inputs.passageBlocked) return false;
    faultReason_.clear();
    trafficOverride_.reset();
    transition(State::Closing, now);
    return true;
}

void Controller::testTrafficSignal(
    TimePoint now,
    bool green,
    Milliseconds duration
) {
    trafficOverride_ = green;
    trafficOverrideEndsAt_ = now + duration;
}

Snapshot Controller::snapshot(TimePoint now, const Inputs& inputs) const {
    Outputs outputs;
    const bool authorizedMovement = state_ == State::Opening ||
        state_ == State::GateOpen || state_ == State::VehiclePassing ||
        state_ == State::ClearanceWait;
    outputs.trafficGreen = authorizedMovement;
    if (trafficOverride_ && now < trafficOverrideEndsAt_) {
        outputs.trafficGreen = *trafficOverride_;
    }
    outputs.requestOpen = state_ == State::Opening && now < relayPulseEndsAt_;
    outputs.requestClose = state_ == State::Closing && now < relayPulseEndsAt_ &&
        !inputs.passageBlocked;

    const bool cycleActive = state_ != State::Startup &&
        state_ != State::IdleClosed && state_ != State::Fault;
    return {state_, outputs, cycleActive, faultReason_};
}

State Controller::state() const {
    return state_;
}

const std::string& Controller::faultReason() const {
    return faultReason_;
}

const char* stateName(State state) {
    switch (state) {
        case State::Startup: return "STARTUP";
        case State::IdleClosed: return "IDLE_CLOSED";
        case State::Recognizing: return "RECOGNIZING";
        case State::Denied: return "DENIED";
        case State::Opening: return "OPENING";
        case State::GateOpen: return "GATE_OPEN";
        case State::VehiclePassing: return "VEHICLE_PASSING";
        case State::ClearanceWait: return "CLEARANCE_WAIT";
        case State::Closing: return "CLOSING";
        case State::Rearming: return "REARMING";
        case State::Fault: return "FAULT";
    }
    return "UNKNOWN";
}

}  // namespace gate
