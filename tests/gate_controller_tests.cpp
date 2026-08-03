#include "gate_gpio.hpp"
#include "rfid_trigger_output.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

using gate::Milliseconds;
using gate::State;

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

void activeLowBarrierLevels() {
    require(gate::barrierLineLevel(false), "inactive barrier request maps to HIGH");
    require(!gate::barrierLineLevel(true), "active barrier request maps to LOW");
}

void activeLowRfidLevels() {
    require(gate::rfidTriggerLineLevel(false),
            "inactive RFID trigger maps to HIGH");
    require(!gate::rfidTriggerLineLevel(true),
            "active RFID trigger maps to LOW");
}

struct Fixture {
    gate::Config config;
    gate::Controller controller{config};
    gate::Inputs inputs;
    gate::TimePoint now{};

    gate::Snapshot update(long long advance = 0) {
        now += Milliseconds(advance);
        return controller.update(now, inputs);
    }

    void startRecognition() {
        inputs.loopPresent = true;
        update();
        update(config.inputDebounce.count());
        require(controller.state() == State::Recognizing, "loop starts recognition after debounce");
    }
};

void authorizedCycleAndWaitingVehicle() {
    Fixture fixture;
    require(fixture.update().state == State::IdleClosed, "healthy startup reaches idle closed");
    fixture.startRecognition();
    require(fixture.controller.recognitionCompleted(fixture.now, true), "authorization is accepted");
    auto status = fixture.update();
    require(status.state == State::Opening && status.outputs.requestOpen, "authorized cycle pulses open");
    require(status.outputs.trafficGreen, "authorized movement shows green");

    status = fixture.update(999);
    require(status.outputs.requestOpen, "open request remains active before one second");
    status = fixture.update(1);
    require(!status.outputs.requestOpen, "open request becomes inactive after one second");

    require(fixture.update(fixture.config.openingTravelTime.count() - 1000).state == State::GateOpen,
            "opening travel delay completes");
    fixture.inputs.loopPresent = true;
    fixture.update(1000);
    require(fixture.controller.state() == State::GateOpen, "second loop activity cannot overlap cycle");

    fixture.inputs.passageBlocked = true;
    require(fixture.update().state == State::VehiclePassing, "passage blockage is detected");
    fixture.inputs.passageBlocked = false;
    require(fixture.update().state == State::ClearanceWait, "clear passage starts clearance timer");
    status = fixture.update(fixture.config.clearanceTime.count());
    require(status.state == State::Closing && status.outputs.requestClose, "clearance requests closing");
    require(!status.outputs.trafficGreen, "closing shows red");

    status = fixture.update(1000);
    require(!status.outputs.requestClose, "close request becomes inactive after one second");
    require(fixture.update(fixture.config.closingTravelTime.count() - 1000).state == State::Rearming,
            "closing travel delay ends movement cycle");
    fixture.update();
    fixture.update(fixture.config.inputDebounce.count());
    require(fixture.controller.state() == State::Recognizing, "waiting vehicle starts after confirmed closure");
}

void denialRequiresLoopClear() {
    Fixture fixture;
    fixture.update();
    fixture.startRecognition();
    fixture.controller.recognitionCompleted(fixture.now, false);
    const auto denied = fixture.update();
    require(denied.state == State::Denied, "denied plate keeps gate denied");
    require(!denied.outputs.trafficGreen && !denied.outputs.requestOpen &&
            !denied.outputs.requestClose,
            "denied plate remains red and never pulses movement outputs");
    fixture.update(5000);
    require(fixture.controller.state() == State::Denied, "occupied loop cannot retrigger denied vehicle");
    fixture.inputs.loopPresent = false;
    fixture.update();
    fixture.update(fixture.config.inputDebounce.count());
    require(fixture.controller.state() == State::IdleClosed, "denied cycle rearms only after loop clears");
}

void obstructionReopensAndBlocksCloseRelay() {
    Fixture fixture;
    fixture.update();
    fixture.startRecognition();
    fixture.controller.recognitionCompleted(fixture.now, true);
    fixture.update(fixture.config.openingTravelTime.count());
    fixture.inputs.passageBlocked = true;
    fixture.update();
    fixture.inputs.passageBlocked = false;
    fixture.update();
    fixture.update(fixture.config.clearanceTime.count());
    fixture.inputs.passageBlocked = true;
    const auto status = fixture.update();
    require(status.state == State::Opening, "obstruction during closing requests reopen");
    require(status.outputs.requestOpen, "reopen relay pulses after obstruction");
    require(!status.outputs.requestClose, "close relay is off whenever passage is blocked");
}

void faultsAreFailSafe() {
    Fixture fixture;
    fixture.update();
    fixture.startRecognition();
    fixture.update(fixture.config.recognitionTimeout.count());
    const auto fault = fixture.update();
    require(fault.state == State::Fault, "recognition timeout enters fault");
    require(!fault.outputs.trafficGreen, "fault defaults to red");
    require(!fault.outputs.requestOpen && !fault.outputs.requestClose,
            "fault removes movement requests");

    Fixture passageTimeout;
    passageTimeout.update();
    passageTimeout.startRecognition();
    passageTimeout.controller.recognitionCompleted(passageTimeout.now, true);
    passageTimeout.update(passageTimeout.config.openingTravelTime.count());
    passageTimeout.update(passageTimeout.config.passageTimeout.count());
    require(passageTimeout.controller.state() == State::Fault,
            "missing IR passage enters fault instead of closing blindly");
}

}  // namespace

int main() {
    activeLowBarrierLevels();
    activeLowRfidLevels();
    authorizedCycleAndWaitingVehicle();
    denialRequiresLoopClear();
    obstructionReopensAndBlocksCloseRelay();
    faultsAreFailSafe();
    std::cout << "All gate-controller safety tests passed.\n";
    return 0;
}
