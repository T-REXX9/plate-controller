# ESP8266 NodeMCU Gate Firmware

This repository contains the ESP8266/NodeMCU firmware for the Plate Program
gate controller. The controller is responsible for the physical gate only:

- inductive-loop vehicle detection
- RFID reader communication
- IR safety beam monitoring
- boom-barrier relay control
- red/green traffic signals
- fail-closed gate state management

The server is responsible for camera capture, YOLO plate detection, OCR,
vehicle lookup, RFID/plate correlation, access history, and authorization.
The controller never stores the server's vehicle database.

## Firmware

Open this file in Arduino IDE or PlatformIO:

```text
firmware/rfid_gate_controller_v4/rfid_gate_controller_v4.ino
```

Select an ESP8266 NodeMCU board, configure the pin assignments and RFID reader
settings near the top of the file, then upload it to the controller.

## Server setup

Before configuring the controller, use the Plate Program web application:

1. Create or select the village and gate.
2. Provision a controller as **Plate + RFID**.
3. Copy the controller ID and one-time controller key.
4. Bind an active network camera to the same gate.

The controller's local System Mode page requires:

- Wi-Fi SSID and password
- Plate Program URL, such as `https://server.example.com`
- provisioned controller ID
- one-time controller key

The key is sent in the `X-Controller-Key` header. The server resolves the
controller to its gate and village; the firmware does not submit or choose
those IDs.

## Access flow

When the loop reports a vehicle, the controller creates a capture attempt and
triggers the RFID reader. The server captures the gate-bound camera, runs YOLO
and OCR, combines the plate and RFID evidence, records one access event, and
returns the authorization decision. The controller opens the barrier only for
an authorized result and keeps it closed on timeout, denial, or communication
failure.

Either a valid plate or a valid RFID may authorize access. A late second
credential is correlated with the same server-side attempt and history event.

## Documentation

- `docs/GATE_WIRING_DIAGRAM.md` — ESP8266 pin and gate wiring
- `docs/GATE_CONTROL_PLAN.md` — safety state machine and timing
- `docs/RFID_READER_README.md` — RFID reader operation
- `docs/RFID_READER_COMMAND_MANUAL.md` — reader command details
- `docs/HQ_SERVER_PLAN.md` — future centralized fleet-management planning

The former Raspberry Pi camera reader and C++ recognition implementation are
not part of this repository's production controller path.
