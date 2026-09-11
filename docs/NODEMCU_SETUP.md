# NodeMCU setup and wiring

The controller is an ESP8266 NodeMCU. It owns the physical gate and communicates
with the centralized Plate Program server over HTTP or HTTPS.

## Upload

Open:

```text
firmware/rfid_gate_controller_v4/rfid_gate_controller_v4.ino
```

In Arduino IDE select an ESP8266 NodeMCU board and upload the sketch. Configure
the pin constants and RFID reader baud rate at the top of the sketch before
connecting the gate equipment.

## Provisioning

In Plate Program:

1. Create or select the village and gate.
2. Provision a controller as **Plate + RFID**.
3. Copy the controller ID and one-time key.
4. Bind one active camera to that gate.

Connect to the controller's `RFID-GATE` Wi-Fi, open `http://192.168.4.1`, and
open **System Mode**. Select **Plate Program** and enter the local Wi-Fi,
server URL, controller ID, and controller key. The server resolves the gate
and village from the credential; do not place those IDs in the firmware.

## Runtime flow

```text
Loop detects vehicle
        |
        +--> server capture request --> camera capture --> YOLO + OCR
        |
        +--> RFID read -------------------------------> server
                                                        |
                                  combine plate + RFID evidence
                                                        |
                           authorize or deny and record event
                                                        |
                         NodeMCU opens or keeps gate closed
```

Either a valid plate or a valid RFID can authorize the vehicle. A second,
late credential is correlated with the same server attempt and history event.
The camera is never connected to the NodeMCU; it is reached by the server.

## Pin assignments

The default sketch assignments are:

| Function | ESP8266 pin |
| --- | --- |
| Inductive loop | D1 / GPIO5 |
| IR safety beam | D2 / GPIO4 |
| RFID RX | D5 / GPIO14 |
| RFID TX | D6 / GPIO12 |
| Open relay | D7 / GPIO13 |
| Close relay | D0 / GPIO16 |
| RFID status | D3 / GPIO0 |
| Red status | D4 / GPIO2 |
| Green status | RX / GPIO3 |
| Vehicle status | TX / GPIO1 |
| Traffic light | D8 / GPIO15 |

Use dry contacts or 3.3 V-safe interfaces for sensor inputs. Never connect
12 V or 24 V directly to an ESP8266 pin. A true RS-232 RFID reader requires a
MAX3232 or equivalent level converter; do not connect RS-232 voltage directly
to the NodeMCU.

## Server endpoints used

The sketch authenticates every request with the controller ID and
`X-Controller-Key` header:

- `POST /api/rfid-controller/status`
- `POST /api/controller/capture-request`
- `POST /api/rfid-controller/recognitions`
- `POST /api/controller/access-result`

If the server, camera, recognition worker, or authorization result is
unavailable, the controller fails closed and does not open the barrier.
