# Raspberry Pi Plate Reader

This repository contains the lightweight Raspberry Pi side of the plate access
control system. The Pi performs only camera capture, YOLO plate detection,
crop/enhancement, and PP-OCRv5 recognition. It sends each result immediately to
the separate PC web server and does not host a website or database.

## Recognition workflow

1. Keep the camera open while YOLO and OCR remain idle.
2. Wait for an administrator to press **Capture plate** on the PC dashboard.
3. Acquire two fresh full-resolution frames, run YOLO at 60% minimum confidence,
   and evaluate the strongest crops with PP-OCRv5.
4. Return immediately when the first plate crop and OCR probabilities are strong.
5. When uncertain, acquire one more frame and use two-sample OCR consensus.
6. Return the final clean alphanumeric value without imposing a plate format.
7. Store only the winning enhanced crop in `Output/Plate-Crops`.
8. Send the plate, detector confidence, crop, raw frame, and annotated frame to
   the PC server. Raw and annotated frames are encoded in memory and are not
   retained on the Raspberry Pi.

## Boom-barrier control development

The safety state machine and macOS/Raspberry Pi simulator are now included.
They implement the cycle lock, authorization/denial paths, one-second open and
close pulses, passage clearance, obstruction reopening, waiting-vehicle
behavior, the single LOW-red/HIGH-green traffic output, and an active-low RFID
trigger synchronized with automatic camera capture.

Build and run the simulator:

```bash
cmake -S . -B build -DPLATE_ENABLE_CAMERA=ON -DBUILD_TESTING=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
./build/gate_simulator
```

The switch connections and GPIO assignments are documented
in [`docs/GATE_WIRING_DIAGRAM.md`](docs/GATE_WIRING_DIAGRAM.md). Physical GPIO
movement remains disabled until `GATE_MODE=1` is set in the private `.env`.

After wiring, enable the automatic inductive-loop sequence by editing `.env`:

```text
GATE_MODE=1
```

`./start_reader.sh` will then wait for BCM17 to be shorted to ground instead of
waiting for the dashboard Capture button. BCM17 and BCM27 use internal pull-ups,
so HIGH is idle and grounded LOW means vehicle present or IR beam broken.

Five active-high status indicators are available:

- Camera detected: BCM25, physical pin 22.
- Server detected: BCM5, physical pin 29.
- Loop detector active: BCM6, physical pin 31.
- Boom barrier open: BCM12, physical pin 32.
- Plate not recognized: BCM13, physical pin 33.

The optional RFID reader is controlled entirely through `/dev/serial0` on
physical pins 8/10. Its tag number is printed in the live controller log.
These are 3.3 V UART pins, so a true RS-232 reader requires an RS-232-to-3.3 V
TTL transceiver. The confirmed UHFReader18-compatible reader is placed in Answer
Mode when the controller starts. On each loop-triggered cycle the controller sends
`04 00 0F A5 A2`, validates the returned CRC, and uploads only the EPC bytes as
uppercase hexadecimal without separators. Physical pin 36 is unused. A
registered plate or an active RFID
sticker independently authorizes the barrier; one credential does not fail merely
because the other is absent or unknown. When RFID is disabled, plate-only
authorization remains active. During
`controller -configure`, select the baud rate stated in the RFID reader manual;
the value is saved in the controller configuration rather than assumed.

Each LED requires its own 220–330 Ω series resistor. Complete wiring and
indicator behavior are documented in
[`docs/GATE_WIRING_DIAGRAM.md`](docs/GATE_WIRING_DIAGRAM.md).

## Raspberry Pi 4 setup

For a completely fresh 64-bit Raspberry Pi OS installation, run this single
command:

```bash
curl -fsSL https://raw.githubusercontent.com/T-REXX9/plate-controller/main/install_controller.sh -o /tmp/install-controller.sh && sudo bash /tmp/install-controller.sh
```

The guided installer asks whether the separate PC server is already installed.
When it is available, the installer scans the Pi's local `/24` network for the
plate-program identity and health endpoint, asks for the camera, builds
and tests everything, installs GPIO, video, and UART permissions, and starts the reader as a
background system service. It supports both Raspberry Pi OS Bookworm and Trixie;
if the operating system's OpenCV is too old, it builds the required minimal
OpenCV 4.10 installation automatically. Long compiler output is kept out of the
terminal and saved to `/var/log/plate-controller-install.log`; if setup fails,
the installer prints the useful final part of that log automatically.

If the PC server has not been installed yet, the Pi setup still completes but
leaves the controller safely stopped. After setting up the server, run:

```bash
controller -configure
```

No project-directory knowledge is needed afterward. Common commands are:

```bash
controller -status
controller -logs
controller -diagnose
controller -update
controller -restart
controller -stop
controller -start
```

`controller -diagnose` requires confirmation before temporarily stopping the
service, then checks the USB camera identity and test frame, Plate Program server,
inductive-loop input, and IR-beam input. It separately warns the operator and
requires the exact confirmation `ACTIVATE` before running a physical barrier
open/close test. If that test is interrupted or the IR beam is broken while
closing, the barrier reopens and the controller remains stopped for inspection.

`controller -update` stops the service, fast-forwards the managed clone from the
GitHub `main` branch, rebuilds and tests it, refreshes the service installation,
and starts it again. A failed build automatically restores the last working
revision.

The older `./build_raspberry_pi.sh` command remains available for developers who
already cloned and configured the repository manually.

## Connect it to the PC server

Start the separate `plate-program` project on the PC. That website
uses native MySQL locally; the Pi never connects to MySQL. It sends recognition
results to the Flask API over the trusted local network.

On the Raspberry Pi, run:

```bash
./configure_reader.sh
```

The setup searches the local network for the website. Confirm the discovered
address or enter one such as `http://192.168.0.103:8080`, then choose the USB
camera index. The controller requests the webcam's 3840×2160 MJPEG mode at
30 FPS and prints both the requested and actually negotiated camera modes when
it starts. The 4K source frame is retained through plate detection and cropping,
so OCR receives the maximum plate detail even though YOLO uses its trained
640×640 inference input. These defaults match the EMEET C950 4K's advertised
4K/30 FPS mode, and the controller explicitly enables the camera's autofocus.
The configuration is stored in a
private `.env` file that Git ignores. The setup checks the website health route
before accepting the configuration.

The defaults can be changed in the private controller configuration when a
camera requires a different mode:

```text
CAMERA_WIDTH=3840
CAMERA_HEIGHT=2160
CAMERA_FPS=30
CAMERA_FOURCC=MJPG
```

On Raspberry Pi OS, list the exact modes exposed by the selected USB camera
with:

```bash
v4l2-ctl --device /dev/video0 --list-formats-ext
```

Start the reader with:

```bash
./start_reader.sh
```

The launcher checks the PC website before opening the camera, then polls its
capture queue while inference stays idle.

To test only the PC connection without opening the camera:

```bash
./start_reader.sh --check
```

Press **Capture plate** on the administrator dashboard. The Pi terminal prints
whether the one-frame fast path or two-frame fallback was used, the final
plate, web server response, and a timing summary for frame capture, YOLO, OCR,
upload, and total processing time.

For temporary maintenance, the server address can be supplied as an environment
variable:

```bash
PLATE_SERVER_URL=http://192.168.0.103:8080 \
./build-pi/plate_reader --camera 0 \
  models/license_plate_detector.onnx \
  models/en_PP-OCRv5_rec_mobile.onnx \
  Output --headless
```

The interactive configuration script is preferred because it validates the
server connection and camera selection.

## macOS build

```bash
brew install cmake opencv curl
cmake -S . -B build -DPLATE_ENABLE_CAMERA=ON
cmake --build build -j
```

Run `./configure_reader.sh`, followed by `./start_reader.sh`. The launcher
automatically chooses the macOS build. Camera access must be allowed for Terminal.

## Process an image folder

Folder mode remains available and does not contact the server:

```bash
./build/plate_reader raw-images Output models/license_plate_detector.onnx
```

Annotated images are written to `Output`, and enhanced plate crops are written
to `Output/Plate-Crops`.

Models are included so the Pi can run YOLO and OCR locally without Python,
EasyOCR, Tesseract, or an internet connection.
