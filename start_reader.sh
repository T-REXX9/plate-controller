#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config_path="${PLATE_READER_CONFIG:-$project_dir/.env}"

if [[ ! -f "$config_path" ]]; then
    echo "Reader configuration is missing. Run ./configure_reader.sh first."
    exit 1
fi

set -a
# shellcheck disable=SC1090
source "$config_path"
set +a

: "${PLATE_SERVER_URL:?PLATE_SERVER_URL is missing from .env}"
: "${CAMERA_INDEX:=0}"
: "${CAMERA_WIDTH:=3840}"
: "${CAMERA_HEIGHT:=2160}"
: "${CAMERA_FPS:=30}"
: "${CAMERA_FOURCC:=MJPG}"
: "${CAMERA_STATUS_LED_GPIO:=25}"
: "${SERVER_STATUS_LED_GPIO:=5}"
: "${LOOP_STATUS_LED_GPIO:=6}"
: "${BARRIER_OPEN_STATUS_LED_GPIO:=12}"
: "${PLATE_UNRECOGNIZED_LED_GPIO:=13}"
: "${GATE_RFID_OUTPUT_GPIO:=16}"
: "${GATE_RFID_PULSE_MS:=1500}"
: "${RFID_ENABLED:=0}"
: "${RFID_SERIAL_DEVICE:=/dev/serial0}"
: "${RFID_BAUD_RATE:=9600}"
: "${RFID_READ_TIMEOUT_MS:=2000}"
: "${RFID_MIN_LENGTH:=4}"
: "${RFID_MAX_LENGTH:=64}"
: "${RFID_TAG_BYTES:=12}"
: "${GATE_MODE:=0}"
: "${PLATE_OUTPUT_DIR:=$project_dir/Output}"

if [[ "$GATE_MODE" != "0" && "$GATE_MODE" != "1" ]]; then
    echo "GATE_MODE must be 0 or 1."
    exit 1
fi
if [[ "$RFID_ENABLED" != "0" && "$RFID_ENABLED" != "1" ]]; then
    echo "RFID_ENABLED must be 0 or 1."
    exit 1
fi
if [[ ! "$CAMERA_WIDTH" =~ ^[1-9][0-9]*$ ]] ||
   [[ ! "$CAMERA_HEIGHT" =~ ^[1-9][0-9]*$ ]] ||
   [[ ! "$CAMERA_FPS" =~ ^[1-9][0-9]*$ ]]; then
    echo "CAMERA_WIDTH, CAMERA_HEIGHT, and CAMERA_FPS must be positive integers."
    exit 1
fi
if [[ ${#CAMERA_FOURCC} -ne 4 ]]; then
    echo "CAMERA_FOURCC must contain exactly four characters, such as MJPG."
    exit 1
fi
for gpio_name in \
    CAMERA_STATUS_LED_GPIO \
    SERVER_STATUS_LED_GPIO \
    LOOP_STATUS_LED_GPIO \
    BARRIER_OPEN_STATUS_LED_GPIO \
    PLATE_UNRECOGNIZED_LED_GPIO \
    GATE_RFID_OUTPUT_GPIO; do
    gpio_value="${!gpio_name}"
    if [[ ! "$gpio_value" =~ ^([0-9]|1[0-9]|2[0-7])$ ]]; then
        echo "$gpio_name must be a BCM GPIO number from 0 to 27."
        exit 1
    fi
done
if [[ "$GATE_RFID_OUTPUT_GPIO" == "14" || "$GATE_RFID_OUTPUT_GPIO" == "15" ]]; then
    echo "GATE_RFID_OUTPUT_GPIO cannot use physical pin 8 or 10 (BCM14/BCM15)."
    exit 1
fi
if [[ ! "$GATE_RFID_PULSE_MS" =~ ^[1-9][0-9]*$ ]]; then
    echo "GATE_RFID_PULSE_MS must be a positive integer."
    exit 1
fi
if [[ "$RFID_SERIAL_DEVICE" != /* || "$RFID_SERIAL_DEVICE" =~ [[:space:]] ]]; then
    echo "RFID_SERIAL_DEVICE must be an absolute path without spaces."
    exit 1
fi
case "$RFID_BAUD_RATE" in
    1200|2400|4800|9600|19200|38400|57600|115200) ;;
    *)
        echo "RFID_BAUD_RATE is unsupported."
        exit 1
        ;;
esac
if [[ ! "$RFID_READ_TIMEOUT_MS" =~ ^[1-9][0-9]*$ ]] ||
   ((RFID_READ_TIMEOUT_MS > 30000)); then
    echo "RFID_READ_TIMEOUT_MS must be from 1 to 30000."
    exit 1
fi
if [[ ! "$RFID_MIN_LENGTH" =~ ^[1-9][0-9]*$ ]] ||
   [[ ! "$RFID_MAX_LENGTH" =~ ^[1-9][0-9]*$ ]] ||
   ((RFID_MIN_LENGTH > RFID_MAX_LENGTH || RFID_MAX_LENGTH > 128)); then
    echo "RFID tag lengths must be valid values from 1 to 128."
    exit 1
fi
if [[ ! "$RFID_TAG_BYTES" =~ ^([0-9]|[1-5][0-9]|6[0-4])$ ]]; then
    echo "RFID_TAG_BYTES must be from 0 to 64."
    exit 1
fi

PLATE_SERVER_URL="${PLATE_SERVER_URL%/}"
export PLATE_SERVER_URL CAMERA_INDEX CAMERA_WIDTH CAMERA_HEIGHT CAMERA_FPS \
    CAMERA_FOURCC CAMERA_STATUS_LED_GPIO SERVER_STATUS_LED_GPIO \
    LOOP_STATUS_LED_GPIO BARRIER_OPEN_STATUS_LED_GPIO \
    PLATE_UNRECOGNIZED_LED_GPIO GATE_RFID_OUTPUT_GPIO \
    GATE_RFID_PULSE_MS RFID_ENABLED RFID_SERIAL_DEVICE RFID_BAUD_RATE \
    RFID_READ_TIMEOUT_MS RFID_MIN_LENGTH RFID_MAX_LENGTH RFID_TAG_BYTES
mkdir -p "$PLATE_OUTPUT_DIR"

server_available=0
if curl --fail --silent --show-error --connect-timeout 3 --max-time 5 \
    "$PLATE_SERVER_URL/health" >/dev/null; then
    server_available=1
fi

if [[ "${1:-}" == "--check" && "$server_available" == "1" ]]; then
    echo "PC website connection successful: $PLATE_SERVER_URL"
    exit 0
fi
if [[ "${1:-}" == "--check" ]]; then
    echo "The PC website is unavailable at $PLATE_SERVER_URL."
    exit 1
fi
if [[ "$server_available" == "0" ]]; then
    echo "Warning: the PC website is unavailable at $PLATE_SERVER_URL."
    echo "The controller will start safely and keep the Server detected LED off."
fi

reader="$project_dir/build-pi/plate_reader"
if [[ "$(uname -s)" == "Darwin" ]]; then
    reader="$project_dir/build/plate_reader"
fi
if [[ ! -x "$reader" ]]; then
    echo "The plate reader is not built. Run ./build_raspberry_pi.sh first."
    exit 1
fi

cd "$project_dir"
if [[ "$server_available" == "1" ]]; then
    echo "PC website connected: $PLATE_SERVER_URL"
else
    echo "Waiting for PC website: $PLATE_SERVER_URL"
fi
mode_argument="--remote-commands"
if [[ "$GATE_MODE" == "1" ]]; then
    mode_argument="--gate"
    echo "Starting automatic gate mode on camera $CAMERA_INDEX at ${CAMERA_WIDTH}x${CAMERA_HEIGHT}."
else
    echo "Starting camera $CAMERA_INDEX at ${CAMERA_WIDTH}x${CAMERA_HEIGHT} in idle mode; Capture requests come from the website."
fi
exec "$reader" --camera "$CAMERA_INDEX" \
  models/license_plate_detector.onnx \
  models/en_PP-OCRv5_rec_mobile.onnx \
  "$PLATE_OUTPUT_DIR" --headless "$mode_argument"
