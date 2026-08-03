#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config_path="${PLATE_READER_CONFIG:-$project_dir/.env}"

echo "Searching the local network for the plate-program server..."
discovered_servers=()
while IFS= read -r discovered_server; do
    [[ -n "$discovered_server" ]] && discovered_servers+=("$discovered_server")
done < <("$project_dir/discover_server.sh" 2>/dev/null || true)

server_url=""
if [[ ${#discovered_servers[@]} -eq 1 ]]; then
    read -r -p "Found ${discovered_servers[0]}. Use this server? [Y/n]: " use_found
    case "${use_found:-y}" in
        n|N|no|NO) ;;
        *) server_url="${discovered_servers[0]}" ;;
    esac
elif [[ ${#discovered_servers[@]} -gt 1 ]]; then
    echo "Found these possible plate-program servers:"
    for index in "${!discovered_servers[@]}"; do
        printf '  %d) %s\n' "$((index + 1))" "${discovered_servers[$index]}"
    done
    read -r -p "Choose a number, or press Enter to type the address: " choice
    if [[ "$choice" =~ ^[0-9]+$ ]] && ((choice >= 1 && choice <= ${#discovered_servers[@]})); then
        server_url="${discovered_servers[$((choice - 1))]}"
    fi
else
    echo "No plate-program server was found automatically."
fi

if [[ -z "$server_url" ]]; then
    read -r -p "PC website address (example http://192.168.0.103:8080): " server_url
fi
server_url="${server_url%/}"
if [[ ! "$server_url" =~ ^https?://[A-Za-z0-9._:-]+$ ]]; then
    echo "Enter a complete http:// or https:// website address."
    exit 1
fi

if command -v v4l2-ctl >/dev/null 2>&1; then
    echo
    echo "Connected cameras:"
    v4l2-ctl --list-devices 2>/dev/null || true
fi

while true; do
    read -r -p "USB camera index [0]: " camera_index
    camera_index="${camera_index:-0}"
    if [[ ! "$camera_index" =~ ^[0-9]+$ ]]; then
        echo "Camera index must be zero or a positive number."
        continue
    fi
    if [[ "$(uname -s)" == "Linux" && ! -e "/dev/video$camera_index" ]]; then
        echo "/dev/video$camera_index does not exist. Connect the camera or choose another index."
        continue
    fi
    break
done

camera_width=3840
camera_height=2160
camera_fps=30
camera_fourcc=MJPG
camera_status_led_gpio=25
server_status_led_gpio=5
loop_status_led_gpio=6
barrier_open_status_led_gpio=12
plate_unrecognized_led_gpio=13
rfid_output_gpio=16
rfid_pulse_ms=1500
echo
echo "Using the EMEET C950 4K profile: 3840x2160, MJPEG, 30 FPS, autofocus."
if [[ "$(uname -s)" == "Linux" ]] && command -v v4l2-ctl >/dev/null 2>&1; then
    if ! v4l2-ctl --device "/dev/video$camera_index" --list-formats-ext 2>/dev/null |
        grep -qE '3840x2160|4096x2160'; then
        echo "Warning: /dev/video$camera_index did not advertise a 4K capture mode."
        echo "If the webcam exposes several video devices, select its 4K-capable index."
    fi
fi

read -r -p "Enable automatic GPIO gate mode? [y/N]: " gate_answer
case "${gate_answer:-n}" in
    y|Y|yes|YES)
        read -r -p "Are the two switches and all four protected 3.3 V output interfaces wired? [y/N]: " wiring_answer
        case "${wiring_answer:-n}" in
            y|Y|yes|YES) gate_mode=1 ;;
            *)
                gate_mode=0
                echo "Gate mode will remain safely disabled until the wiring is ready."
                ;;
        esac
        ;;
    *) gate_mode=0 ;;
esac

mkdir -p "$(dirname "$config_path")"
umask 077
printf 'PLATE_SERVER_URL=%s\nCAMERA_INDEX=%s\nCAMERA_WIDTH=%s\nCAMERA_HEIGHT=%s\nCAMERA_FPS=%s\nCAMERA_FOURCC=%s\nCAMERA_STATUS_LED_GPIO=%s\nSERVER_STATUS_LED_GPIO=%s\nLOOP_STATUS_LED_GPIO=%s\nBARRIER_OPEN_STATUS_LED_GPIO=%s\nPLATE_UNRECOGNIZED_LED_GPIO=%s\nGATE_RFID_OUTPUT_GPIO=%s\nGATE_RFID_PULSE_MS=%s\nGATE_MODE=%s\n' \
    "$server_url" "$camera_index" "$camera_width" "$camera_height" \
    "$camera_fps" "$camera_fourcc" "$camera_status_led_gpio" \
    "$server_status_led_gpio" "$loop_status_led_gpio" \
    "$barrier_open_status_led_gpio" "$plate_unrecognized_led_gpio" \
    "$rfid_output_gpio" "$rfid_pulse_ms" \
    "$gate_mode" > "$config_path"
chmod 600 "$config_path"

if curl --fail --silent --show-error --connect-timeout 3 --max-time 5 \
    "$server_url/health" >/dev/null; then
    echo "Website connection successful. Reader configuration saved to .env."
else
    echo "Configuration saved, but the website health check failed."
    echo "Confirm the PC address, firewall, and that the website is running."
    exit 1
fi
