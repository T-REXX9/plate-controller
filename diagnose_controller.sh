#!/usr/bin/env bash
set -uo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config_path="${PLATE_READER_CONFIG:-$project_dir/.env}"
service_name="plate-controller.service"
diagnostic_binary="$project_dir/build-pi/gate_hardware_diagnostic"
failures=0
warnings=0
service_was_running=0
safe_to_restart=1
cleanup_complete=0

pass() {
    printf '[PASS] %s\n' "$1"
}

fail() {
    printf '[FAIL] %s\n' "$1"
    failures=$((failures + 1))
}

warn() {
    printf '[WARN] %s\n' "$1"
    warnings=$((warnings + 1))
}

info() {
    printf '       %s\n' "$1"
}

restore_service() {
    local previous_status=$?
    if ((cleanup_complete == 1)); then
        return
    fi
    cleanup_complete=1
    echo
    if ((service_was_running == 1 && safe_to_restart == 1)); then
        echo "Restarting the controller service..."
        if systemctl start "$service_name"; then
            pass "Controller service restarted."
        else
            echo "[FAIL] Controller service could not be restarted." >&2
            previous_status=1
        fi
    elif ((safe_to_restart == 0)); then
        echo "DANGER: the barrier position is uncertain or OPEN." >&2
        echo "The controller service has intentionally remained stopped." >&2
        echo "Keep the gate secured and inspect the barrier before restarting it." >&2
    fi
    return "$previous_status"
}

trap restore_service EXIT
trap 'exit 130' INT TERM HUP

echo "Plate Controller maintenance diagnostics"
echo "========================================"
echo
echo "WARNING:"
echo "  This test temporarily STOPS automatic camera recognition and gate safety."
echo "  Confirm the boom barrier is CLOSED and no vehicle or person is beneath it."
echo
read -r -p "Type STOP to authorize stopping the controller service: " stop_answer
if [[ "$stop_answer" != "STOP" ]]; then
    echo "Diagnostic cancelled. The controller was not stopped."
    cleanup_complete=1
    exit 2
fi

if [[ ! -r "$config_path" ]]; then
    fail "Configuration cannot be read: $config_path"
    cleanup_complete=1
    exit 1
fi

set -a
# shellcheck disable=SC1090
source "$config_path"
set +a

: "${PLATE_SERVER_URL:=}"
: "${CAMERA_INDEX:=0}"
: "${CAMERA_WIDTH:=3840}"
: "${CAMERA_HEIGHT:=2160}"
: "${CAMERA_FPS:=30}"
: "${CAMERA_FOURCC:=MJPG}"
: "${GATE_MODE:=0}"
: "${GATE_LOOP_GPIO:=17}"
: "${GATE_PASSAGE_GPIO:=27}"

if systemctl is-active --quiet "$service_name"; then
    service_was_running=1
    echo
    echo "Stopping the controller service..."
    if ! systemctl stop "$service_name"; then
        fail "Controller service could not be stopped."
        cleanup_complete=1
        exit 1
    fi
    pass "Controller service stopped for maintenance."
else
    warn "Controller service was already stopped."
fi

echo
echo "USB camera"
camera_device="/dev/video$CAMERA_INDEX"
if [[ -e "$camera_device" ]]; then
    pass "Configured camera device exists: $camera_device"
    if command -v udevadm >/dev/null 2>&1; then
        camera_properties="$(udevadm info --query=property --name="$camera_device" 2>/dev/null || true)"
        vendor_id="$(awk -F= '$1 == "ID_VENDOR_ID" {print $2; exit}' <<<"$camera_properties")"
        product_id="$(awk -F= '$1 == "ID_MODEL_ID" {print $2; exit}' <<<"$camera_properties")"
        camera_name="$(awk -F= '$1 == "ID_V4L_PRODUCT" {sub(/^[^=]*=/, ""); print; exit}' <<<"$camera_properties")"
        serial="$(awk -F= '$1 == "ID_SERIAL_SHORT" {sub(/^[^=]*=/, ""); print; exit}' <<<"$camera_properties")"
        [[ -n "$camera_name" ]] && info "Name: $camera_name"
        if [[ -n "$vendor_id" || -n "$product_id" ]]; then
            info "USB ID: ${vendor_id:-unknown}:${product_id:-unknown}"
        fi
        if [[ -n "$serial" ]]; then
            info "Serial: $serial"
        else
            warn "The camera does not expose a USB serial number."
        fi
    fi
    if command -v v4l2-ctl >/dev/null 2>&1 &&
       v4l2-ctl --device "$camera_device" \
           --set-fmt-video="width=$CAMERA_WIDTH,height=$CAMERA_HEIGHT,pixelformat=$CAMERA_FOURCC" \
           --set-parm="$CAMERA_FPS" \
           --stream-mmap=3 --stream-count=1 --stream-to=/dev/null \
           >/dev/null 2>&1; then
        pass "Camera delivered a ${CAMERA_WIDTH}x${CAMERA_HEIGHT} test frame."
    else
        fail "Camera exists but failed to deliver a test frame."
    fi
else
    fail "Configured camera is missing: $camera_device"
    info "Connected video devices: $(find /dev -maxdepth 1 -name 'video*' -printf '%f ' 2>/dev/null || true)"
fi

echo
echo "PC server"
if [[ -z "$PLATE_SERVER_URL" ]]; then
    fail "PLATE_SERVER_URL is not configured."
elif curl --fail --silent --show-error --connect-timeout 2 --max-time 4 \
    "${PLATE_SERVER_URL%/}/health" >/dev/null 2>&1; then
    pass "Plate Program server is reachable: ${PLATE_SERVER_URL%/}"
else
    fail "Plate Program server is unavailable: ${PLATE_SERVER_URL%/}"
    info "Check the PC, Plate Program service, network, and firewall."
fi

echo
echo "Gate sensors"
if [[ "$GATE_MODE" != "1" ]]; then
    warn "Automatic GPIO gate mode is disabled."
    info "Camera and server diagnostics were completed; gate hardware was skipped."
elif [[ ! -x "$diagnostic_binary" ]]; then
    fail "GPIO diagnostic program is missing: $diagnostic_binary"
    info "Run controller -update to rebuild the controller."
else
    sensor_output="$("$diagnostic_binary" status 2>&1)"
    sensor_status=$?
    if ((sensor_status != 0)); then
        fail "GPIO sensors could not be read."
        info "$sensor_output"
    else
        pass "Both GPIO sensor inputs are readable."
        loop_present="$(awk -F= '$1 == "LOOP_PRESENT" {print $2; exit}' <<<"$sensor_output")"
        beam_broken="$(awk -F= '$1 == "IR_BEAM_BROKEN" {print $2; exit}' <<<"$sensor_output")"
        if [[ "$loop_present" == "1" ]]; then
            info "Inductive loop (BCM $GATE_LOOP_GPIO): VEHICLE PRESENT"
        else
            info "Inductive loop (BCM $GATE_LOOP_GPIO): clear"
        fi
        if [[ "$beam_broken" == "1" ]]; then
            info "IR beam (BCM $GATE_PASSAGE_GPIO): BROKEN / OBSTRUCTED"
        else
            info "IR beam (BCM $GATE_PASSAGE_GPIO): clear"
        fi

        echo
        echo "PHYSICAL BARRIER TEST WARNING:"
        echo "  The boom barrier will OPEN, remain open for 5 seconds, then CLOSE."
        echo "  Stay clear of the entire barrier path."
        echo "  Keep all people, vehicles, tools, and cables away from the boom."
        echo "  The test will refuse to close while the IR beam is obstructed."
        echo
        read -r -p "Type ACTIVATE only when the barrier area is completely clear: " barrier_answer
        if [[ "$barrier_answer" == "ACTIVATE" ]]; then
            safe_to_restart=0
            "$diagnostic_binary" barrier-test
            barrier_status=$?
            if ((barrier_status == 0)); then
                safe_to_restart=1
                pass "Physical barrier open/close test completed."
            elif ((barrier_status == 2)); then
                safe_to_restart=1
                fail "Barrier test was refused because the IR beam was obstructed."
            else
                fail "Barrier test did not complete safely."
                echo "The controller will remain stopped until the barrier is inspected." >&2
            fi
        else
            warn "Physical barrier test skipped; no barrier output was activated."
        fi
    fi
fi

echo
if ((failures == 0)); then
    if ((warnings == 0)); then
        echo "RESULT: ALL CHECKS PASSED"
    else
        echo "RESULT: PASSED WITH $warnings WARNING(S)"
    fi
    exit 0
fi

echo "RESULT: $failures CHECK(S) FAILED, $warnings WARNING(S)"
exit 1
