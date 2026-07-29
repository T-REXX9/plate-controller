#!/usr/bin/env bash
set -uo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config_path="${PLATE_READER_CONFIG:-$project_dir/.env}"
service_name="plate-controller.service"
diagnostic_binary="$project_dir/build-pi/gate_hardware_diagnostic"
build_label="${PLATE_CONTROLLER_BUILD:-7.4.2-STABLE}"
failures=0
warnings=0
service_was_running=0
safe_to_restart=1
cleanup_complete=0

pass() {
    printf '[ PASS ] %s\n' "$1"
}

fail() {
    printf '[ FAIL ] %s\n' "$1"
    failures=$((failures + 1))
}

warn() {
    printf '[ WARN ] %s\n' "$1"
    warnings=$((warnings + 1))
}

hint() {
    printf '[ HINT ] %s\n' "$1"
}

detail() {
    printf '         %s\n' "$1"
}

module_header() {
    echo
    echo "──────────────────────────────────────────────────────────"
    printf '  MODULE %s/3 :: %s\n' "$1" "$2"
    echo "──────────────────────────────────────────────────────────"
}

diagnostic_summary() {
    echo
    echo "════════════════════════════════════════════════════════════"
    echo "  DIAGNOSTIC COMPLETE"
    if ((failures == 0 && warnings == 0)); then
        echo "  RESULT: ALL CHECKS PASSED"
    elif ((failures == 0)); then
        printf '  RESULT: PASSED WITH %d WARNING(S)\n' "$warnings"
    else
        printf '  RESULT: %d CHECK(S) FAILED, %d WARNING(S)\n' \
            "$failures" "$warnings"
    fi
    echo "════════════════════════════════════════════════════════════"
}

restore_service() {
    local previous_status=$?
    if ((cleanup_complete == 1)); then
        return
    fi
    cleanup_complete=1
    echo
    if ((service_was_running == 1 && safe_to_restart == 1)); then
        printf '[ACTION] Restarting controller service..............'
        if systemctl start "$service_name" >/dev/null 2>&1; then
            echo "[DONE]"
            pass "Controller service restarted."
        else
            echo "[FAILED]"
            echo "[ FAIL ] Controller service could not be restarted." >&2
            previous_status=1
        fi
    elif ((safe_to_restart == 0)); then
        echo "[!!] DANGER :::::::::::::::::::::::::::::::::::::::::::::::::"
        echo "     Barrier position is uncertain or OPEN."
        echo "     Controller service intentionally remains stopped."
        echo "     Secure and inspect the barrier before restarting."
        echo "::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
    fi
    return "$previous_status"
}

trap restore_service EXIT
trap 'exit 130' INT TERM HUP

echo
echo "╔══════════════════════════════════════════════════════════╗"
echo "║        PLATE CONTROLLER :: MAINTENANCE DIAGNOSTICS       ║"
build_text="BUILD $build_label"
build_text="${build_text:0:58}"
build_left_padding=$(((58 - ${#build_text}) / 2))
build_right_padding=$((58 - ${#build_text} - build_left_padding))
printf '║%*s%s%*s║\n' \
    "$build_left_padding" "" "$build_text" "$build_right_padding" ""
echo "╚══════════════════════════════════════════════════════════╝"
echo
printf '>> Session start: %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
printf '>> Node ID: %s@%s\n' "$(id -un)" "$(hostname -s)"
echo
echo "[!!] WARNING ::::::::::::::::::::::::::::::::::::::::::::::::"
echo "     This test temporarily STOPS automatic camera recognition"
echo "     and gate safety systems."
echo "     CONFIRM: boom barrier is CLOSED. No vehicle or person"
echo "     beneath it."
echo "::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
echo
echo ">> Authorization required to suspend controller service"
read -r -p ">> Type STOP to confirm: " stop_answer
if [[ "$stop_answer" != "STOP" ]]; then
    echo ">> Authorization rejected. Diagnostic cancelled."
    cleanup_complete=1
    exit 2
fi
echo ">> Authorization accepted."

if [[ ! -r "$config_path" ]]; then
    echo
    fail "Configuration cannot be read: $config_path"
    hint "Run controller -configure, then try again."
    diagnostic_summary
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

echo
if systemctl is-active --quiet "$service_name"; then
    service_was_running=1
    printf '[ACTION] Stopping controller service...............'
    if systemctl stop "$service_name" >/dev/null 2>&1; then
        echo "[DONE]"
        pass "Controller service stopped for maintenance."
    else
        echo "[FAILED]"
        fail "Controller service could not be stopped."
        diagnostic_summary
        cleanup_complete=1
        exit 1
    fi
else
    echo "[ACTION] Controller service already stopped.........[SKIP]"
    warn "Controller service was already stopped."
fi

module_header 1 "USB CAMERA SUBSYSTEM"
camera_device="/dev/video$CAMERA_INDEX"
if [[ -e "$camera_device" ]]; then
    printf '[ SCAN ] Probing %-29s [ OK ]\n' "$camera_device"
    pass "Configured camera device exists: $camera_device"
    camera_name=""
    vendor_id=""
    product_id=""
    serial=""
    if command -v udevadm >/dev/null 2>&1; then
        camera_properties="$(udevadm info --query=property --name="$camera_device" 2>/dev/null || true)"
        vendor_id="$(awk -F= '$1 == "ID_VENDOR_ID" {print $2; exit}' <<<"$camera_properties")"
        product_id="$(awk -F= '$1 == "ID_MODEL_ID" {print $2; exit}' <<<"$camera_properties")"
        camera_name="$(awk -F= '$1 == "ID_V4L_PRODUCT" {sub(/^[^=]*=/, ""); print; exit}' <<<"$camera_properties")"
        serial="$(awk -F= '$1 == "ID_SERIAL_SHORT" {sub(/^[^=]*=/, ""); print; exit}' <<<"$camera_properties")"
    fi
    detail "├─ Name   : ${camera_name:-unknown}"
    detail "├─ USB ID : ${vendor_id:-unknown}:${product_id:-unknown}"
    detail "└─ Serial : ${serial:-not provided}"
    if [[ -z "$serial" ]]; then
        warn "The camera does not expose a USB serial number."
    fi

    if command -v v4l2-ctl >/dev/null 2>&1 &&
       v4l2-ctl --device "$camera_device" \
           --set-fmt-video="width=$CAMERA_WIDTH,height=$CAMERA_HEIGHT,pixelformat=$CAMERA_FOURCC" \
           --set-parm="$CAMERA_FPS" \
           --stream-mmap=3 --stream-count=1 --stream-to=/dev/null \
           >/dev/null 2>&1; then
        echo "[ SCAN ] Requesting test frame ......[####################] 100%"
        pass "Camera delivered a ${CAMERA_WIDTH}x${CAMERA_HEIGHT} test frame."
    else
        echo "[ SCAN ] Requesting test frame .....................[FAILED]"
        fail "Camera exists but failed to deliver a test frame."
    fi
else
    printf '[ SCAN ] Probing %-29s [MISSING]\n' "$camera_device"
    fail "Configured camera is missing: $camera_device"
    connected_devices="$(find /dev -maxdepth 1 -name 'video*' -printf '%f ' 2>/dev/null || true)"
    detail "└─ Connected devices: ${connected_devices:-none}"
fi

module_header 2 "PC SERVER LINK"
server_url="${PLATE_SERVER_URL%/}"
if [[ -z "$server_url" ]]; then
    echo "[ SCAN ] Reading configured server address.........[MISSING]"
    fail "PLATE_SERVER_URL is not configured."
    hint "Run controller -configure after Plate Program is available."
else
    curl --fail --silent --show-error --connect-timeout 2 --max-time 4 \
        "$server_url/health" >/dev/null 2>&1
    server_status=$?
    if ((server_status == 0)); then
        printf '[ SCAN ] Contacting %-31s[ OK ]\n' "$server_url"
        pass "Plate Program server is reachable."
        detail "└─ $server_url"
    else
        if ((server_status == 28)); then
            scan_result="[TIMEOUT]"
        else
            scan_result="[FAILED]"
        fi
        printf '[ SCAN ] Contacting %-31s%s\n' "$server_url" "$scan_result"
        fail "Plate Program server is unavailable."
        detail "└─ $server_url"
        hint "Check the PC, Plate Program service, network,"
        detail "and firewall."
    fi
fi

module_header 3 "GATE SENSORS"
if [[ "$GATE_MODE" != "1" ]]; then
    echo "[ SCAN ] Querying GPIO gate mode ...................[SKIPPED]"
    warn "Automatic GPIO gate mode is disabled."
    detail "└─ Camera and server diagnostics were completed;"
    detail "   gate hardware was skipped."
elif [[ ! -x "$diagnostic_binary" ]]; then
    echo "[ SCAN ] Loading GPIO diagnostic program...........[MISSING]"
    fail "GPIO diagnostic program is missing."
    detail "└─ $diagnostic_binary"
    hint "Run controller -update to rebuild the controller."
else
    sensor_output="$("$diagnostic_binary" status 2>&1)"
    sensor_status=$?
    if ((sensor_status != 0)); then
        echo "[ SCAN ] Reading GPIO sensor inputs...............[FAILED]"
        fail "GPIO sensors could not be read."
        detail "└─ $sensor_output"
    else
        echo "[ SCAN ] Reading GPIO sensor inputs..................[ OK ]"
        pass "Both GPIO sensor inputs are readable."
        loop_present="$(awk -F= '$1 == "LOOP_PRESENT" {print $2; exit}' <<<"$sensor_output")"
        beam_broken="$(awk -F= '$1 == "IR_BEAM_BROKEN" {print $2; exit}' <<<"$sensor_output")"
        if [[ "$loop_present" == "1" ]]; then
            detail "├─ Inductive loop (BCM $GATE_LOOP_GPIO): VEHICLE PRESENT"
        else
            detail "├─ Inductive loop (BCM $GATE_LOOP_GPIO): clear"
        fi
        if [[ "$beam_broken" == "1" ]]; then
            detail "└─ IR beam (BCM $GATE_PASSAGE_GPIO): BROKEN / OBSTRUCTED"
        else
            detail "└─ IR beam (BCM $GATE_PASSAGE_GPIO): clear"
        fi

        echo
        echo "[!!] PHYSICAL BARRIER TEST :::::::::::::::::::::::::::::::::"
        echo "     The boom barrier will OPEN, remain open for 5 seconds,"
        echo "     then CLOSE. Stay clear of the entire barrier path."
        echo "     Remove all people, vehicles, tools, and cables."
        echo "     Closing is blocked while the IR beam is obstructed."
        echo "::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
        echo
        echo ">> Authorization required to activate barrier outputs"
        read -r -p ">> Type ACTIVATE only when the area is clear: " barrier_answer
        if [[ "$barrier_answer" == "ACTIVATE" ]]; then
            echo ">> Barrier authorization accepted."
            safe_to_restart=0
            "$diagnostic_binary" barrier-test
            barrier_status=$?
            if ((barrier_status == 0)); then
                safe_to_restart=1
                pass "Physical barrier open/close test completed."
            elif ((barrier_status == 2)); then
                safe_to_restart=1
                fail "Barrier test refused because the IR beam was obstructed."
            else
                fail "Barrier test did not complete safely."
                hint "Controller remains stopped until the barrier is inspected."
            fi
        else
            echo ">> Barrier authorization rejected."
            warn "Physical barrier test skipped; outputs were not activated."
        fi
    fi
fi

diagnostic_summary
if ((failures == 0)); then
    exit 0
fi
exit 1
