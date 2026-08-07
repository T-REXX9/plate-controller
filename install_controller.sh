#!/usr/bin/env bash
set -euo pipefail

repository_url="${PLATE_CONTROLLER_REPOSITORY:-https://github.com/T-REXX9/plate-controller.git}"
branch="${PLATE_CONTROLLER_BRANCH:-main}"
opencv_version="4.10.0"
opencv_sha256="b2171af5be6b26f7a06b1229948bbb2bdaa74fcf5cd097e0af6378fce50a6eb9"
project_dir="/opt/plate-controller"
config_dir="/etc/plate-controller"
config_path="$config_dir/controller.env"
data_dir="/var/lib/plate-controller"
service_user="platecontroller"
install_log="/var/log/plate-controller-install.log"
work_dir=""
refresh=0
[[ "${1:-}" == "--refresh" ]] && refresh=1

cleanup() {
    if [[ -n "$work_dir" && -d "$work_dir" ]]; then
        rm -rf "$work_dir"
    fi
}

failure_report() {
    local status=$?
    echo >&2
    echo "Controller setup stopped at line ${BASH_LINENO[0]}." >&2
    echo "The last installation messages are in $install_log." >&2
    if [[ -f "$install_log" ]]; then
        tail -n 40 "$install_log" >&2 || true
    fi
    exit "$status"
}

retry() {
    local attempt=1
    local maximum=3
    until "$@"; do
        if ((attempt >= maximum)); then
            echo "Command failed after $maximum attempts: $*" >&2
            return 1
        fi
        echo "Temporary failure. Retrying in 5 seconds ($attempt/$maximum)..."
        sleep 5
        ((attempt += 1))
    done
}

wait_for_service() {
    local attempt
    local consecutive_active=0
    for attempt in {1..20}; do
        if systemctl is-active --quiet plate-controller.service; then
            ((consecutive_active += 1))
            if ((consecutive_active >= 3)); then
                return 0
            fi
        else
            consecutive_active=0
        fi
        if systemctl is-failed --quiet plate-controller.service; then
            break
        fi
        sleep 1
    done
    systemctl --no-pager --full status plate-controller.service || true
    journalctl -u plate-controller.service --no-pager -n 60 || true
    return 1
}

trap cleanup EXIT
trap failure_report ERR

if [[ $EUID -ne 0 ]]; then
    echo "Administrator permission is required."
    exec sudo bash "$0" "$@"
fi

if [[ "${PLATE_ALLOW_NON_PI:-0}" != "1" ]]; then
    if [[ "$(uname -m)" != "aarch64" ]]; then
        echo "This installer requires 64-bit Raspberry Pi OS (aarch64)." >&2
        exit 1
    fi
    if [[ ! -r /proc/device-tree/model ]] ||
       ! tr -d '\0' </proc/device-tree/model | grep -qi "Raspberry Pi"; then
        echo "This installer must run directly on Raspberry Pi hardware." >&2
        exit 1
    fi
    if [[ ! -d /run/systemd/system ]]; then
        echo "This installer requires Raspberry Pi OS with systemd." >&2
        exit 1
    fi
fi

if [[ ! -r /etc/os-release ]]; then
    echo "Unable to identify the operating system (/etc/os-release is missing)." >&2
    exit 1
fi
# shellcheck disable=SC1091
source /etc/os-release
os_codename="${VERSION_CODENAME:-}"
if [[ "${PLATE_ALLOW_NON_PI:-0}" != "1" ]]; then
    # Official 64-bit Raspberry Pi OS identifies itself as ID=debian, while
    # the 32-bit image identifies itself as ID=raspbian with ID_LIKE=debian.
    if [[ "${ID:-}" != "debian" &&
          "${ID:-}" != "raspbian" &&
          "${ID_LIKE:-}" != *debian* ]]; then
        echo "This installer supports Raspberry Pi OS only." >&2
        exit 1
    fi
    case "$os_codename" in
        bookworm|trixie) ;;
        *)
            echo "Supported Raspberry Pi OS releases are Bookworm and Trixie." >&2
            echo "Detected release: ${PRETTY_NAME:-unknown}" >&2
            exit 1
            ;;
    esac
fi

: >"$install_log"
chmod 600 "$install_log"
echo "Detected ${PRETTY_NAME:-Linux} on $(uname -m)." | tee -a "$install_log"

server_ready="yes"
if ((refresh == 0)); then
    echo
    echo "Plate Controller — complete Raspberry Pi setup"
    echo "This may take a long time on the first installation. Do not turn off the Pi."
    echo
    read -r -p "Is the plate-program website already set up and running on the PC? [y/N]: " answer
    case "${answer:-n}" in
        y|Y|yes|YES) server_ready="yes" ;;
        *) server_ready="no" ;;
    esac
fi

export DEBIAN_FRONTEND=noninteractive
echo "Installing operating-system requirements..."
dpkg --configure -a >>"$install_log" 2>&1 || true
apt-get -f install -y >>"$install_log" 2>&1 || true
retry apt-get -o Acquire::Retries=5 update

required_packages=(
    ca-certificates git curl nmap iproute2 v4l-utils
    build-essential cmake ninja-build pkg-config
    libcurl4-openssl-dev libgpiod-dev gpiod
    libjpeg-dev libpng-dev libtiff-dev libwebp-dev libopenjp2-7-dev
    libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libv4l-dev
)
missing_packages=()
for package in "${required_packages[@]}"; do
    candidate="$(apt-cache policy "$package" 2>/dev/null | awk '/Candidate:/ {print $2; exit}')"
    if [[ -z "$candidate" || "$candidate" == "(none)" ]]; then
        missing_packages+=("$package")
    fi
done
if ((${#missing_packages[@]} > 0)); then
    echo "Required Raspberry Pi OS packages are unavailable:" >&2
    printf '  %s\n' "${missing_packages[@]}" >&2
    echo "Use an unmodified Raspberry Pi OS Bookworm or Trixie package configuration." >&2
    exit 1
fi
retry apt-get install -y --no-install-recommends "${required_packages[@]}"

if ((refresh == 0)); then
    if [[ -d "$project_dir/.git" ]]; then
        echo "The controller repository already exists; updating it."
        retry git -C "$project_dir" fetch origin "$branch"
        git -C "$project_dir" merge --ff-only "origin/$branch"
    else
        rm -rf "$project_dir"
        retry git clone --branch "$branch" --single-branch "$repository_url" "$project_dir"
    fi
fi

opencv_dir=""
candidate="$(apt-cache policy libopencv-dev 2>/dev/null | awk '/Candidate:/ {print $2; exit}')"
if [[ -n "$candidate" && "$candidate" != "(none)" ]] && \
   dpkg --compare-versions "$candidate" ge 4.10; then
    echo "Installing the Raspberry Pi OS OpenCV package ($candidate)..."
    retry apt-get install -y --no-install-recommends libopencv-dev
else
    opencv_prefix="/opt/opencv-$opencv_version"
    opencv_dir="$opencv_prefix/lib/cmake/opencv4"
    if [[ ! -f "$opencv_dir/OpenCVConfig.cmake" ]]; then
        free_kb="$(df --output=avail /opt | tail -1 | tr -d ' ')"
        if ((free_kb < 3500000)); then
            echo "At least 3.5 GB of free disk space is required to build OpenCV." >&2
            exit 1
        fi
        echo "Raspberry Pi OS provides an older OpenCV."
        echo "Building OpenCV $opencv_version once; this can take a while on a Pi 4."
        echo "Detailed build output is being saved to $install_log."
        work_dir="$(mktemp -d /var/tmp/plate-opencv.XXXXXX)"
        curl --fail --location --retry 5 --retry-delay 2 \
            --output "$work_dir/opencv.tar.gz" \
            "https://codeload.github.com/opencv/opencv/tar.gz/refs/tags/$opencv_version"
        printf '%s  %s\n' "$opencv_sha256" "$work_dir/opencv.tar.gz" |
            sha256sum --check -
        tar -xzf "$work_dir/opencv.tar.gz" -C "$work_dir"
        {
            cmake -S "$work_dir/opencv-$opencv_version" -B "$work_dir/build" -G Ninja \
                -DCMAKE_BUILD_TYPE=Release \
                -DCMAKE_INSTALL_PREFIX="$opencv_prefix" \
                -DBUILD_LIST=core,imgproc,imgcodecs,dnn,videoio,highgui \
                -DBUILD_SHARED_LIBS=ON \
                -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF \
                -DBUILD_EXAMPLES=OFF -DBUILD_opencv_apps=OFF \
                -DBUILD_JAVA=OFF -DBUILD_opencv_python2=OFF -DBUILD_opencv_python3=OFF \
                -DWITH_GTK=OFF -DWITH_QT=OFF -DWITH_OPENCL=OFF \
                -DWITH_V4L=ON -DWITH_FFMPEG=ON
            cmake --build "$work_dir/build" --parallel 2
            cmake --install "$work_dir/build"
        } >>"$install_log" 2>&1
        printf '%s\n' "$opencv_prefix/lib" > /etc/ld.so.conf.d/plate-controller-opencv.conf
        ldconfig
        rm -rf "$work_dir"
        work_dir=""
    fi
    printf '%s\n' "$opencv_prefix/lib" > /etc/ld.so.conf.d/plate-controller-opencv.conf
    ldconfig
fi

echo "Building and testing the plate controller..."
if [[ -n "$opencv_dir" ]]; then
    export OpenCV_DIR="$opencv_dir"
fi
PLATE_SKIP_APT=1 bash "$project_dir/build_raspberry_pi.sh"

getent group gpio >/dev/null || groupadd --system gpio
getent group video >/dev/null || groupadd --system video
getent group dialout >/dev/null || groupadd --system dialout
getent group "$service_user" >/dev/null || groupadd --system "$service_user"
if ! id "$service_user" >/dev/null 2>&1; then
    useradd --system --gid "$service_user" --no-create-home \
        --shell /usr/sbin/nologin "$service_user"
fi
usermod -g "$service_user" -a -G gpio,video,dialout "$service_user"
install -d -o "$service_user" -g "$service_user" -m 750 "$data_dir/Output/Plate-Crops"
install -d -o root -g "$service_user" -m 750 "$config_dir"

cat > /etc/udev/rules.d/60-plate-controller.rules <<'EOF'
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"
SUBSYSTEM=="video4linux", KERNEL=="video[0-9]*", GROUP="video", MODE="0660"
EOF
udevadm control --reload-rules
udevadm trigger --subsystem-match=gpio || true
udevadm trigger --subsystem-match=video4linux || true

install -o root -g root -m 755 "$project_dir/controller" /usr/local/bin/controller

cat > /etc/systemd/system/plate-controller.service <<EOF
[Unit]
Description=License Plate Recognition and Gate Controller
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=$service_user
Group=$service_user
SupplementaryGroups=gpio video dialout
WorkingDirectory=$project_dir
Environment=PLATE_READER_CONFIG=$config_path
Environment=PLATE_OUTPUT_DIR=$data_dir/Output
ExecStart=/usr/bin/stdbuf -oL -eL $project_dir/start_reader.sh
Restart=on-failure
RestartSec=5
TimeoutStopSec=15

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload

# Remove the obsolete token from configurations created by older releases.
if [[ -f "$config_path" ]]; then
    sed -i '/^PLATE_API_KEY=/d' "$config_path"
    sed -i '/^GATE_RFID_OUTPUT_GPIO=/d; /^GATE_RFID_PULSE_MS=/d' "$config_path"
    if ! grep -q '^RFID_PROTOCOL=' "$config_path"; then
        printf '%s\n' 'RFID_PROTOCOL=uhfreader18' >>"$config_path"
    fi
    if grep -q '^RFID_READ_TIMEOUT_MS=2000$' "$config_path"; then
        sed -i 's/^RFID_READ_TIMEOUT_MS=2000$/RFID_READ_TIMEOUT_MS=5000/' "$config_path"
    fi
fi

if ((refresh == 1)); then
    if [[ -f "$config_path" ]]; then
        chown root:"$service_user" "$config_path"
        chmod 640 "$config_path"
        systemctl enable --now plate-controller.service
        if ! wait_for_service; then
            echo "The update installed, but the controller service failed to start." >&2
            exit 1
        fi
    fi
    exit 0
fi

if [[ "$server_ready" == "yes" ]]; then
    echo
    PLATE_READER_CONFIG="$config_path" "$project_dir/configure_reader.sh"
    chown root:"$service_user" "$config_path"
    chmod 640 "$config_path"
    systemctl enable --now plate-controller.service
    if ! wait_for_service; then
        echo "Setup finished, but the controller did not stay running. See the message above." >&2
        exit 1
    fi
    echo
    echo "Installation complete. The plate controller is running."
    echo "View live logs with: controller -logs"
else
    systemctl disable --now plate-controller.service 2>/dev/null || true
    echo
    echo "The Raspberry Pi controller is fully installed and safely stopped."
    echo "After installing the PC server, run: controller -configure"
fi

echo "Future updates require only: controller -update"
