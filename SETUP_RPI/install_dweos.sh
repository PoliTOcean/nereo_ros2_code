#!/bin/bash
# Usage: install_dweos.sh <all|configure|identify|autostart>
set -e

DWEOS_VERSION="v0.7.3"  # resolved commit c7aef48a; v0.7.4 has no release asset
STREAM_HOST="10.0.0.69"
DWEOS_API_HOST="127.0.0.1"
DWEOS_UI_PORT="80"
MAIN_CAM_PORT="5001"
CAM_1_PORT="5002"
CAM_2_PORT="5003"
MAIN_CAM_BUS_INFO="usb-xhci-hcd.1-1"
CAM_1_BUS_INFO="usb-xhci-hcd.0-2"
CAM_2_BUS_INFO="usb-xhci-hcd.1-2"
STREAM_WIDTH="1280"
STREAM_HEIGHT="720"
STREAM_FPS="30"
STREAM_ENCODE_TYPE="H264"
SNAPSHOT_DIR="$(dirname "$0")/dweos_evidence"
MODE="${1:-}"

# snapshot_system_state: writes pre-install OS/ROS/systemd facts to
# SNAPSHOT_DIR; each write failure aborts before anything is installed.
snapshot_system_state() {
    mkdir -p "$SNAPSHOT_DIR" || {
        echo "Cannot create $SNAPSHOT_DIR, exiting"
        exit 1
    }

    ( . /etc/os-release && echo "$VERSION_ID $UBUNTU_CODENAME" ) \
        > "$SNAPSHOT_DIR/os-release-before.txt" || {
        echo "os-release snapshot failed, exiting"
        exit 1
    }

    ls /opt/ros/ > "$SNAPSHOT_DIR/ros-distros-before.txt" || {
        echo "ROS distro snapshot failed, exiting"
        exit 1
    }

    cat /etc/apt/sources.list.d/ros2.list \
        > "$SNAPSHOT_DIR/ros2-apt-source-before.txt" || {
        echo "ROS 2 apt source snapshot failed, exiting"
        exit 1
    }

    dpkg -l 'ros-*' > "$SNAPSHOT_DIR/dpkg-ros-before.txt" || {
        echo "dpkg ros-* snapshot failed, exiting"
        exit 1
    }

    systemd-analyze blame \
        > "$SNAPSHOT_DIR/systemd-analyze-blame-before.txt" || {
        echo "systemd-analyze blame snapshot failed, exiting"
        exit 1
    }

    systemd-analyze critical-chain \
        > "$SNAPSHOT_DIR/systemd-analyze-critical-chain-before.txt" || {
        echo "systemd-analyze critical-chain snapshot failed, exiting"
        exit 1
    }

    {
        dpkg -l network-manager 2>&1 || true
        systemctl is-enabled NetworkManager.service 2>&1 || true
    } > "$SNAPSHOT_DIR/networkmanager-before.txt"
}

# assert_network_available: fails fast if github.com is unreachable; the
# DWE OS install is a bench/lab-only step, never expected on the tether.
assert_network_available() {
    if ! curl -sSf --max-time 5 -o /dev/null https://github.com; then
        echo "github.com unreachable: this is a bench-only install step;"
        echo "the isolated competition tether has no internet by design."
        exit 1
    fi
}

# install_dweos: pipes the pinned vendor install.sh into a root shell; the
# version argument is mandatory, no default is used from this call site.
install_dweos() {
    local url="https://raw.githubusercontent.com/DeepwaterExploration"
    url="$url/DWE_OS_2/main/install.sh"
    curl -sSL "$url" | sudo bash -s "$DWEOS_VERSION"
}

# apply_service_override: patches two confirmed v0.7.3 defects in the
# installed release (see below), writes a dwe_os_2.service drop-in
# disabling the bundled web terminal and the network-management
# feature, then reloads and restarts the unit.
#
# Defect 1: server.py's serve() calls network_wrapper.initialize()
# UNCONDITIONALLY - unlike the adjacent ttyd feature, it is never gated on
# self.feature_support.wifi, so passing --no-wifi does not stop it from
# running. Absent NetworkManager (this Pi has none), that call crashes the
# whole server at startup (sdbus.dbus_exceptions.DbusServiceUnknownError on
# org.freedesktop.NetworkManager) instead of degrading gracefully as
# research's Assumption A2 expected. Rather than installing NetworkManager
# (a new always-on service, and one that would leave DWE OS's unauthenticated
# /api/network/* routes fully live, since those are not gated by the flag
# either - a materially larger surface than intended), this
# adds the missing guard directly, mirroring the existing
# "if self.feature_support.ttyd:" pattern already in the same function.
#
# Defect 2: models/cameras.py's DeviceModel.string3 is typed "str = ''",
# but at least one connected exploreHD leaves the underlying attribute as
# None (an unpopulated UVC string descriptor), which pydantic's
# from_attributes validation rejects outright - GET /api/devices and
# /api/devices/map both 500 on every request.
#
# Widening the annotation to "str | None" fixes the 500 but is NOT enough:
# the API then emits "string3": null, and DWE OS's own web UI reads
# .length on that value unguarded, so its Cameras page dies with
# "can't access property length, n is null" and lists no cameras at all.
# The fix therefore coerces None to the field's own default with a
# before-validator, so the attribute stays a str all the way to the
# client and both the API and the vendor UI work.
#
# Both patches must be re-applied after every install_dweos call, since
# install.sh wipes and recreates /opt/DWE_OS_2 from the release tarball
# each time.
apply_service_override() {
    local override_dir="/etc/systemd/system/dwe_os_2.service.d"
    local wrapper="/opt/DWE_OS_2/run_release.sh"
    local server_py="/opt/DWE_OS_2/backend_py/src/server.py"
    local models_py="/opt/DWE_OS_2/backend_py/src/models/cameras.py"
    local exec_line

    if ! sudo grep -q 'feature_support.wifi' "$server_py"; then
        sudo python3 - "$server_py" <<'PATCH'
import sys
path = sys.argv[1]
old = "        await self.network_wrapper.initialize()\n"
new = (
    "        if self.feature_support.wifi:\n"
    "            await self.network_wrapper.initialize()\n"
)
with open(path) as f:
    content = f.read()
if old not in content:
    print("apply_service_override: expected line not found, exiting")
    sys.exit(1)
with open(path, "w") as f:
    f.write(content.replace(old, new, 1))
PATCH
    fi

    if ! sudo grep -q '_string3_never_none' "$models_py"; then
        sudo python3 - "$models_py" <<'PATCH'
import sys
path = sys.argv[1]
old_import = "from pydantic import BaseModel, Field\n"
new_import = "from pydantic import BaseModel, Field, field_validator\n"
old = '    string3: str = ""\n'
new = (
    '    string3: str = ""\n'
    "\n"
    "    # Some exploreHD units leave this UVC string descriptor\n"
    "    # unpopulated, so the underlying attribute is None. Emitting null\n"
    "    # breaks the DWE OS web UI, which reads .length on it unguarded;\n"
    "    # coerce to the field default instead.\n"
    '    @field_validator("string3", mode="before")\n'
    "    @classmethod\n"
    "    def _string3_never_none(cls, v):\n"
    '        return v if v is not None else ""\n'
)
with open(path) as f:
    content = f.read()
if old not in content or old_import not in content:
    print("apply_service_override: string3 anchors not found, exiting")
    sys.exit(1)
content = content.replace(old_import, new_import, 1)
with open(path, "w") as f:
    f.write(content.replace(old, new, 1))
PATCH
    fi

    if sudo grep -qF '"$@"' "$wrapper"; then
        exec_line="$wrapper --no-ttyd --no-wifi"
    else
        exec_line="/opt/DWE_OS_2/.venv/bin/python3"
        exec_line="$exec_line /opt/DWE_OS_2/run_release.py"
        exec_line="$exec_line --no-ttyd --no-wifi"
    fi

    sudo mkdir -p "$override_dir" || {
        echo "Cannot create $override_dir, exiting"
        exit 1
    }

    printf '[Service]\nExecStart=\nExecStart=%s\n' "$exec_line" \
        | sudo tee "$override_dir/override.conf" > /dev/null

    sudo systemctl daemon-reload
    sudo systemctl restart dwe_os_2
}

# snapshot_post_install: writes the -after.txt counterparts of the dpkg and
# systemd-analyze captures to SNAPSHOT_DIR, for comparison.
snapshot_post_install() {
    dpkg -l 'ros-*' > "$SNAPSHOT_DIR/dpkg-ros-after.txt" || {
        echo "dpkg ros-* post-install snapshot failed, exiting"
        exit 1
    }

    systemd-analyze blame \
        > "$SNAPSHOT_DIR/systemd-analyze-blame-after.txt" || {
        echo "systemd-analyze blame post-install snapshot failed, exiting"
        exit 1
    }

    systemd-analyze critical-chain \
        > "$SNAPSHOT_DIR/systemd-analyze-critical-chain-after.txt" || {
        echo "systemd-analyze critical-chain post-install snapshot failed"
        exit 1
    }
}

# list_devices: GETs the DWE OS device list and prints each camera's
# bus_info and device_paths so the bench operator can build the port map.
list_devices() {
    curl -sS "http://$DWEOS_API_HOST:$DWEOS_UI_PORT/api/devices" \
        | python3 -c '
import json, sys
devices = json.load(sys.stdin)
for d in devices:
    print(d.get("bus_info"), d.get("device_paths"))
'
}

# configure_stream: POSTs a UDP/H264 stream config for one camera (bus_info)
# to the given port; refuses if bus_info is empty.
configure_stream() {
    local bus_info="$1"
    local port="$2"
    local body

    if [ -z "$bus_info" ]; then
        echo "configure_stream: empty bus_info, exiting"
        exit 1
    fi

    body=$(cat <<JSON
{
  "bus_info": "$bus_info",
  "stream_type": "UDP",
  "stream_format": {
    "width": $STREAM_WIDTH,
    "height": $STREAM_HEIGHT,
    "interval": {"numerator": 1, "denominator": $STREAM_FPS}
  },
  "encode_type": "$STREAM_ENCODE_TYPE",
  "enabled": true,
  "endpoints": [{"host": "$STREAM_HOST", "port": $port}]
}
JSON
)

    curl -sS -X POST \
        "http://$DWEOS_API_HOST:$DWEOS_UI_PORT/api/devices/configure_stream" \
        -H "Content-Type: application/json" \
        -d "$body"
}

# wait_for_api: blocks until the DWE OS device endpoint answers with at
# least one camera, or gives up after roughly 60 seconds. Needed because the
# service accepts connections before it has finished enumerating devices, so
# a configure issued too early is answered for a device list that is still
# empty.
wait_for_api() {
    local i count
    for i in $(seq 1 60); do
        count=$(curl -sS --max-time 2 \
            "http://$DWEOS_API_HOST:$DWEOS_UI_PORT/api/devices" 2>/dev/null \
            | python3 -c 'import json,sys
try:
    print(len(json.load(sys.stdin)))
except Exception:
    print(0)' 2>/dev/null)
        if [ "${count:-0}" -gt 0 ]; then
            return 0
        fi
        sleep 1
    done
    echo "wait_for_api: no cameras after 60s, configuring anyway"
    return 0
}

# configure_all_streams: configures the UDP/H264 stream of all three cameras
# from the recorded bus_info map, after refusing a destination port set that
# is not three distinct ports or that collides with the DWE OS web UI.
configure_all_streams() {
    wait_for_api

    local ports="$MAIN_CAM_PORT $CAM_1_PORT $CAM_2_PORT"
    local p

    if [ "$(printf '%s\n' $ports | sort -u | wc -l)" -ne 3 ]; then
        echo "configure_all_streams: ports not distinct ($ports), exiting"
        exit 1
    fi

    for p in $ports; do
        if [ "$p" = "$DWEOS_UI_PORT" ]; then
            echo "configure_all_streams: port $p is the DWE OS web UI"
            echo "port, exiting"
            exit 1
        fi
    done

    configure_stream "$MAIN_CAM_BUS_INFO" "$MAIN_CAM_PORT"
    configure_stream "$CAM_1_BUS_INFO" "$CAM_1_PORT"
    configure_stream "$CAM_2_BUS_INFO" "$CAM_2_PORT"
}

# identify_ports: interactive bench aid for the physical port labelling. The
# operator unplugs one camera per role; the bus_info that disappears is that
# role's. Prints the three settings lines to paste back into this script and
# configures nothing.
identify_ports() {
    local role present remaining gone

    present=$(list_devices | awk '{print $1}' | sort)
    echo "Cameras currently seen by DWE OS:"
    printf '%s\n' "$present"

    for role in MAIN_CAM CAM_1 CAM_2; do
        printf 'Unplug the camera to label %s, then press Enter: ' "$role"
        read -r _
        sleep 2
        remaining=$(list_devices | awk '{print $1}' | sort)
        gone=$(comm -23 <(printf '%s\n' "$present") \
                        <(printf '%s\n' "$remaining"))
        if [ "$(printf '%s\n' "$gone" | grep -c .)" -ne 1 ]; then
            echo "identify_ports: expected exactly one camera to vanish,"
            echo "got: $gone"
            exit 1
        fi
        echo "${role}_BUS_INFO=\"$gone\""
        present="$remaining"
    done

    echo "Write the physical port labels on the Pi, paste the three lines"
    echo "above into this script, plug every camera back in, then run:"
    echo "  $0 configure"
}

# install_stream_autostart: writes a second dwe_os_2 drop-in that re-runs
# this script's configure mode after the service starts. DWE OS saves each
# stream with enabled true but does not restart the pipelines when the
# service comes back, so without this the cameras go silent after every
# reboot and every service restart, with the configuration still looking
# correct in the API.
install_stream_autostart() {
    local override_dir="/etc/systemd/system/dwe_os_2.service.d"
    local self
    self="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"

    sudo mkdir -p "$override_dir" || {
        echo "Cannot create $override_dir, exiting"
        exit 1
    }

    printf '[Service]\nExecStartPost=-%s configure\n' "$self" \
        | sudo tee "$override_dir/99-autostream.conf" > /dev/null

    sudo systemctl daemon-reload
    echo "Stream autostart installed. Streams now reconfigure themselves"
    echo "after every start of dwe_os_2."
}

case "$MODE" in
all)
    snapshot_system_state
    assert_network_available
    install_dweos
    apply_service_override
    install_stream_autostart
    snapshot_post_install
    list_devices
    configure_all_streams
    ;;
configure)
    list_devices
    configure_all_streams
    ;;
identify)
    identify_ports
    ;;
autostart)
    install_stream_autostart
    ;;
*)
    echo "Usage: $0 <all|configure|identify|autostart>"
    exit 1
    ;;
esac
