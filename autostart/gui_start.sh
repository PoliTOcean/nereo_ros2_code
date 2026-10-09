#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/../gui_ws"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash}"
CONTROLLER_WS_DIR="${CONTROLLER_WS_DIR:-$SCRIPT_DIR/../../ros2_controller_tuning_aid}"
export CONTROLLER_OVERLAY="$CONTROLLER_WS_DIR/install"

# Discovery via IP esplicito: il multicast in ingresso non passa su WSL.
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS="10.0.0.3"

fail() {
    echo "ERRORE: $*" >&2
    exit 1
}

source_setup() {
    [[ -r "$1" ]] || fail "File di setup mancante: $1. Prepara il workspace manualmente."
    # I setup generati da ROS/colcon possono leggere variabili non definite.
    set +u
    source "$1"
    set -u
}

check_dependencies() {
    local cmd pkg
    for cmd in /usr/bin/python3 colcon tmux; do
        command -v "$cmd" >/dev/null 2>&1 || fail "Comando mancante: $cmd. Installalo manualmente."
    done
    source_setup "$ROS_SETUP"
    source_setup "$CONTROLLER_OVERLAY/local_setup.bash"
    command -v ros2 >/dev/null 2>&1 || fail "ros2 non disponibile dopo il sourcing di $ROS_SETUP."
    for pkg in nereo_controller_node joy rosbridge_server; do
        ros2 pkg prefix "$pkg" >/dev/null 2>&1 || fail "Package ROS mancante: $pkg. Installalo o compilalo manualmente."
    done
    # Gli entry point colcon della GUI usano Python di sistema.
    /usr/bin/python3 - <<'PY'
import importlib
import sys
modules = ('rclpy', 'PyQt6.QtCore', 'PyQt6.QtGui', 'PyQt6.QtQml',
           'PyQt6.QtQuick', 'numpy', 'matplotlib', 'qrcode', 'gi',
           'tf_transformations')
missing = []
for module in modules:
    try:
        importlib.import_module(module)
    except (ImportError, OSError) as exc:
        missing.append(f'{module}: {exc}')
try:
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
except (ImportError, ValueError, OSError) as exc:
    missing.append(f'GStreamer: {exc}')
if missing:
    print('ERRORE: dipendenze Python mancanti o non utilizzabili; preparale manualmente:\n'
          + '\n'.join(missing), file=sys.stderr)
    sys.exit(1)
PY
    [[ -d "$WS_DIR/src" ]] || fail "Workspace GUI mancante: $WS_DIR/src."
}

main() {
    check_dependencies
    if [[ "${1:-}" == "--check" ]]; then
        echo "Controlli dipendenze completati. Nessuna installazione eseguita."
        return
    fi

    # Non ereditare un eventuale virtualenv durante la build.
    if declare -F deactivate >/dev/null; then
        deactivate
    elif [[ -n "${VIRTUAL_ENV:-}" ]]; then
        PATH="${PATH#"$VIRTUAL_ENV/bin:"}"
        unset VIRTUAL_ENV
        export PATH
    fi
    echo "Building gui_ws with system Python..."
    cd "$WS_DIR"
    colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
    source_setup "$WS_DIR/install/local_setup.bash"
    local pkg
    for pkg in gui_pkg joystick_pkg web_pkg nereo_interfaces; do
        ros2 pkg prefix "$pkg" >/dev/null 2>&1 || fail "Package GUI mancante dopo la build: $pkg."
    done

    local session="rosgui" pane_cmd
    if tmux has-session -t "$session" 2>/dev/null; then
        tmux kill-session -t "$session"
    fi
    # Passa i percorsi con quoting shell e ricarica gli overlay nel pannello.
    # Imposta la rete anche nel pannello: un server tmux già attivo può avere
    # un ambiente diverso da quello dello script.
    printf -v pane_cmd 'bash -c %q' "$(printf 'export CONTROLLER_OVERLAY=%q; source %q && source %q && source %q && export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=10.0.0.3 && ros2 launch gui_pkg workstation.launch.py; exec bash' \
        "$CONTROLLER_OVERLAY" "$ROS_SETUP" "$CONTROLLER_OVERLAY/local_setup.bash" "$WS_DIR/install/local_setup.bash")"
    tmux -f /dev/null new-session -d -s "$session" -n gui "$pane_cmd"
    echo "Starting ROS2 GUI in tmux session '$session'..."
    tmux attach -t "$session"
}

main "$@"
