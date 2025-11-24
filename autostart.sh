#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONNECT_SCRIPT="$SCRIPT_DIR/connect_and_run.sh"
GUI_SCRIPT="$SCRIPT_DIR/gui_start.sh"
TARGET_IP="10.0.0.3"

if [[ ! -f "$CONNECT_SCRIPT" ]]; then
    echo "Error: $CONNECT_SCRIPT not found." >&2
    exit 2
fi
if [[ ! -f "$GUI_SCRIPT" ]]; then
    echo "Error: $GUI_SCRIPT not found." >&2
    exit 2
fi

ensure_executable(){
    local f="$1"
    if [[ ! -x "$f" ]]; then
        chmod +x "$f" || true
    fi
}

ensure_executable "$CONNECT_SCRIPT"
ensure_executable "$GUI_SCRIPT"

open_terminal(){
    # open_terminal "Title" "command string"
    local title="$1"; shift
    local cmd="$*"

    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="$title" -- bash -lc "$cmd; exec bash" &
        return 0
    fi
    if command -v konsole >/dev/null 2>&1; then
        konsole --new-tab -p tabtitle="$title" -e bash -lc "$cmd; exec bash" &
        return 0
    fi
    if command -v xfce4-terminal >/dev/null 2>&1; then
        xfce4-terminal --title "$title" --hold -e "bash -lc '$cmd; exec bash'" &
        return 0
    fi
    if command -v xterm >/dev/null 2>&1; then
        xterm -T "$title" -hold -e "bash -lc '$cmd; exec bash'" &
        return 0
    fi
    if command -v alacritty >/dev/null 2>&1; then
        alacritty -t "$title" -e bash -lc "$cmd; exec bash" &
        return 0
    fi
    if command -v kitty >/dev/null 2>&1; then
        kitty --title "$title" bash -lc "$cmd; exec bash" &
        return 0
    fi

    # Fallback: use tmux inside current terminal
    echo "No supported terminal emulator found; falling back to tmux session 'autostart'"
    tmux new-session -d -s autostart -n "$title" "bash -lc '$cmd; read -n1 -r -p \"Press any key to close...\"'"
    tmux attach -t autostart
}

echo "Opening GUI terminal (runs gui_start.sh)..."
open_terminal "ros_gui" "$GUI_SCRIPT"

echo "Opening connect terminal (will wait for $TARGET_IP to respond before running connect)..."
CONNECT_CMD="bash -lc 'echo Waiting for $TARGET_IP; until ping -c1 -W1 $TARGET_IP >/dev/null 2>&1; do sleep 1; done; echo $TARGET_IP reachable; exec \"$CONNECT_SCRIPT\"'"
open_terminal "connect_and_run" "$CONNECT_CMD"

echo "Done. Two terminals should be open: GUI and Connect (connect waits for ping)."

cat <<'USAGE'
 _   _                           _             _            
| \ | | ___ _ __ ___  ___    ___| |_ __ _ _ __| |_ ___ _ __ 
|  \| |/ _ \ '__/ _ \/ _ \  / __| __/ _` | '__| __/ _ \ '__|
| |\  |  __/ | |  __/ (_) | \__ \ || (_| | |  | ||  __/ |   
|_| \_|\___|_|  \___|\___/  |___/\__\__,_|_|   \__\___|_|   

USAGE
