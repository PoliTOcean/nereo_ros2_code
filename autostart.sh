#!/usr/bin/env bash
set -euo pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RPI_SCRIPT="$SCRIPT_DIR/scripts/rpi/start_rpi"
GUI_SCRIPT="$SCRIPT_DIR/scripts/controlstation/start_gui"

echo -e "${YELLOW}"
cat <<'USAGE'
 _   _                           _             _            
| \ | | ___ _ __ ___  ___    ___| |_ __ _ _ __| |_ ___ _ __ 
|  \| |/ _ \ '__/ _ \/ _ \  / __| __/ _` | '__| __/ _ \ '__|
| |\  |  __/ | |  __/ (_) | \__ \ || (_| | |  | ||  __/ |   
|_| \_|\___|_|  \___|\___/  |___/\__\__,_|_|   \__\___|_|   

USAGE
echo -e "${NC}"

# Check and install required dependencies
check_dependencies() {
    local deps_missing=0
    local packages_to_install=()

    echo -e "${BLUE}Checking dependencies...${NC}"

    # Check for tmux
    if ! command -v tmux >/dev/null 2>&1; then
        echo -e "  - tmux: ${RED}not found${NC}"
        packages_to_install+=("tmux")
        deps_missing=1
    else
        echo -e "  - tmux: ${GREEN}OK${NC}"
    fi

    # Check for sshpass
    if ! command -v sshpass >/dev/null 2>&1; then
        echo -e "  - sshpass: ${RED}not found${NC}"
        packages_to_install+=("sshpass")
        deps_missing=1
    else
        echo -e "  - sshpass: ${GREEN}OK${NC}"
    fi

    # Check for gnome-terminal or other terminal emulators
    local terminal_found=0
    for term in gnome-terminal konsole xfce4-terminal xterm alacritty kitty; do
        if command -v "$term" >/dev/null 2>&1; then
            echo -e "  - terminal emulator ($term): ${GREEN}OK${NC}"
            terminal_found=1
            break
        fi
    done
    
    if [[ $terminal_found -eq 0 ]]; then
        echo -e "  - terminal emulator: ${YELLOW}not found, will use tmux fallback${NC}"
    fi

    # Install missing packages
    if [[ $deps_missing -eq 1 ]]; then
        echo ""
        echo -e "${YELLOW}Installing missing dependencies: ${packages_to_install[*]}${NC}"
        sudo apt-get update -qq
        sudo apt-get install -y "${packages_to_install[@]}"
        echo -e "${GREEN}Dependencies installed successfully.${NC}"
    else
        echo -e "${GREEN}All dependencies are installed.${NC}"
    fi
    echo ""
}

check_dependencies
if [[ ! -f "$RPI_SCRIPT" ]]; then
    echo -e "${RED}Error: $RPI_SCRIPT not found.${NC}" >&2
    exit 2
fi
if [[ ! -f "$GUI_SCRIPT" ]]; then
    echo -e "${RED}Error: $GUI_SCRIPT not found.${NC}" >&2
    exit 2
fi

ensure_executable(){
    local f="$1"
    if [[ ! -x "$f" ]]; then
        chmod +x "$f" || true
    fi
}

ensure_executable "$RPI_SCRIPT"
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
    echo -e "${YELLOW}No supported terminal emulator found; falling back to tmux session 'autostart'${NC}"
    tmux new-session -d -s autostart -n "$title" "bash -lc '$cmd; read -n1 -r -p \"Press any key to close...\"'"
    tmux attach -t autostart
}

echo -e "${BLUE}Opening GUI terminal (runs start_gui)...${NC}"
open_terminal "ros_gui" "$GUI_SCRIPT"

echo -e "${BLUE}Opening RPI services terminal (runs start_rpi via SSH)...${NC}"
open_terminal "rpi_services" "$RPI_SCRIPT"

echo -e "${GREEN}Done. Two terminals should be open: GUI and RPI services.${NC}"
