#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/gui_ws"
VENV_DIR="$SCRIPT_DIR/venv"

echo


check_command() {
    command -v "$1" >/dev/null 2>&1
}

ensure_python_tools() {
    if ! command -v python3 >/dev/null 2>&1; then
        sudo apt update && sudo apt install -y python3
    fi
    if ! python3 -m venv --help >/dev/null 2>&1; then
        sudo apt update && sudo apt install -y python3-venv
    fi
    if ! command -v pip3 >/dev/null 2>&1; then
        sudo apt update && sudo apt install -y python3-pip
    fi
}

install_ros2() {
    echo "Installing ROS2 Humble... (requires sudo)"
    sudo apt update
    sudo apt install -y software-properties-common curl gnupg
    sudo add-apt-repository universe -y || true
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
          http://packages.ros.org/ros2/ubuntu \
          $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-humble-ros-base ros-dev-tools
}

main() {
    if ! check_command ros2; then
        echo "ROS2 not found — installing..."
        install_ros2
    fi

    # Source ROS2
    set +u
    source /opt/ros/humble/setup.bash
    set -u

    ensure_python_tools

    # Clone Nereo_interfaces if not present
    if [[ ! -d "$SCRIPT_DIR/nereo_interfaces" ]]; then
        echo "Cloning nereo_interfaces..."
        git clone https://github.com/PoliTOcean/nereo_interfaces.git "$SCRIPT_DIR/nereo_interfaces"
    fi

    # Build interfaces con Python di sistema (ha i pacchetti ROS2 corretti)
    echo "Building nereo_interfaces with system Python..."
    cd "$SCRIPT_DIR/nereo_interfaces"
    colcon build --symlink-install

    # Build workspace con Python di sistema
    echo "Building gui_ws with system Python..."
    cd "$WS_DIR"
    colcon build --symlink-install

    # DOPO i build, crea il venv per runtime
    if [[ ! -d "$VENV_DIR" ]]; then
        echo "Creating virtual environment for runtime..."
        python3 -m venv "$VENV_DIR" --system-site-packages
        touch "$VENV_DIR/COLCON_IGNORE"
    fi

    # Attiva venv e installa solo PyQt6 (le altre dipendenze vengono da system-site-packages)
    source "$VENV_DIR/bin/activate"
    echo "Installing PyQt6 in venv..."
    pip install --upgrade pip
    pip install PyQt6
    
    echo "✓ Setup complete, starting tmux session..."

    ### --------- TMUX PART --------- ###
    # Check tmux
    if ! command -v tmux >/dev/null 2>&1; then
        echo "tmux not installed — installing..."
        sudo apt update && sudo apt install -y tmux
    fi

    # Nome sessione
    SESSION="rosgui"

    # Se esiste, kill e ricrea (gestisci l'exit code senza far fallire lo script)
    if tmux has-session -t $SESSION 2>/dev/null; then
        echo "Killing existing tmux session '$SESSION'..."
        tmux kill-session -t $SESSION
    fi

    echo "DEBUG: Creating tmux session '$SESSION'..."
    echo "DEBUG: VENV_DIR=$VENV_DIR"
    echo "DEBUG: WS_DIR=$WS_DIR"
    echo "DEBUG: SCRIPT_DIR=$SCRIPT_DIR"
    
    # Disabilita set -e temporaneamente per i comandi tmux
    set +e
    
    # Crea la sessione con il primo pannello (usa i valori espansi direttamente)
    tmux -f /dev/null new-session -d -s $SESSION -n gui bash -c "
        source '$VENV_DIR/bin/activate' && \
        source /opt/ros/humble/setup.bash && \
        source '$SCRIPT_DIR/nereo_interfaces/install/setup.bash' && \
        source '$WS_DIR/install/setup.bash' && \
        ros2 run gui_pkg gui_node; \
        exec bash
    "
    
    TMUX_EXIT_CODE=$?
    if [ $TMUX_EXIT_CODE -ne 0 ]; then
        echo "ERROR: Failed to create tmux session (exit code: $TMUX_EXIT_CODE)"
        exit 1
    fi

    # Split verticale con il secondo nodo
    tmux split-window -h -t $SESSION:0 bash -c "
        source '$VENV_DIR/bin/activate' && \
        source /opt/ros/humble/setup.bash && \
        source '$SCRIPT_DIR/nereo_interfaces/install/setup.bash' && \
        source '$WS_DIR/install/setup.bash' && \
        ros2 run joystick_pkg joy_to_cmdvel; \
        exec bash
    "
    
    # Riabilita set -e
    set -e

    # Attacca la sessione
    echo "Starting ROS2 GUI in tmux session '$SESSION'..."
    tmux attach -t $SESSION
}

main "$@"