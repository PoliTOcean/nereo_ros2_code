#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"          # percorso salvato rispetto a dove si trova questo script

LOG_FILE="$SCRIPT_DIR/log_autostart.txt"
GUI_SCRIPT="$SCRIPT_DIR/gui_start.sh"
CONNECT_SCRIPT="$SCRIPT_DIR/connect_and_run.sh"
INSTALL_SCRIPT="$SCRIPT_DIR/install_ros.sh"
TARGET_IP="10.0.0.3"

# Appende sul file di log e se non esiste lo crea
printf "\n\n" >> "$LOG_FILE"  

# ---------- Funzione per loggare su file di log ----------
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" >> "$LOG_FILE"           # scrive solo su file di log
}

log "New autostart attempt started."


# ---------- PULIZIA PRIMA DI OGNI NUOVO AVVIO ----------
# Chiude le sessioni tmux remote e i processi locali del ROV.
stop_previous_autostart() {
    log "Stopping previous ROV processes..."
    echo "Chiusura delle sessioni sul Raspberry..."

    # Richiede la password SSH se non hai configurato le chiavi.
    # Chiude tutte le sessioni tmux dell'utente pi.
    if ! ssh -o ConnectTimeout=10 "pi@$TARGET_IP" \
        'if command -v tmux >/dev/null 2>&1; then
             tmux kill-server 2>/dev/null || true
         else
             echo "ERROR: tmux non installato sul Raspberry." >&2
             exit 1
         fi'; then      # provo a connettermi per 10 secondi alla rasp e se non riesco stampo cose, se riesco killo il tmux   

        log "ERROR: cannot complete Raspberry cleanup."
        echo "Pulizia del Raspberry fallita. Autostart interrotto."
        exit 1
    fi

    log "Raspberry tmux sessions stopped."
    echo "Chiusura dei processi ROS sul PC..."

    pkill -f "gui_start.sh" || true
    pkill -f "connect_and_run.sh" || true

    pkill -f "main.py" || true
    pkill -f "gui_ws" || true
    pkill -f "ros_gui" || true
    pkill -f "rqt" || true

    pkill -f "ros2 launch" || true
    pkill -f "ros2 run" || true

    # Include processi che possono restare attivi senza il launcher.
    pkill -f "web_server_node" || true
    pkill -f "rosbridge_websocket" || true
    pkill -f "nereo_controller_node" || true
    pkill -f "joy_node" || true

    pkill -f "gui_node" || true
    pkill -f "joy_to_cmdvel" || true
    pkill -f "safety_node" || true
    
    sleep 1

    log "Previous ROV processes stopped."
}

cat <<'USAGE'
 _   _                           _             _            
| \ | | ___ _ __ ___  ___    ___| |_ __ _ _ __| |_ ___ _ __ 
|  \| |/ _ \ '__/ _ \/ _ \  / __| __/ _` | '__| __/ _ \ '__|
| |\  |  __/ | |  __/ (_) | \__ \ || (_| | |  | ||  __/ |   
|_| \_|\___|_|  \___|\___/  |___/\__\__,_|_|   \__\___|_|   

USAGE

log "Starting autostart script."


# ---------- LOCK: una sola procedura di riavvio alla volta ----------
LOCK_FILE="/tmp/nereo_autostart_restart.lock"
exec 9>"$LOCK_FILE"

log "Waiting for restart lock..."

if ! flock 9; then
    log "ERROR: cannot acquire restart lock."
    echo "Impossibile acquisire il lock. Autostart interrotto."
    exit 1
fi

log "Restart lock acquired."

# A ogni avvio ferma i processi precedenti, poi prosegue.
stop_previous_autostart


# ---------- CONTROLLO FILE LOCALE: esistenza e permessi di esecuzione ----------
# se il file non esiste, esco con errore
# se esiste ma non è eseguibile, lo rendo eseguibile
check_file() {
    local f="$1"
    local name
    name="$(basename "$f")"

    if [ ! -f "$f" ]; then
        log "ERROR: File '$name' NOT found at path: $f"
        exit 1
    fi

    log "File '$name': found."

    if [ ! -x "$f" ]; then
        log "WARNING: '$name' is not executable. Fixing permissions..."

        if ! chmod +x "$f"; then
            log "ERROR: cannot make '$name' executable."
            exit 1
        fi
    fi

    log "File '$name': executable."
}

# ---------- APERTURA TERMINALE ----------
# funzione per testare la presenza di vari emulatori di terminale
open_terminal() {
    # open_terminal "Title" "command string"
    local title="$1"
    shift
    local cmd="$*"

    log "Opening terminal '$title'."

    if command -v xterm >/dev/null 2>&1; then
        xterm -T "$title" -hold -e bash -lc "$cmd; exec bash" &
        return 0
    fi

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

    if command -v alacritty >/dev/null 2>&1; then
        alacritty -t "$title" -e bash -lc "$cmd; exec bash" &
        return 0
    fi

    if command -v kitty >/dev/null 2>&1; then
        kitty --title "$title" bash -lc "$cmd; exec bash" &
        return 0
    fi

    log "ERROR: no supported terminal emulator found."
    exit 1
}

# ---------- Controlli iniziali ----------
check_file "$GUI_SCRIPT"
check_file "$CONNECT_SCRIPT"
check_file "$INSTALL_SCRIPT"

# ---------- Comando GUI ----------
# crea un comando che verrà eseguito dopo (in open_terminal), e fa scrivere l'output sul file di log
GUI_CMD="\"$GUI_SCRIPT\" >> \"$LOG_FILE\" 2>&1" 

# ---------- Comando connessione ----------
# anche qui viene preparata una stringa di comando, che pinga la rasp finchè non è raggiungibile, poi esegue lo script di connessione
CONNECT_CMD="echo Waiting for $TARGET_IP; until ping -c1 -W1 $TARGET_IP >/dev/null 2>&1; do sleep 1; done; echo $TARGET_IP reachable; exec \"$CONNECT_SCRIPT\""

# ---------- Valutazione ROS installato o meno ----------
log "Checking ROS installation..."
"$INSTALL_SCRIPT"
log "ROS installation check completed."

# ---------- Avvio in terminali separati ----------
log "Opening GUI terminal."
open_terminal "ros_gui" "$GUI_CMD" 9>&-

log "Opening connect terminal."
open_terminal "connect_and_run" "$CONNECT_CMD" 9>&-

log "Done. Two terminals should be open: GUI and Connect."