#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"          # percorso salvato rispetto a dove si trova questo script

LOG_FILE="$SCRIPT_DIR/log_autostart.txt"
GUI_SCRIPT="$SCRIPT_DIR/gui_start.sh"
CONNECT_SCRIPT="$SCRIPT_DIR/connect_and_run.sh"
TARGET_IP="10.0.0.3"

# Appende sul file di log
if [ ! -f "$LOG_FILE" ]; then
    touch "$LOG_FILE"    
else
    printf "\n\n" >> "$LOG_FILE"  
fi

# ---------- Funzione per loggare sia su console sia su file ----------
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"           # scrive sia su console sia su file
}

log "New autostart attempt started."


# ---------- FALLBACK DI EMERGENZA: se c'è un altro autostart in esecuzione, lo killa ----------
# è un po' brutale, ma in caso di problemi con lock file o processi appesi, assicura che alla fine non ci siano più autostart attivi
stop_previous_autostart() {
    log "Stopping previous autostart terminals/processes..."

    pkill -f "gui_start.sh" || true
    pkill -f "connect_and_run.sh" || true

    # Chiusura processi GUI/ROS rimasti attivi
    pkill -f "main.py" || true
    pkill -f "gui_ws" || true
    pkill -f "ros_gui" || true
    pkill -f "rqt" || true

    # eventuali nodi ROS avviati
    pkill -f "ros2 launch" || true
    pkill -f "ros2 run" || true
    #piu specifico



    # breve pausa per assicurarsi che i processi siano terminati
    sleep 1 

    # breve pausa per assicurarsi che i processi siano terminati
    if lsof "$LOCK_FILE" >/dev/null 2>&1; then
        log "Lock still held. Killing processes using lock file..."
        fuser -k "$LOCK_FILE" || true
        sleep 1
    fi

    log "Previous autostart stopped."
}

cat <<'USAGE'
 _   _                           _             _            
| \ | | ___ _ __ ___  ___    ___| |_ __ _ _ __| |_ ___ _ __ 
|  \| |/ _ \ '__/ _ \/ _ \  / __| __/ _` | '__| __/ _ \ '__|
| |\  |  __/ | |  __/ (_) | \__ \ || (_| | |  | ||  __/ |   
|_| \_|\___|_|  \___|\___/  |___/\__\__,_|_|   \__\___|_|   

USAGE

log "Starting autostart script."

# ---------- LOCK: evita avvi multipli contemporanei di autostart.sh ----------
# il senso è che se devo runnare di nuovo autostart, prima chiudo quello vecchio (con stop_previous_autostart che è un po' brutale) e poi esco, così da lasciare la possibilità di runnare un nuovo autostart senza dover killare manualmente processi o terminali appesi

LOCK_FILE="/tmp/nereo_autostart.lock"           # file usato come flag per capire se lo script è già in esecuzione
exec 9>"$LOCK_FILE"         # apro il file di lock sul descrittore 9 e lo tengo aperto finché lo script gira

# provo a prendere il lock: se ci riesco, lo script continua mentre se è già in uso, prima chiudo tutto e poi esco
if ! flock -n 9; then
    log "ERROR: another autostart.sh is already running. Exiting."
    stop_previous_autostart
    log "Old autostart has been stopped. Please run autostart.sh again."
    exit 1
fi

log "Lock acquired."

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
        chmod +x "$f"

        if [ $? -ne 0 ]; then
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
        xterm -T "$title" -hold -e "bash -lc '$cmd; exec bash'" &
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

# ---------- Comando GUI ----------
# crea un comando che verrà eseguito dopo (in open_terminal), e fa scrivere l'output sul file di log
GUI_CMD="\"$GUI_SCRIPT\" >> \"$LOG_FILE\" 2>&1" 

# ---------- Comando connessione ----------
# anche qui viene preparata una stringa di comando, che pinga la rasp finchè non è raggiungibile, poi esegue lo script di connessione
CONNECT_CMD="echo Waiting for $TARGET_IP; until ping -c1 -W1 $TARGET_IP >/dev/null 2>&1; do sleep 1; done; echo $TARGET_IP reachable; exec \"$CONNECT_SCRIPT\""

# ---------- Avvio in terminali separati ----------
log "Opening GUI terminal."
open_terminal "ros_gui" "$GUI_CMD"

log "Opening connect terminal."
open_terminal "connect_and_run" "$CONNECT_CMD"

log "Done. Two terminals should be open: GUI and Connect."