#!/usr/bin/env bash
set -euo pipefail

# ---------- 1. INSTALL EXPECT SE NON PRESENTE ----------
if ! command -v expect >/dev/null 2>&1; then
    echo "[*] Expect non trovato. Installazione in corso..."
    sudo apt update && sudo apt install -y expect
fi

HOST="10.0.0.3"
USER="pi"
PASS="raspberry"
SESSION="nereo_session"

expect <<'EOF'
set timeout 30
set host "10.0.0.3"
set user "pi"
set pass "raspberry"
set session "nereo_session"

# ----- SSH -----
spawn ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $user@$host

# Gestione primo accesso + password
expect {
    "yes/no" { 
        send "yes\r"
        exp_continue 
    }
    "password:" { 
        send "$pass\r" 
    }
}

# ----- ATTENDERE IL PROMPT -----
expect -re "pi@.*:.*\\$ "

# Breve pausa per assicurarsi che il sistema sia pronto
sleep 0.5

# ----- COMANDI TMUX -----
# Controlla se la sessione esiste e uccidila
send "tmux has-session -t $session 2>/dev/null && tmux kill-session -t $session\r"
expect -re "pi@.*:.*\\$ "

# Breve pausa dopo il kill
sleep 0.3

# Crea sessione con primo pannello - usa nome finestra micro_ros
# Avvia lo script dentro tmux (senza piping della password), poi simula la
# digitazione della password con `tmux send-keys` in modo che programmi che
# leggono la password da /dev/tty (es. sudo, prompt interattivi) la ricevano.
send "tmux -f /dev/null new-session -d -s $session -n micro_ros bash -c 'cd ~; ./micro_ros_connect.sh; exec bash'\r"
# lascia un breve intervallo perché tmux abbia avviato il processo e il prompt
sleep 0.8
# invia la password come tasti alla pane (simula l'input utente), seguito da Invio
send "tmux send-keys -t $session:micro_ros \"$pass\" C-m\r"
# opzionale: attendi un momento per permettere allo script remoto di proseguire
sleep 0.5
expect -re "pi@.*:.*\\$ "

# Split orizzontale nella stessa finestra - usa il nome della finestra
send "tmux split-window -h -t $session:micro_ros bash -c 'cd ~/nereo_ros2_code && ./start_imu_bar.sh; exec bash'\r"
expect -re "pi@.*:.*\\$ "

# Attach alla sessione - rimuovi l'expect dopo perché tmux prende il controllo
send "tmux attach -t $session\r"
expect -re "pi@.*:.*\\$ "

# Passa IMMEDIATAMENTE il controllo all'utente senza aspettare nulla
interact
EOF