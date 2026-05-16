#!/bin/bash

# Array per salvare i PID dei processi GStreamer
PIDS=()

# Funzione di pulizia per uccidere i flussi quando si interrompe lo script
cleanup() {
    echo -e "\n\e[31m[!] Arresto di tutte le telecamere simulate...\e[0m"
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid"
        fi
    done
    exit 0
}

# Intercetta il segnale CTRL+C (SIGINT) e chiama la funzione cleanup
trap cleanup SIGINT

echo -e "\e[32m[+] Avvio simulazione multi-camera per il ROV Nereo...\e[0m"

# 1. MAIN CAMERA - Porta 5001 (Barre SMPTE)
gst-launch-1.0 videotestsrc pattern=smpte ! \
    video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! \
    x264enc tune=zerolatency bitrate=500 byte-stream=true ! \
    rtph264pay config-interval=1 ! \
    udpsink host=127.0.0.1 port=5001 > /dev/null 2>&1 &
PIDS+=($!)
echo "[->] Main Camera attiva sulla porta 5001 (Pattern: Barre SMPTE)"

# 2. CAMERA 1 - Porta 5002 (Palla che rimbalza)
gst-launch-1.0 videotestsrc pattern=ball ! \
    video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! \
    x264enc tune=zerolatency bitrate=500 byte-stream=true ! \
    rtph264pay config-interval=1 ! \
    udpsink host=127.0.0.1 port=5002 > /dev/null 2>&1 &
PIDS+=($!)
echo "[->] Camera 1 attiva sulla porta 5002 (Pattern: Palla)"

# 3. CAMERA 2 - Porta 5003 (Effetto neve)
gst-launch-1.0 videotestsrc pattern=snow ! \
    video/x-raw,width=640,height=480,framerate=15/1 ! videoconvert ! \
    x264enc tune=zerolatency bitrate=500 byte-stream=true ! \
    rtph264pay config-interval=1 ! \
    udpsink host=127.0.0.1 port=5003 > /dev/null 2>&1 &
PIDS+=($!)
echo "[->] Camera 2 attiva sulla porta 5003 (Pattern: Neve)"

echo -e "\e[33m[*] Simulatore in esecuzione. Premi [CTRL+C] per spegnere le cam.\e[0m"

while true; do
    sleep 1
done