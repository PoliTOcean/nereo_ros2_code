#!/bin/bash
# Usage: stream_cam.sh <device> <port>
set -e
DEVICE="$1"
PORT="$2"
HOST="10.0.0.69"

if [ ! -e "$DEVICE" ]; then
    echo "Device $DEVICE non presente, esco (systemd ritenterà)"
    exit 1
fi

timeout 3 v4l2-ctl -d "$DEVICE" --info >/dev/null 2>&1 || {
    echo "Device $DEVICE non risponde entro 3s, esco"
    exit 1
}

exec gst-launch-1.0 -e \
    v4l2src device="$DEVICE" ! \
    'video/x-h264,width=1280,height=720,framerate=30/1' ! \
    h264parse config-interval=1 ! \
    rtph264pay pt=96 config-interval=1 ! \
    udpsink host="$HOST" port="$PORT" sync=false async=false
