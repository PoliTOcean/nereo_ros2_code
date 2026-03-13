#!/bin/bash
# Installa tutti i service Nereo
# Eseguire con: sudo bash install_services.sh
# dalla cartella dove si trovano i file

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== Pulizia service esistenti ==="
for SERVICE in nereo-micro-ros nereo-sensors nereo-camera; do
    if systemctl is-active --quiet "$SERVICE" 2>/dev/null; then
        systemctl stop "$SERVICE"
        echo "✓ Fermato $SERVICE"
    fi
    if systemctl is-enabled --quiet "$SERVICE" 2>/dev/null; then
        systemctl disable "$SERVICE"
        echo "✓ Disabilitato $SERVICE"
    fi
    if [ -f "/etc/systemd/system/$SERVICE.service" ]; then
        rm "/etc/systemd/system/$SERVICE.service"
        echo "✓ Rimosso $SERVICE.service"
    fi
done

echo ""
echo "=== Permessi /dev/ttyAMA0 ==="
echo 'KERNEL=="ttyAMA0", GROUP="dialout", MODE="0666"' > /etc/udev/rules.d/99-ttyAMA0.rules
udevadm control --reload-rules
udevadm trigger
chmod 666 /dev/ttyAMA0
echo "✓ Permessi impostati su /dev/ttyAMA0"

echo ""
echo "=== Installazione script ==="
chmod +x "$SCRIPT_DIR/nereo_ros_env.sh"
chmod +x "$SCRIPT_DIR/nereo_sensors_start.sh"
echo "✓ Script già in $SCRIPT_DIR, permessi impostati"

echo ""
echo "=== Installazione service ==="
for SERVICE in nereo-micro-ros nereo-sensors nereo-camera; do
    cp "$SCRIPT_DIR/$SERVICE.service" /etc/systemd/system/
    echo "✓ $SERVICE.service → /etc/systemd/system/"
done

echo ""
echo "=== Abilitazione e avvio ==="
systemctl daemon-reload
for SERVICE in nereo-micro-ros nereo-sensors nereo-camera; do
    systemctl enable "$SERVICE"
    systemctl start "$SERVICE"
    echo "✓ $SERVICE abilitato e avviato"
done

echo ""
echo "=== Stato finale ==="
for SERVICE in nereo-micro-ros nereo-sensors nereo-camera; do
    echo ""
    echo "--- $SERVICE ---"
    systemctl status "$SERVICE" --no-pager -l | head -10
done

echo ""
echo "Installazione completata!"
echo ""
echo "Comandi utili:"
echo "  systemctl status nereo-*           # stato di tutti"
echo "  journalctl -u nereo-sensors -f     # log in tempo reale"
echo "  systemctl restart nereo-micro-ros  # riavvia uno"
echo "  systemctl stop nereo-*             # ferma tutto"