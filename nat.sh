set -euo pipefail

# The tether interface is identified by the address it holds, not by name:
# interface names move between machines and across reinstalls.
TETHER_ADDR="${TETHER_ADDR:-10.0.0.69}"
TETHER_NET="${TETHER_NET:-10.0.0.0/24}"
UNIT=/etc/systemd/system/nereo-tether-nat.service
SYSCTL_FILE=/etc/sysctl.d/99-nereo-tether-forward.conf

die() { echo "error: $*" >&2; exit 1; }

find_tether_iface() {
	ip -brief -4 addr show \
		| awk -v a="$TETHER_ADDR/" '$3 ~ "^"a {print $1; exit}'
}

# The uplink is whatever currently carries the default route. Resolved at run
# time on purpose: it is wifi at the bench and may be ethernet elsewhere.
find_uplink_iface() {
	ip -4 route show default | awk '{for(i=1;i<=NF;i++) if($i=="dev"){print $(i+1); exit}}'
}

need_root() { [ "$(id -u)" -eq 0 ] || die "run this with sudo"; }

do_up() {
	need_root
	local tether uplink
	tether=$(find_tether_iface) || true
	[ -n "$tether" ] || die "no interface holds $TETHER_ADDR -- is the tether plugged in?"
	uplink=$(find_uplink_iface) || true
	[ -n "$uplink" ] || die "no default route on this machine: nothing to share"
	[ "$uplink" != "$tether" ] || die "the tether is also the default route; refusing"

	echo "tether: $tether ($TETHER_ADDR)   uplink: $uplink"

	sysctl -qw net.ipv4.ip_forward=1

	# -C tests for an existing identical rule, which is what makes repeated
	# runs harmless. Without it every invocation would append a duplicate.
	iptables -t nat -C POSTROUTING -s "$TETHER_NET" -o "$uplink" -j MASQUERADE 2>/dev/null \
		|| iptables -t nat -A POSTROUTING -s "$TETHER_NET" -o "$uplink" -j MASQUERADE
	iptables -C FORWARD -i "$tether" -o "$uplink" -j ACCEPT 2>/dev/null \
		|| iptables -A FORWARD -i "$tether" -o "$uplink" -j ACCEPT
	iptables -C FORWARD -i "$uplink" -o "$tether" -m state \
		--state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null \
		|| iptables -A FORWARD -i "$uplink" -o "$tether" -m state \
			--state RELATED,ESTABLISHED -j ACCEPT

	echo "NAT is up. On the Pi:"
	echo "    sudo ip route add default via $TETHER_ADDR dev eth0"
	echo "    sudo resolvectl dns eth0 8.8.8.8 1.1.1.1"
}

do_down() {
	need_root
	local tether uplink
	tether=$(find_tether_iface) || true
	uplink=$(find_uplink_iface) || true
	[ -n "$tether" ] && [ -n "$uplink" ] || die "cannot resolve interfaces; remove rules by hand"

	iptables -t nat -D POSTROUTING -s "$TETHER_NET" -o "$uplink" -j MASQUERADE 2>/dev/null || true
	iptables -D FORWARD -i "$tether" -o "$uplink" -j ACCEPT 2>/dev/null || true
	iptables -D FORWARD -i "$uplink" -o "$tether" -m state \
		--state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true

	# Forwarding is left enabled: Docker and other things may rely on it, and
	# turning it off is not this script's business to decide.
	echo "NAT rules removed. IP forwarding left as it is."
}

do_status() {
	local tether uplink
	tether=$(find_tether_iface) || true
	uplink=$(find_uplink_iface) || true
	echo "tether interface : ${tether:-NOT FOUND ($TETHER_ADDR)}"
	echo "uplink interface : ${uplink:-NONE (no default route)}"
	echo "ip_forward       : $(cat /proc/sys/net/ipv4/ip_forward)"
	echo "boot persistence : $([ -f "$UNIT" ] && echo "installed ($UNIT)" || echo "not installed")"
	if [ -n "$uplink" ]; then
		if iptables -t nat -C POSTROUTING -s "$TETHER_NET" -o "$uplink" \
				-j MASQUERADE 2>/dev/null; then
			echo "masquerade       : present"
		else
			echo "masquerade       : absent"
		fi
	fi
}

do_install() {
	need_root
	cat > "$SYSCTL_FILE" <<-EOF
	# Lets the workstation route the ROV tether ($TETHER_NET) to the internet.
	# See nereo_ros2_code/SETUP_RPI/tether_nat.sh
	net.ipv4.ip_forward = 1
	EOF

	# A oneshot unit rather than iptables-persistent: the rules depend on which
	# interface currently carries the default route, so they have to be
	# recomputed at boot, not replayed from a saved table.
	cat > "$UNIT" <<-EOF
	[Unit]
	Description=NAT for the NEREO ROV tether
	After=network-online.target
	Wants=network-online.target

	[Service]
	Type=oneshot
	RemainAfterExit=yes
	ExecStart=$(readlink -f "$0") up
	ExecStop=$(readlink -f "$0") down

	[Install]
	WantedBy=multi-user.target
	EOF

	systemctl daemon-reload
	systemctl enable --now nereo-tether-nat.service
	echo "installed and started. Check with: systemctl status nereo-tether-nat"
	echo "note: the unit runs this script from its current path"
	echo "      ($(readlink -f "$0")) -- reinstall if the repo moves."
}

do_uninstall() {
	need_root
	systemctl disable --now nereo-tether-nat.service 2>/dev/null || true
	rm -f "$UNIT" "$SYSCTL_FILE"
	systemctl daemon-reload
	echo "unit and sysctl drop-in removed. Live rules are untouched; run 'down' for those."
}

case "${1:-}" in
	up)        do_up ;;
	down)      do_down ;;
	status)    do_status ;;
	install)   do_install ;;
	uninstall) do_uninstall ;;
	*) sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac