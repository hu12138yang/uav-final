#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo "Usage: sudo $0 <physical-lidar-interface>; sets additional 192.168.1.50/24 address, no default route change" >&2
  exit 2
fi
iface=$1
if [[ ! -d /sys/class/net/$iface ]]; then echo "Interface not found: $iface" >&2; exit 2; fi
if grep -qi microsoft /proc/sys/kernel/osrelease; then
  echo "WSL detected. Do not assign the physical LAN IP to a NAT eth0. Enable mirrored networking/restart WSL or use native Ubuntu." >&2
  exit 2
fi
ip link set dev "$iface" up
ip address replace 192.168.1.50/24 dev "$iface"
ip route replace 192.168.1.173/32 dev "$iface" src 192.168.1.50
ip -br address show dev "$iface"
echo "Start PTP: sudo ptp4l -i $iface -f algorithm/onboard/deploy/ptp4l-master.conf -m"
