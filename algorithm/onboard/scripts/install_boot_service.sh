#!/usr/bin/env bash
# One-time native installation. Normal boots never require a login or command.
set -euo pipefail
[[ $# == 3 ]] || { echo 'Usage: sudo install_boot_service.sh <run-user> <physical-lidar-interface> <fcu-device>' >&2; exit 2; }
[[ $(id -u) == 0 ]] || { echo 'Installation requires root on the onboard computer.' >&2; exit 2; }
if grep -qi microsoft /proc/sys/kernel/osrelease; then
  echo 'Install this boot service on the actual RK3588/Linux computer; WSL is not the deployment target.' >&2
  exit 2
fi
[[ $(ps -p 1 -o comm=) == systemd ]] || { echo 'This deployment requires a Linux system booted with systemd.' >&2; exit 2; }
project=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
run_user=$1
lidar_iface=$2
serial=$3
build_jobs=${UAV_BUILD_JOBS:-2}
[[ "$build_jobs" =~ ^[1-9][0-9]?$ ]] && ((build_jobs<=16)) || { echo 'UAV_BUILD_JOBS must be 1..16.' >&2; exit 2; }
id "$run_user" >/dev/null
[[ -d /sys/class/net/$lidar_iface ]] || { echo 'Physical lidar interface does not exist.' >&2; exit 2; }
[[ $serial == /dev/* && $serial != *tcp://* ]] || { echo 'Use a physical /dev/ serial path, preferably /dev/serial/by-id/...' >&2; exit 2; }
for dependency in cmake g++ python3 runuser systemctl systemd-analyze ip stdbuf; do
  command -v "$dependency" >/dev/null || { echo "Missing dependency: $dependency" >&2; exit 2; }
done
[[ -x /usr/sbin/ptp4l ]] || { echo 'Install linuxptp before deployment.' >&2; exit 2; }
[[ ! -e /etc/uav-onboard.env ]] || { echo '/etc/uav-onboard.env already exists; preserve its machine-specific settings and manage the installed service with systemctl.' >&2; exit 2; }
staging=$(mktemp -d)
trap 'rm -rf "$staging"' EXIT
# Validate and render before modifying the target system or building.
python3 - "$project" "$run_user" "$lidar_iface" "$serial" "$staging" <<'PY'
from pathlib import Path
import re
import sys
project,user,interface,serial,staging=sys.argv[1:]
for value in (project,user,interface,serial):
    if not re.fullmatch(r'[A-Za-z0-9_./:@+\-]+',value):
        raise SystemExit('Use deployment paths/interface names without spaces or control characters.')
root=Path(project);out=Path(staging)
for name in ('uav-onboard','uav-lidar-ptp'):
    text=(root/'algorithm/onboard/deploy'/f'{name}.service.in').read_text()
    text=text.replace('@PROJECT_DIR@',project).replace('@RUN_USER@',user)
    (out/(name+'.service')).write_text(text)
(out/'uav-onboard.env').write_text(f'UAV_MODE=control\nUAV_SERIAL={serial}\nUAV_BAUD=921600\nUAV_KML={project}/missions/mission.kml\nUAV_LIDAR_IFACE={interface}\n')
PY
run_group=$(id -gn "$run_user")
install -d -o "$run_user" -g "$run_group" "$project/.build" "$project/.build/onboard" "$project/missions"
echo "Building natively on $(uname -m); this is a one-time deployment step."
runuser -u "$run_user" -- cmake -S "$project/algorithm/onboard" -B "$project/.build/onboard" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
runuser -u "$run_user" -- cmake --build "$project/.build/onboard" -j"$build_jobs"
# Executes no device operation, but catches wrong-architecture binaries and missing libraries.
runuser -u "$run_user" -- "$project/.build/onboard/onboard_main" --help >/dev/null
systemd-analyze verify "$staging/uav-onboard.service" "$staging/uav-lidar-ptp.service"
install -m 644 "$staging/uav-onboard.service" "$staging/uav-lidar-ptp.service" /etc/systemd/system/
install -m 644 "$staging/uav-onboard.env" /etc/uav-onboard.env
systemctl daemon-reload
systemctl enable uav-onboard.service uav-lidar-ptp.service
systemctl is-enabled --quiet uav-onboard.service
systemctl is-enabled --quiet uav-lidar-ptp.service
echo 'Installed and enabled: next boot automatically enters WAIT_KML without login/SSH.'
echo 'No hardware process was started by this installer; validation did not open any FCU or lidar connection.'
echo 'Deliver one mission atomically to missions/mission.kml. GCS owns arming, takeoff and landing.'
