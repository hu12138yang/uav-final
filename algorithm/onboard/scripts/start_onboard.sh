#!/usr/bin/env bash
# One process per boot/task. The executable owns WAIT_KML and task validation.
set -euo pipefail
project=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$project"
binary="$project/.build/onboard/onboard_main"
[[ -x "$binary" ]] || { echo 'Build .build/onboard/onboard_main before deployment.' >&2; exit 2; }
mode=${UAV_MODE:-control}
case "$mode" in
  control) mode_args=(--enable-control) ;;
  observe) mode_args=(--observe-only) ;;
  *) echo 'UAV_MODE must be control or observe.' >&2; exit 2 ;;
esac
exec stdbuf -oL -eL "$binary" \
  --serial "${UAV_SERIAL:-/dev/serial/by-id/usb-ACFLy_ACFLy_Device_123456-if00}" \
  --baud "${UAV_BAUD:-921600}" \
  --config "${UAV_CONFIG:-$project/algorithm/onboard/config/onboard.yaml}" \
  --kml "${UAV_KML:-$project/missions/mission.kml}" \
  --livox-config "$project/algorithm/onboard/deploy/mid360s.json" \
  --lidar-ip 192.168.1.173 "${mode_args[@]}"
