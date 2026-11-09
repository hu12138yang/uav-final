#!/usr/bin/env bash
# Indoor observation only; never uploads missions or sends flight commands.
set -euo pipefail
project=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$project"
if [[ $(id -u) != 0 ]]; then
  echo 'Run through the Windows launcher, or use sudo for the PTP master.' >&2
  exit 2
fi
raw_diagnostics=false
if [[ ${1:-} == --raw-diagnostics ]]; then
  raw_diagnostics=true
  shift
fi
(( $# <= 1 )) || { echo 'Usage: verify_bench.sh [--raw-diagnostics] [seconds]' >&2; exit 2; }
duration=${1:-120}
[[ "$duration" =~ ^[0-9]+$ ]] && ((duration>=5 && duration<=3600)) || { echo 'Duration must be 5..3600 seconds.' >&2; exit 2; }
owner=$(stat -c '%U' "$project")
serial=/dev/serial/by-id/usb-ACFLy_ACFLy_Device_123456-if00
[[ -e "$serial" ]] || { echo 'FCU USB is not attached to WSL.' >&2; exit 2; }
command -v ptp4l >/dev/null || { echo 'linuxptp is required.' >&2; exit 2; }
if pgrep -x ptp4l >/dev/null; then
  echo 'A ptp4l process already exists. Finish that capture before starting another.' >&2
  exit 2
fi
iface=$(ip -o route get 192.168.1.173 | awk '{for(i=1;i<=NF;i++)if($i=="dev"){print $(i+1);exit}}')
ip -o -4 addr show dev "$iface" | grep -q '192.168.1.50/24' || { echo 'Radar interface must carry 192.168.1.50/24.' >&2; exit 2; }
report="algorithm/onboard/test/reports/bench-$(TZ=Asia/Shanghai date +%Y%m%d-%H%M%S)"
runuser -u "$owner" -- mkdir -p "$(dirname "$report")"
runuser -u "$owner" -- mkdir "$report"
runuser -u "$owner" -- cmake -S algorithm/onboard -B .build/check -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON > "$report/build.log" 2>&1
runuser -u "$owner" -- cmake --build .build/check -j4 >> "$report/build.log" 2>&1
(cd .build/check && runuser -u "$owner" -- ctest --output-on-failure) > "$report/ctest.log" 2>&1
ptp4l -i "$iface" -f algorithm/onboard/deploy/ptp4l-master.conf -m > "$report/ptp4l.log" 2>&1 &
ptp_pid=$!
cleanup() { kill "$ptp_pid" 2>/dev/null || true; wait "$ptp_pid" 2>/dev/null || true; chown -R "$owner:$(id -gn "$owner")" "$report"; }
trap cleanup EXIT
args=()
if [[ "$raw_diagnostics" == true ]]; then
  args+=(--diagnose-zero-point-crc)
  echo 'RAW DIAGNOSTICS: capture only; the configured Mid360S sample profile handles absent CRC.'
fi
runuser -u "$owner" -- stdbuf -oL -eL .build/check/onboard_main \
  --serial "$serial" --baud 921600 --config algorithm/onboard/config/onboard.yaml \
  --livox-config algorithm/onboard/deploy/mid360s.json --lidar-ip 192.168.1.173 \
  --observe-only --no-wait-kml --duration "$duration" --record-dir "$report/capture" "${args[@]}" \
  > "$report/observe.log" 2>&1
cat "$report/ctest.log"
tail -n 10 "$report/observe.log"
echo "Capture saved: $project/$report"
if [[ "$raw_diagnostics" == true ]]; then
  echo 'Raw diagnostics captured; trusted input mapping was intentionally disabled.'
else
  python3 algorithm/onboard/scripts/check_link_capture.py "$report/capture"
fi
