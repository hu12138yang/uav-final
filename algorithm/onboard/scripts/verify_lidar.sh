#!/usr/bin/env bash
# Direct SDK2 point-cloud verification; never opens the FCU serial port.
set -euo pipefail
project=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$project"
duration=${1:-60}
[[ "$duration" =~ ^[0-9]+$ ]] && ((duration>=5 && duration<=3600)) || { echo 'Duration must be 5..3600 seconds.' >&2; exit 2; }
report="algorithm/onboard/test/reports/lidar-direct-$(TZ=Asia/Shanghai date +%Y%m%d-%H%M%S)"
mkdir -p "$(dirname "$report")"
mkdir "$report"
cmake -S algorithm/onboard -B .build/onboard -DCMAKE_BUILD_TYPE=Release > "$report/build.log" 2>&1
cmake --build .build/onboard -j4 >> "$report/build.log" 2>&1
echo 'Direct Ethernet/SDK2 capture. No FCU serial, mission upload or motion commands.'
echo 'PTP is managed separately; this command verifies raw sensor-frame reception.'
stdbuf -oL -eL .build/onboard/onboard_main --lidar-only \
  --config algorithm/onboard/config/onboard.yaml \
  --livox-config algorithm/onboard/deploy/mid360s.json --lidar-ip 192.168.1.173 \
  --duration "$duration" --record-dir "$report/capture" 2>&1 | tee "$report/observe.log"
echo "Capture saved: $project/$report"
