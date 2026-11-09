#!/usr/bin/env bash
# Restore the pinned SDK plus the project's measured-packet validation patch.
set -euo pipefail
project=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
sdk="$project/lidar/Livox-SDK2"
patch="$project/lidar/patches/mid360s-data-validation.patch"
if [[ ! -f "$sdk/sdk_core/data_handler/data_handler.h" ]]; then
  git -C "$project" submodule update --init -- lidar/Livox-SDK2
fi
if git -C "$sdk" apply --reverse --check "$patch" 2>/dev/null; then
  echo 'Livox SDK project patch already present.'
elif git -C "$sdk" apply --check "$patch"; then
  git -C "$sdk" apply "$patch"
  echo 'Livox SDK project patch applied.'
else
  echo 'SDK patch conflicts with local changes; SDK files were left untouched.' >&2
  exit 1
fi
