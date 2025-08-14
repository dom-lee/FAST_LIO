#!/usr/bin/env bash
set -euo pipefail

if command -v conda >/dev/null 2>&1; then
  # Proper non-interactive conda activation
  eval "$(conda shell.bash hook)"
  conda activate ros310
else
  echo "[WARN] conda not found; skipping conda activation" >&2
fi

if [ "$#" -lt 1 ]; then
  echo "[USAGE] $0 <bag_dir>"
  echo "  - bag_dir: Directory containing ros2 bags"
  exit 1
fi

BAG_DIR="$(realpath "$1")"
cd "$BAG_DIR"

shopt -s nullglob

for BAGPATH in "$BAG_DIR"/*/; do
  BAGPATH="${BAGPATH%/}"

  if [[ -d "$BAGPATH" ]]; then
    if [[ ! -f "$BAGPATH/metadata.yaml" ]]; then
      continue
    fi

    BAG_NAME="$(basename "$BAGPATH")"
    OUT_BAG="${BAG_NAME}.bag"

    if [[ -e "$OUT_BAG" ]]; then
      echo "[SKIP] $OUT_BAG already exists"
      continue
    fi

    echo "[INFO] Converting: '$BAGPATH' -> '$OUT_BAG'"
    # rosbags-convert: ROS 2 bag dir -> ROS 1 .bag file
    if ! command -v rosbags-convert >/dev/null 2>&1; then
      echo "[ERROR] 'rosbags-convert' not found in PATH" >&2
      exit 2
    fi

    rosbags-convert --src "$BAGPATH" --dst "$OUT_BAG"
  fi
done
