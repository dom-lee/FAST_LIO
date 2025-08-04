#!/bin/bash
set -e

# --------- Parse Arguments ---------
if [ "$#" -lt 2 ]; then
  echo "[USAGE] $0 <bag_dir> <launch_file> [output_dir]"
  echo "  - bag_dir: Directory containing .bag files"
  echo "  - launch_file: e.g. 'mapping_ouster64.launch'"
  echo "  - output_dir: Optional. Defaults to '/output'"
  exit 1
fi

BAG_DIR="$(realpath "$1")"
LAUNCH_FILE="$2"
OUTPUT_DIR="${3:-/output}"

# --------- Setup Paths ---------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ODOM_LOGGER="$SCRIPT_DIR/odom_logger.py"

if [ ! -d "$BAG_DIR" ]; then
  echo "[ERROR] Bag directory not found: $BAG_DIR"
  exit 1
fi

mkdir -p "$OUTPUT_DIR/logs"


# --------- Process Bag Files ---------
for BAGFILE in "$BAG_DIR"/*.bag; do
  if [ ! -f "$BAGFILE" ]; then
    echo "[WARN] No .bag files found in $BAG_DIR"
    break
  fi

  BASENAME=$(basename "$BAGFILE" .bag)
  CSV_OUTPUT="$OUTPUT_DIR/${BASENAME}_fastlio2.csv"
  FASTLIO_LOG="$OUTPUT_DIR/logs/${BASENAME}_fastlio.log"

  echo
  echo "======================================"
  echo "[INFO] Processing $BASENAME"
  echo "[INFO] Logging FAST-LIO to $FASTLIO_LOG"
  echo "[INFO] Output CSV: $CSV_OUTPUT"
  echo "======================================"
  echo

  # Launch FAST-LIO
  roslaunch fast_lio $LAUNCH_FILE > "$FASTLIO_LOG" 2>&1 &
  LIO_PID=$!
  sleep 3

  # Start odom logger
  python3 "$ODOM_LOGGER" --output "$CSV_OUTPUT" &
  LOGGER_PID=$!
  sleep 1

  # Play the bag
  stdbuf -oL rosbag play --clock "$BAGFILE"
  sleep 3

  # Stop processes
  kill -2 $LOGGER_PID 2>/dev/null || true
  kill -2 $LIO_PID 2>/dev/null || true
  sleep 2
  kill -9 $LOGGER_PID $LIO_PID 2>/dev/null || true
done

echo "[DONE] All bags processed. Output saved to $OUTPUT_DIR"