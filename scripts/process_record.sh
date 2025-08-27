#!/bin/bash
set -e

# --------- Parse Arguments ---------
if [ "$#" -lt 2 ]; then
  echo "[USAGE] $0 <launch_file> <bag_file1> [bag_file2] ... [--output <output_dir>] [--config <config_file>]"
  echo "  - launch_file: e.g. 'mapping_ouster64.launch'"
  echo "  - bag_files: One or more .bag files to process"
  echo "  - --output: Optional output directory (defaults to 'output/')"
  echo "  - --config: Optional JSON config file for transformation matrices"
  exit 1
fi

LAUNCH_FILE="$1"
OUTPUT_DIR="output"
CONFIG_FILE=""
shift 1

# Parse remaining arguments
BAG_FILES=()
while [[ $# -gt 0 ]]; do
  case $1 in
    --output)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --config)
      CONFIG_FILE="$2"
      shift 2
      ;;
    *)
      BAG_FILES+=("$1")
      shift
      ;;
  esac
done

# --------- Setup Paths ---------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ODOM_LOGGER="$SCRIPT_DIR/odom_logger.py"

mkdir -p "$OUTPUT_DIR/logs"

# --------- Validate Bag Files ---------
for BAGFILE in "${BAG_FILES[@]}"; do
  if [ ! -f "$BAGFILE" ]; then
    echo "[ERROR] Bag file not found: $BAGFILE"
    exit 1
  fi
done

# --------- Process Bag Files ---------
for BAGFILE in "${BAG_FILES[@]}"; do
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
  if [ -n "$CONFIG_FILE" ]; then
    python3 "$ODOM_LOGGER" --output "$CSV_OUTPUT" --config "$CONFIG_FILE" &
  else
    python3 "$ODOM_LOGGER" --output "$CSV_OUTPUT" &
  fi
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