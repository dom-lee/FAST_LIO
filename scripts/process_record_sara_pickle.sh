#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ODOM_LOGGER="$SCRIPT_DIR/odom_logger.py"

# ----------- Configurable Parameters ------------
INPUT_DIR="/mnt/ARL_SARA/pickle/03-06-2025"
OUTPUT_DIR="/output"
LAUNCH_FILE="fast_lio mapping_sara_pickle.launch"
ODOM_TOPIC="/Odometry"
# -----------------------------------------------

mkdir -p "$OUTPUT_DIR/logs"

# Start roscore if not already running
if ! pgrep -x "roscore" > /dev/null; then
  echo "[INFO] Starting roscore..."
  roscore > "$OUTPUT_DIR/roscore.log" 2>&1 &
  sleep 2
fi

# Process all bag files
for BAGFILE in "$INPUT_DIR"/*.bag; do
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

  # Launch FAST-LIO (log only its output)
  roslaunch $LAUNCH_FILE > "$FASTLIO_LOG" 2>&1 &
  LIO_PID=$!
  sleep 3

  # Start recording user-defined topics
  python3 "$ODOM_LOGGER" --output "$CSV_OUTPUT" --odom_topic "$ODOM_TOPIC" &
  LOGGER_PID=$!
  sleep 1

  # Play input bag
  DURATION=$(rosbag info "$BAGFILE" | grep "duration" | head -1 | awk '{print $2}')
  echo "[INFO] Estimated duration: $DURATION"
  stdbuf -oL rosbag play --clock "$BAGFILE"
  sleep 3

  # Clean up
  kill $LOGGER_PID 2>/dev/null || true
  kill $LIO_PID 2>/dev/null || true
  echo "[INFO] Finished processing $BASENAME"
  echo
done

echo "[DONE] All bags processed. Output saved to $OUTPUT_DIR"