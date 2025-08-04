#!/bin/bash
set -e

# --------- Check args ---------
if [ $# -lt 1 ]; then
  echo "[USAGE] $0 <path_to_data_dir> [optional container command]"
  exit 1
fi

DATA_DIR="$(realpath "$1")"
shift
CMD=("${@:-bash}")

if [ ! -d "$DATA_DIR" ]; then
  echo "[ERROR] Data directory does not exist: $DATA_DIR"
  exit 1
fi

# --------- Detect DISPLAY for GUI ---------
HAS_DISPLAY=0
if [ -n "$DISPLAY" ] && command -v xauth >/dev/null && xauth nlist "$DISPLAY" | grep -q .; then
  HAS_DISPLAY=1
  echo "[INFO] X11 forwarding enabled, DISPLAY=$DISPLAY"
else
  echo "[INFO] X11 forwarding disabled"
fi

# --------- X11 Auth ---------
XAUTH=/tmp/.docker.xauth
if [ "$HAS_DISPLAY" -eq 1 ]; then
  if command -v xhost >/dev/null; then
    xhost +local:root >/dev/null 2>&1
  fi
  touch "$XAUTH"
  xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - >/dev/null
  chmod 644 "$XAUTH"
fi

# --------- Paths ---------
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WS_DIR="$(realpath "$REPO_DIR/../..")"
if [ ! -d "$WS_DIR/src" ]; then
  echo "[ERROR] '$WS_DIR' does not contain a 'src/' folder — not a valid catkin workspace."
  exit 1
fi
if [ ! -d "$WS_DIR/devel" ] || [ ! -d "$WS_DIR/build" ]; then
  echo "[INFO] If you haven't built the workspace yet, run: catkin_make"
fi

USERNAME=$(id -un)
IMAGE_NAME="fastlio2"

# --------- Build image if missing ---------
if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
  echo "[INFO] Building Docker image: $IMAGE_NAME"
  docker build \
    --build-arg USERNAME=$USERNAME \
    --build-arg USER_UID=$(id -u) \
    --build-arg USER_GID=$(id -g) \
    -t $IMAGE_NAME . || exit 1
fi

# --------- Ensure output dir exists ---------
mkdir -p "$REPO_DIR/output"

# --------- Docker Run ---------
docker run --rm -it \
  --privileged \
  --gpus all \
  --user $(id -u):$(id -g) \
  --env DISABLE_ROS1_EOL_WARNINGS=1 \
  --env TZ=Etc/UTC \
  -v "$DATA_DIR":/mnt/data \
  -v "$WS_DIR":/home/$USERNAME/FAST_LIO_ws \
  -v "$REPO_DIR/output":/output \
  $(if [ "$HAS_DISPLAY" -eq 1 ]; then
      echo \
        --env DISPLAY=$DISPLAY \
        --env XAUTHORITY=/tmp/.docker.xauth \
        -v "$XAUTH":/tmp/.docker.xauth \
        -v /tmp/.X11-unix:/tmp/.X11-unix
    fi) \
  "$IMAGE_NAME" "${CMD[@]}"