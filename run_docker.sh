#!/bin/bash
set -e

# --------- Detect if DISPLAY is set (local X11 GUI) ---------
HAS_DISPLAY=0
if [ -n "$DISPLAY" ] && xauth nlist "$DISPLAY" | grep -q .; then
  HAS_DISPLAY=1
  echo "[INFO] X11 forwarding detected, DISPLAY=$DISPLAY"
else
  echo "[INFO] No X11 forwarding detected, DISPLAY is not set"
fi

# --------- X11 config ---------
XAUTH=/tmp/.docker.xauth
if [ "$HAS_DISPLAY" -eq 1 ]; then
  xhost +local:root >/dev/null 2>&1
  touch "$XAUTH"
  xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - >/dev/null
  chmod 644 "$XAUTH"
fi

# --------- Docker Volume Setup ---------
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
TMP_WS="${TMP_WS:-$(mktemp -d)}"
mkdir -p "$TMP_WS/src"
trap 'rm -rf "$TMP_WS"' EXIT

CONTAINER_USER="domlee"

# --------- Docker Run Command ---------
docker run --rm -it \
  --privileged \
  --gpus all \
  --user $(id -u):$(id -g) \
  --env DISABLE_ROS1_EOL_WARNINGS=1 \
  -v "$TMP_WS":/home/$CONTAINER_USER/catkin_ws \
  -v "$REPO_DIR":/home/$CONTAINER_USER/catkin_ws/src/FAST_LIO \
  -v /home/domlee/mnt/ARL_SARA:/mnt/ARL_SARA \
  -v "$REPO_DIR/output":/output \
  $( # GUI options only if DISPLAY is valid
    if [ "$HAS_DISPLAY" -eq 1 ]; then
      echo \
        --env DISPLAY=$DISPLAY \
        --env XAUTHORITY=/tmp/.docker.xauth \
        -v "$XAUTH":/tmp/.docker.xauth \
        -v /tmp/.X11-unix:/tmp/.X11-unix
    fi
  ) \
  fastlio2 \
  bash