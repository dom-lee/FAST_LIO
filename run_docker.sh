#!/bin/bash
set -e

xhost +local:root >/dev/null 2>&1

XAUTH=/tmp/.docker.xauth
touch "$XAUTH"
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - >/dev/null
chmod 644 "$XAUTH"

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
TMP_WS="${TMP_WS:-$(mktemp -d)}"
mkdir -p "$TMP_WS/src"
trap 'rm -rf "$TMP_WS"' EXIT

CONTAINER_USER="domlee"

# Run the Docker Container
docker run --rm -it \
  --privileged \
  --gpus all \
  --user $(id -u):$(id -g) \
  --env DISPLAY=$DISPLAY \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --env XAUTHORITY=/tmp/.docker.xauth \
  --env DISABLE_ROS1_EOL_WARNINGS=1 \
  -v "$XAUTH":/tmp/.docker.xauth \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$TMP_WS":/home/$CONTAINER_USER/catkin_ws \
  -v "$REPO_DIR":/home/$CONTAINER_USER/catkin_ws/src/FAST_LIO \
  -v /home/domlee/mnt/ARL_SARA:/mnt/ARL_SARA \
  fastlio2