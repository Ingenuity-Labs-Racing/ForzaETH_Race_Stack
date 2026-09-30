#! /bin/bash

# Script to launch the main docker instance on macOS (Apple Silicon)
# Differences to main_dock.sh: ARM image, repo path detected automatically,
# no /dev or X11 socket mounts, display through XQuartz over TCP and
# software OpenGL so RViz/Gazebo can render without a GPU.

CONTAINER_USER=racer # must match USER used when building the image
IMAGE=jet_forzaeth_racestack_ros2
FORZAETH_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CACHE_DIR="$FORZAETH_DIR/../cache/humble"

mkdir -p "$CACHE_DIR/build" "$CACHE_DIR/install" "$CACHE_DIR/log"

docker run --tty \
    --interactive \
    --env DISPLAY=host.docker.internal:0 \
    --env LIBGL_ALWAYS_SOFTWARE=1 \
    --env USER=$CONTAINER_USER \
    --volume "$CACHE_DIR/build:/home/$CONTAINER_USER/ws/build" \
    --volume "$CACHE_DIR/install:/home/$CONTAINER_USER/ws/install" \
    --volume "$CACHE_DIR/log:/home/$CONTAINER_USER/ws/log" \
    --volume "$FORZAETH_DIR:/home/$CONTAINER_USER/ws/src/race_stack" \
    --name forzaeth_racestack_ros2_humble \
    --entrypoint /bin/bash \
    ${IMAGE}:humble
