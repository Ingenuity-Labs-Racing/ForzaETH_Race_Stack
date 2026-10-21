#! /bin/bash

# Script to launch the main docker instance on macOS (Apple Silicon)
# Differences to main_dock.sh: ARM image, repo path detected automatically,
# no /dev or X11 socket mounts. XQuartz has no usable OpenGL for RViz/Gazebo,
# so GUIs run on a virtual display inside the container instead
# (see start_vnc_display.sh). View it with Screen Sharing: vnc://localhost:5901, password: racer
# Port 8765 is for the Foxglove app (ws://localhost:8765), see foxglove_bridge.

CONTAINER_USER=racer # must match USER used when building the image
IMAGE=jet_forzaeth_racestack_ros2
FORZAETH_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CACHE_DIR="$FORZAETH_DIR/../cache/humble"

mkdir -p "$CACHE_DIR/build" "$CACHE_DIR/install" "$CACHE_DIR/log"

docker run --tty \
    --interactive \
    --publish 127.0.0.1:5901:5900 \
    --publish 127.0.0.1:8765:8765 \
    --env DISPLAY=:1 \
    --env LIBGL_ALWAYS_SOFTWARE=1 \
    --env USER=$CONTAINER_USER \
    --volume "$CACHE_DIR/build:/home/$CONTAINER_USER/ws/build" \
    --volume "$CACHE_DIR/install:/home/$CONTAINER_USER/ws/install" \
    --volume "$CACHE_DIR/log:/home/$CONTAINER_USER/ws/log" \
    --volume "$FORZAETH_DIR:/home/$CONTAINER_USER/ws/src/race_stack" \
    --name forzaeth_racestack_ros2_humble \
    --entrypoint /bin/bash \
    ${IMAGE}:humble \
    -c "~/ws/src/race_stack/.docker_utils/start_vnc_display.sh; exec bash"
