#! /bin/bash

# Script to launch the AutoDRIVE RoboRacer simulator container (run on the host, next to the race stack container)
# Requires an NVIDIA GPU with the NVIDIA Container Toolkit
IMAGE=autodriveecosystem/autodrive_roboracer_sim
TAG=${TAG:-2026-iros-practice} # see https://hub.docker.com/r/autodriveecosystem/autodrive_roboracer_sim/tags

xhost +local:root

docker run --tty \
    --interactive \
    --rm \
    --network=host \
    --ipc=host \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --privileged \
    --gpus all \
    --name autodrive_roboracer_sim \
    --entrypoint /bin/bash \
    ${IMAGE}:${TAG}

# Inside the container, from /home/autodrive_simulator:
#   ./AutoDRIVE\ Simulator.x86_64                                     # with graphics
#   xvfb-run ./AutoDRIVE\ Simulator.x86_64 -ip 127.0.0.1 -port 4567   # headless
