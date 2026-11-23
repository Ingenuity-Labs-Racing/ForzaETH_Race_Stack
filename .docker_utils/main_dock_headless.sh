#! /bin/bash

# Script to launch the race stack for local Foxglove testing
IMAGE=nuc_forzaeth_racestack_ros2 # for x86
# IMAGE=jet_forzaeth_racestack_ros2 # for ARM
FORZETH_DIR=/home/severn/race_stack/ForzaETH_Race_Stack

mkdir -p "$FORZETH_DIR"/../cache/humble-foxglove/{build,install,log} || exit 1

docker run --tty \
    --interactive \
    --network=host \
    --env USER=$USER \
    --env ROS_LOCALHOST_ONLY=1 \
    --env ROS_DOMAIN_ID=42 \
    --volume $FORZETH_DIR/../cache/humble-foxglove/build:/home/$USER/ws/build \
    --volume $FORZETH_DIR/../cache/humble-foxglove/install:/home/$USER/ws/install \
    --volume $FORZETH_DIR/../cache/humble-foxglove/log:/home/$USER/ws/log \
    --volume $FORZETH_DIR:/home/$USER/ws/src/race_stack \
    --name forzaeth_foxglove_dev \
    --entrypoint /bin/bash \
    ${IMAGE}:humble
