#! /bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# ROS dependencies of the AutoDRIVE devkit bridge
sudo apt-get install -y ros-humble-tf-transformations ros-humble-imu-tools

# python dependencies of the AutoDRIVE devkit bridge
pip3 install -r "$SCRIPT_DIR/autodrive_req.txt"
