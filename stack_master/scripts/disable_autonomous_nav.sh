#!/bin/bash

# Script to disable autonomous navigation and enable teleop control
# This publishes zero commands to Nav_0 (highest priority) to override
# the autonomous navigation system and allow manual teleop control

echo "=========================================="
echo "Disabling Autonomous Navigation"
echo "=========================================="
echo "Publishing zero commands to Nav_0 input..."
echo "This will override autonomous commands and allow teleop control."
echo ""
echo "Press Ctrl+C to stop and re-enable autonomous navigation."
echo "=========================================="
echo ""

# Publish zero ackermann commands to the highest priority nav input
# This runs at 10 Hz to maintain active status
rostopic pub /vesc/high_level/ackermann_cmd_mux/input/nav_0 \
  ackermann_msgs/AckermannDriveStamped \
  "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
drive: 
  steering_angle: 0.0
  steering_angle_velocity: 0.0
  speed: 0.0
  acceleration: 0.0
  jerk: 0.0" \
  -r 10