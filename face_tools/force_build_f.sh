#!/bin/bash
set -e
. /opt/ros/humble/setup.bash
colcon build \
  --base-paths /home/racecar/src/encoder_imu /home/racecar/src/odom_tf \
  --build-base /home/racecar/build \
  --install-base /home/racecar/install \
  --cmake-clean-first