#!/bin/bash
cd /home/unicorn/catkin_ws/src/race_stack/bag
rosbag record \
    /scan \
    /imu/data \
    /vesc/sensors/core \
    /vesc/sensors/servo_position_command \
    /car_state/odom \
