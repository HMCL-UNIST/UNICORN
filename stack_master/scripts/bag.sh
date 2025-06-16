#!/bin/bash
cd /home/unicorn/catkin_ws/src/race_stack/bag
rosbag record \
    /scan \
    /imu/data \
    /vesc/low_level/ackermann_cmd_mux/output \
    /car_state/odom \
    /car_state/odom \
    /car_state/odom_frenet \
    /behavior_strategy \
