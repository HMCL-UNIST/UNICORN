#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from vesc_ackermann.cfg import dyn_wheel_odomConfig

def callback(config, level):
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_traction_tuner_node", anonymous=False)
    print('Dynamic Traction Server Launched...')
    srv = Server(dyn_wheel_odomConfig, callback)
    rospy.spin()

