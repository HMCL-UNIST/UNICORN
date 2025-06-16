#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from recovery_spliner.cfg import dyn_recovery_spliner_tunerConfig

def callback(config, level):
    # Ensuring nice rounding by either 0.05 or 0.5

    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_recovery_spline_tuner_node", anonymous=False)
    print('[Planner] Dynamic Recovery Spline Server Launched...')
    srv = Server(dyn_recovery_spliner_tunerConfig, callback)
    rospy.spin()

