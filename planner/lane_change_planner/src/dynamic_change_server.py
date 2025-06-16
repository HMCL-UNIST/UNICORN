#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from lane_change_planner.cfg import dyn_change_tunerConfig

def callback(config, level):
    config.evasion_dist = round(config.evasion_dist, 2)
    config.obs_traj_tresh = round(config.obs_traj_tresh,2)
    config.spline_bound_mindist = round(config.spline_bound_mindist, 3)

    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_change_tuner_node", anonymous=False)
    print('[Planner] Dynamic change Server Launched...')
    srv = Server(dyn_change_tunerConfig, callback)
    rospy.spin()