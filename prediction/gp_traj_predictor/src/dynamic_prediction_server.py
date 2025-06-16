#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from gp_traj_predictor.cfg import dyn_prediction_tunerConfig

def callback(config, level):
    config.n_time_steps = round(config.n_time_steps)
    config.dt= config.dt
    config.save_distance_front = round(config.save_distance_front, 2)
    config.max_expire_counter = round(config.max_expire_counter)  
    config.update_waypoints = config.update_waypoints
    config.speed_offset = round(config.speed_offset, 3)
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_prediction_tuner_node", anonymous=False)
    print('[Planner] Dynamic Prediction Server Launched...')
    srv = Server(dyn_prediction_tunerConfig, callback)
    rospy.spin()
