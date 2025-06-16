#!/usr/bin/env python3
import rospy
import rospkg
import yaml
from dynamic_reconfigure.server import Server
# from state_machine.cfg import dyn_statemachine_tunerConfig
from state_machine.cfg import dyn_planner_tunerConfig
from std_msgs.msg import Bool


class DynamicStateMachineConfigServer:
    def __init__(self):
        self.planners_name = rospy.get_param("~planners_name", "global_tracking") 
        
        self.srv = Server(dyn_planner_tunerConfig, self.callback)
        # Get corresponding yaml file path
        pkg_path = rospkg.RosPack().get_path("stack_master")
        self.yaml_file_path = pkg_path + "/config/planners/" + self.planners_name +".yaml" 
        self.yaml_data = self.get_yaml_values(self.yaml_file_path)
        self.default_config = self.decode_yaml(self.yaml_data)
        self.srv.update_configuration(self.default_config) 

        self.true_msg = Bool()
        self.true_msg.data = True
        # self.save_start_traj_pub = rospy.Publisher("/save_start_traj", Bool, queue_size=1)



    def save_yaml(self, config):
        try:
            self.yaml_data["min_horizon"] = config.min_horizon
            self.yaml_data["max_horizon"] = config.max_horizon
            
            self.yaml_data["lateral_width_m"] = config.lateral_width_m
            self.yaml_data["free_scaling_reference_distance_m"] = config.free_scaling_reference_distance_m
            self.yaml_data["latest_threshold"] = config.latest_threshold
            self.yaml_data["hyst_timer_sec"] = config.hyst_timer_sec
            
            self.yaml_data["killing_timer_sec"] = config.killing_timer_sec
            self.yaml_data["on_spline_front_horizon_thres_m"] = config.on_spline_front_horizon_thres_m
            
            self.yaml_data["on_spline_min_dist_thres_m"] = config.on_spline_min_dist_thres_m
            # self.yaml_data["force_GBTRACK"] = config.force_GBTRACK
                
            with open(self.yaml_file_path, "w") as yaml_file:
                yaml.dump(self.yaml_data, yaml_file, default_flow_style=False)
            rospy.loginfo("Configuration saved to YAML file: %s", self.yaml_file_path)

        except Exception as e:
            rospy.logerr("Failed to save configuration to YAML: %s", str(e))
            
    def get_yaml_values(self, yaml_file_path):
        # Get and return data
        with open(yaml_file_path, "r") as file:
            data = yaml.safe_load(file)
        return data
    
    def decode_yaml(self, yaml_data):
        default_config = {
            "min_horizon": float(yaml_data["min_horizon"]),
            "max_horizon": float(yaml_data["max_horizon"]),
            
            "lateral_width_m": float(yaml_data["lateral_width_m"]),
            "free_scaling_reference_distance_m": int(yaml_data["free_scaling_reference_distance_m"]),
            "latest_threshold": float(yaml_data["latest_threshold"]),
            "hyst_timer_sec": float(yaml_data["hyst_timer_sec"]),
            "killing_timer_sec": float(yaml_data["killing_timer_sec"]),

            "on_spline_front_horizon_thres_m": float(yaml_data["on_spline_front_horizon_thres_m"]),
            "on_spline_min_dist_thres_m": float(yaml_data["on_spline_min_dist_thres_m"]),

        }
        return default_config

    def callback(self, config, level):
        if config.save_params:
            self.save_yaml(config)
            config.save_params = False

        return config  


if __name__ == "__main__":
    rospy.init_node("dynamic_statemachine_tuner_node", anonymous=False)
    print('[dynamic_statemachine_tuner_node] State Machine Parameter Server Launched')
    srv = DynamicStateMachineConfigServer()
    rospy.spin()

