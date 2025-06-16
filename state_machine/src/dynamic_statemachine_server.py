#!/usr/bin/env python3
import rospy
import rospkg
import yaml
from dynamic_reconfigure.server import Server
from state_machine.cfg import dyn_statemachine_tunerConfig
from std_msgs.msg import Bool


class DynamicStateMachineConfigServer:
    def __init__(self):
        self.srv = Server(dyn_statemachine_tunerConfig, self.callback)
        # Get corresponding yaml file path
        pkg_path = rospkg.RosPack().get_path("stack_master")
        self.yaml_file_path = pkg_path + "/config/state_machine_params.yaml" 
        self.yaml_data = self.get_yaml_values(self.yaml_file_path)
        self.default_config = self.decode_yaml(self.yaml_data)
        self.srv.update_configuration(self.default_config) 

        self.true_msg = Bool()
        self.true_msg.data = True
        self.save_start_traj_pub = rospy.Publisher("/save_start_traj", Bool, queue_size=1)



    def save_yaml(self, config):
        try:
            self.yaml_data["lateral_width_gb_m"] = config.lateral_width_gb_m
            self.yaml_data["lateral_width_ot_m"] = config.lateral_width_ot_m
            
            self.yaml_data["overtaking_ttl_sec"] = config.overtaking_ttl_sec
            self.yaml_data["splini_hyst_timer_sec"] = config.splini_hyst_timer_sec
            self.yaml_data["splini_ttl"] = config.splini_ttl
            self.yaml_data["pred_splini_ttl"] = config.pred_splini_ttl
            self.yaml_data["emergency_break_horizon"] = config.emergency_break_horizon
            
            self.yaml_data["ftg_speed_mps"] = config.ftg_speed_mps
            self.yaml_data["ftg_timer_sec"] = config.ftg_timer_sec
            
            self.yaml_data["ftg_active"] = config.ftg_active
            self.yaml_data["force_GBTRACK"] = config.force_GBTRACK

                
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
            "lateral_width_gb_m": float(yaml_data["lateral_width_gb_m"]),
            "lateral_width_ot_m": float(yaml_data["lateral_width_ot_m"]),
            
            
            "overtaking_ttl_sec": float(yaml_data["overtaking_ttl_sec"]),
            "splini_hyst_timer_sec": float(yaml_data["splini_hyst_timer_sec"]),
            "splini_ttl": int(yaml_data["splini_ttl"]),
            "pred_splini_ttl": float(yaml_data["pred_splini_ttl"]),
            "emergency_break_horizon": float(yaml_data["emergency_break_horizon"]),

            "ftg_speed_mps": float(yaml_data["ftg_speed_mps"]),
            "ftg_timer_sec": float(yaml_data["ftg_timer_sec"]),
            
            "ftg_active": bool(yaml_data["ftg_active"]),
            "force_GBTRACK": bool(yaml_data["force_GBTRACK"]),
        }
        return default_config

    def callback(self, config, level):
        if config.save_params:
            self.save_yaml(config)
            config.save_params = False
       
        if config.save_start_traj:
            self.save_start_traj_pub.publish(self.true_msg)
            config.save_start_traj = False
        # Ensuring nice rounding by either 0.05
        config.lateral_width_gb_m = round(config.lateral_width_gb_m * 20) / 20
        config.lateral_width_ot_m = round(config.lateral_width_ot_m * 20) / 20

        config.splini_hyst_timer_sec = round(config.splini_hyst_timer_sec * 20) / 20
        config.splini_ttl = round(config.splini_ttl * 20) / 20
        config.pred_splini_ttl = round(config.pred_splini_ttl, 3)
        config.emergency_break_horizon = round(config.emergency_break_horizon, 2)

        config.ftg_speed_mps = round(config.ftg_speed_mps * 20) / 20
        config.ftg_timer_sec = round(config.ftg_timer_sec * 20) / 20

        if not config.ftg_active:
            rospy.logdebug_throttle_identical(30, "FTG IS NOT ACTIVE")

        return config  


if __name__ == "__main__":
    rospy.init_node("dynamic_statemachine_tuner_node", anonymous=False)
    print('[dynamic_statemachine_tuner_node] State Machine Parameter Server Launched')
    srv = DynamicStateMachineConfigServer()
    rospy.spin()

