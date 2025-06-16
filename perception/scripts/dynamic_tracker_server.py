#!/usr/bin/env python3
import rospy
import rospkg
import yaml
from dynamic_reconfigure.server import Server
from perception.cfg import dyn_tracker_tunerConfig

class DynamicPerceptionConfigServer:
    def __init__(self):
        self.srv = Server(dyn_tracker_tunerConfig, self.callback)
        # Get corresponding yaml file path
        self.sectors = None
        pkg_path = rospkg.RosPack().get_path("stack_master")
        self.yaml_file_path = pkg_path + "/config/opponent_tracker_params.yaml" 
        self.yaml_data = self.get_yaml_values(self.yaml_file_path)
        self.default_config = self.decode_yaml(self.yaml_data)
        self.srv.update_configuration(self.default_config)    

    def save_yaml(self, config):
        try:
            self.yaml_data["filter_kernel_size"] = config.filter_kernel_size
            self.yaml_data["min_size_n"] = config.min_size_n
            self.yaml_data["min_size_m"] = config.min_size_m
            self.yaml_data["max_size_m"] = config.max_size_m
            self.yaml_data["lambda_deg"] = config.lambda_deg
            self.yaml_data["sigma"] = config.sigma
            self.yaml_data["new_cluster_threshold_m"] = config.new_cluster_threshold_m
            self.yaml_data["max_viewing_distance"] = config.max_viewing_distance
            self.yaml_data["boundaries_inflation"] = config.boundaries_inflation
            
            self.yaml_data["ttl_dynamic"] = config.ttl_dynamic
            self.yaml_data["ratio_to_glob_path"] = config.ratio_to_glob_path
            self.yaml_data["ttl_static"] = config.ttl_static
            self.yaml_data["min_nb_meas"] = config.min_nb_meas
            self.yaml_data["dist_deletion"] = config.dist_deletion
            self.yaml_data["dist_infront"] = config.dist_infront
            self.yaml_data["min_std"] = config.min_std
            self.yaml_data["max_std"] = config.max_std
            self.yaml_data["vs_reset"] = config.vs_reset
            self.yaml_data["aggro_multi"] = config.aggro_multi
            self.yaml_data["debug_mode"] = config.debug_mode
            self.yaml_data["publish_static"] = config.publish_static
            self.yaml_data["noMemoryMode"] = config.noMemoryMode
                
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
            "min_size_n": int(yaml_data["min_size_n"]),
            "min_size_m": float(yaml_data["min_size_m"]),
            "max_size_m": float(yaml_data["max_size_m"]),
            "lambda_deg": float(yaml_data["lambda_deg"]),
            "sigma": float(yaml_data["sigma"]),
            "filter_kernel_size": int(yaml_data["filter_kernel_size"]),
            "new_cluster_threshold_m": float(yaml_data["new_cluster_threshold_m"]),

            "max_viewing_distance": float(yaml_data["max_viewing_distance"]),
            "boundaries_inflation": float(yaml_data["boundaries_inflation"]),
            
            "ttl_dynamic": int(yaml_data["ttl_dynamic"]),
            "ratio_to_glob_path": float(yaml_data["ratio_to_glob_path"]),
            "ttl_static": int(yaml_data["ttl_static"]),
            "min_nb_meas": int(yaml_data["min_nb_meas"]),
            "dist_deletion": float(yaml_data["dist_deletion"]),
            "dist_infront": float(yaml_data["dist_infront"]),
            "min_std": float(yaml_data["min_std"]),
            "max_std": float(yaml_data["max_std"]),
            "vs_reset": float(yaml_data["vs_reset"]),
            "aggro_multi": float(yaml_data["aggro_multi"]),
            "debug_mode": bool(yaml_data["debug_mode"]),
            "publish_static": bool(yaml_data["publish_static"]),
            "noMemoryMode": bool(yaml_data["noMemoryMode"]),
        }
        return default_config


    def callback(self, config, level):
        if config.save_params:
            self.save_yaml(config)
            config.save_params = False
            
        # tracking
        config.dist_deletion = round(config.dist_deletion * 20) / 20
        config.dist_infront = round(config.dist_infront * 20) /20
        config.min_std = round(min(config.min_std, config.max_std) * 100) / 100
        config.max_std = round(max(config.min_std + 0.01, config.max_std) * 100) / 100
        config.vs_reset = round(config.vs_reset * 100) / 100
        config.aggro_multi = round(config.aggro_multi *10) / 10
        config.ratio_to_glob_path = round(config.ratio_to_glob_path * 10) / 10

        # detection
        config.max_size_m = round(config.max_size_m * 10) / 10
        config.max_viewing_distance = round(config.max_viewing_distance * 20) / 20
        config.boundaries_inflation = round(config.boundaries_inflation *100) / 100
        return config

if __name__ == "__main__":
    rospy.init_node("dynamic_tracker_server", anonymous=False)
    print('[Opponent Tracking] Dynamic Tracker Server Launched...')
    srv = DynamicPerceptionConfigServer()
    rospy.spin()