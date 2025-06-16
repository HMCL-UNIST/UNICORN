#!/usr/bin/env python3
import rospy
import rospkg
import yaml
from dynamic_reconfigure.server import Server
from controller.cfg import controllerConfig


class DynamicControllerConfigServer:
    def __init__(self):
        self.srv = Server(controllerConfig, self.callback)
        # Get corresponding yaml file path
        self.sectors = None
        pkg_path = rospkg.RosPack().get_path("stack_master")
        car_name = rospy.get_param('/racecar_version')
        self.yaml_file_path = pkg_path + "/config/" + car_name + "/controller.yaml" 
        self.yaml_data = self.get_yaml_values(self.yaml_file_path)
        self.default_config = self.decode_yaml(self.yaml_data)
        self.srv.update_configuration(self.default_config)    

    def save_yaml(self, config):
        try:
            self.yaml_data["t_clip_min"] = config.t_clip_min
            self.yaml_data["t_clip_max"] = config.t_clip_max
            self.yaml_data["m_l1"] = config.m_l1
            self.yaml_data["q_l1"] = config.q_l1
            self.yaml_data["curvature_factor"] = config.curvature_factor
            self.yaml_data["speed_factor_for_lat_err"] = config.speed_factor_for_lat_err
            self.yaml_data["speed_factor_for_curvature"] = config.speed_factor_for_curvature

            self.yaml_data["KP"] = config.KP
            self.yaml_data["KI"] = config.KI
            self.yaml_data["KD"] = config.KD

            self.yaml_data["heading_error_thres"] = config.heading_error_thres
            self.yaml_data["steer_gain_for_speed"] = config.steer_gain_for_speed

            self.yaml_data["future_constant"] = config.future_constant

            self.yaml_data["speed_diff_thres"] = config.speed_diff_thres
            self.yaml_data["start_speed"] = config.start_speed
            self.yaml_data["start_curvature_factor"] = config.start_curvature_factor
            self.yaml_data["AEB_thres"] = config.AEB_thres

            self.yaml_data["speed_lookahead"] = config.speed_lookahead
            self.yaml_data["lat_err_coeff"] = config.lat_err_coeff
            self.yaml_data["acc_scaler_for_steer"] = config.acc_scaler_for_steer
            self.yaml_data["dec_scaler_for_steer"] = config.dec_scaler_for_steer
            self.yaml_data["start_scale_speed"] = config.start_scale_speed
            self.yaml_data["end_scale_speed"] = config.end_scale_speed
            self.yaml_data["downscale_factor"] = config.downscale_factor
            self.yaml_data["speed_lookahead_for_steer"] = config.speed_lookahead_for_steer
            self.yaml_data["trailing_gap"] = config.trailing_gap 
            self.yaml_data["trailing_vel_gain"] = config.trailing_vel_gain 
            self.yaml_data["trailing_p_gain"] = config.trailing_p_gain
            self.yaml_data["trailing_i_gain"] = config.trailing_i_gain
            self.yaml_data["trailing_d_gain"] = config.trailing_d_gain
            self.yaml_data["blind_trailing_speed"] = config.blind_trailing_speed


                
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
            ## L1 Controller Parameters
            "t_clip_min": float(yaml_data["t_clip_min"]),
            "t_clip_max": float(yaml_data["t_clip_max"]),
            "m_l1": float(yaml_data["m_l1"]),
            "q_l1": float(yaml_data["q_l1"]),
            "curvature_factor": float(yaml_data["curvature_factor"]),
            "speed_factor_for_lat_err": float(yaml_data["speed_factor_for_lat_err"]),
            "speed_factor_for_curvature": float(yaml_data["speed_factor_for_curvature"]),

            "KP": float(yaml_data["KP"]),
            "KI": float(yaml_data["KI"]),
            "KD": float(yaml_data["KD"]),

            "heading_error_thres": float(yaml_data["heading_error_thres"]),
            "steer_gain_for_speed": float(yaml_data["steer_gain_for_speed"]),

            "future_constant": float(yaml_data["future_constant"]),

            "speed_diff_thres": float(yaml_data["speed_diff_thres"]),
            "start_speed": float(yaml_data["start_speed"]),
            "start_curvature_factor": float(yaml_data["start_curvature_factor"]),
            "AEB_thres": float(yaml_data["AEB_thres"]),

            "speed_lookahead": float(yaml_data["speed_lookahead"]),
            "lat_err_coeff": float(yaml_data["lat_err_coeff"]),
            "acc_scaler_for_steer": float(yaml_data["acc_scaler_for_steer"]),
            "dec_scaler_for_steer": float(yaml_data["dec_scaler_for_steer"]),
            "start_scale_speed": float(yaml_data["start_scale_speed"]),
            "end_scale_speed": float(yaml_data["end_scale_speed"]),
            "downscale_factor": float(yaml_data["downscale_factor"]),
            "speed_lookahead_for_steer": float(yaml_data["speed_lookahead_for_steer"]),
            ## Trailing Controller Parameters
            "trailing_gap": float(yaml_data["trailing_gap"]), 
            "trailing_vel_gain": float(yaml_data["trailing_vel_gain"]), 
            "trailing_p_gain": float(yaml_data["trailing_p_gain"]),
            "trailing_i_gain": float(yaml_data["trailing_i_gain"]),
            "trailing_d_gain": float(yaml_data["trailing_d_gain"]),
            "blind_trailing_speed": float(yaml_data["blind_trailing_speed"])

        }          
        return default_config

    # Set l1 parameter values to the values of the corresponding .yaml file
    # float() is need as we want to ensure its type to access it in L1_controller with "params.doubles[idx].value"
    def callback(self, config, level):
        if config.save_params:
            self.save_yaml(config)
            config.save_params = False
            
        rospy.loginfo("L1 Parameter Updated")
        # Ensuring nice rounding by 0.05
        config.t_clip_min = round(config.t_clip_min * 200) / 200 # round to 0.005
        config.t_clip_max = round(config.t_clip_max * 200) / 200
        config.m_l1 = round(config.m_l1 * 200) / 200
        config.q_l1 = round(config.q_l1 * 200) / 200
        config.curvature_factor = round(config.curvature_factor * 200) / 200
        config.future_constant = round(config.future_constant*200) / 200

        config.speed_diff_thres = round(config.speed_diff_thres*200) / 200
        config.start_speed = round(config.start_speed*200) / 200
        config.start_curvature_factor = round(config.start_curvature_factor*200) / 200

        config.speed_factor_for_lat_err = round(config.speed_factor_for_lat_err*200) / 200
        config.speed_factor_for_curvature = round(config.speed_factor_for_curvature*200) / 200
        config.AEB_thres = round(config.AEB_thres * 200) / 200 

        config.KP = round(config.KP * 200) / 200
        config.KI = round(config.KI * 200) / 200
        config.KD = round(config.KD * 200) / 200

        config.heading_error_thres = round(config.heading_error_thres * 200) / 200
        config.steer_gain_for_speed = round(config.steer_gain_for_speed * 200) / 200

        config.speed_lookahead = round(config.speed_lookahead * 20) / 20
        config.lat_err_coeff = round(config.lat_err_coeff * 20) / 20
        config.acc_scaler_for_steer = round(config.acc_scaler_for_steer * 20) / 20
        config.dec_scaler_for_steer = round(config.dec_scaler_for_steer * 20) / 20
        config.start_scale_speed = round(config.start_scale_speed * 20) / 20
        config.end_scale_speed = round(config.end_scale_speed * 20) / 20
        config.downscale_factor = round(config.downscale_factor * 20) / 20
        config.speed_lookahead_for_steer = round(config.speed_lookahead_for_steer * 20) / 20
        #Trailing Controller
        config.trailing_gap = round(config.trailing_gap * 20) / 20 
        config.trailing_vel_gain = round(config.trailing_vel_gain * 100) / 100
        config.trailing_p_gain = round(config.trailing_p_gain * 100) / 100
        config.trailing_i_gain = round(config.trailing_i_gain * 100) / 100
        config.trailing_d_gain = round(config.trailing_d_gain * 100) / 100
        config.blind_trailing_speed = round(config.blind_trailing_speed * 10) / 10

        return config

if __name__ == "__main__":
    rospy.init_node("dyn_l1_params_tuner_server", anonymous=False)
    
    srv = DynamicControllerConfigServer()
    rospy.spin()



