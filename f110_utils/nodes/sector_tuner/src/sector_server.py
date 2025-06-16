#!/usr/bin/env python3
import rospy
import rospkg
import yaml
import numpy as np
from f110_msgs.msg import Wpnt, WpntArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from dynamic_reconfigure.server import Server
from sector_tuner.cfg import dyn_sect_tunerConfig
from visualization_msgs.msg import Marker, MarkerArray

class SectorPublisher:
    def __init__(self):
        self.srv = Server(dyn_sect_tunerConfig, self.callback)
        # Get corresponding yaml file path
        self.sectors = None
        pkg_path = rospkg.RosPack().get_path("stack_master")
        map_name = rospy.get_param('/map')
        self.yaml_file_path = pkg_path + "/maps/" + map_name + "/speed_scaling.yaml" 
        self.yaml_data = self.get_yaml_values(self.yaml_file_path)
        self.default_config = self.decode_yaml(self.yaml_data)
        self.srv.update_configuration(self.default_config)
        
        
        self.glb_waypoints = None
        self.sector_pub = rospy.Publisher('/sector_markers', MarkerArray, queue_size=10)
        rospy.Subscriber('/global_waypoints', WpntArray, self.glb_wpnts_cb)

    def callback(self, config, level):
        if config.save_params:
            self.save_yaml(config)
            config.save_params = False
        return config
    
    def save_yaml(self, config):
        try:
            self.yaml_data["global_limit"] = config.global_limit
                
            i = 0
            for key, item in self.sectors.items():
                self.yaml_data[key]['scaling'] = float(getattr(config, key, None)) 
                self.yaml_data[key]['only_FTG'] = bool(getattr(config, "OnlyFTG" + str(i), None)) 
                self.yaml_data[key]['no_FTG'] = bool(getattr(config, "NoFTG" + str(i), None)) 
                i += 1

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
            "global_limit": float(yaml_data["global_limit"]),
        }
        self.sectors = {k: v for k, v in yaml_data.items() if k.startswith('Sector')}

        i = 0
        for key, item in self.sectors.items():
            default_config[key] = float(item['scaling'])            
            default_config["OnlyFTG" + str(i)] = bool(item['only_FTG'])            
            default_config["NoFTG" + str(i)] = bool(item['no_FTG'])    
            i += 1
        return default_config
    
    def glb_wpnts_cb(self, data):
        self.glb_waypoints = []
        for waypoint in data.wpnts:
            self.glb_waypoints.append([waypoint.x_m, waypoint.y_m, waypoint.s_m])

    def pub_sector_markers(self):
        rate = rospy.Rate(1)
        while (not rospy.is_shutdown()):
            if self.glb_waypoints is None:
                rate.sleep()
                continue

            n_sectors = self.yaml_data['n_sectors']
            sec_markers = MarkerArray()

            for i in range(n_sectors):
                s = self.yaml_data[f"Sector{i}"]['start']
                if s == (len(self.glb_waypoints) - 1):
                    theta = np.arctan2((self.glb_waypoints[0][1] - self.glb_waypoints[s][1]),(self.glb_waypoints[0][0] - self.glb_waypoints[s][0]))
                else:
                    theta = np.arctan2((self.glb_waypoints[s+1][1] - self.glb_waypoints[s][1]),(self.glb_waypoints[s+1][0] - self.glb_waypoints[s][0]))
                quaternions = quaternion_from_euler(0, 0, theta)
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.type = marker.ARROW
                marker.scale.x = 0.5
                marker.scale.y = 0.05
                marker.scale.z = 0.15
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.pose.position.x = self.glb_waypoints[s][0]
                marker.pose.position.y = self.glb_waypoints[s][1]
                marker.pose.position.z = 0
                marker.pose.orientation.x = quaternions[0]
                marker.pose.orientation.y = quaternions[1]
                marker.pose.orientation.z = quaternions[2]
                marker.pose.orientation.w = quaternions[3]
                marker.id = i
                sec_markers.markers.append(marker)

                marker_text = Marker()
                marker_text.header.frame_id = "map"
                marker_text.header.stamp = rospy.Time.now()
                marker_text.type = marker_text.TEXT_VIEW_FACING
                marker_text.text = f"Start Sector {i}"
                marker_text.scale.z = 0.4
                marker_text.color.r = 0.2
                marker_text.color.g = 0.1
                marker_text.color.b = 0.1
                marker_text.color.a = 1.0
                marker_text.pose.position.x = self.glb_waypoints[s][0]
                marker_text.pose.position.y = self.glb_waypoints[s][1]
                marker_text.pose.position.z = 1.5
                marker_text.pose.orientation.x = 0.0
                marker_text.pose.orientation.y = 0.0
                marker_text.pose.orientation.z = 0.0436194
                marker_text.pose.orientation.w = 0.9990482
                marker_text.id = i + n_sectors
                sec_markers.markers.append(marker_text)
            self.sector_pub.publish(sec_markers)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("dynamic_sector_tuner", anonymous=False)
    print('Dynamic Sector Server Launched...')

    sec_pub = SectorPublisher()
    sec_pub.pub_sector_markers()
