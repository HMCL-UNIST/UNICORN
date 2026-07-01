#!/usr/bin/env python3
import os
import yaml
import rclpy
from rcl_interfaces.msg import ParameterType, ParameterDescriptor, FloatingPointRange
from rclpy.node import Node
from f110_msgs.msg import WpntArray
from std_msgs.msg import Bool
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from tf_transformations import quaternion_from_euler

from sector_tuner.parameter_event_handler import ParameterEventHandler


class SectorTuner(Node):
    """
    Sector scaler for the velocity of the global waypoints.

    Merges the ROS1 unicorn `sector_server` (markers + yaml save-back via
    dynamic_reconfigure) and `vel_scaler_node` (velocity scaling) into a single
    ROS2 node, following the race_stack ROS2 style.

    Publishes:
        /global_waypoints_scaled (and /shortest_path) : scaled WpntArray
        /sector_markers                               : MarkerArray
    Parameters come from the speed_scaling.yaml installed in the package config
    (loaded as launch parameter overrides). When the `save_params` parameter is
    set True, the current sector configuration is written back to the map's
    speed_scaling.yaml (unicorn-specific behaviour).
    """

    def __init__(self):
        super().__init__('speed_sector_tuner',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.vis_timer = self.create_timer(1.0, self.marker_callback)

        # sectors params
        self.glb_wpnts_og = None
        self.glb_wpnts_scaled = None
        self.glb_wpnts_sp_og = None
        self.glb_wpnts_sp_scaled = None
        self.update_map = False

        # get initial scaling
        self.sectors_params = self.parameters_to_dict()
        self.n_sectors = self.sectors_params['n_sectors']
        # apply the same clip the dyn callback does, so the yaml is in effect at
        # startup (otherwise scaling stays raw until the first param change).
        for i in range(self.n_sectors):
            self.sectors_params[f"Sector{i}"]['scaling'] = np.clip(
                self.sectors_params[f"Sector{i}"]['scaling'], 0, self.sectors_params['global_limit'])

        # unicorn-specific: path to the yaml that can be written back to disk
        # (the map's speed_scaling.yaml). Empty -> save-back disabled.
        try:
            self.yaml_file_path = self.get_parameter('save_yaml_path').value
        except Exception:
            self.yaml_file_path = ''

        desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=2.0, step=0.01)])
        self.set_descriptor('global_limit', descriptor=desc)
        for i in range(self.n_sectors):
            self.set_descriptor('Sector' + str(i) + '.scaling', descriptor=desc)

        # dyn params sub
        self.glb_wpnts_name = "/global_waypoints"
        self.handler = ParameterEventHandler(self)
        self.callback_handle = self.handler.add_parameter_event_callback(
            callback=self.dyn_param_cb,
        )
        self.global_waypoint_sub = self.create_subscription(
            WpntArray, self.glb_wpnts_name, self.global_waypoints_cb, 10)
        self.global_waypoint_sp_sub = self.create_subscription(
            WpntArray, self.glb_wpnts_name + "/shortest_path", self.global_waypoints_sp_cb, 10)
        # unicorn-specific: signal to re-take the original (unscaled) waypoints
        self.update_map_sub = self.create_subscription(
            Bool, "update_map", self.update_map_cb, 10)

        # new glb_waypoints pub
        self.scaled_points_pub = self.create_publisher(WpntArray, "/global_waypoints_scaled", 10)
        self.scaled_points_sp_pub = self.create_publisher(
            WpntArray, "/global_waypoints_scaled/shortest_path", 10)

        # Visualizations
        self.sector_visualization_pub = self.create_publisher(MarkerArray, '/sector_markers', 10)

        self.get_logger().info("Waiting for global waypoints...")

    def parameters_to_dict(self):
        params = {}
        for key in self._parameters:
            keylist = key.split('.')
            paramit = params
            for subkey in keylist[:-1]:
                paramit = paramit.setdefault(subkey, {})
            paramit[keylist[-1]] = self.get_parameter(key).value
        return params

    def update_map_cb(self, data: Bool):
        # unicorn-specific: force re-taking the original waypoints next scale
        self.update_map = True

    def global_waypoints_cb(self, data: WpntArray):
        """Saves the global waypoints of the main trajectory (e.g. min curvature)."""
        self.glb_wpnts_og = data

    def global_waypoints_sp_cb(self, data: WpntArray):
        """Saves the global waypoints of the shortest path."""
        self.glb_wpnts_sp_og = data

    def dyn_param_cb(self, parameter_event):
        """Notices the change in the parameters and scales the global waypoints."""
        if (parameter_event.node != '/speed_sector_tuner'):
            return
        self.sectors_params = self.parameters_to_dict()

        # unicorn-specific: save params to yaml on request
        if self.sectors_params.get('save_params', False):
            self.save_yaml()
            self.set_parameters(
                [rclpy.parameter.Parameter('save_params', rclpy.Parameter.Type.BOOL, False)])

        # update params
        for i in range(self.n_sectors):
            self.sectors_params[f"Sector{i}"]['scaling'] = np.clip(
                self.sectors_params[f"Sector{i}"]['scaling'], 0, self.sectors_params['global_limit'])

        self.get_logger().info(str(self.sectors_params))

    def save_yaml(self):
        """unicorn-specific: dump the current sector configuration to the map yaml."""
        if not self.yaml_file_path:
            self.get_logger().warn("No save_yaml_path configured; skipping save.")
            return
        try:
            yaml_data = {
                'save_params': False,
                'global_limit': float(self.sectors_params['global_limit']),
                'n_sectors': int(self.n_sectors),
            }
            for i in range(self.n_sectors):
                sec = self.sectors_params[f"Sector{i}"]
                yaml_data[f"Sector{i}"] = {
                    'start': int(sec['start']),
                    'end': int(sec['end']),
                    'scaling': float(sec['scaling']),
                    'only_FTG': bool(sec.get('only_FTG', False)),
                    'no_FTG': bool(sec.get('no_FTG', False)),
                }
            wrapped = {'speed_sector_tuner': {'ros__parameters': yaml_data}}
            os.makedirs(os.path.dirname(self.yaml_file_path), exist_ok=True)
            with open(self.yaml_file_path, "w") as yaml_file:
                yaml.dump(wrapped, yaml_file, default_flow_style=False, sort_keys=False)
            self.get_logger().info(f"Configuration saved to YAML file: {self.yaml_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save configuration to YAML: {e}")

    def get_vel_scaling(self, s):
        """
        Gets the dynamically reconfigured velocity scaling for the points.
        Linearly interpolates for points between two sectors.

        Parameters
        ----------
        s
            s parameter whose sector we want to find
        """
        hl_change = 10

        if self.n_sectors > 1:
            for i in range(self.n_sectors):
                if i == 0:
                    if (s >= self.sectors_params[f'Sector{i}']['start']) and (s < self.sectors_params[f'Sector{i}']['start'] + hl_change):
                        scaler = np.interp(
                            x=s,
                            xp=[self.sectors_params[f'Sector{i}']['start'] - hl_change, self.sectors_params[f'Sector{i}']['start'] + hl_change],
                            fp=[self.sectors_params[f'Sector{self.n_sectors-1}']['scaling'], self.sectors_params[f'Sector{i}']['scaling']]
                        )
                    elif (s >= self.sectors_params[f'Sector{i}']['start'] + hl_change) and (s < self.sectors_params[f'Sector{i+1}']['start'] - hl_change):
                        scaler = self.sectors_params[f"Sector{i}"]['scaling']
                    elif (s >= self.sectors_params[f'Sector{i+1}']['start'] - hl_change) and (s < self.sectors_params[f'Sector{i+1}']['start']):
                        scaler = np.interp(
                            x=s,
                            xp=[self.sectors_params[f'Sector{i+1}']['start'] - hl_change, self.sectors_params[f'Sector{i+1}']['start'] + hl_change],
                            fp=[self.sectors_params[f'Sector{i}']['scaling'], self.sectors_params[f'Sector{i+1}']['scaling']]
                        )
                elif i != self.n_sectors - 1:
                    if (s >= self.sectors_params[f'Sector{i}']['start']) and (s < self.sectors_params[f'Sector{i}']['start'] + hl_change):
                        scaler = np.interp(
                            x=s,
                            xp=[self.sectors_params[f'Sector{i}']['start'] - hl_change, self.sectors_params[f'Sector{i}']['start'] + hl_change],
                            fp=[self.sectors_params[f'Sector{i-1}']['scaling'], self.sectors_params[f'Sector{i}']['scaling']]
                        )
                    elif (s >= self.sectors_params[f'Sector{i}']['start'] + hl_change) and (s < self.sectors_params[f'Sector{i+1}']['start'] - hl_change):
                        scaler = self.sectors_params[f"Sector{i}"]['scaling']
                    elif (s >= self.sectors_params[f'Sector{i+1}']['start'] - hl_change) and (s < self.sectors_params[f'Sector{i+1}']['start']):
                        scaler = np.interp(
                            x=s,
                            xp=[self.sectors_params[f'Sector{i+1}']['start'] - hl_change, self.sectors_params[f'Sector{i+1}']['start'] + hl_change],
                            fp=[self.sectors_params[f'Sector{i}']['scaling'], self.sectors_params[f'Sector{i+1}']['scaling']]
                        )
                else:
                    if (s >= self.sectors_params[f'Sector{i}']['start']) and (s < self.sectors_params[f'Sector{i}']['start'] + hl_change):
                        scaler = np.interp(
                            x=s,
                            xp=[self.sectors_params[f'Sector{i}']['start'] - hl_change, self.sectors_params[f'Sector{i}']['start'] + hl_change],
                            fp=[self.sectors_params[f'Sector{i-1}']['scaling'], self.sectors_params[f'Sector{i}']['scaling']]
                        )
                    elif (s >= self.sectors_params[f'Sector{i}']['start'] + hl_change) and (s < self.sectors_params[f'Sector{i}']['end'] - hl_change):
                        scaler = self.sectors_params[f"Sector{i}"]['scaling']
                    elif (s >= self.sectors_params[f'Sector{i}']['end'] - hl_change):
                        scaler = np.interp(
                            x=s,
                            xp=[self.sectors_params[f'Sector{i}']['end'] - hl_change, self.sectors_params[f'Sector{i}']['end'] + hl_change],
                            fp=[self.sectors_params[f'Sector{i}']['scaling'], self.sectors_params[f'Sector{0}']['scaling']]
                        )
        elif self.n_sectors == 1:
            scaler = self.sectors_params["Sector0"]['scaling']

        return scaler

    def scale_points(self):
        """Scales the global waypoints' velocities."""
        if self.glb_wpnts_scaled is None or self.update_map:
            self.glb_wpnts_scaled = self.glb_wpnts_og
            self.glb_wpnts_sp_scaled = self.glb_wpnts_sp_og
            self.update_map = False

        for i, wpnt in enumerate(self.glb_wpnts_og.wpnts):
            vel_scaling = self.get_vel_scaling(i)
            new_vel = wpnt.vx_mps * vel_scaling
            self.glb_wpnts_scaled.wpnts[i].vx_mps = new_vel

    def timer_callback(self):
        if (self.glb_wpnts_og is None):
            return
        self.scale_points()
        self.scaled_points_pub.publish(self.glb_wpnts_scaled)

    def marker_callback(self):
        if self.glb_wpnts_og is None:
            return

        global_waypoints_vis = []
        for waypoint in self.glb_wpnts_og.wpnts:
            global_waypoints_vis.append([waypoint.x_m, waypoint.y_m, waypoint.s_m])

        n_sectors = self.sectors_params['n_sectors']
        sec_markers = MarkerArray()

        for i in range(n_sectors):
            s = self.sectors_params[f"Sector{i}"]['start']
            if s == (len(global_waypoints_vis) - 1):
                theta = np.arctan2((global_waypoints_vis[0][1] - global_waypoints_vis[s][1]), (global_waypoints_vis[0][0] - global_waypoints_vis[s][0]))
            else:
                theta = np.arctan2((global_waypoints_vis[s+1][1] - global_waypoints_vis[s][1]), (global_waypoints_vis[s+1][0] - global_waypoints_vis[s][0]))
            quaternions = quaternion_from_euler(0, 0, theta)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = marker.ARROW
            marker.scale.x = 0.5
            marker.scale.y = 0.05
            marker.scale.z = 0.15
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.position.x = global_waypoints_vis[s][0]
            marker.pose.position.y = global_waypoints_vis[s][1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = quaternions[0]
            marker.pose.orientation.y = quaternions[1]
            marker.pose.orientation.z = quaternions[2]
            marker.pose.orientation.w = quaternions[3]
            marker.id = i
            sec_markers.markers.append(marker)

            marker_text = Marker()
            marker_text.header.frame_id = "map"
            marker_text.header.stamp = self.get_clock().now().to_msg()
            marker_text.type = marker_text.TEXT_VIEW_FACING
            marker_text.text = f"Start Sector {i}"
            marker_text.scale.z = 0.4
            marker_text.color.r = 0.2
            marker_text.color.g = 0.1
            marker_text.color.b = 0.1
            marker_text.color.a = 1.0
            marker_text.pose.position.x = global_waypoints_vis[s][0]
            marker_text.pose.position.y = global_waypoints_vis[s][1]
            marker_text.pose.position.z = 1.5
            marker_text.pose.orientation.x = 0.0
            marker_text.pose.orientation.y = 0.0
            marker_text.pose.orientation.z = 0.0436194
            marker_text.pose.orientation.w = 0.9990482
            marker_text.id = i + n_sectors
            sec_markers.markers.append(marker_text)
        self.sector_visualization_pub.publish(sec_markers)


def main():
    rclpy.init()
    node = SectorTuner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
