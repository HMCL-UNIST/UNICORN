#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from f110_msgs.msg import LapData, WpntArray
from std_msgs.msg import Float32, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker

from collections import deque
from copy import deepcopy
import numpy as np

from datetime import datetime
import os

import pitwall  # one-line telemetry: pitwall.log(...) -> /pitwall/<key>, pitwall.event(...) -> /pitwall/events


class LapAnalyser(Node):
    def __init__(self):
        super().__init__('lap_analyser',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.get_logger().info("Lap_analyser node started")

        # Bind pitwall telemetry to this node so lap metrics get recorded to the
        # pitwall MCAP (cheap no-op unless a recorder is subscribed to /pitwall/*).
        pitwall.init(self)

        # Where to place the lap-stats text marker. ROS1 blocked here with
        # wait_for_message("/state_marker") to grab the state machine's marker pose;
        # rclpy can't block in __init__, so we latch it lazily on the first
        # /state_marker (marker_cb) instead. Until then vis_pos stays unset so the
        # text isn't pinned to the origin.
        self.vis_pos = None
        self.state_marker = None
        self.marker_sub = self.create_subscription(Marker, '/state_marker', self.marker_cb, 10)

        # stuff for min distance to track boundary
        self.wp_flag = False
        self.car_distance_to_boundary = []
        self.global_lateral_waypoints = None
        self.gb_wpnts_sub = self.create_subscription(WpntArray, "/global_waypoints", self.waypoints_cb, 10)

        # car odom in frenet frame (ROS1 unicorn authoritative topic name)
        self.odom_frenet_sub = self.create_subscription(Odometry, '/car_state/odom_frenet', self.frenet_odom_cb, 10)

        self.lap_analy_sub = self.create_subscription(Empty, '/lap_analyser/start', self.start_log_cb, 10)

        # New subscriber for x,y coordinate odom (unicorn-specific)
        self.odom_xy_sub = self.create_subscription(Odometry, '/car_state/odom', self.odom_xy_cb, 10)
        self.latest_odom = None
        self.odom_points = []  # store current odom (x,y) along with d and dist_to_boundary

        # publishes once when a lap is completed
        self.lap_data_pub = self.create_publisher(LapData, 'lap_data', 10)
        self.min_car_distance_to_boundary_pub = self.create_publisher(Float32, 'min_car_distance_to_boundary', 10)
        self.lap_start_time = self.get_clock().now()
        self.last_s = 0
        self.accumulated_error = 0
        self.max_error = 0
        self.n_datapoints = 0
        self.lap_count = -1

        self.NUM_LAPS_ANALYSED = 10
        '''The number of laps to analyse and compute statistics for'''
        self.lap_time_acc = deque(maxlen=self.NUM_LAPS_ANALYSED)
        self.lat_err_acc = deque(maxlen=self.NUM_LAPS_ANALYSED)
        self.max_lat_err_acc = deque(maxlen=self.NUM_LAPS_ANALYSED)

        # unicorn-specific localization method param
        self.LOC_METHOD = self._get_param_default('loc_algo', 'slam')

        # Publish stuff to RViz
        self.lap_data_vis = self.create_publisher(Marker, 'lap_data_vis', 5)

        # New publisher for odom trajectory and other markers (unicorn-specific)
        self.lap_marker_pub = self.create_publisher(Marker, 'lap_marker', 5)

        # Open up logfile (in the source package's data/ dir, matching ROS1)
        pkg_root = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))
        data_path = os.path.join(pkg_root, 'data')
        self.get_logger().info(f"lap_analyser data dir: {data_path}")
        if not os.path.exists(data_path):
            os.makedirs(data_path)

        self.logfile_name = f"lap_analyzer_{datetime.now().strftime('%d%m_%H%M')}.txt"
        self.logfile_dir = os.path.join(data_path, self.logfile_name)
        with open(self.logfile_dir, 'w') as f:
            f.write("Laps done on " + datetime.now().strftime('%d %b %H:%M:%S') + '\n')

    def _get_param_default(self, name, default):
        try:
            val = self.get_parameter(name).value
            return val if val is not None else default
        except Exception:
            return default

    def marker_cb(self, data: Marker):
        self.state_marker = data
        # Latch the text-marker position once, from the first state marker (ROS1
        # did this via wait_for_message in __init__).
        if self.vis_pos is None:
            self.vis_pos = deepcopy(data.pose)
            self.vis_pos.position.z += 1.5  # appear on top of the state marker
            self.get_logger().info(
                f"LapAnalyser will be centered at {self.vis_pos.position.x}, "
                f"{self.vis_pos.position.y}, {self.vis_pos.position.z}")

    def waypoints_cb(self, data: WpntArray):
        """
        Callback function of /global_waypoints subscriber.

        Parameters
        ----------
        data
            Data received from /global_waypoints topic
        """
        if not self.wp_flag:
            # Store original waypoint array
            self.global_lateral_waypoints = np.array([
                [w.s_m, w.d_right, w.d_left] for w in data.wpnts
            ])
            self.wp_flag = True
        else:
            pass

    def odom_xy_cb(self, msg):
        # Callback for x,y coordinate odom; simply store the latest message
        self.latest_odom = msg

    def frenet_odom_cb(self, msg):
        if not self.wp_flag:
            self.get_logger().warn("frenet cb waiting for gb wpnts...", throttle_duration_sec=0.5)
            return

        current_s = msg.pose.pose.position.x
        current_d = msg.pose.pose.position.y
        if self.check_for_finish_line_pass(current_s):
            if (self.lap_count == -1):
                self.lap_start_time = self.get_clock().now()
                self.get_logger().info("LapAnalyser: started first lap")
                self.lap_count = 0
            else:
                self.lap_count += 1
                self.publish_lap_info()

                if self.lap_count >= 2:
                    self.publish_min_distance()
                    # Publish the stored odom trajectory as markers
                    self.publish_odom_marker()

                # Reset stored odom trajectory after publishing
                self.odom_points = []
                self.car_distance_to_boundary = []
                self.lap_start_time = self.get_clock().now()
                self.max_error = abs(current_d)
                self.accumulated_error = abs(current_d)
                self.n_datapoints = 1

                # Compute and publish statistics. Perhaps publish to a file?
                if self.lap_count > 0 and self.lap_count % self.NUM_LAPS_ANALYSED == 0:
                    lap_time_str = f"Lap time over the past {self.NUM_LAPS_ANALYSED} laps: Mean: {np.mean(self.lap_time_acc):.4f}, Std: {np.std(self.lap_time_acc):.4f}"
                    avg_err_str = f"Avg Lat Error over the past {self.NUM_LAPS_ANALYSED} laps: Mean: {np.mean(self.lat_err_acc):.4f}, Std: {np.std(self.lat_err_acc):.4f}"
                    max_err_str = f"Max Lat Error over the past {self.NUM_LAPS_ANALYSED} laps: Mean: {np.mean(self.max_lat_err_acc):.4f}, Std: {np.std(self.max_lat_err_acc):.4f}"
                    self.get_logger().warn(lap_time_str)
                    self.get_logger().warn(avg_err_str)
                    self.get_logger().warn(max_err_str)

                    with open(self.logfile_dir, 'a') as f:
                        f.write(lap_time_str + '\n')
                        f.write(avg_err_str + '\n')
                        f.write(max_err_str + '\n')
        else:
            self.accumulated_error += abs(current_d)
            self.n_datapoints += 1
            if self.max_error < abs(current_d):
                self.max_error = abs(current_d)
        self.last_s = current_s

        # search for closest s value: s values of global waypoints do not match the s values of car position exactly
        s_ref_line_values = np.array(self.global_lateral_waypoints)[:, 0]
        index_of_interest = np.argmin(np.abs(s_ref_line_values - current_s))  # index where s car state value is closest to s ref line

        d_right = self.global_lateral_waypoints[index_of_interest, 1]  # [w.s_m, w.d_right, w.d_left]
        d_left = self.global_lateral_waypoints[index_of_interest, 2]

        dist_to_bound = self.get_distance_to_boundary(current_d, d_left, d_right)
        self.car_distance_to_boundary.append(dist_to_bound)

        # Continuously store the current odom (x,y) along with d and dist_to_boundary if available
        if self.latest_odom is not None:
            x = self.latest_odom.pose.pose.position.x
            y = self.latest_odom.pose.pose.position.y
            self.odom_points.append({'x': x, 'y': y, 'd': current_d, 'dist_to_boundary': dist_to_bound})

    def start_log_cb(self, _):
        '''Start logging. Reset all metrics.'''
        self.get_logger().info(
            f"LapAnalyser: Start logging statistics for {self.NUM_LAPS_ANALYSED} laps.")
        self.accumulated_error = 0
        self.max_error = 0
        self.lap_count = -1
        self.n_datapoints = 0

    def check_for_finish_line_pass(self, current_s):
        # detect wrapping of the track, should happen exactly once per round
        if (self.last_s - current_s) > 1.0:
            return True
        else:
            return False

    def publish_lap_info(self):
        msg = LapData()
        lap_time = self.get_clock().now() - self.lap_start_time
        msg.lap_time = lap_time.nanoseconds / 1e9
        self.get_logger().info(
            f"LapAnalyser: completed lap #{self.lap_count} in {msg.lap_time}")

        with open(self.logfile_dir, 'a') as f:
            f.write(f"Lap #{self.lap_count}: {msg.lap_time:.4f}" + '\n')

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.lap_count = self.lap_count
        msg.average_lateral_error_to_global_waypoints = self.accumulated_error / self.n_datapoints
        msg.max_lateral_error_to_global_waypoints = self.max_error
        self.lap_data_pub.publish(msg)

        # append to deques for statistics
        self.lap_time_acc.append(msg.lap_time)
        self.lat_err_acc.append(msg.average_lateral_error_to_global_waypoints)
        self.max_lat_err_acc.append(msg.max_lateral_error_to_global_waypoints)

        # pitwall telemetry: record the lap metrics (to MCAP / Foxglove when a
        # recorder is up; no-op otherwise). Fires every lap, independent of the
        # RViz text marker below (which is skipped until /state_marker is seen).
        pitwall.log("lap/time", msg.lap_time)
        pitwall.log("lap/count", self.lap_count)
        pitwall.log("lap/avg_lateral_error", msg.average_lateral_error_to_global_waypoints)
        pitwall.log("lap/max_lateral_error", msg.max_lateral_error_to_global_waypoints)
        pitwall.event(f"lap {self.lap_count} completed: {msg.lap_time:.3f}s")

        mark = Marker()
        mark.header.stamp = self.get_clock().now().to_msg()
        mark.header.frame_id = 'map'
        mark.id = 0
        mark.ns = 'lap_info'
        mark.type = Marker.TEXT_VIEW_FACING
        mark.action = Marker.ADD
        if self.vis_pos is None:
            # /state_marker not seen yet -> skip this lap's text marker rather than
            # pinning it to the origin (it latches on the first /state_marker).
            self.get_logger().warn("vis_pos not set (no /state_marker yet); skipping lap text marker",
                                    throttle_duration_sec=5.0)
            return
        mark.pose = self.vis_pos
        mark.scale.x = 0.0
        mark.scale.y = 0.0
        mark.scale.z = 0.5  # Upper case A
        mark.color.a = 1.0
        mark.color.r = 0.2
        mark.color.g = 0.2
        mark.color.b = 0.2
        mark.text = f"Lap {self.lap_count:02d} {msg.lap_time:.3f}s"
        self.lap_data_vis.publish(mark)

    def publish_min_distance(self):
        min_msg = Float32()
        min_msg.data = float(np.min(self.car_distance_to_boundary))
        self.min_car_distance_to_boundary_pub.publish(min_msg)
        pitwall.log("lap/min_dist_to_boundary", min_msg.data)

    def publish_odom_marker(self):
        # Publish the trajectory marker for the current lap using stored odom points
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'lap_trajectory'
        marker.id = 10
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 1.0
        for pt in self.odom_points:
            p = Point()
            p.x = pt['x']
            p.y = pt['y']
            p.z = 0.0
            marker.points.append(p)
        self.lap_marker_pub.publish(marker)

        # Find and publish marker for min distance to boundary point
        if len(self.odom_points) > 0:
            min_idx = np.argmin([pt['dist_to_boundary'] for pt in self.odom_points])
            min_pt = self.odom_points[min_idx]
            marker_min = Marker()
            marker_min.header.stamp = self.get_clock().now().to_msg()
            marker_min.header.frame_id = 'map'
            marker_min.ns = 'lap_min_dist'
            marker_min.id = 11
            marker_min.type = Marker.SPHERE
            marker_min.action = Marker.ADD
            marker_min.pose.position.x = min_pt['x']
            marker_min.pose.position.y = min_pt['y']
            marker_min.pose.position.z = 0.0
            marker_min.scale.x = 0.3
            marker_min.scale.y = 0.3
            marker_min.scale.z = 0.3
            marker_min.color.r = 1.0
            marker_min.color.g = 0.0
            marker_min.color.b = 0.0
            marker_min.color.a = 1.0
            self.lap_marker_pub.publish(marker_min)

            # Publish text marker for min boundary point
            marker_min_text = Marker()
            marker_min_text.header.stamp = self.get_clock().now().to_msg()
            marker_min_text.header.frame_id = 'map'
            marker_min_text.ns = 'lap_min_dist_text'
            marker_min_text.id = 13
            marker_min_text.type = Marker.TEXT_VIEW_FACING
            marker_min_text.action = Marker.ADD
            # Position text slightly above the sphere marker
            marker_min_text.pose.position.x = min_pt['x']
            marker_min_text.pose.position.y = min_pt['y'] + 0.5
            marker_min_text.pose.position.z = 0.0
            marker_min_text.scale.z = 0.5  # text height
            marker_min_text.color.r = 0.2
            marker_min_text.color.g = 0.2
            marker_min_text.color.b = 0.2
            marker_min_text.color.a = 1.0
            marker_min_text.text = f"Min boundary: {min_pt['dist_to_boundary']:.2f}m"
            self.lap_marker_pub.publish(marker_min_text)

            # Find and publish marker for max lateral error (max abs(d)) point
            max_idx = np.argmax([abs(pt['d']) for pt in self.odom_points])
            max_pt = self.odom_points[max_idx]
            marker_max = Marker()
            marker_max.header.stamp = self.get_clock().now().to_msg()
            marker_max.header.frame_id = 'map'
            marker_max.ns = 'lap_max_d'
            marker_max.id = 12
            marker_max.type = Marker.SPHERE
            marker_max.action = Marker.ADD
            marker_max.pose.position.x = max_pt['x']
            marker_max.pose.position.y = max_pt['y']
            marker_max.pose.position.z = 0.0
            marker_max.scale.x = 0.3
            marker_max.scale.y = 0.3
            marker_max.scale.z = 0.3
            marker_max.color.r = 0.0
            marker_max.color.g = 1.0
            marker_max.color.b = 0.0
            marker_max.color.a = 1.0
            self.lap_marker_pub.publish(marker_max)

            # Publish text marker for max lateral error point
            marker_max_text = Marker()
            marker_max_text.header.stamp = self.get_clock().now().to_msg()
            marker_max_text.header.frame_id = 'map'
            marker_max_text.ns = 'lap_max_d_text'
            marker_max_text.id = 14
            marker_max_text.type = Marker.TEXT_VIEW_FACING
            marker_max_text.action = Marker.ADD
            marker_max_text.pose.position.x = max_pt['x']
            marker_max_text.pose.position.y = max_pt['y'] + 0.5
            marker_max_text.pose.position.z = 0.0
            marker_max_text.scale.z = 0.5
            marker_max_text.color.r = 0.2
            marker_max_text.color.g = 0.2
            marker_max_text.color.b = 0.2
            marker_max_text.color.a = 1.0
            marker_max_text.text = f"Max d: {max_pt['d']:.2f}m"
            self.lap_marker_pub.publish(marker_max_text)

    def get_distance_to_boundary(self, current_d, d_left, d_right):
        """
        ----------
        Input:
            current_d: lateral distance to reference line
            d_left: distance from ref. line to left track boundary
            d_right: distance from ref. line to right track boundary
        Output:
            distance: critical distance to track boundary (whichever is smaller, to the right or left)
        """
        # calculate distance from car to boundary
        car_dist_to_bound_left = d_left - current_d
        car_dist_to_bound_right = d_right + current_d

        # select whichever distance is smaller (to the right or left)
        if car_dist_to_bound_left > car_dist_to_bound_right:  # car is closer to right boundary
            return car_dist_to_bound_right
        else:
            return car_dist_to_bound_left


def main():
    rclpy.init()
    lap_analyser = LapAnalyser()
    rclpy.spin(lap_analyser)
    lap_analyser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
