#!/usr/bin/env python3
import time
from copy import deepcopy
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import (
    FloatingPointRange,
    ParameterDescriptor,
    ParameterType,
    SetParametersResult,
)
from rclpy.parameter import Parameter

from nav_msgs.msg import Odometry
from f110_msgs.msg import (
    Wpnt,
    WpntArray,
    Obstacle,
    ObstacleArray,
    OTWpntArray,
    OpponentTrajectory,
    OppWpnt,
    BehaviorStrategy,
    PredictionArray,
)
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Float32, Bool, Header

from scipy.optimize import minimize
from scipy.interpolate import BPoly
from frenet_conversion.frenet_converter import FrenetConverter

from ccma import CCMA
import trajectory_planning_helpers as tph
from transforms3d.euler import quat2euler
from grid_filter.grid_filter import GridFilter
import matplotlib.pyplot as plt


def euler_from_quaternion(quaternion):
    """quaternion is [x, y, z, w]; returns (roll, pitch, yaw)."""
    x, y, z, w = quaternion
    return quat2euler([w, x, y, z])


class ChangeAvoidanceNode(Node):
    def __init__(self):
        # Initialize node
        super().__init__('change_avoidance_node')

        # Params
        self.local_wpnts = None
        self.lookahead = 15
        self.past_avoidance_d = []

        # Scaled waypoints params
        self.scaled_wpnts = None
        self.scaled_wpnts_msg = WpntArray()
        self.scaled_vmax = None
        self.scaled_max_idx = None
        self.scaled_max_s = None
        self.scaled_delta_s = None

        self.center_wpnts_msg = WpntArray()
        self.outer_lane_wpnts_msg = WpntArray()
        self.inner_lane_wpnts_msg = WpntArray()
        self.center_wpnts_received = False

        # Updated waypoints params
        self.wpnts_updated = None
        self.max_s_updated = None
        self.max_idx_updated = None

        # Obstalces params
        self.obs = ObstacleArray()
        self.obs_perception = ObstacleArray()
        self.obs_prediction = ObstacleArray()
        self.obs_prediction_pred = PredictionArray()

        # Opponent waypoint params
        self.opponent_waypoints = OpponentTrajectory()
        self.max_opp_idx = None
        self.opponent_wpnts_sm = None

        # OT params
        self.last_ot_side = ""
        self.ot_section_check = False

        # Solver params
        self.min_radius = 0.55  # wheelbase / np.tan(max_steering)
        self.max_kappa = 1 / self.min_radius
        self.width_car = 0.30
        self.safety_margin = 0.1
        self.avoidance_resolution = 20
        self.back_to_raceline_before = 5
        self.back_to_raceline_after = 5
        self.obs_traj_tresh = 2

        # Dynamic sovler params
        self.down_sampled_delta_s = 0.1
        self.global_traj_kappas = None

        # State variables (filled by callbacks)
        self.current_s = None
        self.current_d = None
        self.current_vs = None
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.current_vx = None
        self.local_wpnts_msg = None
        self.ego_prediction = None

        # Dynamic reconf params (defaults from cfg/dyn_change_tuner.cfg)
        self.evasion_dist = 0.3
        self.obs_traj_tresh = 1.0
        self.spline_bound_mindist = 0.3

        # Dynamic reconf params
        self.avoid_static_obs = True

        self.converter = None
        self.global_waypoints = None
        self.gb_max_idx = None
        self.gb_max_s = None

        # CCMA init
        self.ccma = CCMA(w_ma=10, w_cc=5)

        # ROS Parameters
        self.declare_parameter('measure', False,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.measure = self.get_parameter('measure').get_parameter_value().bool_value

        # Show the matplotlib lane-visualization window in generate_lanes().
        # Default OFF: it blocks on plt.show() at startup and pops a GUI window.
        self.declare_parameter('vis', False,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.vis = self.get_parameter('vis').get_parameter_value().bool_value

        # Dynamic reconfigure -> declared parameters with FloatingPointRange
        param_dicts = [
            {
                'name': 'evasion_dist',
                'default': self.evasion_dist,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Orthogonal distance of the apex to the obstacle",
                    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.25, step=0.001)],
                ),
            },
            {
                'name': 'obs_traj_tresh',
                'default': self.obs_traj_tresh,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Threshold of the obstacle towards raceline to be considered for evasion",
                    floating_point_range=[FloatingPointRange(from_value=0.1, to_value=2.0, step=0.001)],
                ),
            },
            {
                'name': 'spline_bound_mindist',
                'default': self.spline_bound_mindist,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Splines may never be closer to the track bounds than this param in meters",
                    floating_point_range=[FloatingPointRange(from_value=0.05, to_value=1.0, step=0.001)],
                ),
            },
        ]
        self.declare_all_parameters(param_dicts=param_dicts)
        self.add_on_set_parameters_callback(self.dyn_param_cb)

        # Publishers
        self.mrks_pub = self.create_publisher(MarkerArray, "/planner/avoidance/markers_sqp", QoSProfile(depth=10))
        self.evasion_pub = self.create_publisher(OTWpntArray, "/planner/avoidance/otwpnts", QoSProfile(depth=10))
        self.merger_pub = self.create_publisher(Float32MultiArray, "/planner/avoidance/merger", QoSProfile(depth=10))
        if self.measure:
            self.measure_pub = self.create_publisher(Float32, "/planner/pspliner_sqp/latency", QoSProfile(depth=10))

        self.spline_sample_pub = self.create_publisher(MarkerArray, "/spline_sample_points", QoSProfile(depth=10))

        # Subscribers
        self.create_subscription(ObstacleArray, "/tracking/obstacles", self.obs_perception_cb, QoSProfile(depth=10))
        self.create_subscription(ObstacleArray, "/opponent_prediction/obstacles", self.obs_prediction_cb, QoSProfile(depth=10))
        self.create_subscription(PredictionArray, "/opponent_prediction/obstacles_pred", self.obstacle_prediction_cb, QoSProfile(depth=10))
        self.create_subscription(PredictionArray, "/mpc_controller/ego_prediction", self.ego_prediction_cb, QoSProfile(depth=10))
        self.create_subscription(Odometry, "/car_state/odom_frenet", self.state_frenet_cb, QoSProfile(depth=10))
        self.create_subscription(Odometry, "/car_state/odom", self.state_cartesian_cb, QoSProfile(depth=10))
        self.create_subscription(BehaviorStrategy, "/behavior_strategy", self.behavior_cb, QoSProfile(depth=10))
        self.create_subscription(WpntArray, "/global_waypoints", self.gb_cb, QoSProfile(depth=10))
        self.create_subscription(WpntArray, "/global_waypoints_scaled", self.scaled_wpnts_cb, QoSProfile(depth=10))
        self.create_subscription(WpntArray, "/global_waypoints_updated", self.updated_wpnts_cb, QoSProfile(depth=10))
        self.create_subscription(WpntArray, "/centerline_waypoints", self.center_wpnts_cb, QoSProfile(depth=10))
        self.create_subscription(OpponentTrajectory, "/opponent_trajectory", self.opponent_trajectory_cb, QoSProfile(depth=10))
        self.create_subscription(Bool, "/ot_section_check", self.ot_sections_check_cb, QoSProfile(depth=10))
        self.create_subscription(Marker, "/lookahead_point", self.lookahead_callback, QoSProfile(depth=10))

        self.converter = self.initialize_converter()

        self.map_filter = GridFilter(node=self, map_topic="/map", debug=False)
        self.map_filter.set_erosion_kernel_size(7)

        # Wait for the centerline waypoints, then generate the inner/outer lanes
        self.wait_for_message_attr('center_wpnts_received')
        self.generate_lanes(center_wpnts=self.center_wpnts_msg)

        self.num_extra_points = 100
        self.num_extra_global_points = 20

        self.lookahead_point_x = None
        self.lookahead_point_y = None

        # Wait for critical messages and services
        self.get_logger().info("[OBS Spliner] Waiting for messages and services...")
        self.wait_for_loop_messages()
        self.get_logger().info("[OBS Spliner] Ready!")

        # Main loop timer at 20 Hz
        self.create_timer(1.0 / 20.0, self.loop)

    #################### DYNAMIC PARAMS ####################
    def declare_all_parameters(self, param_dicts: List[dict]):
        params = []
        for param_dict in param_dicts:
            param = self.declare_parameter(
                param_dict['name'], param_dict['default'], param_dict['descriptor'])
            params.append(param)
        return params

    def dyn_param_cb(self, params: List[Parameter]):
        for param in params:
            if param.name == 'evasion_dist':
                self.evasion_dist = param.value
            elif param.name == 'obs_traj_tresh':
                self.obs_traj_tresh = param.value
            elif param.name == 'spline_bound_mindist':
                self.spline_bound_mindist = param.value

        self.get_logger().info(
            f"[Planner] Dynamic reconf triggered new spline params: \n"
            f" Evasion apex distance: {self.evasion_dist} [m],\n"
            f" Obstacle trajectory treshold: {self.obs_traj_tresh} [m]\n"
            f" Spline boundary mindist: {self.spline_bound_mindist} [m]\n"
        )
        return SetParametersResult(successful=True)

    ### Callbacks ###
    def obs_perception_cb(self, data: ObstacleArray):
        self.obs_perception = data
        self.obs_perception.obstacles = [obs for obs in data.obstacles if obs.is_static == False]

    def obs_prediction_cb(self, data: ObstacleArray):
        self.obs_prediction = data

    def obstacle_prediction_cb(self, data: PredictionArray):
        self.obs_prediction_pred = data

    def ego_prediction_cb(self, data: PredictionArray):
        self.ego_prediction = data

    def state_frenet_cb(self, data: Odometry):
        self.current_s = data.pose.pose.position.x
        self.current_d = data.pose.pose.position.y
        self.current_vs = data.twist.twist.linear.x

    def state_cartesian_cb(self, data: Odometry):
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y
        quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        self.current_roll, self.current_pitch, self.current_yaw = euler_from_quaternion(quaternion)
        self.current_vx = data.twist.twist.linear.x

    def gb_cb(self, data: WpntArray):
        self.global_waypoints = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in data.wpnts])
        self.gb_max_idx = data.wpnts[-1].id
        self.gb_max_s = data.wpnts[-1].s_m

    def scaled_wpnts_cb(self, data: WpntArray):
        self.scaled_wpnts = np.array([[wpnt.s_m, wpnt.d_m] for wpnt in data.wpnts])
        self.scaled_wpnts_msg = data
        v_max = np.max(np.array([wpnt.vx_mps for wpnt in data.wpnts]))
        if self.scaled_vmax != v_max:
            self.scaled_vmax = v_max
            self.scaled_max_idx = data.wpnts[-1].id
            self.scaled_max_s = data.wpnts[-1].s_m
            self.scaled_delta_s = data.wpnts[1].s_m - data.wpnts[0].s_m

    def updated_wpnts_cb(self, data: WpntArray):
        self.wpnts_updated = data.wpnts[:-1]
        self.max_s_updated = self.wpnts_updated[-1].s_m
        self.max_idx_updated = self.wpnts_updated[-1].id

    def center_wpnts_cb(self, data: WpntArray):
        self.center_wpnts_msg = data
        self.center_wpnts_max_s = data.wpnts[-1].s_m
        self.center_wpnts_max_idx = data.wpnts[-1].id
        self.center_wpnts_received = True

    def behavior_cb(self, data: BehaviorStrategy):
        self.local_wpnts_msg = data
        self.local_wpnts = np.array([[wpnt.s_m, wpnt.d_m] for wpnt in data.local_wpnts])

    def opponent_trajectory_cb(self, data: OpponentTrajectory):
        self.opponent_waypoints = data.oppwpnts
        self.max_opp_idx = len(data.oppwpnts) - 1
        self.opponent_wpnts_sm = np.array([wpnt.s_m for wpnt in data.oppwpnts])

    def ot_sections_check_cb(self, data: Bool):
        self.ot_section_check = data.data

    def lookahead_callback(self, data):
        if data.id == 100:
            self.lookahead_point_x = data.pose.position.x
            self.lookahead_point_y = data.pose.position.y

    ### Common Functions ###
    def wait_for_message_attr(self, attr_name: str):
        """Spin until the named instance attribute has been set by a callback."""
        while not getattr(self, attr_name, False):
            rclpy.spin_once(self)

    def wait_for_loop_messages(self):
        """Spin until all critical messages have been received before starting the loop."""
        while (self.scaled_wpnts is None or self.current_x is None
               or self.local_wpnts is None):
            rclpy.spin_once(self)

    def initialize_converter(self) -> "FrenetConverter":
        """
        Initialize the FrenetConverter object"""
        # Wait for the global waypoints to arrive
        while self.global_waypoints is None:
            rclpy.spin_once(self)

        # Initialize the FrenetConverter object
        converter = FrenetConverter(self.global_waypoints[:, 0], self.global_waypoints[:, 1])
        self.get_logger().info("[Spliner] initialized FrenetConverter object")

        return converter

    def obstacle_preprocessing(self, obs: ObstacleArray):
        obs.obstacles = sorted(obs.obstacles, key=lambda obs: obs.s_start)

        considered_obs = []
        for obs in obs.obstacles:
            if (obs.s_start - self.current_s) % self.scaled_max_s < self.lookahead and abs(obs.d_center - self.current_d) < self.obs_traj_tresh:
                if False and obs.id == self.obs_prediction_pred.id and len(self.obs_prediction.obstacles) == 20:
                    for opd in self.obs_prediction.obstacles:
                        considered_obs.append(opd)
                else:
                    considered_obs.append(obs)

        return considered_obs

    def more_space(self, obstacle: Obstacle, gb_wpnts, gb_idxs):
        left_gap = abs(gb_wpnts[gb_idxs[0]].d_left - obstacle.d_left)
        right_gap = abs(gb_wpnts[gb_idxs[0]].d_right + obstacle.d_right)
        min_space = self.spline_bound_mindist + self.width_car / 2 + self.safety_margin

        if right_gap > min_space and left_gap < min_space:
            # Compute apex distance to the right of the opponent
            d_apex_right = obstacle.d_right - (self.width_car / 2 + self.safety_margin + 0.2)
            # If we overtake to the right of the opponent BUT the apex is to the left of the raceline, then we set the apex to 0
            if d_apex_right > 0 and right_gap < abs(d_apex_right):
                d_apex_right = 0
            return "right", d_apex_right, left_gap, right_gap

        elif left_gap > min_space and right_gap < min_space:
            # Compute apex distance to the left of the opponent
            d_apex_left = obstacle.d_left + (self.width_car / 2 + self.safety_margin + 0.2)
            # If we overtake to the left of the opponent BUT the apex is to the right of the raceline, then we set the apex to 0
            if d_apex_left < 0 and left_gap < abs(d_apex_left):
                d_apex_left = 0
            return "left", d_apex_left, left_gap, right_gap
        elif left_gap < min_space and right_gap < min_space:
            # self.get_logger().warn("No enough gap!")
            return None, 0.0, left_gap, right_gap
        else:
            # self.get_logger().warn("This happen!")
            return None, 0.0, left_gap, right_gap

    ### Visualize SPL Rviz ###
    def visualize_dynamic_spliner(self, evasion_s, evasion_d, evasion_x, evasion_y, evasion_v):
        mrks = MarkerArray()
        if len(evasion_s) == 0:
            pass
        else:
            resp = self.converter.get_cartesian(evasion_s, evasion_d)
            for i in range(len(evasion_s)):
                mrk = Marker(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"))
                mrk.type = mrk.CYLINDER
                mrk.scale.x = 0.1
                mrk.scale.y = 0.1
                mrk.scale.z = evasion_v[i] / self.scaled_vmax
                mrk.color.a = 1.0
                mrk.color.g = 0.13
                mrk.color.r = 0.63
                mrk.color.b = 0.94

                mrk.id = i
                mrk.pose.position.x = evasion_x[i]
                mrk.pose.position.y = evasion_y[i]
                mrk.pose.position.z = evasion_v[i] / self.scaled_vmax / 2
                mrk.pose.orientation.w = 1.0
                mrks.markers.append(mrk)
            self.mrks_pub.publish(mrks)

    def visualize_spline_samples(self, x_vals, y_vals):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(zip(x_vals, y_vals)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "spline_samples"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1  # optional: small height
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.2
            marker.color.g = 0.8
            marker.color.b = 0.2
            marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()  # stays for 0.2 sec
            marker_array.markers.append(marker)

        self.spline_sample_pub.publish(marker_array)

    ### Lane Change Avoidance ###
    def lane_change(self, considered_obs: list, cur_s: float):
        # Get the initial guess of the overtaking side
        initial_guess_object_start_idx = np.abs(self.scaled_wpnts - considered_obs[0].s_start).argmin()
        initial_guess_object_end_idx = np.abs(self.scaled_wpnts - considered_obs[-1].s_end).argmin()

        # Get array of indexes of the global waypoints overlapping with the ROC
        gb_idxs = np.array(range(initial_guess_object_start_idx, initial_guess_object_start_idx + (initial_guess_object_end_idx - initial_guess_object_start_idx) % self.scaled_max_idx)) % self.scaled_max_idx

        if len(gb_idxs) < 20:
            gb_idxs = [int(considered_obs[0].s_center / self.scaled_delta_s + i) % self.scaled_max_idx for i in range(20)]

        side, initial_apex, left_gap, right_gap = self.more_space(considered_obs[0], self.scaled_wpnts_msg.wpnts, gb_idxs)
        kappas = np.array([self.scaled_wpnts_msg.wpnts[gb_idx].kappa_radpm for gb_idx in gb_idxs])
        max_kappa = np.max(np.abs(kappas))
        outside = "left" if np.sum(kappas) < 0 else "right"

        # Enlongate the ROC if our initial guess suggests that we are overtaking on the outside
        if side == outside:
            for i in range(len(considered_obs)):
                considered_obs[i].s_end = considered_obs[i].s_end + (considered_obs[i].s_end - considered_obs[i].s_start) % self.max_s_updated * max_kappa * (self.width_car + self.evasion_dist)

        min_s_obs_start = self.scaled_max_s
        max_s_obs_end = 0

        for obs in considered_obs:
            if obs.s_start < min_s_obs_start:
                min_s_obs_start = obs.s_start
            if obs.s_end > max_s_obs_end:
                max_s_obs_end = obs.s_end

        # Get local waypoints to check where we are and where we are heading
        # If we are closer than threshold to the opponent use the first two local waypoints as start points
        start_avoidance = min_s_obs_start
        end_avoidance = max_s_obs_end

        # Get a downsampled version for s avoidance points
        s_avoidance = np.arange(start_avoidance, end_avoidance + self.down_sampled_delta_s, self.down_sampled_delta_s)

        if s_avoidance.size == 0:
            self.get_logger().warn("s_avoidance is empty! Skipping avoidance.")
            return [], [], [], [], []

        # Get the closest scaled waypoint for every s avoidance point (down sampled)
        scaled_wpnts_indices = np.array([np.abs(self.scaled_wpnts[:, 0] - s % self.scaled_max_s).argmin() for s in s_avoidance])

        # Get the min radius
        # # Clip speed
        clipped_speed = np.clip(self.current_vs, 1.0, a_max=None)
        # Get the minimum of clipped speed and the updated speed of the first waypoints
        radius_speed = min([clipped_speed, self.wpnts_updated[(scaled_wpnts_indices[0]) % self.max_idx_updated].vx_mps])
        # Interpolate the min_radius with speeds between 0.2 and 7 m
        self.min_radius = np.interp(radius_speed, [1, 6, 7], [0.2, 2, 4])
        self.max_kappa = 1 / self.min_radius

        initial_guess = np.zeros(len(s_avoidance))
        max_obs_idx_center = 0
        left_count = 0
        right_count = 0
        left_gap_sum = 0
        right_gap_sum = 0
        for obs in considered_obs:
            side, apex, left_gap, right_gap = self.more_space(obs, self.scaled_wpnts_msg.wpnts, gb_idxs)
            left_gap_sum += left_gap
            right_gap_sum += right_gap
            if side is None:
                continue
            elif side == "left":
                left_count += 1
            elif side == "right":
                right_count += 1
            obs_idx_start = np.abs(s_avoidance - obs.s_start).argmin()
            obs_idx_center = np.abs(s_avoidance - obs.s_center).argmin()
            obs_idx_end = np.abs(s_avoidance - obs.s_end).argmin()
            if obs_idx_start >= obs_idx_end or obs_idx_end >= len(s_avoidance):
                continue

            initial_guess[obs_idx_center] = apex

            if obs_idx_center > max_obs_idx_center:
                max_obs_idx_center = obs_idx_center

        left_gap_avg = left_gap_sum / left_count if left_count > 0 else 0
        right_gap_avg = right_gap_sum / right_count if right_count > 0 else 0

        if left_count > right_count and left_gap_avg > right_gap_avg and left_gap_avg > self.width_car:
            preferred_side = "left"
        elif right_count > left_count and right_gap_avg > left_gap_avg and right_gap_avg > self.width_car:
            preferred_side = "right"
        else:
            return [], [], [], [], []

        # ---- Monotonic-s sampling (SQP-style): define an increasing raceline s up front, then
        # ---- pull the target lane's d at each s, ease in/out around the obstacle span, and convert
        # ---- to cartesian. s is monotonic by construction so the published path can never reverse.
        target_wpnts = self.inner_lane_wpnts_msg.wpnts if preferred_side == "left" else self.outer_lane_wpnts_msg.wpnts

        # Obstacle span unwrapped relative to the car so it stays monotonic across the s=0 seam.
        start_s = self.current_s
        obs_start_u = obs_start = min(o.s_start for o in considered_obs)
        obs_end_u = max(o.s_end for o in considered_obs)
        if obs_start_u < start_s:
            obs_start_u += self.scaled_max_s
        while obs_end_u < obs_start_u:
            obs_end_u += self.scaled_max_s
        end_s = obs_end_u + self.back_to_raceline_after

        n_samples = max(int((end_s - start_s) / self.scaled_delta_s), 5)
        s_lin = np.linspace(start_s, end_s, n_samples)            # monotonic, unwrapped

        # Target lane d along s (full avoidance offset); raceline d = 0.
        lane_d = self._lane_d_at_s(target_wpnts, s_lin)

        # Cosine ease: 0 (raceline) before the obstacle, ramp to lane_d across the obstacle, ramp back.
        ramp = self.back_to_raceline_before
        d_arr = np.zeros_like(s_lin)
        for i, s in enumerate(s_lin):
            if s < obs_start_u - ramp or s > obs_end_u + ramp:
                w = 0.0
            elif s < obs_start_u:
                w = 0.5 * (1 - np.cos(np.pi * (s - (obs_start_u - ramp)) / ramp))
            elif s > obs_end_u:
                w = 0.5 * (1 + np.cos(np.pi * (s - obs_end_u) / ramp))
            else:
                w = 1.0
            d_arr[i] = w * lane_d[i]

        s_wrapped = s_lin % self.scaled_max_s
        resp = self.converter.get_cartesian(s_wrapped, d_arr)
        resp = resp.T if resp.ndim == 2 else resp
        samples = np.asarray(resp, dtype=float).reshape(-1, 2)
        if samples.shape[0] < 3 or not np.all(np.isfinite(samples)):
            return [], [], [], [], []

        for i in range(samples.shape[0]):
            inside = self.map_filter.is_point_inside(samples[i, 0], samples[i, 1])

            if not inside:
                evasion_x = []
                evasion_y = []
                evasion_s = []
                evasion_d = []
                evasion_v = []
                return evasion_x, evasion_y, evasion_s, evasion_d, evasion_v

        self.visualize_spline_samples(samples[:, 0], samples[:, 1])

        smoothed_xy_points = self.ccma.filter(samples)
        smoothed_sd_points = self.converter.get_frenet(smoothed_xy_points[:, 0], smoothed_xy_points[:, 1])
        # samples are already s-monotonic; keep x/y and s/d in that order (no sort) and wrap s.
        evasion_x = np.asarray(smoothed_xy_points[:, 0])
        evasion_y = np.asarray(smoothed_xy_points[:, 1])
        evasion_s = np.asarray(smoothed_sd_points[0]) % self.scaled_max_s
        evasion_d = np.asarray(smoothed_sd_points[1])
        evasion_coords = np.column_stack((evasion_x, evasion_y))

        # Guard: drop coincident points (zero-length segments -> NaN psi/kappa/velocity).
        if len(evasion_coords) >= 2:
            seg = np.linalg.norm(np.diff(evasion_coords, axis=0), axis=1)
            keep = np.concatenate([[True], seg > 1e-6])
            if not np.all(keep):
                evasion_x = evasion_x[keep]
                evasion_y = evasion_y[keep]
                evasion_s = evasion_s[keep]
                evasion_d = evasion_d[keep]
                evasion_coords = evasion_coords[keep]
        if len(evasion_coords) < 3 or not np.all(np.isfinite(evasion_coords)):
            return [], [], [], [], []

        evasion_psi, evasion_kappa = tph.calc_head_curv_num.calc_head_curv_num(
            path=evasion_coords,
            el_lengths=0.1 * np.ones(len(evasion_coords) - 1),
            is_closed=False
        )
        evasion_psi += np.pi / 2
        evasion_v = np.zeros(len(evasion_s))

        temp_idx = np.argmin([abs(evs - self.current_s) for evs in evasion_s])
        if abs(self.current_d - evasion_d[temp_idx]) > 0.5:
            # print(abs(self.current_d - evasion_d[temp_idx]))
            evasion_x = []
            evasion_y = []
            evasion_s = []
            evasion_d = []
            evasion_v = []
            return evasion_x, evasion_y, evasion_s, evasion_d, evasion_v

        # Create a new evasion waypoint message
        evasion_wpnts_msg = OTWpntArray(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"))
        evasion_wpnts = []
        evasion_wpnts = [Wpnt(id=len(evasion_wpnts), s_m=s, d_m=d, x_m=x, y_m=y, psi_rad=p, kappa_radpm=k, vx_mps=v) for x, y, s, d, p, k, v in zip(evasion_x, evasion_y, evasion_s, evasion_d, evasion_psi, evasion_kappa, evasion_v)]
        evasion_wpnts_msg.wpnts = evasion_wpnts
        # self.past_avoidance_d = initial_guess
        mean_d = np.mean(evasion_d)
        if mean_d > 0:
            self.last_ot_side = "left"
        else:
            self.last_ot_side = "right"

        self.evasion_pub.publish(evasion_wpnts_msg)
        self.visualize_dynamic_spliner(evasion_s, evasion_d, evasion_x, evasion_y, evasion_v)

        return evasion_x, evasion_y, evasion_s, evasion_d, evasion_v

    def resample_lane(self, xy: np.ndarray, resolution: float = 0.1) -> np.ndarray:
        deltas = np.diff(xy, axis=0)
        dists = np.hypot(deltas[:, 0], deltas[:, 1])
        s = np.concatenate([[0], np.cumsum(dists)])
        s_new = np.arange(0, s[-1], resolution)
        x_new = np.interp(s_new, s, xy[:, 0])
        y_new = np.interp(s_new, s, xy[:, 1])
        return np.stack([x_new, y_new], axis=1)

    def _lane_d_at_s(self, lane_wpnts, s_query):
        # Interpolate a lane's d over raceline s. Lane s wraps [0, scaled_max_s); duplicate one
        # lap ahead so an UNWRAPPED s_query (can exceed scaled_max_s) still interpolates cleanly.
        ls = np.array([w.s_m for w in lane_wpnts])
        ld = np.array([w.d_m for w in lane_wpnts])
        order = np.argsort(ls)
        ls, ld = ls[order], ld[order]
        ls_ext = np.concatenate([ls, ls + self.scaled_max_s])
        ld_ext = np.concatenate([ld, ld])
        return np.interp(s_query, ls_ext, ld_ext)

    def generate_lanes(self, center_wpnts: WpntArray):
        original_center_wpnts = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in center_wpnts.wpnts])
        original_center_psi = np.array([wpnt.psi_rad for wpnt in center_wpnts.wpnts])

        min_center_left_gap = np.min([wpnt.d_left for wpnt in center_wpnts.wpnts])
        min_center_right_gap = np.min([wpnt.d_right for wpnt in center_wpnts.wpnts])

        # Map boundary min value show
        # print(f"min_center_left_gap: {min_center_left_gap}, min_center_right_gap: {min_center_right_gap}")

        normals = np.stack([np.sin(original_center_psi), -np.cos(original_center_psi)], axis=1)

        # 1. Map boundary min value
        # outer_lane = original_center_wpnts + normals * min_center_left_gap
        # inner_lane = original_center_wpnts - normals * min_center_right_gap

        # 2. Constant othogonal evasion
        outer_lane = original_center_wpnts + normals * 0.35
        inner_lane = original_center_wpnts - normals * 0.35

        outer_lane_resampled = self.resample_lane(outer_lane, resolution=0.1)
        inner_lane_resampled = self.resample_lane(inner_lane, resolution=0.1)

        outer_s, outer_d = self.converter.get_frenet(outer_lane_resampled[:, 0], outer_lane_resampled[:, 1])
        inner_s, inner_d = self.converter.get_frenet(inner_lane_resampled[:, 0], inner_lane_resampled[:, 1])

        outer_lane_resampled_psi, outer_lane_resampled_kappa = tph.calc_head_curv_num.calc_head_curv_num(
                path=outer_lane_resampled,
                el_lengths=0.1 * np.ones(len(outer_lane_resampled) - 1),
                is_closed=False,
                stepsize_curv_preview=5.0,
                stepsize_curv_review=5.0
            )
        outer_lane_resampled_psi += np.pi / 2

        inner_lane_resampled_psi, inner_lane_resampled_kappa = tph.calc_head_curv_num.calc_head_curv_num(
                path=inner_lane_resampled,
                el_lengths=0.1 * np.ones(len(inner_lane_resampled) - 1),
                is_closed=False,
                stepsize_curv_preview=5.0,
                stepsize_curv_review=5.0
            )
        inner_lane_resampled_psi += np.pi / 2

        self.outer_lane_wpnts_msg.header.stamp = self.get_clock().now().to_msg()
        self.inner_lane_wpnts_msg.header.stamp = self.get_clock().now().to_msg()

        self.outer_lane_wpnts_msg.header.frame_id = "map"
        self.inner_lane_wpnts_msg.header.frame_id = "map"

        for i in range(len(outer_lane_resampled)):
            wpnt = Wpnt()
            wpnt.id = i
            wpnt.x_m = outer_lane_resampled[i, 0]
            wpnt.y_m = outer_lane_resampled[i, 1]
            wpnt.s_m = outer_s[i]
            wpnt.d_m = outer_d[i]
            wpnt.psi_rad = outer_lane_resampled_psi[i]
            wpnt.kappa_radpm = outer_lane_resampled_kappa[i]
            wpnt.d_left = 0.0
            wpnt.d_right = 0.0
            wpnt.vx_mps = 0.0
            wpnt.ax_mps2 = 0.0
            self.outer_lane_wpnts_msg.wpnts.append(wpnt)

        for i in range(len(inner_lane_resampled)):
            wpnt = Wpnt()
            wpnt.id = i
            wpnt.x_m = inner_lane_resampled[i, 0]
            wpnt.y_m = inner_lane_resampled[i, 1]
            wpnt.s_m = inner_s[i]
            wpnt.d_m = inner_d[i]
            wpnt.psi_rad = inner_lane_resampled_psi[i]
            wpnt.kappa_radpm = inner_lane_resampled_kappa[i]
            wpnt.d_left = 0.0
            wpnt.d_right = 0.0
            wpnt.vx_mps = 0.0
            wpnt.ax_mps2 = 0.0
            self.inner_lane_wpnts_msg.wpnts.append(wpnt)

        # ------------- Plot (only when vis:=true) -------------
        if self.vis:
            plt.figure(figsize=(8, 6))
            plt.plot(original_center_wpnts[:, 0], original_center_wpnts[:, 1], 'k-', label='Center Line')
            plt.plot(outer_lane_resampled[:, 0], outer_lane_resampled[:, 1], 'g--', label='Outer Lane')
            plt.plot(inner_lane_resampled[:, 0], inner_lane_resampled[:, 1], 'b--', label='Inner Lane')

            plt.scatter(original_center_wpnts[:, 0], original_center_wpnts[:, 1], c='k', s=10)
            plt.scatter(outer_lane_resampled[:, 0], outer_lane_resampled[:, 1], c='g', s=10)
            plt.scatter(inner_lane_resampled[:, 0], inner_lane_resampled[:, 1], c='b', s=10)

            plt.axis('equal')
            plt.xlabel('X [m]')
            plt.ylabel('Y [m]')
            plt.title('Lane Visualization')
            plt.legend()
            plt.grid(True)
            plt.show()
        # ------------- Plot -------------

    ### Main Loop ###
    def loop(self):
        start_time = time.perf_counter()

        # Obstacle pre-processing
        obs = deepcopy(self.obs_perception)
        considered_obs = self.obstacle_preprocessing(obs=obs)

        # If there is an obstacle and we are in OT section
        # if len(considered_obs) > 0 and self.ot_section_check == True:
        if len(considered_obs) > 0:
            evasion_x, evasion_y, evasion_s, evasion_d, evasion_v = self.lane_change(considered_obs, self.current_s)
            # Publish merge reagion if evasion track has been found
            if len(evasion_s) > 0:
                self.merger_pub.publish(Float32MultiArray(data=[considered_obs[-1].s_end % self.scaled_max_s, evasion_s[-1] % self.scaled_max_s]))
        # If there is no point in overtaking anymore delte all markers
        else:
            mrks = MarkerArray()
            del_mrk = Marker(header=Header(stamp=self.get_clock().now().to_msg()))
            del_mrk.action = Marker.DELETEALL
            mrks.markers = []
            mrks.markers.append(del_mrk)
            self.mrks_pub.publish(mrks)

        # publish latency
        if self.measure:
            self.measure_pub.publish(Float32(data=time.perf_counter() - start_time))


def main(args=None):
    rclpy.init(args=args)
    node = ChangeAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
