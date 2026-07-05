#!/usr/bin/env python3
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor, SetParametersResult, ParameterType
from rclpy.parameter import Parameter

import numpy as np
from nav_msgs.msg import Odometry
from f110_msgs.msg import Wpnt, WpntArray, Obstacle, ObstacleArray, OTWpntArray, OpponentTrajectory, OppWpnt, BehaviorStrategy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float32MultiArray, Float32, Header
from scipy.optimize import minimize
from frenet_conversion.frenet_converter import FrenetConverter
from std_msgs.msg import Bool
from copy import deepcopy

from ccma import CCMA
# NumPy 2.0 removed the np.row_stack alias, but the ccma library still calls it.
if not hasattr(np, "row_stack"):
    np.row_stack = np.vstack
import trajectory_planning_helpers as tph
from transforms3d.euler import quat2euler


class SQPAvoidanceNode(Node):
    def __init__(self):
        # Initialize node
        super().__init__('sqp_avoidance_node')

        # Params
        self.frenet_state = Odometry()
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

        # Updated waypoints params
        self.wpnts_updated = None
        self.max_s_updated = None
        self.max_idx_updated = None

        # Obstalces params
        self.obs = ObstacleArray()
        self.obs_perception = ObstacleArray()
        self.obs_predict = ObstacleArray()

        # Opponent waypoint params
        self.opponent_waypoints = OpponentTrajectory()
        self.max_opp_idx = None
        self.opponent_wpnts_sm = None

        # OT params
        self.last_ot_side = ""
        self.ot_section_check = False

        # Solver params
        self.min_radius = 0.55  # wheelbase / np.tan(max_steering)
        self.max_kappa = 1/self.min_radius
        self.width_car = 0.30
        self.avoidance_resolution = 20
        self.back_to_raceline_before = 5
        self.back_to_raceline_after = 5
        self.obs_traj_tresh = 2

        # Dynamic sovler params
        self.down_sampled_delta_s = None
        self.global_traj_kappas = None

        # ROS Parameters
        self.declare_parameter('measure', False)
        self.measure = self.get_parameter('measure').get_parameter_value().bool_value

        # Dynamic reconf params (defaults from cfg/dyn_sqp_tuner.cfg)
        self.evasion_dist = 0.3
        self.spline_bound_mindist = 0.3
        self.avoid_static_obs = True

        self.converter = None
        self.global_waypoints = None
        self.global_waypoints_psi = None

        # CCMA init
        self.ccma = CCMA(w_ma=10, w_cc=3)

        # Subscribers
        self.create_subscription(ObstacleArray, "/tracking/obstacles", self.obs_perception_cb, QoSProfile(depth=10))
        self.create_subscription(ObstacleArray, "/opponent_prediction/obstacles", self.obs_prediction_cb, QoSProfile(depth=10))
        self.create_subscription(Odometry, "/car_state/odom_frenet", self.state_frenet_cb, QoSProfile(depth=10))
        self.create_subscription(Odometry, "/car_state/odom", self.state_cartesian_cb, QoSProfile(depth=10))
        self.create_subscription(WpntArray, "/global_waypoints_scaled", self.scaled_wpnts_cb, QoSProfile(depth=10))
        self.create_subscription(BehaviorStrategy, "/behavior_strategy", self.behavior_cb, QoSProfile(depth=10))
        self.create_subscription(WpntArray, "/global_waypoints", self.gb_cb, QoSProfile(depth=10))
        self.create_subscription(WpntArray, "/global_waypoints_updated", self.updated_wpnts_cb, QoSProfile(depth=10))
        self.create_subscription(OpponentTrajectory, "/opponent_trajectory", self.opponent_trajectory_cb, QoSProfile(depth=10))
        self.create_subscription(Bool, "/ot_section_check", self.ot_sections_check_cb, QoSProfile(depth=10))

        # Publishers
        self.mrks_pub = self.create_publisher(MarkerArray, "/planner/avoidance/markers_sqp", QoSProfile(depth=10))
        self.evasion_pub = self.create_publisher(OTWpntArray, "/planner/avoidance/otwpnts", QoSProfile(depth=10))
        self.merger_pub = self.create_publisher(Float32MultiArray, "/planner/avoidance/merger", QoSProfile(depth=10))
        if self.measure:
            self.measure_pub = self.create_publisher(Float32, "/planner/pspliner_sqp/latency", QoSProfile(depth=10))

        # Dynamic reconfigure -> declared parameters with callback
        double_descriptor = lambda lo, hi: ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(from_value=lo, to_value=hi, step=0.001)])
        param_dicts = [
            {'name': 'evasion_dist', 'default': 0.3, 'descriptor': double_descriptor(0.0, 1.25)},
            {'name': 'obs_traj_tresh', 'default': 1.0, 'descriptor': double_descriptor(0.1, 2.0)},
            {'name': 'spline_bound_mindist', 'default': 0.3, 'descriptor': double_descriptor(0.05, 1.0)},
            {'name': 'lookahead_dist', 'default': 10.0, 'descriptor': double_descriptor(1.0, 50.0)},
            {'name': 'avoidance_resolution', 'default': 10,
             'descriptor': ParameterDescriptor(
                 type=ParameterType.PARAMETER_INTEGER,
                 integer_range=[IntegerRange(from_value=10, to_value=100, step=1)])},
            {'name': 'back_to_raceline_before', 'default': 6.0, 'descriptor': double_descriptor(0.5, 10.0)},
            {'name': 'back_to_raceline_after', 'default': 8.0, 'descriptor': double_descriptor(0.5, 10.0)},
            {'name': 'avoid_static_obs', 'default': False,
             'descriptor': ParameterDescriptor(
                 type=ParameterType.PARAMETER_BOOL, description="Avoid static obstacles")},
        ]
        self.declare_all_parameters(param_dicts=param_dicts)
        self.read_dyn_params()
        self.add_on_set_parameters_callback(self.dyn_param_cb)

        # Wait for messages and initialize the converter
        self.wait_for_messages()
        self.converter = self.initialize_converter()

        # Main loop timer (ROS1 rospy.Rate(20))
        self.create_timer(1.0 / 20.0, self.loop)

    #################### DYNAMIC PARAMS ####################
    def declare_all_parameters(self, param_dicts: List[dict]):
        params = []
        for param_dict in param_dicts:
            param = self.declare_parameter(
                param_dict['name'], param_dict['default'], param_dict['descriptor'])
            params.append(param)
        return params

    def read_dyn_params(self):
        self.evasion_dist = self.get_parameter('evasion_dist').get_parameter_value().double_value
        self.obs_traj_tresh = self.get_parameter('obs_traj_tresh').get_parameter_value().double_value
        self.spline_bound_mindist = self.get_parameter('spline_bound_mindist').get_parameter_value().double_value
        self.lookahead = self.get_parameter('lookahead_dist').get_parameter_value().double_value
        self.avoidance_resolution = self.get_parameter('avoidance_resolution').get_parameter_value().integer_value
        self.back_to_raceline_before = self.get_parameter('back_to_raceline_before').get_parameter_value().double_value
        self.back_to_raceline_after = self.get_parameter('back_to_raceline_after').get_parameter_value().double_value
        self.avoid_static_obs = self.get_parameter('avoid_static_obs').get_parameter_value().bool_value

    # Callback triggered by dynamic reconf
    def dyn_param_cb(self, params: List[Parameter]):
        for param in params:
            if param.name == 'evasion_dist':
                self.evasion_dist = param.value
            elif param.name == 'obs_traj_tresh':
                self.obs_traj_tresh = param.value
            elif param.name == 'spline_bound_mindist':
                self.spline_bound_mindist = param.value
            elif param.name == 'lookahead_dist':
                self.lookahead = param.value
            elif param.name == 'avoidance_resolution':
                self.avoidance_resolution = param.value
            elif param.name == 'back_to_raceline_before':
                self.back_to_raceline_before = param.value
            elif param.name == 'back_to_raceline_after':
                self.back_to_raceline_after = param.value
            elif param.name == 'avoid_static_obs':
                self.avoid_static_obs = param.value

        self.get_logger().info(
            f"[Planner] Dynamic reconf triggered new spline params: \n"
            f" Evasion apex distance: {self.evasion_dist} [m],\n"
            f" Obstacle trajectory treshold: {self.obs_traj_tresh} [m]\n"
            f" Spline boundary mindist: {self.spline_bound_mindist} [m]\n"
            f" Lookahead distance: {self.lookahead} [m]\n"
            f" Avoid static obstacles: {self.avoid_static_obs}\n"
            f" Avoidance resolution: {self.avoidance_resolution}\n"
            f" Back to raceline before: {self.back_to_raceline_before} [m]\n"
            f" Back to raceline after: {self.back_to_raceline_after} [m]\n"
        )
        return SetParametersResult(successful=True)

    ### Callbacks ###
    def obs_perception_cb(self, data: ObstacleArray):
        self.obs_perception = data
        self.obs_perception.obstacles = [obs for obs in data.obstacles if obs.is_static == True]
        if self.avoid_static_obs == True:
            self.obs.header = data.header
            self.obs.obstacles = self.obs_perception.obstacles + self.obs_predict.obstacles

    def obs_prediction_cb(self, data: ObstacleArray):
        self.obs_predict = data
        self.obs = self.obs_predict
        if self.avoid_static_obs == True:
            self.obs.obstacles = self.obs.obstacles + self.obs_perception.obstacles

    def state_frenet_cb(self, data: Odometry):
        self.frenet_state = data

        quaternion = [data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]
        roll, pitch, yaw = quat2euler(quaternion)

        self.cur_yaw = yaw

    def state_cartesian_cb(self, msg):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        self.cur_v = msg.twist.twist.linear.x

    def gb_cb(self, data: WpntArray):
        self.global_waypoints = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in data.wpnts])
        self.global_waypoints_psi = np.array([wpnt.psi_rad for wpnt in data.wpnts])

    # Everything is refered to the SCALED global waypoints
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

    def behavior_cb(self, data: BehaviorStrategy):
        self.local_wpnts = np.array([[wpnt.s_m, wpnt.d_m] for wpnt in data.local_wpnts])

    def opponent_trajectory_cb(self, data: OpponentTrajectory):
        self.opponent_waypoints = data.oppwpnts
        self.max_opp_idx = len(data.oppwpnts)-1
        self.opponent_wpnts_sm = np.array([wpnt.s_m for wpnt in data.oppwpnts])

    def ot_sections_check_cb(self, data: Bool):
        self.ot_section_check = data.data

    ### Helper functions ###
    def wait_for_messages(self):
        # Wait for critical Messages and services
        self.get_logger().info("[OBS Spliner] Waiting for messages and services...")
        while rclpy.ok() and (
            self.scaled_wpnts is None
            or self.scaled_max_s is None
            or self.local_wpnts is None
            or not hasattr(self, "cur_x")
            or self.global_waypoints is None
        ):
            rclpy.spin_once(self)
        self.get_logger().info("[OBS Spliner] Ready!")

    def initialize_converter(self) -> FrenetConverter:
        """
        Initialize the FrenetConverter object"""
        # Initialize the FrenetConverter object
        converter = FrenetConverter(self.global_waypoints[:, 0], self.global_waypoints[:, 1], self.global_waypoints_psi)
        self.get_logger().info("[Spliner] initialized FrenetConverter object")

        return converter

    def loop(self):
        start_time = time.perf_counter()
        obs = deepcopy(self.obs)
        mrks = MarkerArray()
        frenet_state = self.frenet_state
        self.current_d = frenet_state.pose.pose.position.y
        self.cur_s = frenet_state.pose.pose.position.x

        # Obstacle pre-processing
        obs.obstacles = sorted(obs.obstacles, key=lambda obs: obs.s_start)
        considered_obs = []
        for obs in obs.obstacles:
            if abs(obs.d_center) < self.obs_traj_tresh and (obs.s_start - self.cur_s) % self.scaled_max_s < self.lookahead:
                considered_obs.append(obs)

        # If there is an obstacle and we are in OT section
        if len(considered_obs) > 0 and self.ot_section_check == True:
            evasion_x, evasion_y, evasion_s, evasion_d, evasion_v = self.sqp_solver(considered_obs, frenet_state.pose.pose.position.x)
            # Publish merge reagion if evasion track has been found
            if len(evasion_s) > 0:
                self.merger_pub.publish(Float32MultiArray(data=[considered_obs[-1].s_end % self.scaled_max_s, evasion_s[-1] % self.scaled_max_s]))

        # IF there is no point in overtaking anymore delte all markers
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

    def sqp_solver(self, considered_obs: list, cur_s: float):
        danger_flag = False
        # Get the initial guess of the overtaking side (see spliner)
        initial_guess_object = self.group_objects(considered_obs)
        # BUGFIX: scaled_wpnts is (N,2) [[s,d],...]. Subtracting a scalar from the whole 2D
        # array and calling argmin() returns a FLATTENED index (0..2N-1) that mixes the s and
        # d columns and is ~2x the real waypoint index. Downstream (gb_idxs, scaled_wpnts_msg
        # indexing) then reads the wrong / out-of-range waypoints, so the solver only produced
        # a valid path near s=0 (where the 2x error is small). Slice the s column ONLY, exactly
        # like the scaled_wpnts_indices computation further down.
        initial_guess_object_start_idx = np.abs(self.scaled_wpnts[:, 0] - initial_guess_object.s_start).argmin()
        initial_guess_object_end_idx = np.abs(self.scaled_wpnts[:, 0] - initial_guess_object.s_end).argmin()
        # Get array of indexes of the global waypoints overlapping with the ROC
        gb_idxs = np.array(range(initial_guess_object_start_idx, initial_guess_object_start_idx + (initial_guess_object_end_idx - initial_guess_object_start_idx)%self.scaled_max_idx))%self.scaled_max_idx
        # If the ROC is too short, we take the next 20 waypoints
        if len(gb_idxs) < 20:
            gb_idxs = [int(initial_guess_object.s_center / self.scaled_delta_s + i) % self.scaled_max_idx for i in range(20)]

        side, initial_apex = self._more_space(initial_guess_object, self.scaled_wpnts_msg.wpnts, gb_idxs)
        self.desired_side = side
        kappas = np.array([self.scaled_wpnts_msg.wpnts[gb_idx].kappa_radpm for gb_idx in gb_idxs])
        max_kappa = np.max(np.abs(kappas))
        outside = "left" if np.sum(kappas) < 0 else "right"

        # Enlongate the ROC if our initial guess suggests that we are overtaking on the outside.
        # NOTE: wrap-safe. obs_len is the (positive) obstacle length in s; if the obstacle
        # straddles the start/finish line s_end < s_start, so we take it modulo the track
        # length. The enlongation is then clamped so a single obstacle can never push s_end
        # more than a fraction of a lap ahead (previously it could explode when the raw
        # (s_end - s_start) went negative and %max_s_updated blew it up to ~one lap).
        if side == outside:
            for i in range(len(considered_obs)):
                obs_len = (considered_obs[i].s_end - considered_obs[i].s_start) % self.scaled_max_s
                enlongation = obs_len * max_kappa * (self.width_car + self.evasion_dist)
                enlongation = min(enlongation, self.scaled_max_s * 0.15)
                considered_obs[i].s_end = considered_obs[i].s_end + enlongation

        min_s_obs_start = self.scaled_max_s
        max_s_obs_end = 0
        for obs in considered_obs:
            if obs.s_start < min_s_obs_start:
                min_s_obs_start = obs.s_start
            if obs.s_end > max_s_obs_end:
                max_s_obs_end = obs.s_end
            # Check if it is a really wide obstacle
            if obs.d_left > 3 or obs.d_right < -3:
                danger_flag = True

        # Get local waypoints to check where we are and where we are heading.
        # Work in an UNWRAPPED s frame that starts at the car (start_avoidance = cur_s) and
        # only ever grows forward. The obstacle end is measured as a forward distance from the
        # car (wrap-safe), then the whole avoidance window is clamped to at most half a lap so
        # the path can never span more than one lap and produce duplicate/wrapped s values
        # downstream (this was the root cause of the "path longer than one lap" bug).
        start_avoidance = cur_s
        fwd_to_obs_end = (max_s_obs_end - cur_s) % self.scaled_max_s
        avoidance_len = fwd_to_obs_end + self.back_to_raceline_after
        avoidance_len = min(avoidance_len, self.scaled_max_s * 0.5)
        end_avoidance = start_avoidance + avoidance_len

        # Get a downsampled version for s avoidance points
        s_avoidance = np.linspace(start_avoidance, end_avoidance, self.avoidance_resolution)
        self.down_sampled_delta_s = s_avoidance[1] - s_avoidance[0]
        # Get the closest scaled waypoint for every s avoidance point (down sampled)
        scaled_wpnts_indices = np.array([np.abs(self.scaled_wpnts[:, 0] - s % self.scaled_max_s).argmin() for s in s_avoidance])
        # Get the scaled waypoints for every s avoidance point idx
        corresponding_scaled_wpnts = [self.scaled_wpnts_msg.wpnts[i] for i in scaled_wpnts_indices]
        # Get the boundaries for every s avoidance point
        bounds = np.array([(-wpnt.d_right + self.spline_bound_mindist, wpnt.d_left - self.spline_bound_mindist) for wpnt in corresponding_scaled_wpnts])

        # Calculate curvature at each point using numerical differentiation
        # k = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
        x_global_points = np.array([wpnt.x_m for wpnt in corresponding_scaled_wpnts])
        y_global_points = np.array([wpnt.y_m for wpnt in corresponding_scaled_wpnts])
        x_prime = np.diff(x_global_points)
        x_prime = np.where(x_prime == 0, 1e-6, x_prime) # Avoid division by zero
        y_prime = np.diff(y_global_points)
        y_prime = np.where(y_prime == 0, 1e-6, y_prime) # Avoid division by zero
        x_prime_prime = np.diff(x_prime)
        y_prime_prime = np.diff(y_prime)
        x_prime = x_prime[:-1] # Make it the same length as x_prime_prime
        y_prime = y_prime[:-1] # Make it the same length as y_prime_prime
        self.global_traj_kappas = (x_prime*y_prime_prime - y_prime*x_prime_prime) / ((x_prime**2 + y_prime**2)**(3/2))

        # Create a list of indices which overlap with the obstacles
        # Get the centerline of the obstacles and enforce a min distance to the obstacles
        self.obs_downsampled_indices = np.array([])
        self.obs_downsampled_center_d = np.array([])
        self.obs_downsampled_min_dist = np.array([])

        for obs in considered_obs:
            obs_idx_start = np.abs(s_avoidance - obs.s_start).argmin()
            obs_idx_end = np.abs(s_avoidance - obs.s_end).argmin()

            if obs_idx_start < len(s_avoidance) - 2: # Sanity check
                if obs.is_static == True or obs_idx_end == obs_idx_start:
                    if obs_idx_end == obs_idx_start:
                        obs_idx_end = obs_idx_start + 1
                    self.obs_downsampled_indices = np.append(self.obs_downsampled_indices, np.arange(obs_idx_start, obs_idx_end + 1))
                    self.obs_downsampled_center_d = np.append(self.obs_downsampled_center_d, np.full(obs_idx_end - obs_idx_start + 1, (obs.d_left + obs.d_right) / 2))
                    self.obs_downsampled_min_dist = np.append(self.obs_downsampled_min_dist, np.full(obs_idx_end - obs_idx_start + 1, (obs.d_left - obs.d_right) / 2 + self.width_car + self.evasion_dist))
                else:
                    indices = np.arange(obs_idx_start, obs_idx_end + 1)
                    self.obs_downsampled_indices = np.append(self.obs_downsampled_indices, indices)
                    opp_wpnts_idx = [np.abs(self.opponent_wpnts_sm - s_avoidance[int(idx)]%self.max_opp_idx).argmin() for idx in indices]
                    d_opp_downsampled_array = np.array([self.opponent_waypoints[opp_idx].d_m for opp_idx in opp_wpnts_idx])
                    self.obs_downsampled_center_d = np.append(self.obs_downsampled_center_d, d_opp_downsampled_array)
                    self.obs_downsampled_min_dist = np.append(self.obs_downsampled_min_dist, np.full(obs_idx_end - obs_idx_start + 1, self.width_car + self.evasion_dist))
            else:
                self.get_logger().info("[OBS Spliner] Obstacle end index is smaller than start index")
                self.get_logger().info("[OBS Spliner] len obs: " + str(len(considered_obs)) + "obs_start:" + str(obs.s_start) + "obs_end:" + str(obs.s_end) + " obs_idx_start: " + str(obs_idx_start) + " obs_idx_end: " + str(obs_idx_end) + " len s_avoidance: " + str(len(s_avoidance)) + "s avoidance 0:" + str(s_avoidance[0]) + " s avoidance -1: " + str(s_avoidance[-1]))


        self.obs_downsampled_indices = self.obs_downsampled_indices.astype(int)

        # Get the min radius
        clipped_speed = np.clip(self.frenet_state.twist.twist.linear.x, 1, a_max=None)
        # Get the minimum of clipped speed and the updated speed of the first waypoints
        radius_speed = min([clipped_speed, self.wpnts_updated[(scaled_wpnts_indices[0])%self.max_idx_updated].vx_mps])
        # Interpolate the min_radius with speeds between 0.2 and 7 m
        self.min_radius = np.interp(radius_speed, [1, 6, 7], [0.2, 2, 4])
        self.max_kappa = 1/self.min_radius

        if len(self.past_avoidance_d) == 0:
            initial_guess = np.full(len(s_avoidance), initial_apex)

        elif len(self.past_avoidance_d) > 0:
            initial_guess = self.past_avoidance_d
        else:
            #TODO: Remove -> print("this happend")
            if self.last_ot_side == "left":
                initial_guess = np.full(len(s_avoidance), 2)
            else:
                initial_guess = np.full(len(s_avoidance), -2)

        result = self.solve_sqp(initial_guess, bounds)

        if result.success == True:
            # Create a new s array for the global waypoints as close to delta s as possible
            n_global_avoidance_points = int((end_avoidance - start_avoidance) / self.scaled_delta_s)
            s_array = np.linspace(start_avoidance, end_avoidance, n_global_avoidance_points)
            # Interpolate corresponding d values
            evasion_d = np.interp(s_array, s_avoidance, result.x)
            # Solve rap around problem
            evasion_s = np.mod(s_array, self.scaled_max_s)
            # Get the corresponding x and y values
            resp = self.converter.get_cartesian(evasion_s, evasion_d)
            resp = resp.transpose()
            smoothed_xy_points = self.ccma.filter(resp)
            smoothed_sd_points = self.converter.get_frenet(smoothed_xy_points[:, 0], smoothed_xy_points[:, 1])
            # Keep the ORIGINAL generation order for every array. The s values already grow
            # monotonically along the path (the avoidance window is < half a lap), so they are
            # already ordered. Do NOT sort s/d independently: x/y/psi/kappa/v are not sorted, so
            # sorting only s/d would misalign every waypoint's (s,d) from its (x,y). The only
            # thing get_frenet does is wrap s into [0, max_s); a single start/finish crossing is
            # left for the downstream merger to handle, exactly like the static planner.
            evasion_s = np.asarray(smoothed_sd_points[0])
            evasion_d = np.asarray(smoothed_sd_points[1])
            evasion_x = smoothed_xy_points[:, 0]
            evasion_y = smoothed_xy_points[:, 1]
            evasion_coords = np.column_stack((evasion_x, evasion_y))
            evasion_psi, evasion_kappa = tph.calc_head_curv_num.calc_head_curv_num(
                path=evasion_coords,
                el_lengths=0.1 * np.ones(len(evasion_coords) - 1),
                is_closed=False
            )
            evasion_psi += np.pi / 2
            # Get the corresponding v values
            downsampled_v = np.array([wpnt.vx_mps for wpnt in corresponding_scaled_wpnts])
            evasion_v = np.interp(s_array, s_avoidance, downsampled_v)
            # Create a new evasion waypoint message
            evasion_wpnts_msg = OTWpntArray(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"))
            evasion_wpnts = []
            evasion_wpnts = [Wpnt(id=len(evasion_wpnts), s_m=s, d_m=d, x_m=x, y_m=y, psi_rad=p, kappa_radpm=k, vx_mps= v) for x, y, s, d, p, k, v in zip(evasion_x, evasion_y, evasion_s, evasion_d, evasion_psi, evasion_kappa, evasion_v)]
            evasion_wpnts_msg.wpnts = evasion_wpnts
            self.past_avoidance_d = result.x
            mean_d = np.mean(evasion_d)
            if mean_d > 0:
                self.last_ot_side = "left"
            else:
                self.last_ot_side = "right"
            # print("[OBS Spliner] SQP solver successfull")

        else:
            evasion_x = []
            evasion_y = []
            evasion_s = []
            evasion_d = []
            evasion_coords = []
            evasion_psi = []
            evasion_kappa = []
            evasion_v = []
            evasion_wpnts_msg = OTWpntArray(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"))
            evasion_wpnts_msg.wpnts = []
            self.past_avoidance_d = []

        self.evasion_pub.publish(evasion_wpnts_msg)
        self.visualize_sqp(evasion_s, evasion_d, evasion_x, evasion_y, evasion_v)

        return evasion_x, evasion_y, evasion_s, evasion_d, evasion_v


    ### Optimal Trajectory Solver ###
    def objective_function(self, d):
        return np.sum((d) ** 2) * 10  + np.sum(np.diff(np.diff(d))**2) * 100 + (np.diff(d)[0] ** 2) * 1000

    ## Constraint functions ##
    def start_on_raceline_constraint(self, d): # And end on raceline
        return np.array([0.02 - abs(d[0] - self.current_d), 0.02 - abs(d[-2]), 0.02 - abs(d[-1])])

    def psi_constraint(self, d):
        delta_s = self.down_sampled_delta_s
        e_psi = self.converter.get_e_psi(self.cur_x, self.cur_y, self.cur_yaw)
        desired_dd = np.tan(e_psi) * delta_s
        return np.array([0.02 - abs((d[1] - d[0]) - abs(desired_dd))])

    def obstacle_constraint(self, d):
        distance_to_obstacle = np.abs(d[self.obs_downsampled_indices] - self.obs_downsampled_center_d)
        violation = distance_to_obstacle - self.obs_downsampled_min_dist
        return violation

    # Prevents points from jumping trhough obstacles due to resoultion isses
    def consecutive_points_constraint(self, d):
        # Extract the relevant points
        points = d[self.obs_downsampled_indices]

        # Check the condition for each pair of consecutive points
        violations = []
        for i in range(len(points) - 1):
            if not ((points[i] > self.obs_downsampled_center_d[i] and points[i+1] > self.obs_downsampled_center_d[i+1]) or
                    (points[i] < self.obs_downsampled_center_d[i] and points[i+1] < self.obs_downsampled_center_d[i+1])):
                violations.append(-1)  # Add a violation as a negative value if the condition is not met
            else:
                violations.append(1)
        return violations

    def turning_radius_constraint(self, d):
        # Calculate curvature at each point using numerical differentiation
        # k = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
        # x' = self.down_sampled_delta_s, x'' = 0

        y_prime = np.diff(d)
        y_prime = np.where(y_prime == 0, 1e-6, y_prime) # Avoid division by zero
        y_prime_prime = np.diff(y_prime)
        y_prime = y_prime[:-1] # Make it the same length as y_prime_prime

        kappa = (self.down_sampled_delta_s * y_prime_prime) / ((self.down_sampled_delta_s ** 2) ** (3/2))

        mu = 0.318
        g = 9.81
        kappa_limit = mu * g / ((self.frenet_state.twist.twist.linear.x + 1e-6) ** 2)
        return abs(kappa_limit) - abs(kappa)

    # The arctan of of (d[1]-d[0])/ delta_s_sample_points < than 45 degrees
    def first_point_constraint(self, d):
        return np.array([self.down_sampled_delta_s - abs(d[1]-d[0])])

    def side_consistency_constraint(self, d):
        if self.desired_side == "left":
            return d
        elif self.desired_side == "right":
            return -d
        else:
            return d

    def solve_sqp(self, d_array, track_boundaries):
        result = minimize(
        self.objective_function, d_array, method='SLSQP', jac='10-point',
        bounds=track_boundaries,
        constraints=[
            {'type': 'eq', 'fun': self.start_on_raceline_constraint},
            {'type': 'eq', 'fun': self.psi_constraint},
            {'type': 'ineq', 'fun': self.obstacle_constraint},
            {'type': 'ineq', 'fun': self.consecutive_points_constraint},
            {'type': 'ineq', 'fun': self.turning_radius_constraint},
            {'type': 'ineq', 'fun': self.first_point_constraint},
            {'type': 'ineq', 'fun': self.side_consistency_constraint},
            ],
        options={'ftol': 1e-1, 'maxiter': 20, 'disp': False},
        )
        return result

    def group_objects(self, obstacles: list):
        # Group obstacles that are close to each other
        initial_guess_object = obstacles[0]
        for obs in obstacles:
            if obs.d_left > initial_guess_object.d_left:
                initial_guess_object.d_left = obs.d_left
            if obs.d_right < initial_guess_object.d_right:
                initial_guess_object.d_right = obs.d_right
            if obs.s_start < initial_guess_object.s_start:
                initial_guess_object.s_start = obs.s_start
            if obs.s_end > initial_guess_object.s_end:
                initial_guess_object.s_end = obs.s_end
        initial_guess_object.s_center = (initial_guess_object.s_start + initial_guess_object.s_end) / 2
        return initial_guess_object

    def _more_space(self, obstacle: Obstacle, gb_wpnts, gb_idxs):
        # d-sign matches bounds at ~line 378: +d = left wall (+d_left), -d = right wall (-d_right).
        # Nearest single waypoint, not the ROC mean (mean across a corner flips the side).
        near_idx = gb_idxs[len(gb_idxs) // 2]
        wall_left = gb_wpnts[near_idx].d_left
        wall_right = gb_wpnts[near_idx].d_right

        # Signed free room: <=0 when the obstacle already fills that side.
        left_gap = wall_left - obstacle.d_left
        right_gap = obstacle.d_right + wall_right
        min_space = self.evasion_dist + self.spline_bound_mindist

        left_ok = left_gap >= min_space
        right_ok = right_gap >= min_space
        if left_ok and not right_ok:
            side = "left"
        elif right_ok and not left_ok:
            side = "right"
        else:
            side = "left" if left_gap >= right_gap else "right"

        if side == "left":
            d_apex = obstacle.d_left + self.evasion_dist
            d_apex = max(d_apex, 0.0)
            d_apex = min(d_apex, wall_left - self.spline_bound_mindist)
            return "left", d_apex
        else:
            d_apex = obstacle.d_right - self.evasion_dist
            d_apex = min(d_apex, 0.0)
            d_apex = max(d_apex, -wall_right + self.spline_bound_mindist)
            return "right", d_apex

    ### Visualize SQP Rviz###
    def visualize_sqp(self, evasion_s, evasion_d, evasion_x, evasion_y, evasion_v):
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


def main(args=None):
    rclpy.init(args=args)
    node = SQPAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
