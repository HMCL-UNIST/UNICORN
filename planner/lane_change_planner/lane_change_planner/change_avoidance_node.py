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
    BehaviorStrategy,
    PredictionArray,
)
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Float32, Header, Bool

from frenet_conversion.frenet_converter import FrenetConverter

from ccma import CCMA
# NumPy 2.0 removed the np.row_stack alias, but the ccma library still calls it.
if not hasattr(np, "row_stack"):
    np.row_stack = np.vstack
import trajectory_planning_helpers as tph
from grid_filter.grid_filter import GridFilter
import matplotlib.pyplot as plt


class ChangeAvoidanceNode(Node):
    def __init__(self):
        # Initialize node
        super().__init__('change_avoidance_node')

        # Params
        self.local_wpnts = None
        self.lookahead = 15

        # Side hysteresis: only switch preferred_side after the new side persists this many
        # consecutive loops (20 Hz), to reject brief gap-driven side flips that jitter the path.
        self.committed_side = None
        self.pending_side = None
        self.pending_count = 0
        self.side_switch_frames = 10

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

        # Cached lane polylines for the fixed-rate lane-marker publisher (set in generate_lanes).
        self._center_xy = None
        self._outer_xy = None
        self._inner_xy = None

        # Updated waypoints params
        self.wpnts_updated = None
        self.max_s_updated = None

        # Obstalces params
        self.obs_perception = ObstacleArray()
        self.obs_prediction = ObstacleArray()
        self.obs_prediction_pred = PredictionArray()

        # Solver params
        self.width_car = 0.30
        self.safety_margin = 0.1
        self.back_to_raceline_before = 3.0
        self.back_to_raceline_after = 3.0
        self.obs_traj_tresh = 2.0

        # Dynamic sovler params
        self.down_sampled_delta_s = 0.1

        # State variables (filled by callbacks)
        self.current_s = None
        self.current_d = None
        self.current_x = None

        # Dynamic reconf params (defaults from cfg/dyn_change_tuner.cfg)
        self.evasion_dist = 0.3
        self.spline_bound_mindist = 0.3
        # Max allowed |current_d - evasion_d| at the car before the evasion path is
        # rejected (guards against a too-abrupt lateral jump onto the path). Larger =
        # allows starting an evasion while closer/tighter to the opponent.
        self.max_evasion_start_offset = 0.8

        # Symmetric lane perturbation (centerline +/- lane_offset -> outer/inner lanes).
        # Adjustable live via rqt: changing it regenerates the two lanes.
        self.lane_offset = 0.35

        # Require fresh GP prediction before overtaking. Without it we only trail.
        # Prediction is considered stale if the latest prediction msg is older than this.
        self.pred_timeout = 0.5  # [s]
        self.last_pred_stamp = None
        # Default True (trail) until told otherwise, so a missing predictor never enables overtaking.
        self.force_trailing = True

        # Only clear the avoidance markers after this many consecutive failed/idle frames, so a
        # single dropped frame doesn't make the markers flicker.
        self.no_evasion_count = 0
        self.clear_after_frames = 5

        self.converter = None
        self.global_waypoints = None

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

        # Dynamic reconfigure -> declared ROS2 parameters
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
                'name': 'safety_margin',
                'default': self.safety_margin,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Safety margin around the vehicle and obstacles",
                    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.001)],
                ),
            },
            {
                'name': 'back_to_raceline_before',
                'default': self.back_to_raceline_before,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Distance before the obstacle used to return to the raceline",
                    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=30.0, step=0.001)],
                ),
            },
            {
                'name': 'back_to_raceline_after',
                'default': self.back_to_raceline_after,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Distance after the obstacle used to return to the raceline",
                    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=30.0, step=0.001)],
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
            {
                'name': 'lane_offset',
                'default': self.lane_offset,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Symmetric perturbation of the centerline to build outer/inner lanes [m]. Live editable, regenerates lanes.",
                    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.5, step=0.001)],
                ),
            },
            {
                'name': 'pred_timeout',
                'default': self.pred_timeout,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Overtake only if a GP prediction arrived within this time, else trail [s]",
                    floating_point_range=[FloatingPointRange(from_value=0.05, to_value=5.0, step=0.001)],
                ),
            },
            {
                'name': 'max_evasion_start_offset',
                'default': self.max_evasion_start_offset,
                'descriptor': ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description="Reject the evasion path if |current_d - evasion_d| at the car exceeds this [m]. Larger allows starting an evasion while closer to the opponent.",
                    floating_point_range=[FloatingPointRange(from_value=0.1, to_value=2.0, step=0.001)],
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

        # Lane visualization (always published, independent of the `vis` matplotlib flag).
        # Outer/inner perturbed lanes in distinct colors + how far each is offset from center.
        self.lane_mrks_pub = self.create_publisher(MarkerArray, "/planner/avoidance/lane_markers", QoSProfile(depth=10))
        self.lane_offset_pub = self.create_publisher(Float32MultiArray, "/planner/avoidance/lane_offset", QoSProfile(depth=10))
        # Large spheres at the avoidance start/end so we can see exactly which points are picked.
        self.avoidance_pts_pub = self.create_publisher(MarkerArray, "/planner/avoidance/start_end_markers", QoSProfile(depth=10))

        # Subscribers
        self.create_subscription(ObstacleArray, "/tracking/obstacles", self.obs_perception_cb, QoSProfile(depth=10))
        self.create_subscription(ObstacleArray, "/opponent_prediction/obstacles", self.obs_prediction_cb, QoSProfile(depth=10))
        self.create_subscription(PredictionArray, "/opponent_prediction/obstacles_pred", self.obstacle_prediction_cb, QoSProfile(depth=10))
        # True means the prediction is a constant-velocity fallback (opponent not yet on a learned
        # GP trajectory). Only the GP-trajectory prediction is trusted for overtaking.
        self.create_subscription(Bool, "/opponent_prediction/force_trailing", self.force_trailing_cb, QoSProfile(depth=10))
        self.create_subscription(Odometry, "/car_state/odom_frenet", self.state_frenet_cb, QoSProfile(depth=10))
        self.create_subscription(Odometry, "/car_state/odom", self.state_cartesian_cb, QoSProfile(depth=10))
        self.create_subscription(BehaviorStrategy, "/behavior_strategy", self.behavior_cb, QoSProfile(depth=10))
        self.create_subscription(WpntArray, "/global_waypoints", self.gb_cb, QoSProfile(depth=10))
        self.create_subscription(WpntArray, "/global_waypoints_scaled", self.scaled_wpnts_cb, QoSProfile(depth=10))
        self.create_subscription(WpntArray, "/global_waypoints_updated", self.updated_wpnts_cb, QoSProfile(depth=10))
        self.create_subscription(WpntArray, "/centerline_waypoints", self.center_wpnts_cb, QoSProfile(depth=10))

        self.converter = self.initialize_converter()

        self.map_filter = GridFilter(node=self, map_topic="/map", debug=False)
        self.map_filter.set_erosion_kernel_size(7)

        # Wait for the centerline waypoints, then generate the inner/outer lanes
        self.wait_for_message_attr('center_wpnts_received')
        self.generate_lanes(center_wpnts=self.center_wpnts_msg)

        # Wait for critical messages and services
        self.get_logger().info("[OBS Spliner] Waiting for messages and services...")
        self.wait_for_loop_messages()
        self.get_logger().info("[OBS Spliner] Ready!")

        # Main loop timer at 20 Hz
        self.create_timer(1.0 / 20.0, self.loop)

        # Republish the lane markers + offset at a fixed rate so they stay visible in rviz
        # without touching rqt or regenerating the lanes.
        self.lane_viz_hz = 5.0
        self.create_timer(1.0 / self.lane_viz_hz, self.lane_viz_loop)

    #################### DYNAMIC PARAMS ####################
    def declare_all_parameters(self, param_dicts: List[dict]):
        params = []
        for param_dict in param_dicts:
            param = self.declare_parameter(
                param_dict['name'], param_dict['default'], param_dict['descriptor'])
            params.append(param)
        return params

    def dyn_param_cb(self, params: List[Parameter]):
        regenerate_lanes = False
        for param in params:
            if param.name == 'evasion_dist':
                self.evasion_dist = param.value
            elif param.name == 'safety_margin':
                self.safety_margin = param.value
            elif param.name == 'back_to_raceline_before':
                self.back_to_raceline_before = param.value
            elif param.name == 'back_to_raceline_after':
                self.back_to_raceline_after = param.value
            elif param.name == 'obs_traj_tresh':
                self.obs_traj_tresh = param.value
            elif param.name == 'spline_bound_mindist':
                self.spline_bound_mindist = param.value
            elif param.name == 'lane_offset':
                if param.value != self.lane_offset:
                    regenerate_lanes = True
                self.lane_offset = param.value
            elif param.name == 'pred_timeout':
                self.pred_timeout = param.value
            elif param.name == 'max_evasion_start_offset':
                self.max_evasion_start_offset = param.value

        # Rebuild the outer/inner lanes with the new offset. Only once the centerline
        # is available (i.e. after the initial generate_lanes at startup).
        if regenerate_lanes and self.center_wpnts_received:
            self.generate_lanes(center_wpnts=self.center_wpnts_msg)
            self.get_logger().info(f"[Planner] Regenerated lanes with lane_offset={self.lane_offset} [m]")

        self.get_logger().info(
            f"[Planner] Dynamic reconf triggered new spline params: \n"
            f" Evasion apex distance: {self.evasion_dist} [m],\n"
            f" Safety margin: {self.safety_margin} [m],\n"
            f" Back to raceline before: {self.back_to_raceline_before} [m],\n"
            f" Back to raceline after: {self.back_to_raceline_after} [m],\n"
            f" Obstacle trajectory treshold: {self.obs_traj_tresh} [m]\n"
            f" Spline boundary mindist: {self.spline_bound_mindist} [m]\n"
            f" Lane offset: {self.lane_offset} [m]\n"
            f" Prediction timeout: {self.pred_timeout} [s]\n"
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
        # GP 예측이 실제로 도착한 시각을 기록 (예측 신선도 판정용). predictions가 비어있으면
        # "예측 없음"으로 간주하므로 stale 타임스탬프를 갱신하지 않는다.
        if len(data.predictions) > 0:
            self.last_pred_stamp = self.get_clock().now()

    def force_trailing_cb(self, data: Bool):
        self.force_trailing = data.data

    def prediction_is_fresh(self) -> bool:
        """최근 pred_timeout 초 이내에 비어있지 않은 GP 예측이 도착했으면 True."""
        if self.last_pred_stamp is None:
            return False
        dt = (self.get_clock().now() - self.last_pred_stamp).nanoseconds * 1e-9
        return dt <= self.pred_timeout

    def state_frenet_cb(self, data: Odometry):
        self.current_s = data.pose.pose.position.x
        self.current_d = data.pose.pose.position.y

    def state_cartesian_cb(self, data: Odometry):
        self.current_x = data.pose.pose.position.x

    def gb_cb(self, data: WpntArray):
        self.global_waypoints = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in data.wpnts])

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

    def center_wpnts_cb(self, data: WpntArray):
        self.center_wpnts_msg = data
        self.center_wpnts_received = True

    def behavior_cb(self, data: BehaviorStrategy):
        self.local_wpnts = np.array([[wpnt.s_m, wpnt.d_m] for wpnt in data.local_wpnts])

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
                if obs.id == self.obs_prediction_pred.id and len(self.obs_prediction.obstacles) > 0:
                    considered_obs.extend(self.obs_prediction.obstacles)
                else:
                    considered_obs.append(obs)

        return considered_obs

    def _apply_side_hysteresis(self, raw_side: str) -> str:
        if self.committed_side is None:
            self.committed_side = raw_side
            self.pending_side = None
            self.pending_count = 0
        elif raw_side == self.committed_side:
            self.pending_side = None
            self.pending_count = 0
        else:
            if raw_side == self.pending_side:
                self.pending_count += 1
            else:
                self.pending_side = raw_side
                self.pending_count = 1
            if self.pending_count >= self.side_switch_frames:
                self.committed_side = raw_side
                self.pending_side = None
                self.pending_count = 0
        return self.committed_side

    def more_space(self, obstacle: Obstacle, gb_wpnts, gb_idxs):
        left_gap = gb_wpnts[gb_idxs[0]].d_left - obstacle.d_left
        right_gap = gb_wpnts[gb_idxs[0]].d_right + obstacle.d_right
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
            # Both sides too narrow: genuinely no room -> don't overtake.
            return None, 0.0, left_gap, right_gap
        else:
            # Both sides wide enough: pick the wider one instead of giving up (old code returned
            # None here, so a clearly-open side was wasted). Only the ambiguous "both open" case
            # falls here; the "both narrow" case above still returns None.
            if left_gap >= right_gap:
                d_apex_left = obstacle.d_left + (self.width_car / 2 + self.safety_margin + 0.2)
                if d_apex_left < 0 and left_gap < abs(d_apex_left):
                    d_apex_left = 0
                return "left", d_apex_left, left_gap, right_gap
            else:
                d_apex_right = obstacle.d_right - (self.width_car / 2 + self.safety_margin + 0.2)
                if d_apex_right > 0 and right_gap < abs(d_apex_right):
                    d_apex_right = 0
                return "right", d_apex_right, left_gap, right_gap

    ### Visualize SPL Rviz ###
    def visualize_dynamic_spliner(self, evasion_s, evasion_d, evasion_x, evasion_y, evasion_v):
        mrks = MarkerArray()
        if len(evasion_s) == 0:
            pass
        else:
            # DELETEALL first so a shorter path this frame leaves no leftover cylinders.
            del_mrk = Marker(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"))
            del_mrk.action = Marker.DELETEALL
            mrks.markers.append(del_mrk)
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
                mrk.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
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
    def _unwrap_forward(self, s: float, ref_s: float) -> float:
        """Return s expressed as ref_s + forward-distance, so it is always >= ref_s and monotonic
        across the s=0 seam. A point up to `back_tol` behind ref_s (perception jitter) is clamped
        to ref_s instead of being wrapped a whole lap forward (which flashed the path backwards)."""
        back_tol = 1.0
        fwd = (s - ref_s) % self.scaled_max_s
        if fwd > self.scaled_max_s - back_tol:
            fwd = 0.0
        return ref_s + fwd

    def lane_change(self, considered_obs: list, cur_s: float):
        # Normalize every obstacle's s to be unwrapped and forward of the car up front, so the
        # ROC index math, the s_end elongation and s_avoidance below all stay monotonic and can
        # never span a lap. Downstream still wraps at publish time.
        for obs in considered_obs:
            obs.s_start = self._unwrap_forward(obs.s_start, self.current_s)
            obs.s_end = self._unwrap_forward(obs.s_end, self.current_s)
            obs.s_center = self._unwrap_forward(obs.s_center, self.current_s)
            if obs.s_end < obs.s_start:
                obs.s_end += self.scaled_max_s

        # Get the initial guess of the overtaking side.
        # scaled_wpnts is [[s, d], ...]; match on the s column only. np.abs(2D-scalar).argmin()
        # would return a flattened index (~2x) and pick the wrong waypoint.
        scaled_s = self.scaled_wpnts[:, 0]
        initial_guess_object_start_idx = np.abs(scaled_s - considered_obs[0].s_start % self.scaled_max_s).argmin()
        initial_guess_object_end_idx = np.abs(scaled_s - considered_obs[-1].s_end % self.scaled_max_s).argmin()

        # Get array of indexes of the global waypoints overlapping with the ROC
        gb_idxs = np.array(range(initial_guess_object_start_idx, initial_guess_object_start_idx + (initial_guess_object_end_idx - initial_guess_object_start_idx) % self.scaled_max_idx)) % self.scaled_max_idx

        if len(gb_idxs) < 20:
            gb_idxs = [int(considered_obs[0].s_center / self.scaled_delta_s + i) % self.scaled_max_idx for i in range(20)]

        side, initial_apex, left_gap, right_gap = self.more_space(considered_obs[0], self.scaled_wpnts_msg.wpnts, gb_idxs)
        kappas = np.array([self.scaled_wpnts_msg.wpnts[gb_idx].kappa_radpm for gb_idx in gb_idxs])
        max_kappa = np.max(np.abs(kappas))
        outside = "left" if np.sum(kappas) < 0 else "right"

        # Enlongate the ROC if our initial guess suggests that we are overtaking on the outside.
        # s_start/s_end are already unwrapped-forward here, so obs_len is a small positive span
        # (no modulo needed); the enlongation is capped so a corner's max_kappa can't blow s_end
        # up by ~a lap and make the path circle the track.
        if side == outside:
            for i in range(len(considered_obs)):
                obs_len = considered_obs[i].s_end - considered_obs[i].s_start
                enlongation = obs_len * max_kappa * (self.width_car + self.evasion_dist)
                enlongation = min(enlongation, self.scaled_max_s * 0.15)
                considered_obs[i].s_end = considered_obs[i].s_end + enlongation

        # obs.s_* are unwrapped forward-of-car (can exceed scaled_max_s), so seed with +/-inf.
        min_s_obs_start = np.inf
        max_s_obs_end = -np.inf

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
            raw_side = "left"
        elif right_count > left_count and right_gap_avg > left_gap_avg and right_gap_avg > self.width_car:
            raw_side = "right"
        else:
            return [], [], [], [], []

        preferred_side = self._apply_side_hysteresis(raw_side)

        # ---- Monotonic-s sampling (SQP-style): define an increasing raceline s up front, then
        # ---- pull the target lane's d at each s, ease in/out around the obstacle span, and convert
        # ---- to cartesian. s is monotonic by construction so the published path can never reverse.
        target_wpnts = self.outer_lane_wpnts_msg.wpnts if preferred_side == "left" else self.inner_lane_wpnts_msg.wpnts

        # obs.s_* were already unwrapped forward-of-car at the top of lane_change, so the span is
        # monotonic and cannot circle the track. Just take the min/max directly.
        start_s = self.current_s
        obs_start_u = min(o.s_start for o in considered_obs)
        obs_end_u = max(o.s_end for o in considered_obs)
        end_s = obs_end_u + self.back_to_raceline_after
        end_s = min(end_s, start_s + self.scaled_max_s * 0.5)

        n_samples = max(int((end_s - start_s) / self.scaled_delta_s), 5)
        s_lin = np.linspace(start_s, end_s, n_samples)            # monotonic, unwrapped

        # Target lane d along s (full avoidance offset); raceline d = 0.
        lane_d = self._lane_d_at_s(target_wpnts, s_lin)

        # Cosine ease: 0 (raceline) before the obstacle, ramp to lane_d across the obstacle, ramp
        # back. Entry ramp width = back_to_raceline_before, exit ramp width = back_to_raceline_after
        # (separate, so the approach and the return to the raceline can be tuned independently).
        ramp_in = self.back_to_raceline_before
        ramp_out = self.back_to_raceline_after
        d_arr = np.zeros_like(s_lin)
        for i, s in enumerate(s_lin):
            if s < obs_start_u - ramp_in or s > obs_end_u + ramp_out:
                w = 0.0
            elif s < obs_start_u:
                w = 0.5 * (1 - np.cos(np.pi * (s - (obs_start_u - ramp_in)) / ramp_in))
            elif s > obs_end_u:
                w = 0.5 * (1 + np.cos(np.pi * (s - obs_end_u) / ramp_out))
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
        evasion_x = np.asarray(smoothed_xy_points[:, 0])
        evasion_y = np.asarray(smoothed_xy_points[:, 1])
        evasion_d = np.asarray(smoothed_sd_points[1])
        # get_frenet returns wrapped s; CCMA can locally reorder s near the apex. Unwrap, then
        # keep only strictly-increasing s to drop duplicate/reversing points (NaN source).
        evasion_s_raw = np.asarray(smoothed_sd_points[0])
        evasion_s = start_s + (evasion_s_raw - start_s) % self.scaled_max_s

        keep = np.ones(len(evasion_s), dtype=bool)
        last_s = -np.inf
        for i in range(len(evasion_s)):
            if evasion_s[i] - last_s > 1e-4:
                last_s = evasion_s[i]
            else:
                keep[i] = False
        evasion_x = evasion_x[keep]
        evasion_y = evasion_y[keep]
        evasion_s = evasion_s[keep]
        evasion_d = evasion_d[keep]
        evasion_coords = np.column_stack((evasion_x, evasion_y))

        # Guard: drop coincident xy points (zero-length segments -> NaN psi/kappa).
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

        evasion_s = evasion_s % self.scaled_max_s

        evasion_psi, evasion_kappa = tph.calc_head_curv_num.calc_head_curv_num(
            path=evasion_coords,
            el_lengths=0.1 * np.ones(len(evasion_coords) - 1),
            is_closed=False
        )
        evasion_psi += np.pi / 2
        evasion_v = np.zeros(len(evasion_s))

        temp_idx = np.argmin([abs(evs - self.current_s) for evs in evasion_s])
        if abs(self.current_d - evasion_d[temp_idx]) > self.max_evasion_start_offset:
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

        self.evasion_pub.publish(evasion_wpnts_msg)
        self.visualize_dynamic_spliner(evasion_s, evasion_d, evasion_x, evasion_y, evasion_v)
        # Only publish the start/end debug spheres once the path actually succeeds, so they track
        # the published path instead of flickering on every early-return frame.
        self.publish_start_end_markers(start_s, obs_start_u, obs_end_u, end_s)

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

        # 2. Constant othogonal evasion (symmetric perturbation, live-tunable via lane_offset)
        outer_lane = original_center_wpnts + normals * self.lane_offset
        inner_lane = original_center_wpnts - normals * self.lane_offset

        outer_lane_resampled = self.resample_lane(outer_lane, resolution=0.1)
        inner_lane_resampled = self.resample_lane(inner_lane, resolution=0.1)

        outer_s, outer_d = self.converter.get_frenet(outer_lane_resampled[:, 0], outer_lane_resampled[:, 1])
        inner_s, inner_d = self.converter.get_frenet(inner_lane_resampled[:, 0], inner_lane_resampled[:, 1])

        # Robustness: the xy-normal above ([sin,-cos]) assumes a specific psi convention/track
        # direction (CW vs CCW), so "outer" can come out on the wrong side. Downstream this
        # module treats outer_lane as the LEFT (+d) lane (preferred_side=="left" -> outer). f1tenth
        # frenet has d>0 to the left, so if the measured mean d of outer is negative the lanes are
        # flipped -> swap them. This keeps left/right correct for any map orientation.
        if np.mean(outer_d) < np.mean(inner_d):
            outer_lane_resampled, inner_lane_resampled = inner_lane_resampled, outer_lane_resampled
            outer_s, inner_s = inner_s, outer_s
            outer_d, inner_d = inner_d, outer_d
            self.get_logger().info("[Planner] outer/inner lanes swapped to match frenet +d=left convention")

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

        # Clear so a live regeneration (lane_offset change) doesn't append onto the old lanes.
        self.outer_lane_wpnts_msg.wpnts = []
        self.inner_lane_wpnts_msg.wpnts = []

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

        # Cache the polylines so the lane-marker timer can republish them at a fixed rate,
        # independent of the vis flag and without needing an rqt/lane_offset change.
        self._center_xy = original_center_wpnts
        self._outer_xy = outer_lane_resampled
        self._inner_xy = inner_lane_resampled

    def publish_start_end_markers(self, start_s, obs_start_u, obs_end_u, end_s):
        """회피 구간의 핵심 s 포인트 4개를 큰 구 마커로 발행 (디버깅용).
        path start(=차량, 청록), obstacle start(노랑), obstacle end(주황), path end(자홍).
        모두 raceline(d=0) 위에 찍는다."""
        pts_s = np.array([start_s, obs_start_u, obs_end_u, end_s]) % self.scaled_max_s
        try:
            resp = self.converter.get_cartesian(pts_s, np.zeros_like(pts_s))
        except Exception:
            return
        resp = np.asarray(resp, dtype=float)
        xy = resp.T if resp.shape[0] == 2 else resp
        colors = [(0.0, 1.0, 1.0), (1.0, 1.0, 0.0), (1.0, 0.5, 0.0), (1.0, 0.0, 1.0)]
        # Fixed 4 ids, always overwritten in place (no DELETEALL, no lifetime) so the spheres
        # sit steady instead of flickering when this is called every frame. Cleared explicitly
        # by clear_all_markers() when overtaking fully stops.
        mrks = MarkerArray()
        for i in range(min(len(xy), 4)):
            mrk = Marker(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"))
            mrk.ns = "avoidance_start_end"
            mrk.id = i
            mrk.type = Marker.SPHERE
            mrk.action = Marker.ADD
            mrk.scale.x = mrk.scale.y = mrk.scale.z = 0.4
            mrk.color.a = 0.9
            mrk.color.r, mrk.color.g, mrk.color.b = colors[i]
            mrk.pose.position.x = float(xy[i][0])
            mrk.pose.position.y = float(xy[i][1])
            mrk.pose.position.z = 0.2
            mrk.pose.orientation.w = 1.0
            mrks.markers.append(mrk)
        self.avoidance_pts_pub.publish(mrks)

    def lane_viz_loop(self):
        """Fixed-rate republish of the cached lane markers and current offset."""
        if self._center_xy is None:
            return
        self.publish_lane_markers(self._center_xy, self._outer_xy, self._inner_xy)
        self.lane_offset_pub.publish(Float32MultiArray(data=[float(self.lane_offset), float(-self.lane_offset)]))

    def publish_lane_markers(self, center_xy, outer_xy, inner_xy):
        """Publish center / outer(left) / inner(right) lanes as colored line strips.
        Center: white, outer/left: green, inner/right: red."""
        mrks = MarkerArray()
        specs = [
            ("center", center_xy, (1.0, 1.0, 1.0)),
            ("outer_left", outer_xy, (0.1, 0.9, 0.1)),
            ("inner_right", inner_xy, (0.9, 0.1, 0.1)),
        ]
        for mid, (ns, xy, (r, g, b)) in enumerate(specs):
            mrk = Marker(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"))
            mrk.ns = ns
            mrk.id = mid
            mrk.type = Marker.LINE_STRIP
            mrk.action = Marker.ADD
            mrk.scale.x = 0.03
            mrk.color.a = 1.0
            mrk.color.r, mrk.color.g, mrk.color.b = r, g, b
            mrk.pose.orientation.w = 1.0
            mrk.points = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in xy]
            mrks.markers.append(mrk)
        self.lane_mrks_pub.publish(mrks)

    def clear_all_markers(self):
        """Send DELETEALL to every avoidance marker topic so nothing lingers in rviz when
        we stop overtaking. Also publishes an empty evasion path to reset the merger side."""
        for pub in (self.mrks_pub, self.spline_sample_pub, self.avoidance_pts_pub):
            mrks = MarkerArray()
            del_mrk = Marker(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"))
            del_mrk.action = Marker.DELETEALL
            mrks.markers.append(del_mrk)
            pub.publish(mrks)
        self.evasion_pub.publish(
            OTWpntArray(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"), wpnts=[])
        )

    ### Main Loop ###
    def loop(self):
        start_time = time.perf_counter()

        # Obstacle pre-processing
        obs = deepcopy(self.obs_perception)
        considered_obs = self.obstacle_preprocessing(obs=obs)

        # Overtake only with a fresh GP-trajectory prediction. Perception-only overtaking is
        # unstable, and force_trailing means the prediction is only a constant-velocity fallback
        # (opponent not yet on a learned trajectory) -- in both cases we trail instead of evade.
        evasion_ok = False
        if len(considered_obs) > 0 and self.prediction_is_fresh() and not self.force_trailing:
            evasion_x, evasion_y, evasion_s, evasion_d, evasion_v = self.lane_change(considered_obs, self.current_s)
            # Publish merge reagion if evasion track has been found
            if len(evasion_s) > 0:
                evasion_ok = True
                self.merger_pub.publish(Float32MultiArray(data=[considered_obs[-1].s_end % self.scaled_max_s, evasion_s[-1] % self.scaled_max_s]))

        # Debounce clearing: only wipe markers after several consecutive idle frames so a single
        # failed frame does not flicker the markers off and back on.
        if evasion_ok:
            self.no_evasion_count = 0
        else:
            self.no_evasion_count += 1
            if self.no_evasion_count == self.clear_after_frames:
                self.clear_all_markers()
                self.committed_side = None
                self.pending_side = None
                self.pending_count = 0

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
