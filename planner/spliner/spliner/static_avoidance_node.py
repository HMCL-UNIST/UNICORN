#!/usr/bin/env python3
import time
from typing import List, Any, Tuple
import copy

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    ParameterType,
    SetParametersResult,
)

import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from scipy.interpolate import InterpolatedUnivariateSpline as Spline
from scipy.interpolate import BPoly
from scipy.signal import argrelextrema, savgol_filter
from f110_msgs.msg import Obstacle, ObstacleArray, OTWpntArray, Wpnt, WpntArray, BehaviorStrategy
from frenet_conversion.frenet_converter import FrenetConverter
from transforms3d.euler import quat2euler
from grid_filter.grid_filter import GridFilter
import trajectory_planning_helpers as tph

# --- Evasion path smoothing ---
SMOOTH_OTWPNTS = True
SMOOTH_OTWPNTS_WINDOW = 51       # Savitzky-Golay window (must be odd)
SMOOTH_OTWPNTS_POLYORDER = 2
GB_BLEND_LEN = 40                # waypoints over which to quad-ease back onto the GB line


class ObstacleSpliner(Node):
    """
    This class implements a ROS node that performs splining around static obstacles.

    It subscribes to the following topics:
        - `/behavior_strategy`: Subscribes to the behavior strategy (overtaking targets).
        - `/car_state/odom_frenet`: Subscribes to the car state in Frenet coordinates.
        - `/car_state/odom`: Subscribes to the car state in cartesian coordinates.
        - `/global_waypoints`: Subscribes to global waypoints.
        - `/global_waypoints_scaled`: Subscribes to the scaled global waypoints.

    The node publishes the following topics:
        - `/planner/avoidance/markers`: Publishes spline markers.
        - `/planner/avoidance/otwpnts`: Publishes splined waypoints.
        - `/planner/avoidance/considered_OBS`: Publishes markers for the closest obstacle.
        - `/planner/avoidance/propagated_obs`: Publishes markers for the propagated obstacle.
        - `/planner/avoidance/latency`: Publishes the latency of the spliner node. (only if measuring is enabled)
    """

    def __init__(self):
        """
        Initialize the node, subscribe to topics, and create publishers and service proxies.
        """
        # Initialize the node
        self.name = "static_avoidance_planner"
        super().__init__('static_avoidance_planner')

        # initialize the instance variable
        self.obs_in_interest = None
        self.gb_wpnts = None
        self.gb_vmax = None
        self.gb_max_idx = None
        self.gb_max_s = None
        self.cur_s = None
        self.cur_d = None
        self.cur_vs = None
        self.cur_x = None
        self.cur_y = None
        self.cur_yaw = None
        self.gb_scaled_wpnts = None
        self.waypoints = None
        self.lookahead = 10  # in meters [m]
        self.last_switch_time = self.get_clock().now().to_msg()
        self.last_ot_side = ""

        # Static parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('from_bag', False),
                ('measure', False),
            ])
        self.from_bag = self.get_parameter('from_bag').get_parameter_value().bool_value
        self.measuring = self.get_parameter('measure').get_parameter_value().bool_value

        # dyn params defaults
        self.save_params = False
        self.kernel_size = 8
        self.post_sampling_dist = 5.0
        self.sampling_dist = 5.0
        self.post_min_dist = 1.5
        self.post_max_dist = 5.0
        self.spline_scale = 0.8
        self.evasion_dist = 0.65
        self.obs_traj_tresh = 0.3
        self.spline_bound_mindist = 0.2
        self.kd_obs_pred = 1.0
        self.fixed_pred_time = 0.15
        self.n_loc_wpnts = 80
        self.width_car = 0.30

        self.map_filter = GridFilter(node=self, map_topic="/map", debug=False)
        self.map_filter.set_erosion_kernel_size(self.kernel_size)

        self.declare_all_parameters()
        # Apply loaded params to working members at startup (callback only fires on later set).
        self.dyn_param_cb(self.get_parameters([
            'save_params', 'kernel_size', 'post_sampling_dist', 'post_min_dist',
            'post_max_dist', 'spline_scale', 'evasion_dist', 'obs_traj_tresh',
            'spline_bound_mindist', 'kd_obs_pred', 'fixed_pred_time',
        ]))
        self.add_on_set_parameters_callback(self.dyn_param_cb)

        # Subscribe to the topics
        self.create_subscription(BehaviorStrategy, "/behavior_strategy", self.behavior_cb, 10)
        self.create_subscription(Odometry, "/car_state/odom_frenet", self.state_frenet_cb, 10)
        self.create_subscription(Odometry, "/car_state/odom", self.state_cb, 10)
        self.create_subscription(WpntArray, "/global_waypoints", self.gb_cb, 10)
        self.create_subscription(WpntArray, "/global_waypoints_scaled", self.gb_scaled_cb, 10)

        self.mrks_pub = self.create_publisher(MarkerArray, "/planner/avoidance/markers", 10)
        self.evasion_pub = self.create_publisher(OTWpntArray, "/planner/avoidance/otwpnts", 10)
        self.closest_obs_pub = self.create_publisher(Marker, "/planner/avoidance/considered_OBS", 10)
        self.pub_propagated = self.create_publisher(Marker, "/planner/avoidance/propagated_obs", 10)
        # Debug: per-sample spline bounds-check viz (green=pass, red=fail, blue=unchecked tail)
        self.spline_samples_pub = self.create_publisher(MarkerArray, "/planner/avoidance/spline_samples", 10)
        if self.measuring:
            self.latency_pub = self.create_publisher(Float32, "/planner/avoidance/latency", 10)

        # Wait for critical messages
        self.wait_for_messages()

        self.converter = self.initialize_converter()

        # Set the rate at which the loop runs
        self.create_timer(1.0 / 20.0, self.loop)

    #####################
    # DYNAMIC PARAMETERS #
    #####################
    def declare_all_parameters(self):
        """
        Declare the dynamic-reconfigure tunables (from cfg/dyn_spliner_tuner.cfg) as ROS2
        parameters with proper descriptor ranges.
        """
        def dbl(min_v, max_v, desc=""):
            return ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description=desc,
                floating_point_range=[FloatingPointRange(from_value=float(min_v),
                                                         to_value=float(max_v),
                                                         step=0.001)],
            )

        def intd(min_v, max_v, desc=""):
            return ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description=desc,
                integer_range=[IntegerRange(from_value=int(min_v),
                                            to_value=int(max_v),
                                            step=1)],
            )

        self.declare_parameter('save_params', False,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL,
                                                   description="Save params"))
        self.declare_parameter('kernel_size', 8, intd(1, 20))
        self.declare_parameter('post_sampling_dist', 5.0, dbl(0.5, 20.0))
        self.declare_parameter('post_min_dist', 1.5, dbl(0.5, 3.0))
        self.declare_parameter('post_max_dist', 5.0, dbl(3.0, 20.0))
        self.declare_parameter('spline_scale', 0.8, dbl(0.5, 2.0))
        self.declare_parameter('evasion_dist', 0.6, dbl(0.25, 1.25))
        self.declare_parameter('obs_traj_tresh', 1.0, dbl(0.1, 1.5))
        self.declare_parameter('spline_bound_mindist', 0.30, dbl(0.05, 1.0))
        self.declare_parameter('pre_apex_dist0', 4.0, dbl(0.5, 8.0))
        self.declare_parameter('pre_apex_dist1', 3.0, dbl(0.5, 8.0))
        self.declare_parameter('pre_apex_dist2', 2.0, dbl(0.5, 8.0))
        self.declare_parameter('post_apex_dist0', 4.5, dbl(0.5, 12.0))
        self.declare_parameter('post_apex_dist1', 5.0, dbl(0.5, 12.0))
        self.declare_parameter('post_apex_dist2', 5.5, dbl(0.5, 12.0))
        self.declare_parameter('kd_obs_pred', 1.0, dbl(0.1, 10.0))
        self.declare_parameter('fixed_pred_time', 0.15, dbl(0.0, 1.0))

    # Callback triggered by dynamic spline reconf
    def dyn_param_cb(self, params: List[Parameter]):
        """
        Notices the change in the parameters and changes spline params.
        """
        for param in params:
            if param.name == 'evasion_dist':
                self.evasion_dist = round(param.value * 20) / 20
            elif param.name == 'obs_traj_tresh':
                self.obs_traj_tresh = round(param.value * 20) / 20
            elif param.name == 'spline_bound_mindist':
                self.spline_bound_mindist = round(param.value * 20) / 20
            elif param.name == 'kd_obs_pred':
                self.kd_obs_pred = round(param.value * 20) / 20
            elif param.name == 'fixed_pred_time':
                self.fixed_pred_time = round(param.value * 100) / 100
            elif param.name == 'post_sampling_dist':
                self.sampling_dist = param.value
                self.post_sampling_dist = param.value
            elif param.name == 'spline_scale':
                self.spline_scale = param.value
            elif param.name == 'post_min_dist':
                self.post_min_dist = param.value
            elif param.name == 'post_max_dist':
                self.post_max_dist = param.value
            elif param.name == 'kernel_size':
                self.kernel_size = param.value
                self.map_filter.set_erosion_kernel_size(self.kernel_size)
            elif param.name == 'save_params':
                self.save_params = param.value

        self.get_logger().info(
            f"[{self.name}] evasion apex distance: {self.evasion_dist} [m],\n"
            f" obstacle trajectory treshold: {self.obs_traj_tresh} [m]\n"
            f" obstacle prediciton k_d: {self.kd_obs_pred},    obstacle prediciton constant time: {self.fixed_pred_time} [s] "
        )
        return SetParametersResult(successful=True)

    #############
    # CALLBACKS #
    #############
    def behavior_cb(self, data: BehaviorStrategy):
        if len(data.overtaking_targets) != 0:
            self.obs_in_interest = data.overtaking_targets[0]
        else:
            self.obs_in_interest = None

    def state_frenet_cb(self, data: Odometry):
        self.cur_s = data.pose.pose.position.x
        self.cur_d = data.pose.pose.position.y
        self.cur_vs = data.twist.twist.linear.x

    def state_cb(self, data: Odometry):
        self.cur_x = data.pose.pose.position.x
        self.cur_y = data.pose.pose.position.y
        quat = data.pose.pose.orientation
        # transforms3d uses (w, x, y, z) quaternion ordering
        euler = quat2euler([quat.w, quat.x, quat.y, quat.z])
        self.cur_yaw = euler[2]

    # Callback for global waypoint topic
    def gb_cb(self, data: WpntArray):
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in data.wpnts])
        self.gb_wpnts = data
        if self.gb_vmax is None:
            self.gb_vmax = np.max(np.array([wpnt.vx_mps for wpnt in data.wpnts]))
            self.gb_max_idx = data.wpnts[-1].id
            self.gb_max_s = data.wpnts[-1].s_m

    # Callback for scaled global waypoint topic
    def gb_scaled_cb(self, data: WpntArray):
        self.gb_scaled_wpnts = data

    #############
    # MAIN LOOP #
    #############
    def loop(self):
        if self.measuring:
            start = time.perf_counter()
        # Sample data
        gb_scaled_wpnts = self.gb_scaled_wpnts.wpnts
        wpnts = OTWpntArray()
        mrks = MarkerArray()

        # If obs then do splining around it
        if self.obs_in_interest is not None:
            wpnts, mrks = self.do_spline(obs=copy.deepcopy(self.obs_in_interest), gb_wpnts=gb_scaled_wpnts)
        # Else delete spline markers
        else:
            del_mrk = Marker()
            del_mrk.header.stamp = self.get_clock().now().to_msg()
            del_mrk.action = Marker.DELETEALL
            mrks.markers.append(del_mrk)

        # Publish wpnts and markers
        if self.measuring:
            end = time.perf_counter()
            self.latency_pub.publish(Float32(data=float(end - start)))
        self.evasion_pub.publish(wpnts)
        self.mrks_pub.publish(mrks)

    #########
    # UTILS #
    #########
    def wait_for_messages(self):
        self.get_logger().info(f"[{self.name}] Waiting for messages and services...")
        waitlist = [self.cur_s, self.cur_x, self.gb_wpnts, self.gb_scaled_wpnts]
        while None in waitlist:
            rclpy.spin_once(self)
            waitlist = [self.cur_s, self.cur_x, self.gb_wpnts, self.gb_scaled_wpnts]
        self.get_logger().info(f"[{self.name}] Ready!")

    def initialize_converter(self) -> FrenetConverter:
        """
        Initialize the FrenetConverter object"""
        waypoint_array = self.gb_wpnts.wpnts
        waypoints_x = np.array([wpnt.x_m for wpnt in waypoint_array])
        waypoints_y = np.array([wpnt.y_m for wpnt in waypoint_array])
        waypoints_psi = np.array([wpnt.psi_rad for wpnt in waypoint_array])
        converter = FrenetConverter(waypoints_x, waypoints_y, waypoints_psi)
        self.get_logger().info(f"[{self.name}] initialized FrenetConverter object")
        return converter

    def _more_space(self, obstacle: Obstacle, gb_wpnts: List[Any], obs_s_idx: int) -> Tuple[str, float]:
        # Evade toward the side with more empty space. f1tenth frenet: +d is LEFT, -d is RIGHT.
        # gb_wp.d_left / gb_wp.d_right are POSITIVE wall distances (widths), so the left wall
        # sits at signed +d_left and the right wall at signed -d_right. Obstacle edges =
        # d_center +/- size/2. gap and apex use this convention so the apex never lands on the
        # narrow side.
        gb_wp = gb_wpnts[obs_s_idx]
        obs_radius = obstacle.size / 2

        pos_gap = gb_wp.d_left - (obstacle.d_center + obs_radius)   # free room toward +d (left) wall
        neg_gap = (obstacle.d_center - obs_radius) + gb_wp.d_right  # free room toward -d (right) wall
        min_space = self.evasion_dist + self.spline_bound_mindist

        pos_ok = pos_gap >= min_space
        neg_ok = neg_gap >= min_space
        if pos_ok and not neg_ok:
            side = "left"           # +d
        elif neg_ok and not pos_ok:
            side = "right"          # -d
        else:
            side = "left" if pos_gap >= neg_gap else "right"

        if side == "left":
            d_apex = (obstacle.d_center + obs_radius) + self.evasion_dist
            if d_apex < 0:
                d_apex = 0.0        # never flip across the raceline to the wrong side
            d_apex = min(d_apex, gb_wp.d_left)   # clamp to +d wall (signed +d_left)
        else:
            d_apex = (obstacle.d_center - obs_radius) - self.evasion_dist
            if d_apex > 0:
                d_apex = 0.0
            d_apex = max(d_apex, -gb_wp.d_right)  # clamp to -d wall (signed -d_right)

        return side, d_apex

    def do_spline(self, obs: Obstacle, gb_wpnts: WpntArray) -> Tuple[WpntArray, MarkerArray]:
        """
        Creates an evasion trajectory for a static obstacle by splining between current pose and post-apex points.

        Returns:
        - wpnts (WpntArray): An array of waypoints that describe the evasion trajectory to the closest obstacle.
        - mrks (MarkerArray): An array of markers that represent the waypoints in a visualization format.
        """
        # Return wpnts and markers
        mrks = MarkerArray()
        wpnts = OTWpntArray()
        wpnts.header.stamp = self.get_clock().now().to_msg()
        wpnts.header.frame_id = "map"
        # Get spacing between wpnts for rough approximations
        wpnt_dist = gb_wpnts[1].s_m - gb_wpnts[0].s_m

        # If there are obstacles within the lookahead distance, then we need to generate an evasion trajectory considering the closest one
        if obs.is_static == True:
            pre_dist = (obs.s_center - self.cur_s) % self.gb_max_s

            if pre_dist < 0.5 or pre_dist > self.gb_max_s / 2:
                wpnts.wpnts = []
                mrks.markers = []
                return wpnts, mrks

            obs_s_idx = int(obs.s_center / wpnt_dist) % self.gb_max_idx

            more_space, d_apex = self._more_space(obs, gb_wpnts, obs_s_idx)
            s_list = [obs.s_center]
            d_list = [d_apex]

            post_dist = min(min(max(pre_dist, self.post_min_dist), self.post_max_dist), self.gb_max_s / 2)

            num_post_ref = int((post_dist // self.sampling_dist)) + 1

            for i in range(num_post_ref):
                s_list.append(obs.s_center + post_dist * ((i + 1) / num_post_ref))
                d_list.append((d_apex * (1 - (i + 1) / num_post_ref)))

            s_array = np.array(s_list)
            d_array = np.array(d_list)

            s_array = s_array % self.gb_max_s

            s_idx = np.round((s_array / wpnt_dist)).astype(int) % self.gb_max_idx

            # Choose the correct side and compute the distance to the apex based on left of right of the obstacle

            # Do frenet conversion via conversion service for spline and create markers and wpnts
            danger_flag = False
            resp = self.converter.get_cartesian(s_array, d_array)

            points = [[self.cur_x, self.cur_y]]
            tangents = [[np.cos(self.cur_yaw), np.sin(self.cur_yaw)]]

            for i in range(len(s_idx)):
                points.append(resp[:, i])
                tangents.append(np.array([np.cos(gb_wpnts[s_idx[i]].psi_rad), np.sin(gb_wpnts[s_idx[i]].psi_rad)]))

            tangents = np.dot(tangents, self.spline_scale * np.eye(2))
            points = np.asarray(points)
            nPoints, dim = points.shape

            # Parametrization parameter s.
            dp = np.diff(points, axis=0)                 # difference between points
            dp = np.linalg.norm(dp, axis=1)              # distance between points
            d = np.cumsum(dp)                            # cumsum along the segments
            d = np.hstack([[0], d])                      # add distance from first point
            l = d[-1]                                    # length of point sequence
            nSamples = int(l / wpnt_dist)                # number of samples
            s, r = np.linspace(0, l, nSamples, retstep=True)  # sample parameter and step

            # Bring points and (optional) tangent information into correct format.
            assert(len(points) == len(tangents))
            spline_result = np.empty([nPoints, dim], dtype=object)
            for i, ref in enumerate(points):
                t = tangents[i]
                # Either tangent is None or has the same
                # number of dimensions as the point ref.
                assert(t is None or len(t) == dim)
                fuse = list(zip(ref, t) if t is not None else zip(ref,))
                spline_result[i, :] = fuse

            # Compute splines per dimension separately.
            samples = np.zeros([nSamples, dim])
            for i in range(dim):
                poly = BPoly.from_derivatives(d, spline_result[:, i])
                samples[:, i] = poly(s)

            # Savitzky-Golay smoothing; pin + taper the endpoints so the filter edge
            # transient doesn't kink the car pose / last apex.
            if SMOOTH_OTWPNTS and len(samples) >= SMOOTH_OTWPNTS_WINDOW:
                start_pt = samples[0].copy()
                end_pt = samples[-1].copy()
                samples[:, 0] = savgol_filter(samples[:, 0], SMOOTH_OTWPNTS_WINDOW, SMOOTH_OTWPNTS_POLYORDER)
                samples[:, 1] = savgol_filter(samples[:, 1], SMOOTH_OTWPNTS_WINDOW, SMOOTH_OTWPNTS_POLYORDER)
                blend_len = min(5, len(samples) - 1)
                for bi in range(blend_len):
                    w = bi / blend_len
                    samples[bi] = start_pt * (1 - w) + samples[bi] * w
                samples[0] = start_pt
                for bi in range(blend_len):
                    idx = len(samples) - blend_len + bi
                    w = bi / blend_len
                    samples[idx] = samples[idx] * (1 - w) + end_pt * w
                samples[-1] = end_pt

            n_additional = 100
            xy_additional = np.array([
                (
                    gb_wpnts[(s_idx[-1] + i + 1) % self.gb_max_idx].x_m,
                    gb_wpnts[(s_idx[-1] + i + 1) % self.gb_max_idx].y_m
                )
                for i in range(n_additional)
            ])

            # Quad-ease the spline tail onto the GB line so the handoff to xy_additional is smooth.
            if SMOOTH_OTWPNTS and len(samples) > 0:
                blend_to_gb_len = min(GB_BLEND_LEN, len(samples) - 1)
                for bi in range(blend_to_gb_len):
                    idx = len(samples) - blend_to_gb_len + bi
                    t = (bi + 1) / (blend_to_gb_len + 1)
                    w = t * t  # quadratic easing
                    gb_idx_for_blend = (s_idx[-1] - blend_to_gb_len + bi + 1) % self.gb_max_idx
                    target_pt = np.array([
                        gb_wpnts[gb_idx_for_blend].x_m,
                        gb_wpnts[gb_idx_for_blend].y_m])
                    samples[idx] = samples[idx] * (1 - w) + target_pt * w

            samples = np.vstack([samples, xy_additional])

            s_, d_ = self.converter.get_frenet(samples[:, 0], samples[:, 1])

            psi_, kappa_ = tph.calc_head_curv_num.\
                calc_head_curv_num(
                    path=samples,
                    el_lengths=0.1 * np.ones(len(samples) - 1),
                    is_closed=False
                )

            danger_flag = False
            bounds_check_results = []  # debug: True if sample passed is_point_inside
            for i in range(samples.shape[0]):
                gb_wpnt_i = int((s_[i] / wpnt_dist) % self.gb_max_idx)

                inside = self.map_filter.is_point_inside(samples[i, 0], samples[i, 1])
                bounds_check_results.append(inside)
                if not inside:
                    self.get_logger().info(
                        f"[{self.name}]: Evasion trajectory too close to TRACKBOUNDS, aborting evasion",
                        throttle_duration_sec=2,
                    )
                    danger_flag = True
                    break
                outside = True
                # Get V from gb wpnts and go slower if we are going through the inside
                vi = gb_wpnts[gb_wpnt_i].vx_mps if outside else gb_wpnts[gb_wpnt_i].vx_mps * 0.9  # TODO make speed scaling ros param

                wpnts.wpnts.append(
                    self.xyv_to_wpnts(x=samples[i, 0], y=samples[i, 1], s=s_[i], d=d_[i], v=2, psi=psi_[i] + np.pi / 2, kappa=kappa_[i], wpnts=wpnts)
                )
                mrks.markers.append(self.xyv_to_markers(x=samples[i, 0], y=samples[i, 1], v=vi, mrks=mrks))

            # Debug: visualize every spline sample colored by bounds check
            self._publish_spline_samples_markers(samples, bounds_check_results)

            # Fill the rest of OTWpnts
            if danger_flag:
                wpnts.wpnts = []
                mrks.markers = []
        return wpnts, mrks

    def _publish_spline_samples_markers(self, samples: np.ndarray, bounds_check_results: List[bool]):
        """Debug viz: each spline sample as a sphere. green=passed bounds check,
        red=failed (the point that aborted evasion), blue=unchecked tail point."""
        markers = MarkerArray()
        del_mrk = Marker()
        del_mrk.header.frame_id = "map"
        del_mrk.action = Marker.DELETEALL
        markers.markers.append(del_mrk)
        for i in range(samples.shape[0]):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "spline_samples"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(samples[i, 0])
            marker.pose.position.y = float(samples[i, 1])
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            if i < len(bounds_check_results):
                if bounds_check_results[i]:
                    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 0.8  # green: passed
                else:
                    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 1.0  # red: failed
            else:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.0, 1.0, 0.5  # blue: unchecked tail
            markers.markers.append(marker)
        self.spline_samples_pub.publish(markers)

    def _obs_filtering(self, obstacles: ObstacleArray) -> List[Obstacle]:
        # Only use obstacles that are within a threshold of the raceline, else we don't care about them
        obs_on_traj = [obs for obs in obstacles.obstacles if abs(obs.d_center) < self.obs_traj_tresh]

        # Only use obstacles that within self.lookahead in front of the car
        close_obs = []
        for obs in obs_on_traj:
            obs = self._predict_obs_movement(obs)
            # Handle wraparound
            dist_in_front = (obs.s_center - self.cur_s) % self.gb_max_s
            # dist_in_back = abs(dist_in_front % (-self.gb_max_s)) # distance from ego to obstacle in the back
            if dist_in_front < self.lookahead:
                close_obs.append(obs)
                # Not within lookahead
            else:
                pass
        return close_obs

    def _predict_obs_movement(self, obs: Obstacle, mode: str = "constant") -> Obstacle:
        """
        Predicts the movement of an obstacle based on the current state and mode.

        TODO: opponent prediction should be completely isolated for added modularity

        Args:
            obs (Obstacle): The obstacle to predict the movement for.
            mode (str, optional): The mode for predicting the movement. Defaults to "constant".

        Returns:
            Obstacle: The updated obstacle with the predicted movement.
        """
        # propagate opponent by time dependent on distance
        if (obs.s_center - self.cur_s) % self.gb_max_s < 10:  # TODO make param
            if mode == "adaptive":
                # distance in s coordinate
                cur_s = self.cur_s
                ot_distance = (obs.s_center - cur_s) % self.gb_max_s
                rel_speed = np.clip(self.gb_scaled_wpnts.wpnts[int(cur_s * 10)].vx_mps - obs.vs, 0.1, 10)
                ot_time_distance = np.clip(ot_distance / rel_speed, 0, 5) * 0.5

                delta_s = ot_time_distance * obs.vs
                delta_d = ot_time_distance * obs.vd
                delta_d = -(obs.d_center + delta_d) * np.exp(-np.abs(self.kd_obs_pred * obs.d_center))

            elif mode == "adaptive_velheuristic":
                opponent_scaler = 0.7
                cur_s = self.cur_s
                ego_speed = self.gb_scaled_wpnts.wpnts[int(cur_s * 10)].vx_mps

                # distance in s coordinate
                ot_distance = (obs.s_center - cur_s) % self.gb_max_s
                rel_speed = (1 - opponent_scaler) * ego_speed
                ot_time_distance = np.clip(ot_distance / rel_speed, 0, 5)

                delta_s = ot_time_distance * opponent_scaler * ego_speed
                delta_d = -(obs.d_center) * np.exp(-np.abs(self.kd_obs_pred * obs.d_center))

            # propagate opponent by constant time
            elif mode == "constant":
                delta_s = self.fixed_pred_time * obs.vs
                delta_d = self.fixed_pred_time * obs.vd
                # delta_d = -(obs.d_center+delta_d)*np.exp(-np.abs(self.kd_obs_pred*obs.d_center))

            elif mode == "heuristic":
                # distance in s coordinate
                ot_distance = (obs.s_center - self.cur_s) % self.gb_max_s
                rel_speed = 3
                ot_time_distance = ot_distance / rel_speed

                delta_d = ot_time_distance * obs.vd
                delta_d = -(obs.d_center + delta_d) * np.exp(-np.abs(self.kd_obs_pred * obs.d_center))

            # update
            obs.s_start += delta_s
            obs.s_center += delta_s
            obs.s_end += delta_s
            obs.s_start %= self.gb_max_s
            obs.s_center %= self.gb_max_s
            obs.s_end %= self.gb_max_s

            obs.d_left += delta_d
            obs.d_center += delta_d
            obs.d_right += delta_d

            resp = self.converter.get_cartesian([obs.s_center], [obs.d_center])

            marker = self.xy_to_point(resp[0], resp[1], opponent=True)
            self.pub_propagated.publish(marker)

        return obs

    def _check_ot_side_possible(self, more_space) -> bool:
        if abs(self.cur_d) > 0.25 and more_space != self.last_ot_side:  # TODO make rosparam for cur_d threshold
            self.get_logger().info(f"[{self.name}]: Can't switch sides, because we are not on the raceline")
            return False
        return True

    ######################
    # VIZ + MSG WRAPPING #
    ######################
    def xyv_to_markers(self, x: float, y: float, v: float, mrks: MarkerArray) -> Marker:
        mrk = Marker()
        mrk.header.frame_id = "map"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.type = mrk.CYLINDER
        mrk.scale.x = 0.1
        mrk.scale.y = 0.1
        mrk.scale.z = float(v / self.gb_vmax)
        mrk.color.a = 1.0
        mrk.color.b = 0.75
        mrk.color.r = 0.75
        if self.from_bag:
            mrk.color.g = 0.75

        mrk.id = len(mrks.markers)
        mrk.pose.position.x = float(x)
        mrk.pose.position.y = float(y)
        mrk.pose.position.z = float(v / self.gb_vmax / 2)
        mrk.pose.orientation.w = 1.0

        return mrk

    def xy_to_point(self, x: float, y: float, opponent=True) -> Marker:
        mrk = Marker()
        mrk.header.frame_id = "map"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.type = mrk.SPHERE
        mrk.scale.x = 0.5
        mrk.scale.y = 0.5
        mrk.scale.z = 0.5
        mrk.color.a = 0.8
        mrk.color.b = 0.65
        mrk.color.r = 1.0 if opponent else 0.0
        mrk.color.g = 0.65

        mrk.pose.position.x = float(x)
        mrk.pose.position.y = float(y)
        mrk.pose.position.z = 0.01
        mrk.pose.orientation.w = 1.0

        return mrk

    def xyv_to_wpnts(self, s: float, d: float, x: float, y: float, v: float, psi: float, kappa: float, wpnts: WpntArray) -> Wpnt:
        wpnt = Wpnt()
        wpnt.id = len(wpnts.wpnts)
        wpnt.x_m = float(x)
        wpnt.y_m = float(y)
        wpnt.s_m = float(s)
        wpnt.d_m = float(d)
        wpnt.vx_mps = float(v)
        wpnt.psi_rad = float(psi)
        wpnt.kappa_radpm = float(kappa)

        return wpnt


def main(args=None):
    rclpy.init(args=args)
    spliner = ObstacleSpliner()
    try:
        rclpy.spin(spliner)
    except KeyboardInterrupt:
        pass
    spliner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
