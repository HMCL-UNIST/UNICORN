#!/usr/bin/env python3
import time
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from f110_msgs.msg import Wpnt, WpntArray, Obstacle, ObstacleArray, OTWpntArray, OpponentTrajectory, OppWpnt, BehaviorStrategy, PredictionArray
from dynamic_reconfigure.msg import Config
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Float32
from scipy.optimize import minimize
from frenet_converter.frenet_converter import FrenetConverter
from std_msgs.msg import Bool
from copy import deepcopy

from ccma import CCMA
import trajectory_planning_helpers as tph
from tf.transformations import euler_from_quaternion
from scipy.interpolate import BPoly
from grid_filter.grid_filter import GridFilter


class DynamicAvoidanceNode:
    def __init__(self):
        # Initialize node
        rospy.init_node('dynamic_avoidance_node')
        self.rate = rospy.Rate(20)

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
        self.max_kappa = 1/self.min_radius
        self.width_car = 0.30
        self.safety_margin = 0.1
        self.avoidance_resolution = 20
        self.back_to_raceline_before = 5
        self.back_to_raceline_after = 5
        self.obs_traj_tresh = 2

        # Dynamic sovler params
        self.down_sampled_delta_s = 0.1
        self.global_traj_kappas = None

        # ROS Parameters
        self.measure = rospy.get_param("/measure", False)

        # Dynamic reconf params
        self.avoid_static_obs = True

        self.converter = None
        self.global_waypoints = None

        # CCMA init
        self.ccma = CCMA(w_ma=10, w_cc=3)
        
        # Publishers
        self.mrks_pub = rospy.Publisher("/planner/avoidance/markers_sqp", MarkerArray, queue_size=10)
        self.evasion_pub = rospy.Publisher("/planner/avoidance/otwpnts", OTWpntArray, queue_size=10)
        self.merger_pub = rospy.Publisher("/planner/avoidance/merger", Float32MultiArray, queue_size=10)
        if self.measure:
            self.measure_pub = rospy.Publisher("/planner/pspliner_sqp/latency", Float32, queue_size=10)
            
        self.spline_sample_pub = rospy.Publisher("/spline_sample_points", MarkerArray, queue_size=10)

        # Subscribers
        rospy.Subscriber("/tracking/obstacles", ObstacleArray, self.obs_perception_cb)
        rospy.Subscriber("/opponent_prediction/obstacles", ObstacleArray, self.obs_prediction_cb)
        rospy.Subscriber("/opponent_prediction/obstacles_pred", PredictionArray, self.obstacle_prediction_cb)
        rospy.Subscriber("/mpc_controller/ego_prediction", PredictionArray, self.ego_prediction_cb)
        rospy.Subscriber("/car_state/odom_frenet", Odometry, self.state_frenet_cb)
        rospy.Subscriber("/car_state/odom", Odometry, self.state_cartesian_cb)
        rospy.Subscriber("/global_waypoints_scaled", WpntArray, self.scaled_wpnts_cb)
        rospy.Subscriber("/behavior_strategy", BehaviorStrategy, self.behavior_cb)
        rospy.Subscriber("/dynamic_avoidance_tuner_node/parameter_updates", Config, self.dyn_param_cb)
        rospy.Subscriber("/global_waypoints", WpntArray, self.gb_cb)
        rospy.Subscriber("/global_waypoints_updated", WpntArray, self.updated_wpnts_cb)
        rospy.Subscriber("/opponent_trajectory", OpponentTrajectory, self.opponent_trajectory_cb)
        rospy.Subscriber("/ot_section_check", Bool, self.ot_sections_check_cb)

        self.converter = self.initialize_converter()
        
        self.map_filter = GridFilter(map_topic="/map", debug=False)
        self.map_filter.set_erosion_kernel_size(1)


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
        
    def behavior_cb(self, data: BehaviorStrategy):
        self.local_wpnts = np.array([[wpnt.s_m, wpnt.d_m] for wpnt in data.local_wpnts])

    def opponent_trajectory_cb(self, data: OpponentTrajectory):
        self.opponent_waypoints = data.oppwpnts
        self.max_opp_idx = len(data.oppwpnts)-1
        self.opponent_wpnts_sm = np.array([wpnt.s_m for wpnt in data.oppwpnts])

    def ot_sections_check_cb(self, data: Bool):
        self.ot_section_check = data.data

    def dyn_param_cb(self, params: Config):
        self.evasion_dist = rospy.get_param("dynamic_avoidance_tuner_node/evasion_dist", 0.65)
        self.obs_traj_tresh = rospy.get_param("dynamic_avoidance_tuner_node/obs_traj_tresh", 1.5)
        self.spline_bound_mindist = rospy.get_param("dynamic_avoidance_tuner_node/spline_bound_mindist", 0.2)
        self.lookahead = rospy.get_param("dynamic_avoidance_tuner_node/lookahead_dist", 15)
        self.back_to_raceline_after = rospy.get_param("dynamic_avoidance_tuner_node/back_to_raceline_after", 5)

        print(
            f"[Planner] Dynamic reconf triggered new spline params: \n"
            f" Evasion apex distance: {self.evasion_dist} [m],\n"
            f" Obstacle trajectory treshold: {self.obs_traj_tresh} [m]\n"
            f" Spline boundary mindist: {self.spline_bound_mindist} [m]\n"
            f" Lookahead distance: {self.lookahead} [m]\n"
            f" Back to raceline after: {self.back_to_raceline_after} [m]\n"
        )


    ### Common Functions ###
    def initialize_converter(self) -> bool:
            """
            Initialize the FrenetConverter object"""
            rospy.wait_for_message("/global_waypoints", WpntArray)

            # Initialize the FrenetConverter object
            converter = FrenetConverter(self.global_waypoints[:, 0], self.global_waypoints[:, 1])
            rospy.loginfo("[Spliner] initialized FrenetConverter object")

            return converter

    def obstacle_preprocessing(self, obs: ObstacleArray):
        obs.obstacles = sorted(obs.obstacles, key=lambda obs: obs.s_start)
        
        considered_obs = []
        for obs in obs.obstacles:
            if (obs.s_start - self.current_s) % self.scaled_max_s < self.lookahead and abs(obs.d_center - self.current_d) < self.obs_traj_tresh:
                if obs.id == self.obs_prediction_pred.id and len(self.obs_prediction.obstacles) == 20:
                    for opd in self.obs_prediction.obstacles:
                        considered_obs.append(opd)
                else:
                    considered_obs.append(obs)

        return considered_obs

    def more_space(self, obstacle: Obstacle, gb_wpnts, gb_idxs):
        left_gap = abs(gb_wpnts[gb_idxs[0]].d_left - obstacle.d_left)
        right_gap = abs(gb_wpnts[gb_idxs[0]].d_right + obstacle.d_right)
        min_space = self.spline_bound_mindist + self.width_car/2 + self.safety_margin

        if right_gap > min_space and left_gap < min_space:
            # Compute apex distance to the right of the opponent
            d_apex_right = obstacle.d_right - (self.width_car/2 + self.safety_margin + self.evasion_dist)
            # If we overtake to the right of the opponent BUT the apex is to the left of the raceline, then we set the apex to 0
            if d_apex_right > 0 and right_gap < abs(d_apex_right):
                d_apex_right = 0
            return "right", d_apex_right

        elif left_gap > min_space and right_gap < min_space:
            # Compute apex distance to the left of the opponent
            d_apex_left = obstacle.d_left + (self.width_car/2 + self.safety_margin + self.evasion_dist)
            # If we overtake to the left of the opponent BUT the apex is to the right of the raceline, then we set the apex to 0
            if d_apex_left < 0 and left_gap < abs(d_apex_left):
                d_apex_left = 0
            return "left", d_apex_left
        elif left_gap < min_space and right_gap < min_space:
            # rospy.logwarn("No enough gap!")
            return None, 0.0
        else:
            # rospy.logwarn("This happen!")
            return None, 0.0


    ### Visualize SPL Rviz###
    def visualize_dynamic_spliner(self, evasion_s, evasion_d, evasion_x, evasion_y, evasion_v):
        mrks = MarkerArray()
        if len(evasion_s) == 0:
            pass
        else:
            resp = self.converter.get_cartesian(evasion_s, evasion_d)
            for i in range(len(evasion_s)):
                mrk = Marker(header=rospy.Header(stamp=rospy.Time.now(), frame_id="map"))
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
                mrk.pose.orientation.w = 1
                mrks.markers.append(mrk)
            self.mrks_pub.publish(mrks)
            
            
    def visualize_spline_samples(self, x_vals, y_vals):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(zip(x_vals, y_vals)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
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
            marker.lifetime = rospy.Duration(0.2)  # stays for 0.2 sec
            marker_array.markers.append(marker)

        self.spline_sample_pub.publish(marker_array)


    ### Dynamic Spliner Avoidance ###
    def dynamic_spliner(self, considered_obs: list, cur_s: float):
        # Get the initial guess of the overtaking side
        initial_guess_object_start_idx = np.abs(self.scaled_wpnts - considered_obs[0].s_start).argmin()
        initial_guess_object_end_idx = np.abs(self.scaled_wpnts - considered_obs[-1].s_end).argmin()

        # Get array of indexes of the global waypoints overlapping with the ROC
        gb_idxs = np.array(range(initial_guess_object_start_idx, initial_guess_object_start_idx + (initial_guess_object_end_idx - initial_guess_object_start_idx)%self.scaled_max_idx))%self.scaled_max_idx

        if len(gb_idxs) < 20:
            gb_idxs = [int(considered_obs[0].s_center / self.scaled_delta_s + i) % self.scaled_max_idx for i in range(20)]

        side, initial_apex = self.more_space(considered_obs[0], self.scaled_wpnts_msg.wpnts, gb_idxs)
        kappas = np.array([self.scaled_wpnts_msg.wpnts[gb_idx].kappa_radpm for gb_idx in gb_idxs])
        max_kappa = np.max(np.abs(kappas))
        outside = "left" if np.sum(kappas) < 0 else "right"

        # Enlongate the ROC if our initial guess suggests that we are overtaking on the outside
        if side == outside:
            for i in range(len(considered_obs)):
                considered_obs[i].s_end = considered_obs[i].s_end + (considered_obs[i].s_end - considered_obs[i].s_start)%self.max_s_updated * max_kappa * (self.width_car + self.evasion_dist)

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
            rospy.logwarn("s_avoidance is empty! Skipping avoidance.")
            return [], [], [], [], []

        # Get the closest scaled waypoint for every s avoidance point (down sampled)
        scaled_wpnts_indices = np.array([np.abs(self.scaled_wpnts[:, 0] - s % self.scaled_max_s).argmin() for s in s_avoidance])

        # Get the min radius
        # # Clip speed
        clipped_speed = np.clip(self.current_vs, 1.0, a_max=None)
        # Get the minimum of clipped speed and the updated speed of the first waypoints
        radius_speed = min([clipped_speed, self.wpnts_updated[(scaled_wpnts_indices[0])%self.max_idx_updated].vx_mps])
        # Interpolate the min_radius with speeds between 0.2 and 7 m
        self.min_radius = np.interp(radius_speed, [1, 6, 7], [0.2, 2, 4])
        self.max_kappa = 1/self.min_radius

        initial_guess = np.zeros(len(s_avoidance))
        max_obs_idx_center = 0
        for obs in considered_obs:
            side, apex = self.more_space(obs, self.scaled_wpnts_msg.wpnts, gb_idxs)
            if side is None:
                continue
            obs_idx_start = np.abs(s_avoidance - obs.s_start).argmin()
            obs_idx_center = np.abs(s_avoidance - obs.s_center).argmin()
            obs_idx_end = np.abs(s_avoidance - obs.s_end).argmin()
            if obs_idx_start >= obs_idx_end or obs_idx_end >= len(s_avoidance):
                continue

            initial_guess[obs_idx_center] = apex

            if obs_idx_center > max_obs_idx_center:
                max_obs_idx_center = obs_idx_center

        resp = self.converter.get_cartesian(s_avoidance, initial_guess)
        resp = resp.transpose()
        
        psi_resp, kappa_resp = tph.calc_head_curv_num.calc_head_curv_num(
                path=resp,
                el_lengths=0.1*np.ones(len(resp)-1),
                is_closed=False,
                stepsize_curv_preview = 5.0,
                stepsize_curv_review = 5.0
            )
        psi_resp += np.pi / 2
        
        nonzero_apex_indices = np.where(initial_guess != 0.0)[0]
        
        points=[[self.current_x, self.current_y]]
        tangents=[[np.cos(self.current_yaw), np.sin(self.current_yaw)]]
        
        for i in nonzero_apex_indices:
            points.append(resp[i])
            tangents.append([np.cos(psi_resp[i]), np.sin(psi_resp[i])])

        temp_s = s_avoidance[-1] + max(considered_obs[0].s_start - self.current_s, self.back_to_raceline_after)
        temp_d = 0.0
        temp_resp = self.converter.get_cartesian(temp_s, temp_d)
        temp_resp = temp_resp.T if temp_resp.ndim == 2 else temp_resp

        points.append(temp_resp.tolist())
        temp_s_mod = temp_s % self.scaled_max_s
        temp_idx = np.argmin(np.abs(np.array([wpnt.s_m for wpnt in self.scaled_wpnts_msg.wpnts]) - temp_s_mod))
        psi_rad = self.scaled_wpnts_msg.wpnts[temp_idx].psi_rad
        tangents.append([np.cos(psi_rad), np.sin(psi_rad)])
        
        tangents = np.dot(tangents, 1.0*np.eye(2))
        points = np.asarray(points)
        nPoints, dim = points.shape

        # Parametrization parameter s.
        dp = np.diff(points, axis=0)                 # difference between points
        dp = np.linalg.norm(dp, axis=1)              # distance between points
        d = np.cumsum(dp)                            # cumsum along the segments
        d = np.hstack([[0],d])                       # add distance from first point
        l = d[-1]                                    # length of point sequence
        nSamples =  int(l/self.scaled_delta_s)       # number of samples 
        s,r = np.linspace(0,l,nSamples,retstep=True) # sample parameter and step

        # Bring points and (optional) tangent information into correct format.
        assert(len(points) == len(tangents))
        spline_result = np.empty([nPoints, dim], dtype=object)
        for i,ref in enumerate(points):
            t = tangents[i]
            # Either tangent is None or has the same
            # number of dimensions as the point ref.
            assert(t is None or len(t)==dim)
            fuse = list(zip(ref,t) if t is not None else zip(ref,))
            spline_result[i,:] = fuse

        # Compute splines per dimension separately.
        samples = np.zeros([nSamples, dim])
        for i in range(dim):
            poly = BPoly.from_derivatives(d, spline_result[:,i])
            samples[:,i] = poly(s)

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
        evasion_s, evasion_d = zip(*sorted(zip(smoothed_sd_points[0], smoothed_sd_points[1])))
        evasion_x = smoothed_xy_points[:, 0]
        evasion_y = smoothed_xy_points[:, 1]
        evasion_coords = np.column_stack((evasion_x, evasion_y))
        evasion_psi, evasion_kappa = tph.calc_head_curv_num.calc_head_curv_num(
            path=evasion_coords,
            el_lengths=0.1 * np.ones(len(evasion_coords) - 1),
            is_closed=False
        )
        evasion_psi += np.pi / 2
        evasion_v = np.zeros(len(evasion_s))
        
        # Create a new evasion waypoint message
        evasion_wpnts_msg = OTWpntArray(header=rospy.Header(stamp=rospy.Time.now(), frame_id="map"))
        evasion_wpnts = []
        evasion_wpnts = [Wpnt(id=len(evasion_wpnts), s_m=s, d_m=d, x_m=x, y_m=y, psi_rad=p, kappa_radpm=k, vx_mps= v) for x, y, s, d, p, k, v in zip(evasion_x, evasion_y, evasion_s, evasion_d, evasion_psi, evasion_kappa, evasion_v)]
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


    ### Main Loop ###
    def loop(self):
        # Wait for critical Messages and services
        rospy.loginfo("[OBS Spliner] Waiting for messages and services...")
        rospy.wait_for_message("/global_waypoints_scaled", WpntArray)
        rospy.wait_for_message("/car_state/odom", Odometry)
        rospy.wait_for_message("/dynamic_avoidance_tuner_node/parameter_updates", Config)
        rospy.wait_for_message("/behavior_strategy", BehaviorStrategy)
        rospy.loginfo("[OBS Spliner] Ready!")

        while not rospy.is_shutdown():
            start_time = time.perf_counter()

            # Obstacle pre-processing
            obs = deepcopy(self.obs_perception)
            considered_obs = self.obstacle_preprocessing(obs=obs)

            # If there is an obstacle and we are in OT section
            if len(considered_obs) > 0 and self.ot_section_check == True:
                evasion_x, evasion_y, evasion_s, evasion_d, evasion_v = self.dynamic_spliner(considered_obs, self.current_s)
                # Publish merge reagion if evasion track has been found
                if len(evasion_s) > 0:
                    self.merger_pub.publish(Float32MultiArray(data=[considered_obs[-1].s_end%self.scaled_max_s, evasion_s[-1]%self.scaled_max_s]))
            # If there is no point in overtaking anymore delte all markers
            else:
                mrks = MarkerArray()
                del_mrk = Marker(header=rospy.Header(stamp=rospy.Time.now()))
                del_mrk.action = Marker.DELETEALL
                mrks.markers = []
                mrks.markers.append(del_mrk)
                self.mrks_pub.publish(mrks)

            # publish latency
            if self.measure:
                self.measure_pub.publish(Float32(data=time.perf_counter() - start_time))

            self.rate.sleep()


if __name__ == "__main__":
    DynamicAvoidance = DynamicAvoidanceNode()
    DynamicAvoidance.loop()