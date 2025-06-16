#!/usr/bin/env python3
import time
from typing import List, Any, Tuple
import copy
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Bool
from visualization_msgs.msg import Marker, MarkerArray
from scipy.interpolate import InterpolatedUnivariateSpline as Spline
from scipy.interpolate import BPoly
from scipy.signal import argrelextrema
from dynamic_reconfigure.msg import Config
from f110_msgs.msg import Obstacle, ObstacleArray, OTWpntArray, Wpnt, WpntArray, BehaviorStrategy
from frenet_converter.frenet_converter import FrenetConverter
import tf.transformations as tf_trans
from grid_filter.grid_filter import GridFilter
import trajectory_planning_helpers as tph


class ObstacleSpliner:
    """
    This class implements a ROS node that performs splining around obstacles.

    It subscribes to the following topics:
        - `/tracking/obstacles`: Subscribes to the obstacle array.
        - `/car_state/odom_frenet`: Subscribes to the car state in Frenet coordinates.
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
        self.name = "start_spliner_node"
        rospy.init_node(self.name)

        # initialize the instance variable
        # self.obs = ObstacleArray()
        self.obs_in_interest = None
        self.gb_wpnts = WpntArray()
        self.gb_vmax = None
        self.gb_max_idx = None
        self.gb_max_s = None
        self.cur_s = 0
        self.cur_d = 0
        self.cur_vs = 0
        self.start_target_m = 1.0
        self.gb_scaled_wpnts = WpntArray()
        self.lookahead = 10  # in meters [m]
        self.last_switch_time = rospy.Time.now()
        self.last_ot_side = ""
        self.from_bag = rospy.get_param("/from_bag", False)
        self.measuring = rospy.get_param("/measure", False)
        self.points_without_pose = []
        self.tangents_without_pose = []
        self.sampling_dist = rospy.get_param("/sampling_dist", 20.0)
        self.spline_scale = rospy.get_param("/spline_scale", 0.8)
        self.post_min_dist = rospy.get_param("/post_min_dist", 1.5)
        self.post_max_dist = rospy.get_param("/post_max_dist", 5.0)
        self.kernel_size = rospy.get_param("/kernel_size", 4)
        
        self.map_filter = GridFilter(map_topic="/map", debug=False)
        self.map_filter.set_erosion_kernel_size(self.kernel_size)
        
        # Subscribe to the topics
        # rospy.Subscriber("/tracking/obstacles", ObstacleArray, self.obs_cb)
        rospy.Subscriber("/behavior_strategy", BehaviorStrategy, self.behavior_cb)
        rospy.Subscriber("/car_state/odom_frenet", Odometry, self.state_frenet_cb)
        rospy.Subscriber("/car_state/odom", Odometry, self.state_cb)        
        rospy.Subscriber("/global_waypoints", WpntArray, self.gb_cb)
        rospy.Subscriber("/global_waypoints_scaled", WpntArray, self.gb_scaled_cb)
        # dyn params sub
        self.evasion_dist = 0.65
        self.obs_traj_tresh = 0.3
        self.spline_bound_mindist = 0.2
        self.n_loc_wpnts = 80
        self.width_car = 0.30
        # if not self.from_bag:
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.pose_cb)
        rospy.Subscriber("/save_start_traj", Bool, self.save_start_traj_cb)
        rospy.Subscriber("/dyn_statemachine/parameter_updates", Config, self.dyn_param_cb)

        self.mrks_pub = rospy.Publisher("/planner/start_wpnts/markers", MarkerArray, queue_size=10)
        self.evasion_pub = rospy.Publisher("/planner/start_wpnts", OTWpntArray, queue_size=10)
        # self.closest_obs_pub = rospy.Publisher("/planner/avoidance/considered_OBS", Marker, queue_size=10)
        # self.pub_propagated = rospy.Publisher("/planner/avoidance/propagated_obs", Marker, queue_size=10)
        if self.measuring:
            self.latency_pub = rospy.Publisher("/planner/avoidance/latency", Float32, queue_size=10)


        self.converter = self.initialize_converter()


        # Set the rate at which the loop runs
        self.rate = rospy.Rate(20)  # Hz

    #############
    # CALLBACKS #
    #############
    # Callback for obstacle topic
    # def obs_cb(self, data: ObstacleArray):
    #     self.obs = data

    def save_start_traj_cb(self, pose):
        self.points_without_pose = []
        self.tangents_without_pose = []

    def pose_cb(self, data):
        quat = data.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        euler = tf_trans.euler_from_quaternion(q)

        self.points_without_pose.append([data.pose.position.x, data.pose.position.y])
        self.tangents_without_pose.append([np.cos(euler[2] ), np.sin(euler[2] )])

    def behavior_cb(self, data: BehaviorStrategy):
        if len(data.overtaking_targets)!= 0:
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
        q = [quat.x, quat.y, quat.z, quat.w]
        euler = tf_trans.euler_from_quaternion(q)
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

    # Callback triggered by dynamic spline reconf
    def dyn_param_cb(self, params: Config):
        """
        Notices the change in the parameters and changes spline params
        """
        self.start_target_m = rospy.get_param("dyn_statemachine/start_target_m")


    #############
    # MAIN LOOP #
    #############
    def loop(self):
        # Wait for critical Messages and services
        rospy.loginfo(f"[{self.name}] Waiting for messages and services...")
        rospy.wait_for_message("/global_waypoints", WpntArray)
        rospy.wait_for_message("/global_waypoints_scaled", WpntArray)
        rospy.wait_for_message("/car_state/odom", Odometry)
        # rospy.wait_for_message("/dynamic_spline_tuner_node/parameter_updates", Config)
        rospy.loginfo(f"[{self.name}] Ready!")

        while not rospy.is_shutdown():
            if self.measuring:
                start = time.perf_counter()
            # Sample data
            # obs = self.obs
            gb_scaled_wpnts = self.gb_scaled_wpnts.wpnts
            wpnts = OTWpntArray()
            mrks = MarkerArray()

            # If obs then do splining around it
            # if self.obs_in_interest is not None:
                # obs_in_interest = copy.deepcopy(self.obs_in_interest)
            wpnts, mrks = self.do_spline(obs=copy.deepcopy(self.obs_in_interest), gb_wpnts=gb_scaled_wpnts)
            # Else delete spline markers
            # else:
            #     del_mrk = Marker()
            #     del_mrk.header.stamp = rospy.Time.now()
            #     del_mrk.action = Marker.DELETEALL
            #     mrks.markers.append(del_mrk)

            # Publish wpnts and markers
            if self.measuring:
                end = time.perf_counter()
                self.latency_pub.publish(end - start)
            self.evasion_pub.publish(wpnts)
            self.mrks_pub.publish(mrks)
            self.rate.sleep()
    
    #########
    # UTILS #
    #########
    def initialize_converter(self) -> FrenetConverter:
        """
        Initialize the FrenetConverter object"""
        rospy.wait_for_message("/global_waypoints", WpntArray)

        # Initialize the FrenetConverter object
        converter = FrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1])
        rospy.loginfo(f"[{self.name}] initialized FrenetConverter object")

        return converter

    def _more_space(self, obstacle: Obstacle, gb_wpnts: List[Any], obs_s_idx: int) -> Tuple[str, float]:
        left_gap = abs(gb_wpnts[obs_s_idx].d_left - obstacle.d_left)
        right_gap = abs(gb_wpnts[obs_s_idx].d_right + obstacle.d_right)
        min_space = self.evasion_dist + self.spline_bound_mindist

        if right_gap > min_space and left_gap < min_space:
            # Compute apex distance to the right of the opponent
            d_apex_right = obstacle.d_right - self.evasion_dist
            # If we overtake to the right of the opponent BUT the apex is to the left of the raceline, then we set the apex to 0
            if d_apex_right > 0:
                d_apex_right = 0
            return "right", d_apex_right

        elif left_gap > min_space and right_gap < min_space:
            # Compute apex distance to the left of the opponent
            d_apex_left = obstacle.d_left + self.evasion_dist
            # If we overtake to the left of the opponent BUT the apex is to the right of the raceline, then we set the apex to 0
            if d_apex_left < 0:
                d_apex_left = 0
            return "left", d_apex_left
        else:
            candidate_d_apex_left = obstacle.d_left + self.evasion_dist
            candidate_d_apex_right = obstacle.d_right - self.evasion_dist

            if abs(candidate_d_apex_left) <= abs(candidate_d_apex_right):
                # If we overtake to the left of the opponent BUT the apex is to the right of the raceline, then we set the apex to 0
                if candidate_d_apex_left < 0:
                    candidate_d_apex_left = 0
                return "left", candidate_d_apex_left
            else:
                # If we overtake to the right of the opponent BUT the apex is to the left of the raceline, then we set the apex to 0
                if candidate_d_apex_right > 0:
                    candidate_d_apex_right = 0
                return "right", candidate_d_apex_right

    def do_spline(self, obs: Obstacle, gb_wpnts: WpntArray) -> Tuple[WpntArray, MarkerArray]:
        """
        Creates an evasion trajectory for the closest obstacle by splining between pre- and post-apex points.

        This function takes as input the obstacles to be evaded, and a list of global waypoints that describe a reference raceline.
        It only considers obstacles that are within a threshold of the raceline and generates an evasion trajectory for each of these obstacles.
        The evasion trajectory consists of a spline between pre- and post-apex points that are spaced apart from the obstacle.
        The spatial and velocity components of the spline are calculated using the `Spline` class, and the resulting waypoints and markers are returned.

        Args:
        - obstacles (ObstacleArray): An array of obstacle objects to be evaded.
        - gb_wpnts (WpntArray): A list of global waypoints that describe a reference raceline.
        - state (Odometry): The current state of the car.

        Returns:
        - wpnts (WpntArray): An array of waypoints that describe the evasion trajectory to the closest obstacle.
        - mrks (MarkerArray): An array of markers that represent the waypoints in a visualization format.

        """
        # Return wpnts and markers
        mrks = MarkerArray()
        wpnts = OTWpntArray()
        wpnts.header.stamp = rospy.Time.now()
        wpnts.header.frame_id = "map"
        # Get spacing between wpnts for rough approximations
        wpnt_dist = gb_wpnts[1].s_m - gb_wpnts[0].s_m

        if len(self.points_without_pose) > 0:
            # rospy.logwarn(self.start_target_m)
            s_list = [self.cur_s + self.start_target_m]
            d_list = [0]

            s_array = np.array(s_list)
            d_array = np.array(d_list)
            
            s_array = s_array % self.gb_max_s
            
            s_idx = np.round((s_array / wpnt_dist)).astype(int) % self.gb_max_idx

            danger_flag = False
            resp = self.converter.get_cartesian(s_array, d_array)

            points=[[self.cur_x,self.cur_y]]
            tangents=[[np.cos(self.cur_yaw), np.sin(self.cur_yaw)]]
            
            points.extend(self.points_without_pose[:-1])  # 마지막 항목 제외
            tangents.extend(self.tangents_without_pose[:-1])  # 마지막 항목 제외

            # 이제 마지막 항목을 gb_wpnts에서 가장 가까운 포인트로 찾기
            last_point = self.points_without_pose[-1]  # 마지막 항목
            last_tangent = self.tangents_without_pose[-1]  # 마지막 항목
            s_, d_ = self.converter.get_frenet([last_point[0]], [last_point[1]])
            
            last_s_idx = int((s_[0] / wpnt_dist) % self.gb_max_idx)

            last_wpnt = gb_wpnts[last_s_idx]
            points.append([last_wpnt.x_m, last_wpnt.y_m])
            tangents.append([np.cos(last_wpnt.psi_rad), np.sin(last_wpnt.psi_rad)])  
            # points.extend(self.points_without_pose)
            # tangents.extend(self.tangents_without_pose)


            # tangents = np.dot(tangents, self.spline_scale*np.eye(len(points)))
            tangents = np.dot(tangents, self.spline_scale*np.eye(2))
            points = np.asarray(points)
            nPoints, dim = points.shape

            # Parametrization parameter s.
            dp = np.diff(points, axis=0)                 # difference between points
            dp = np.linalg.norm(dp, axis=1)              # distance between points
            d = np.cumsum(dp)                            # cumsum along the segments
            d = np.hstack([[0],d])                       # add distance from first point
            l = d[-1]                                    # length of point sequence
            nSamples =  int(l/wpnt_dist)                 # number of samples 
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

            n_additional = 40

            xy_additional = np.array([
                (
                    gb_wpnts[(last_s_idx + i + 1) % self.gb_max_idx].x_m,
                    gb_wpnts[(last_s_idx + i + 1) % self.gb_max_idx].y_m
                )
                for i in range(n_additional)
            ])

            samples = np.vstack([samples, xy_additional])


            s_, d_ = self.converter.get_frenet(samples[:, 0], samples[:, 1])

            psi_, kappa_ = tph.calc_head_curv_num.\
                calc_head_curv_num(
                    path=samples,
                    el_lengths=0.1*np.ones(len(samples)-1),
                    is_closed=False
                )
            
            danger_flag = False
            for i in range(samples.shape[0]):
                gb_wpnt_i = int((s_[i] / wpnt_dist) % self.gb_max_idx)
                
                inside = self.map_filter.is_point_inside(samples[i, 0], samples[i, 1])
                if not inside:
                    rospy.loginfo_throttle_identical(
                        2, f"[{self.name}]: Evasion trajectory too close to TRACKBOUNDS, aborting evasion"
                    )            # if abs(evasion_d[i]) > abs(tb_dist) - self.spline_bound_mindist:
                    danger_flag = True
                    break
                outside = True
                # Get V from gb wpnts and go slower if we are going through the inside
                vi = gb_wpnts[gb_wpnt_i].vx_mps if outside else gb_wpnts[gb_wpnt_i].vx_mps * 0.9 # TODO make speed scaling ros param

                wpnts.wpnts.append(
                    self.xyv_to_wpnts(x=samples[i, 0], y=samples[i, 1], s=s_[i], d=d_[i], v=2, psi=psi_[i] + np.pi/2 , kappa= kappa_[i], wpnts=wpnts)
                )
                mrks.markers.append(self.xyv_to_markers(x=samples[i, 0], y=samples[i, 1], v=vi, mrks=mrks))

            # Fill the rest of OTWpnts

            
            if danger_flag:
                wpnts.wpnts = []
                mrks.markers = []
        return wpnts, mrks


    ######################
    # VIZ + MSG WRAPPING #
    ######################
    def xyv_to_markers(self, x:float, y:float, v:float, mrks: MarkerArray) -> Marker:
        mrk = Marker()
        mrk.header.frame_id = "map"
        mrk.header.stamp = rospy.Time.now()
        mrk.type = mrk.CYLINDER
        mrk.scale.x = 0.1
        mrk.scale.y = 0.1
        mrk.scale.z = v / self.gb_vmax
        mrk.color.a = 1.0
        mrk.color.b = 0.75
        mrk.color.r = 0.75
        if self.from_bag:
            mrk.color.g = 0.75

        mrk.id = len(mrks.markers)
        mrk.pose.position.x = x
        mrk.pose.position.y = y
        mrk.pose.position.z = v / self.gb_vmax / 2
        mrk.pose.orientation.w = 1

        return mrk

    def xy_to_point(self, x: float, y: float, opponent=True) -> Marker:
        mrk = Marker()
        mrk.header.frame_id = "map"
        mrk.header.stamp = rospy.Time.now()
        mrk.type = mrk.SPHERE
        mrk.scale.x = 0.5
        mrk.scale.y = 0.5
        mrk.scale.z = 0.5
        mrk.color.a = 0.8
        mrk.color.b = 0.65
        mrk.color.r = 1 if opponent else 0
        mrk.color.g = 0.65

        mrk.pose.position.x = x
        mrk.pose.position.y = y
        mrk.pose.position.z = 0.01
        mrk.pose.orientation.w = 1

        return mrk

    def xyv_to_wpnts(self, s: float, d: float, x: float, y: float, v: float, psi: float, kappa: float ,wpnts: WpntArray) -> Wpnt:
        wpnt = Wpnt()
        wpnt.id = len(wpnts.wpnts)
        wpnt.x_m = x
        wpnt.y_m = y
        wpnt.s_m = s
        wpnt.d_m = d
        wpnt.vx_mps = v
        wpnt.psi_rad = psi
        wpnt.kappa_radpm = kappa
        

        return wpnt

if __name__ == "__main__":
    spliner = ObstacleSpliner()
    spliner.loop()
