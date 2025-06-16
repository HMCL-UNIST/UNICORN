#!/usr/bin/env python3

import threading
import time
import os
import copy
import math
import numpy as np
import rospy
from rospkg import RosPack
from ackermann_msgs.msg import AckermannDriveStamped
from dynamic_reconfigure.msg import Config
from f110_msgs.msg import ObstacleArray, PidData, WpntArray, BehaviorStrategy, Wpnt
from sensor_msgs.msg import LaserScan
from frenet_converter.frenet_converter import FrenetConverter
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String, Float32, Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from combined.src.Controller import Controller
from ftg.ftg import FTG

class Controller_manager:
    """This class is the main controller manager for the car. It is responsible for selecting the correct controller $
    and publishing the corresponding commands to the actuators.
    
    It subscribes to the following topics:
    - /car_state/odom:  get ego car speed
    - /car_state/pose:  get ego car position (x, y, theta)
    - /local_waypoints: get waypoints starting at car's position in map frame
    - /vesc/sensors/imu/raw: get acceleration for steer scaling
    - /car_state/odom_frenet: get ego car frenet coordinates
    - /tracking/obstacles: get opponent information (position, speed, static/dynamic)
    - /state_machine: get state of the car
    - /scan: get lidar scan data

    It publishes the following topics:
    - /lookahead_point: publish the lookahead point for visualization
    - /trailing_opponent_marker: publish the trailing opponent marker for visualization
    - /my_waypoints: publish the waypoints for visualization
    - /l1_distance: publish the l1 distance from the Controller for visualization
    - /vesc/high_level/ackermann_cmd_mux/input/nav_1: publish the steering and speed command
    - /controller/latency: publish the latency of the controller for measuring if launched with measure:=true

    """
    def __init__(self):
        self.name = "control_node"
        rospy.init_node(self.name, anonymous=True)
        self.lock = threading.Lock()
        self.loop_rate = 50 # rate in hertz
        self.ros_time = rospy.Time()
        self.scan = None
        
        self.mapping = rospy.get_param('controller_manager/mapping', False)
        if self.mapping:
            self.init_mapping()
        else:
            self.init_controller()


    def init_controller(self):
        self.racecar_version = rospy.get_param('racecar_version') # NUCX
        self.ctrl_algo = rospy.get_param('controller_manager/ctrl_algo', 'PP') # default controller

        # only for MAP Controller
        self.LUT_name = rospy.get_param('controller_manager/LU_table') # name of lookup table
        rospy.loginfo(f"[{self.name}] Using {self.LUT_name}")
        
        self.use_sim = rospy.get_param('/sim')
        self.wheelbase = rospy.get_param('/vesc/wheelbase', 0.321) # NUCX
        self.measuring = rospy.get_param('/measure', False)
        
        self.state_machine_rate = rospy.get_param('state_machine/rate') #rate in hertz
        self.position_in_map = [] # current position in map frame
        self.position_in_map_frenet = [] # current position in frenet coordinates
        self.waypoint_list_in_map = [] # waypoints starting at car's position in map frame
        self.speed_now = 0 # current speed
        self.acc_now = np.zeros(10) # last 5 accleration values
        self.speed_now_y =0 
        self.yaw_rate = 0 
        self.waypoint_safety_counter = 0

        # Trailing related variables
        self.opponent = [0,0,0,False, True] #s, d, vs, is_static
        self.state = ""
        self.trailing_command = 2
        self.i_gap = 0

        self.converter = None

        # initializing l1 parameter
        # This step could be removed with rospy.wait_for_message() in control loop
        self.t_clip_min = rospy.get_param('L1_controller/t_clip_min')
        self.t_clip_max = rospy.get_param('L1_controller/t_clip_max')
        self.m_l1 = rospy.get_param('L1_controller/m_l1')
        self.q_l1 = rospy.get_param('L1_controller/q_l1')
        self.speed_lookahead = rospy.get_param('L1_controller/speed_lookahead')
        self.lat_err_coeff = rospy.get_param('L1_controller/lat_err_coeff')
        self.acc_scaler_for_steer = rospy.get_param('L1_controller/acc_scaler_for_steer')
        self.dec_scaler_for_steer = rospy.get_param('L1_controller/dec_scaler_for_steer')
        self.start_scale_speed = rospy.get_param('L1_controller/start_scale_speed')
        self.end_scale_speed = rospy.get_param('L1_controller/end_scale_speed')
        self.downscale_factor = rospy.get_param('L1_controller/downscale_factor')
        self.speed_lookahead_for_steer = rospy.get_param('L1_controller/speed_lookahead_for_steer')
        self.trailing_gap = rospy.get_param('L1_controller/trailing_gap')
        self.trailing_vel_gain = rospy.get_param('L1_controller/trailing_vel_gain')
        self.trailing_p_gain = rospy.get_param('L1_controller/trailing_p_gain')
        self.trailing_i_gain = rospy.get_param('L1_controller/trailing_i_gain')
        self.trailing_d_gain = rospy.get_param('L1_controller/trailing_d_gain')
        self.blind_trailing_speed = rospy.get_param('L1_controller/blind_trailing_speed')
        
        # L1 dist calc param
        self.curvature_factor = rospy.get_param('L1_controller/curvature_factor')

        self.speed_factor_for_lat_err = rospy.get_param('L1_controller/speed_factor_for_lat_err')
        self.speed_factor_for_curvature = rospy.get_param('L1_controller/speed_factor_for_curvature')

        # steering_compensation
        self.KP = rospy.get_param('L1_controller/KP')
        self.KI = rospy.get_param('L1_controller/KI')
        self.KD = rospy.get_param('L1_controller/KD')

        self.heading_error_thres = rospy.get_param('L1_controller/heading_error_thres')
        self.steer_gain_for_speed = rospy.get_param('L1_controller/steer_gain_for_speed')

        self.future_constant = rospy.get_param('L1_controller/future_constant')
        
        self.AEB_thres = rospy.get_param('L1_controller/AEB_thres')


        self.speed_diff_thres = rospy.get_param('L1_controller/speed_diff_thres')
        self.start_speed = rospy.get_param('L1_controller/start_speed')
        self.start_curvature_factor = rospy.get_param('L1_controller/start_curvature_factor')

        # Parameters
        for i in range(5):
            # waiting for this message twice, as the republisher needs it first to compute the wanted param
            waypoints = rospy.wait_for_message('/global_waypoints', WpntArray)
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in waypoints.wpnts])

        self.track_length = rospy.get_param("/global_republisher/track_length")

        self.converter = FrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1])


        # FTG
        self.ftg_controller = FTG()
        #  initialize controller

        self.controller = Controller(
            self.t_clip_min, 
            self.t_clip_max, 
            self.m_l1, 
            self.q_l1, 
            
            self.curvature_factor,
            
            self.KP,
            self.KI,
            self.KD,
            self.heading_error_thres,
            self.steer_gain_for_speed,

            self.future_constant,

            self.speed_lookahead, 
            self.lat_err_coeff, 
            self.acc_scaler_for_steer, 
            self.dec_scaler_for_steer, 
            self.start_scale_speed, 
            self.end_scale_speed, 
            self.downscale_factor, 
            self.speed_lookahead_for_steer,

            self.trailing_gap,
            self.trailing_vel_gain,
            self.trailing_p_gain,
            self.trailing_i_gain,
            self.trailing_d_gain,
            self.blind_trailing_speed,

            self.loop_rate,
            self.LUT_name,
            self.wheelbase,

            self.speed_factor_for_lat_err,
            self.speed_factor_for_curvature,
            self.ctrl_algo,

            self.speed_diff_thres,
            self.start_speed,
            self.start_curvature_factor,

            self.AEB_thres,

            self.converter,

            logger_info=rospy.loginfo,
            logger_warn=rospy.logwarn
        )


        # Publishers to view data
        self.lookahead_pub = rospy.Publisher('lookahead_point', Marker, queue_size=10)
        self.future_position_pub = rospy.Publisher('future_position', Marker, queue_size=10)
        self.trailing_pub = rospy.Publisher('trailing_opponent_marker', Marker, queue_size=10)

        self.l1_pub = rospy.Publisher('l1_distance', Point, queue_size=10)    
        # Publisher for steering and speed command
        self.publish_topic = rospy.get_param("~drive_topic", '/vesc/high_level/ackermann_cmd_mux/input/nav_1')
        self.drive_pub = rospy.Publisher(self.publish_topic, AckermannDriveStamped, queue_size=10)
        if self.measuring:
            self.measure_pub = rospy.Publisher('/controller/latency', Float32, queue_size=10)

        
        # Subscribers
        rospy.Subscriber('/behavior_strategy', BehaviorStrategy, self.behavior_cb) # waypoints (x, y, v, norm trackbound, s, kappa)
        rospy.Subscriber('/car_state/odom', Odometry, self.odom_cb) # car speed
        rospy.Subscriber('/car_state/pose', PoseStamped, self.car_state_cb) # car position (x, y, theta)
        rospy.Subscriber('/imu/data', Imu, self.imu_cb) # acceleration subscriber for steer change
        rospy.Subscriber('/car_state/odom_frenet', Odometry, self.car_state_frenet_cb) # car frenet coordinates
        rospy.Subscriber("/dyn_controller/parameter_updates", Config, self.l1_params_cb) #l1 param tuning/updating
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        rospy.Subscriber("/vesc/odom", Odometry, self.vesc_odom_cb)
        rospy.Subscriber("/save_start_traj", Bool, self.save_start_traj_cb)

        self.converter = FrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1])
        rospy.loginfo(f"[{self.name}] initialized FrenetConverter object")
        
    def init_mapping(self):
        rospy.logwarn(f"[{self.name}] Initializing for mapping")
        # Use FTG for mapping
        self.ftg_controller = FTG(mapping=False)
        
        # Publisher
        self.publish_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'
        self.drive_pub = rospy.Publisher(self.publish_topic, AckermannDriveStamped, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/car_state/odom', Odometry, self.odom_mapping_cb) # car speed
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        
        
        rospy.loginfo(f"[{self.name}] initialized for mapping")

    ############################################CALLBACKS############################################
    def save_start_traj_cb(self, msg):
        self.controller.boost_mode = True
        self.controller.cur_state_speed = self.controller.start_speed
        
        
    def scan_cb(self, data: LaserScan):
        self.scan = data
          
    def l1_params_cb(self, params:Config):
        """
        Here the l1 parameters are updated if changed with rqt (dyn reconfigure)
        Values from .yaml file are set in l1_params_server.py      
        """
        self.t_clip_min = rospy.get_param('dyn_controller/t_clip_min')
        self.t_clip_max = rospy.get_param('dyn_controller/t_clip_max')
        self.m_l1 = rospy.get_param('dyn_controller/m_l1')
        self.q_l1 = rospy.get_param('dyn_controller/q_l1')
        self.speed_lookahead = rospy.get_param('dyn_controller/speed_lookahead')
        self.lat_err_coeff = rospy.get_param('dyn_controller/lat_err_coeff')
        self.acc_scaler_for_steer = rospy.get_param('dyn_controller/acc_scaler_for_steer')
        self.dec_scaler_for_steer = rospy.get_param('dyn_controller/dec_scaler_for_steer')
        self.start_scale_speed = rospy.get_param('dyn_controller/start_scale_speed')
        self.end_scale_speed = rospy.get_param('dyn_controller/end_scale_speed')
        self.downscale_factor = rospy.get_param('dyn_controller/downscale_factor')
        self.speed_lookahead_for_steer = rospy.get_param('dyn_controller/speed_lookahead_for_steer')
        self.trailing_gap = rospy.get_param('dyn_controller/trailing_gap')
        self.trailing_vel_gain = rospy.get_param('dyn_controller/trailing_vel_gain')
        self.trailing_p_gain = rospy.get_param('dyn_controller/trailing_p_gain')
        self.trailing_i_gain = rospy.get_param('dyn_controller/trailing_i_gain')
        self.trailing_d_gain = rospy.get_param('dyn_controller/trailing_d_gain')
        self.blind_trailing_speed = rospy.get_param('dyn_controller/blind_trailing_speed')
        self.future_constant = rospy.get_param('dyn_controller/future_constant')
        
        self.speed_diff_thres = rospy.get_param('dyn_controller/speed_diff_thres')
        self.start_speed = rospy.get_param('dyn_controller/start_speed')

        # steering_compensation
        self.KP = rospy.get_param('dyn_controller/KP')
        self.KI = rospy.get_param('dyn_controller/KI')
        self.KD = rospy.get_param('dyn_controller/KD')

        self.heading_error_thres = rospy.get_param('dyn_controller/heading_error_thres')
        self.steer_gain_for_speed = rospy.get_param('dyn_controller/steer_gain_for_speed')

        # L1 dist calc param
        self.curvature_factor = rospy.get_param('dyn_controller/curvature_factor')

        self.AEB_thres = rospy.get_param('dyn_controller/AEB_thres')

        self.speed_factor_for_lat_err = rospy.get_param('dyn_controller/speed_factor_for_lat_err')
        self.speed_factor_for_curvature = rospy.get_param('dyn_controller/speed_factor_for_curvature')

        ## Updating params for map and pp controller
        ## Lateral Control Parameters
        self.controller.t_clip_min = self.t_clip_min  
        self.controller.t_clip_max = self.t_clip_max   
        self.controller.m_l1 = self.m_l1
        self.controller.q_l1 = self.q_l1
        
        self.controller.curvature_factor = self.curvature_factor     

        self.controller.speed_factor_for_lat_err = self.speed_factor_for_lat_err
        self.controller.speed_factor_for_curvature = self.speed_factor_for_curvature
        
        self.controller.KP = self.KP 
        self.controller.KI = self.KI 
        self.controller.KD = self.KD 

        self.controller.heading_error_thres = self.heading_error_thres 
        self.controller.steer_gain_for_speed = self.steer_gain_for_speed 
        
        self.controller.speed_lookahead = self.speed_lookahead
        self.controller.lat_err_coeff = self.lat_err_coeff
        self.controller.acc_scaler_for_steer = self.acc_scaler_for_steer
        self.controller.dec_scaler_for_steer = self.dec_scaler_for_steer
        self.controller.start_scale_speed = self.start_scale_speed
        self.controller.end_scale_speed = self.end_scale_speed
        self.controller.downscale_factor = self.downscale_factor
        self.controller.speed_lookahead_for_steer = self.speed_lookahead_for_steer
        self.controller.future_constant = self.future_constant

        self.controller.speed_diff_thres = self.speed_diff_thres
        self.controller.start_speed = self.start_speed
        self.controller.start_curvature_factor = self.start_curvature_factor

        self.controller.AEB_thres = self.AEB_thres

        ## Trailing Control Parameters
        self.controller.trailing_gap = self.trailing_gap # Distance in meters
        self.controller.trailing_vel_gain = self.trailing_vel_gain # Distance in meters
        self.controller.trailing_p_gain = self.trailing_p_gain
        self.controller.trailing_i_gain = self.trailing_i_gain
        self.controller.trailing_d_gain = self.trailing_d_gain
        self.controller.blind_trailing_speed = self.blind_trailing_speed

    def odom_mapping_cb(self, data: Odometry):
        # velocity for follow the gap (needed to set gap radius)
        self.ftg_controller.set_vel(data.twist.twist.linear.x)

    def odom_cb(self, data: Odometry):
        self.speed_now = data.twist.twist.linear.x
        self.speed_now_y = data.twist.twist.linear.y
        self.controller.speed_now = self.speed_now
        
        # velocity for follow the gap (needed to set gap radius)
        self.ftg_controller.set_vel(data.twist.twist.linear.x)
        
    def vesc_odom_cb(self, data: Odometry):
        self.wheelspeed_now = data.twist.twist.linear.x
        
        # velocity for follow the gap (needed to set gap radius)
        self.ftg_controller.set_vel(data.twist.twist.linear.x)

    def car_state_cb(self, data: PoseStamped):
        x = data.pose.position.x
        y = data.pose.position.y
        theta = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, 
                                       data.pose.orientation.z, data.pose.orientation.w])[2]
        self.position_in_map = np.array([x, y, theta])[np.newaxis]

    def car_state_frenet_cb(self, data: Odometry):
        s = data.pose.pose.position.x
        d = data.pose.pose.position.y
        vs = data.twist.twist.linear.x
        vd = data.twist.twist.linear.y
        self.position_in_map_frenet = np.array([s,d,vs,vd]) 


    def behavior_cb(self, data: BehaviorStrategy):
        if len(data.trailing_targets) != 0:
            opponent= data.trailing_targets[0]
            opponent_s = opponent.s_center
            opponent_d = opponent.d_center
            opponent_vs = opponent.vs
            opponent_visible = opponent.is_visible
            opponent_static = opponent.is_static
            self.opponent = [opponent_s, opponent_d, opponent_vs, opponent_static, opponent_visible]
        else:
            self.opponent = None

        self.waypoint_list_in_map = []
        
        for waypoint in data.local_wpnts:
            waypoint_in_map = [waypoint.x_m, waypoint.y_m]
            speed = waypoint.vx_mps
            if waypoint.d_right + waypoint.d_left != 0:
                self.waypoint_list_in_map.append([waypoint_in_map[0],
                                                  waypoint_in_map[1], 
                                                  speed, 
                                                  min(waypoint.d_left, waypoint.d_right)/(waypoint.d_right + waypoint.d_left), 
                                                  waypoint.s_m, waypoint.kappa_radpm, waypoint.psi_rad, waypoint.ax_mps2, waypoint.d_m]
                                                )
            else:
                self.waypoint_list_in_map.append([waypoint_in_map[0], waypoint_in_map[1], speed, 0, waypoint.s_m, waypoint.kappa_radpm, waypoint.psi_rad, waypoint.ax_mps2, waypoint.d_m])
        self.waypoint_array_in_map = np.array(self.waypoint_list_in_map)
        self.waypoint_safety_counter = 0
        self.state = data.state
        
    def imu_cb(self, data):
        self.acc_now[1:] = self.acc_now[:-1]
        self.acc_now[0] = -data.linear_acceleration.x # vesc is rotated 90 deg, so (-acc_y) == (long_acc)
        
        self.yaw_rate = -data.angular_velocity.z # vesc is rotated 90 deg, so (-acc_y) == (long_acc)
        self.controller.yaw_rate = self.yaw_rate

    ############################################MAIN LOOP############################################

    def control_loop(self):
        rate = rospy.Rate(self.loop_rate)  
        if self.mapping:
            self.mapping_loop(rate)
        else:
            self.controller_loop(rate)
    
    def mapping_loop(self, rate: rospy.Rate):
        rospy.wait_for_message('/scan', LaserScan)
        rospy.wait_for_message('/car_state/odom', Odometry)
        rospy.loginfo(f"[{self.name}] Ready for mapping!")
        
        while not rospy.is_shutdown():
            speed, acceleration, jerk, steering_angle = 0, 0, 0, 0
            speed, steering_angle = self.ftg_controller.process_lidar(self.scan.ranges)
            ack_msg = self.create_ack_msg(speed, acceleration, jerk, steering_angle)
            self.drive_pub.publish(ack_msg)
            rate.sleep()
    
    def controller_loop(self, rate: rospy.Rate):
        rospy.loginfo(f"[{self.name}] Waiting for behavior_strategy")
        rospy.wait_for_message('/behavior_strategy', BehaviorStrategy)
        # rospy.wait_for_message('/local_waypoints', WpntArray)        
        rospy.wait_for_message('/global_waypoints', WpntArray)
        rospy.wait_for_message('/car_state/odom', Odometry)
        # rospy.wait_for_service("convert_glob2frenet_service")
        rospy.loginfo(f"[{self.name}] BehaviorStrategy received")
        rospy.loginfo(f"[{self.name}] Waiting for car_state/pose")
        rospy.wait_for_message('/car_state/pose', PoseStamped)
        self.track_length = rospy.get_param("/global_republisher/track_length")   
        rospy.loginfo(f"[{self.name}] Ready!")

        while not rospy.is_shutdown():
            if self.measuring:
                start = time.perf_counter()
            speed, acceleration, jerk, steering_angle = 0, 0, 0, 0

            #Logic to select controller
            if self.state != "FTGONLY":
                speed, acceleration, jerk, steering_angle = self.controller_cycle()

            else:
                speed, steering_angle = self.ftg_cycle()
                
            if self.measuring:
                end = time.perf_counter()
                self.measure_pub.publish(end-start)
                
            ack_msg = self.create_ack_msg(speed, acceleration, jerk, steering_angle)

            self.drive_pub.publish(ack_msg)
            if self.measuring:
                end = time.perf_counter()
                self.measure_pub.publish(1/(end-start))
            rate.sleep()
    
    def controller_cycle(self):
        speed, acceleration, jerk, steering_angle, L1_point, L1_distance, idx_nearest_waypoint, curvature_waypoints, future_position = self.controller.main_loop(self.state, 
                                                                                                                    self.position_in_map, 
                                                                                                                    self.waypoint_array_in_map, 
                                                                                                                    self.speed_now, 
                                                                                                                    self.opponent, 
                                                                                                                    self.position_in_map_frenet, 
                                                                                                                    self.acc_now,
                                                                                                                    self.track_length)
                
        self.set_lookahead_marker(L1_point, 100)
        self.visualize_steering(steering_angle)
        self.visualize_trailing_opponent()

        self.viz_future_position(future_position, 200)

        self.curvature_waypoints = curvature_waypoints
        self.l1_pub.publish(Point(x=idx_nearest_waypoint, y=L1_distance, z=self.curvature_waypoints))

        
        self.waypoint_safety_counter += 1
        if self.waypoint_safety_counter >= self.loop_rate/self.state_machine_rate* 10: #we can use the same waypoints for 5 cycles
            rospy.logerr_throttle(0.5, f"[{self.name}] Received no local wpnts. STOPPING!!") 
            speed = 0
            steering_angle = 0
        
        return speed, acceleration, jerk, steering_angle
    

    def ftg_cycle(self):
        speed, steer = self.ftg_controller.process_lidar(self.scan.ranges)
        rospy.logwarn(f"[{self.name}] FTGONLY!!!")
        return speed, steer 
        
    def create_ack_msg(self, speed, acceleration, jerk, steering_angle):
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = self.ros_time.now()
        ack_msg.header.frame_id = 'base_link'
        ack_msg.drive.steering_angle = steering_angle
        ack_msg.drive.speed = speed
        ack_msg.drive.jerk = jerk
        ack_msg.drive.acceleration = acceleration
        return ack_msg

############################################MSG CREATION############################################
# visualization utilities
    def visualize_steering(self, theta):
        quaternions = quaternion_from_euler(0, 0, theta)

        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "base_link"
        lookahead_marker.header.stamp = self.ros_time.now()
        lookahead_marker.type = Marker.ARROW
        lookahead_marker.id = 50
        lookahead_marker.scale.x = 0.6
        lookahead_marker.scale.y = 0.05
        lookahead_marker.scale.z = 0
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.lifetime = rospy.Duration()
        lookahead_marker.pose.position.x = 0
        lookahead_marker.pose.position.y = 0
        lookahead_marker.pose.position.z = 0
        lookahead_marker.pose.orientation.x = quaternions[0]
        lookahead_marker.pose.orientation.y = quaternions[1]
        lookahead_marker.pose.orientation.z = quaternions[2]
        lookahead_marker.pose.orientation.w = quaternions[3]
        self.lookahead_pub.publish(lookahead_marker)

    def set_lookahead_marker(self, lookahead_point, id):
        
        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "map"
        lookahead_marker.header.stamp = self.ros_time.now()
        lookahead_marker.type = 2
        lookahead_marker.id = id
        lookahead_marker.scale.x = 0.35
        lookahead_marker.scale.y = 0.35
        lookahead_marker.scale.z = 0.35
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.pose.position.x = lookahead_point[0]
        lookahead_marker.pose.position.y = lookahead_point[1]
        lookahead_marker.pose.position.z = 0

        lookahead_marker.pose.orientation.x = 0
        lookahead_marker.pose.orientation.y = 0
        lookahead_marker.pose.orientation.z = 0
        lookahead_marker.pose.orientation.w = 1

        self.lookahead_pub.publish(lookahead_marker)

    def viz_future_position(self, future_position,id):

        quaternions = quaternion_from_euler(0, 0, future_position[0,2])

        future_position_marker = Marker()
        future_position_marker.header.frame_id = "map"
        future_position_marker.header.stamp = self.ros_time.now()
        future_position_marker.type = Marker.ARROW
        future_position_marker.id = id
        future_position_marker.scale.x = 1.2
        future_position_marker.scale.y = 0.06
        future_position_marker.scale.z = 0
        future_position_marker.color.r = 0.5
        future_position_marker.color.g = 0.0
        future_position_marker.color.b = 0.5
        future_position_marker.color.a = 1.0
        future_position_marker.pose.position.x = future_position[0,0]
        future_position_marker.pose.position.y = future_position[0,1]
        future_position_marker.pose.position.z = 0

        future_position_marker.pose.orientation.x = quaternions[0]
        future_position_marker.pose.orientation.y = quaternions[1]
        future_position_marker.pose.orientation.z = quaternions[2]
        future_position_marker.pose.orientation.w = quaternions[3]

        self.future_position_pub.publish(future_position_marker)



    def set_test_lookahead_marker(self, lookahead_point, id):
        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "map"
        lookahead_marker.header.stamp = self.ros_time.now()
        lookahead_marker.type = 2
        lookahead_marker.id = id
        lookahead_marker.scale.x = 0.35
        lookahead_marker.scale.y = 0.35
        lookahead_marker.scale.z = 0.35
        lookahead_marker.color.r = 0.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 1.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.pose.position.x = lookahead_point[0]
        lookahead_marker.pose.position.y = lookahead_point[1]
        lookahead_marker.pose.position.z = 0
        lookahead_marker.pose.orientation.x = 0
        lookahead_marker.pose.orientation.y = 0
        lookahead_marker.pose.orientation.z = 0
        lookahead_marker.pose.orientation.w = 1
        self.lookahead_pub.publish(lookahead_marker)

    def visualize_trailing_opponent(self):
        if(self.state == "TRAILING" and (self.opponent is not None)):
            on = True
        else:
            on = False
        opponent_marker = Marker()
        opponent_marker.header.frame_id = "map"
        opponent_marker.header.stamp = self.ros_time.now()
        opponent_marker.type = 2
        opponent_marker.scale.x = 0.3
        opponent_marker.scale.y = 0.3
        opponent_marker.scale.z = 0.3
        opponent_marker.color.r = 1.0
        opponent_marker.color.g = 0.0
        opponent_marker.color.b = 0.0
        opponent_marker.color.a = 1.0
        if self.opponent is not None:
            pos = self.converter.get_cartesian([self.opponent[0]], [self.opponent[1]])
            opponent_marker.pose.position.x = pos[0]
            opponent_marker.pose.position.y = pos[1]
            opponent_marker.pose.position.z = 0

        opponent_marker.pose.orientation.x = 0
        opponent_marker.pose.orientation.y = 0
        opponent_marker.pose.orientation.z = 0
        opponent_marker.pose.orientation.w = 1
        if on == False:
            opponent_marker.action = Marker.DELETE
        self.trailing_pub.publish(opponent_marker)


if __name__ == "__main__":
    controller_manager = Controller_manager()
    controller_manager.control_loop()
 
