#!/usr/bin/env python3

import rospy
import os
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped    
from std_msgs.msg import Float64    
import trajectory_planning_helpers as tph
from rospkg import RosPack
from f110_msgs.msg import ObstacleArray, OTWpntArray, WpntArray, Wpnt
# from calc_vel_profile import calc_vel_profile
from vel_planner.vel_planner import calc_vel_profile

from copy import deepcopy
class VelocityPlanner:

    def __init__(self):
        """
        Initialize the node, subscribe to topics, create publishers and set up member variables.
        """

        # Initialize the node
        self.name = "velocity_planner"
        rospy.init_node(self.name, anonymous=True)
        
        # self.out_topic  = rospy.get_param("/vesc/out_topic", "low_level/ackermann_cmd_mux/output")
        # self.in_topic  = rospy.get_param("/vesc/in_topic", "high_level/ackermann_cmd_mux/input/nav_1")
        # self.joy_topic  = rospy.get_param("/vesc/joy_topic", "/joy")
        self.rate_hz = rospy.get_param("/vesc/rate_hz", 50.0)
        self.max_speed = rospy.get_param("/vesc/joy_max_speed", 4.0)
        self.max_steer = rospy.get_param("/vesc/joy_max_steer", 0.4)
        self.joy_freshness_threshold = rospy.get_param("/vesc/joy_freshness_threshold", 1.0)

        self.autodrive = None
        self.zero_msg = AckermannDriveStamped()
        self.zero_msg.header.stamp = rospy.Time.now() 
        self.zero_msg.drive.steering_angle = 0
        self.zero_msg.drive.speed = 0
        self.cur_v = 0
        self.prev_del_v = 0 
        self.filtered_acc = 0


        self.a_y_max = rospy.get_param("/vel_planner/a_y_max", 5.0) # friction coefficient
        self.a_x_max = rospy.get_param("/vel_planner/a_x_max", 5.0) # friction coefficient



        self.v_max = rospy.get_param("/vel_planner/v_max", 10.0)
        self.drag_coeff = rospy.get_param("/vel_planner/drag_coeff", 0.0136)
        self.m_veh = rospy.get_param("/vel_planner/m_veh", 3.5)
        self.dyn_model_exp = rospy.get_param("/vel_planner/dyn_model_exp", 1.3)


        ggv_path = os.path.join(RosPack().get_path('stack_master'), 'config', 'gb_optimizer', "veh_dyn_info", "ggv.csv")
        ax_max_path = os.path.join(RosPack().get_path('stack_master'), 'config', 'gb_optimizer', "veh_dyn_info", "ax_max_machines.csv")
        self.ggv, self.ax_max_machines = tph.import_veh_dyn_info.\
            import_veh_dyn_info(ggv_import_path=ggv_path,
                                ax_max_machines_import_path=ax_max_path)
        
        self.ggv[:,1] = self.a_x_max
        self.ggv[:,2] = self.a_y_max

        self.ax_max_machines[:,1][self.ax_max_machines[:,1] > 5.0] = 5.0


        # Subscribe to the topics
        rospy.Subscriber("/local_waypoints", WpntArray, self.wpnts_callback)
        rospy.Subscriber("/car_state/odom", Odometry, self.odom_callback)

        self.speed_pub = rospy.Publisher("vel_planner", Float64, queue_size=10)
        self.acc_pub = rospy.Publisher("/vel_planner_acc", Float64, queue_size=10)

    def odom_callback(self, msg):
        self.cur_v = msg.twist.twist.linear.x

    def wpnts_callback(self, msg):
        # 2. 웨이포인트 경로 정보 준비
        wpnts = msg.wpnts
        # x_list = np.array([wp.x_m for wp in wpnts])
        # y_list = np.array([wp.y_m for wp in wpnts])
        kappa = np.array([wp.kappa_radpm for wp in wpnts])
        print(kappa[:10])
        el_lengths = 0.1 * np.ones(len(kappa)-1)
        # 차량 파라미터와 제약 조건 설정
        vx_profile = calc_vel_profile(
            ax_max_machines=self.ax_max_machines,    # [vx, ax_max] 형태의 numpy 배열
            kappa=kappa,
            el_lengths=el_lengths,
            closed=False,
            drag_coeff=self.drag_coeff,
            m_veh=self.m_veh,
            ggv=self.ggv,                            # [vx, ax_max, ay_max]
            v_max=self.v_max,                        # 최대 속도
            v_start=self.cur_v,
            dyn_model_exp=self.dyn_model_exp
        )
        # 3. 속도 결과를 웨이포인트에 반영
        for i in range(len(vx_profile)):
            wpnts[i].vx_mps = vx_profile[i]

        # speed_lookahead_idx = int(self.cur_v * self.speed_lookahead * 10)
        
        # calculate longitudinal acceleration profile
        # vx_profile_opt_cl = np.append(vx_profile, vx_profile[0])
        ax_profile_opt = tph.calc_ax_profile.calc_ax_profile(vx_profile=vx_profile,
                                                            el_lengths=el_lengths,
                                                            eq_length_output=False)
   
        # print(f"speed_lookahead_idx : {speed_lookahead_idx}")
        # print(f"del speed : {wpnts[speed_lookahead_idx].vx_mps - self.cur_v}")
        # print(f"10 step speed : {vx_profile[:10]}")
        # print(f"10 step accel : {ax_profile_opt[0]}")


        # print(f"speed command : {self.cur_v + ax_profile_opt[0]/4.0}")
        # print(f"accel : {ax_profile_opt[0]}")

        acceleration = ax_profile_opt[0]
        acceleration = np.mean(ax_profile_opt[0:15])
        if abs(acceleration) < 0.1:
            acceleration = 0.0
        # current = 10 * acceleration + 0.9 * self.cur_v + 10.0
        # speed = self.cur_v + acceleration/2.0


        alpha = 0.2  # 0.0 ~ 1.0 사이 값, 작을수록 부드러움
        self.filtered_acc = (1 - alpha) * self.filtered_acc + alpha * acceleration
        if self.filtered_acc > 0:
            target_speed = self.cur_v + self.filtered_acc/8.0
        else:
            target_speed = self.cur_v + self.filtered_acc/1.4

        # self.filtered_acc = target_speed
        # print(current)
        # if self.filtered_speed > 0:
        speed_msg = Float64()
        speed_msg.data = target_speed
        self.speed_pub.publish(speed_msg)
        # else:

            # self.speed_pub.publish(brake_smg)

        brake_msg = Float64()
        brake_msg.data = self.filtered_acc 
        self.acc_pub.publish(brake_msg)

        


if __name__ == '__main__':
    simple_mux = VelocityPlanner()
    rospy.spin()
