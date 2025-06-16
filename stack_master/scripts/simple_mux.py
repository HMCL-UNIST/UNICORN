#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped    
from copy import deepcopy
class SimpleMuxNode:

    def __init__(self):
        """
        Initialize the node, subscribe to topics, create publishers and set up member variables.
        """

        # Initialize the node
        self.name = "simple_mux"
        rospy.init_node(self.name, anonymous=True)
        
        self.out_topic  = rospy.get_param("/vesc/out_topic", "low_level/ackermann_cmd_mux/output")
        self.in_topic  = rospy.get_param("/vesc/in_topic", "high_level/ackermann_cmd_mux/input/nav_1")
        self.joy_topic  = rospy.get_param("/vesc/joy_topic", "/joy")
        self.rate_hz = rospy.get_param("/vesc/rate_hz", 50.0)
        self.max_speed = rospy.get_param("/vesc/joy_max_speed", 4.0)
        self.max_steer = rospy.get_param("/vesc/joy_max_steer", 0.4)
        self.joy_freshness_threshold = rospy.get_param("/vesc/joy_freshness_threshold", 1.0)
        
        
        vesc_servo_min = rospy.get_param("/vesc/vesc_driver/servo_min", 1.0)
        vesc_servo_max = rospy.get_param("/vesc/vesc_driver/servo_max", 1.0)
        steering_angle_to_servo_offset = rospy.get_param("/vesc/steering_angle_to_servo_offset", 1.0)
        steering_angle_to_servo_gain = rospy.get_param("/vesc/steering_angle_to_servo_gain", 1.0)
        
        servo_max_rad = (vesc_servo_max - steering_angle_to_servo_offset) / steering_angle_to_servo_gain
        servo_min_rad = (vesc_servo_min - steering_angle_to_servo_offset) / steering_angle_to_servo_gain
        
        self.servo_max_abs = min(abs(servo_max_rad), abs(servo_min_rad))

        self.current_host = None
        
        self.human_drive = None
        self.autodrive = None
        self.zero_msg = AckermannDriveStamped()
        self.zero_msg.header.stamp = rospy.Time.now() 
        self.zero_msg.drive.steering_angle = 0
        self.zero_msg.drive.speed = 0
        self.cur_v = 0
        self.prev_del_v = 0
        self.vel_planner = 0
        # Subscribe to the topics
        rospy.Subscriber(self.in_topic, AckermannDriveStamped, self.drive_callback)
        rospy.Subscriber("/vesc/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/vel_planner", Float64, self.planner_callback)

        # Do not use filtered velocity
        # rospy.Subscriber("/car_state/odom", Odometry, self.odom_callback)
        
        rospy.Subscriber(self.joy_topic, Joy, self.joy_callback)
        
        self.drive_pub = rospy.Publisher(self.out_topic, AckermannDriveStamped, queue_size=10)
        self.current_pub = rospy.Publisher("/vesc/commands/motor/current", Float64, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.timer_callback)
        
    def check_uptodate(self, drive_msg):
        # return True
        if drive_msg is None:
            return False
        
        if abs(drive_msg.header.stamp.to_sec() - rospy.Time.now().to_sec()) < self.joy_freshness_threshold:
            return True
        else:
            return False
        
    def clip_servo(self, in_drive_msg):
        drive_msg = AckermannDriveStamped()
        drive_msg = deepcopy(in_drive_msg)
        
        if drive_msg.drive.steering_angle > 0 and drive_msg.drive.steering_angle > self.servo_max_abs:
            drive_msg.drive.steering_angle = self.servo_max_abs
        elif drive_msg.drive.steering_angle < 0 and drive_msg.drive.steering_angle < - self.servo_max_abs:
            drive_msg.drive.steering_angle = -self.servo_max_abs
        
        return drive_msg
        
    def timer_callback(self, event):
        # human_drive_ = self.human_drive
        
        if self.current_host is None:
            return
        if self.current_host == "autodrive" and self.check_uptodate(self.autodrive):
            drive_msg = self.clip_servo(self.autodrive)
            drive_msg.drive.steering_angle *= 1.1
            self.drive_pub.publish(drive_msg)                    
        elif self.current_host == "humandrive" and self.check_uptodate(self.human_drive):
            drive_msg = self.clip_servo(self.human_drive)
            # if drive_msg.drive.speed > 0 and self.cur_v < 3.0:
            #     # current_msg = Float64()
            #     # current_msg.data = 50.0
            #     # self.current_pub.publish(current_msg)
            #     # rospy.logwarn(f"joy_command : {drive_msg.drive.speed}" )

            #     drive_msg.drive.speed = 6.0
            #     # self.drive_pub.publish(drive_msg)
            # else:
            self.drive_pub.publish(drive_msg)
            # self.human_drive = None
        # else:
        #     self.drive_pub.publish(self.zero_msg)
    def planner_callback(self, msg):
        self.vel_planner = msg.data

    def odom_callback(self, msg):
        self.cur_v = msg.twist.twist.linear.x

    def joy_callback(self, msg):
        # prev_host = deepcopy(self.current_host)
        use_human_drive = msg.buttons[4]
        use_controller = msg.buttons[5]
        
        if use_human_drive:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now() 
            drive_msg.drive.steering_angle = msg.axes[3] * self.max_steer
            drive_msg.drive.speed = msg.axes[1] * self.max_speed


            # drive_msg.drive.jerk = 512.0
            # del_v = drive_msg.drive.speed -self.cur_v
            # drive_msg.drive.acceleration = del_v * 4.0 + (del_v - self.prev_del_v) * 0.1

            self.human_drive = drive_msg
            self.current_host = "humandrive"
        elif use_controller:
            self.current_host = "autodrive"

    # def drive_callback(self, msg):
    #     self.autodrive = msg
        

    def drive_callback(self, msg):
        drive_msg = AckermannDriveStamped()
        drive_msg = msg
        # drive_msg.drive.speed = self.vel_planner
        # drive_msg.drive.jerk = 512.0
        # del_v = drive_msg.drive.speed -self.cur_v
        # drive_msg.drive.acceleration = del_v * 4.0 + (del_v - self.prev_del_v) * 0.1
        # if drive_msg.drive.acceleration >0:
        #     drive_msg.drive.steering_angle *= (1+drive_msg.drive.acceleration*0.4)
        # else:
        #     drive_msg.drive.steering_angle *= (1+drive_msg.drive.acceleration*0.2)

        # self.prev_del_v = del_v

        self.autodrive = drive_msg
        


if __name__ == '__main__':
    simple_mux = SimpleMuxNode()
    rospy.spin()
