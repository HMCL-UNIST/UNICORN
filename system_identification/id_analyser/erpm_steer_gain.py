#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry

class OdomGainCalculator:
    def __init__(self):
        rospy.init_node('odom_gain_calculator', anonymous=True)

        # Odometry subscribers
        rospy.Subscriber("/vesc/odom", Odometry, self.vesc_odom_callback)
        rospy.Subscriber("/car_state/odom", Odometry, self.car_state_odom_callback)

        # Initialize velocity values
        self.vesc_speed = None
        self.car_state_speed = None

        # Get user input for wheelbase and steering angle
        self.wheelbase = 0.321
        self.steering_angle = 0.3

        rospy.spin()

    def vesc_odom_callback(self, msg):
        self.vesc_speed = msg.twist.twist.linear.x
        self.calculate_and_print()

    def car_state_odom_callback(self, msg):
        self.car_state_speed = msg.twist.twist.linear.x
        self.calculate_and_print()

    def calculate_and_print(self):
        if self.vesc_speed is not None and self.car_state_speed is not None:
            # Gain Calculation
            gain = self.vesc_speed / self.car_state_speed if self.car_state_speed != 0 else float('nan')

            # Turning Diameter Calculation
            turning_radius = self.wheelbase / np.tan(self.steering_angle) if np.tan(self.steering_angle) != 0 else float('inf')
            turning_diameter = 2 * turning_radius

            # Clean output
            print(f"\n⚙️  Gain: {gain:.4f}")
            # print(f"🔄 Turning Diameter: {turning_diameter:.4f}m\n")

if __name__ == '__main__':
    try:
        OdomGainCalculator()
    except rospy.ROSInterruptException:
        pass
