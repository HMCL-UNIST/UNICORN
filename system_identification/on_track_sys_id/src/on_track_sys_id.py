#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np
import os
import yaml
import csv
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu
from helpers.train_model import nn_train
from datetime import datetime
from tqdm import tqdm

class On_Track_Sys_Id:
    def __init__(self):

        rospy.init_node('on_track_sys_id', anonymous=True)
        ############## YH ##############
        # if rospy.has_param('/racecar_version'):
        # if not rospy.has_param('/racecar_version'):
        # ############## YH ##############
        #     self.racecar_version = rospy.get_param('/racecar_version')
        # else:
        self.racecar_version = rospy.get_param('~racecar_version')

        print("Racecar_version: ", self.racecar_version)

        # self.rate = 70
        self.rate = rospy.get_param('~loop_rate')
        
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('on_track_sys_id')

        self.load_parameters()
        self.setup_data_storage()

        self.loop_rate = rospy.Rate(self.rate)

        self.save_LUT_name = rospy.get_param('~save_LUT_name')
        self.save_LUT_name = f"{self.save_LUT_name}_pacejka"
        
        self.plot_model = rospy.get_param('~plot_model')

        # Get topic names from parameters (with default values if not set)
        odom_topic = rospy.get_param('~odom_topic', '/car_state/odom')
        ackermann_topic = rospy.get_param('~ackermann_cmd_topic', '/vesc/low_level/ackermann_cmd_mux/output')

        # Subscribe to the topics using the loaded parameter values
        rospy.Subscriber(odom_topic, Odometry, self.odom_cb)
        rospy.Subscriber(ackermann_topic, AckermannDriveStamped, self.ackermann_cb)
        
        #--------------------------------------HJ----------------------------------------
        rospy.Subscriber('/imu/data', Imu, self.imu_cb)
        #--------------------------------------HJ----------------------------------------

    def setup_data_storage(self):
        """
        Set up storage for collected data.

        Initializes data storage and related variables based on parameters loaded from 'nn_params'.
        """
        self.data_duration = self.nn_params['data_collection_duration']
        self.timesteps = self.data_duration * self.rate
        self.data = np.zeros((self.timesteps, 5))
        self.counter = 0
        self.current_state = np.zeros(5)

    def load_parameters(self):
        """
        This function loads parameters neural network parameters from 'params/nn_params.yaml' and stores them in self.nn_params.
        """

        yaml_file = os.path.join(self.package_path, 'params/nn_params.yaml')
        with open(yaml_file, 'r') as file:
            self.nn_params = yaml.safe_load(file)
    
    def export_data_as_csv(self):
        """
        Export collected data as a CSV file.

        Prompts the user to confirm exporting data. If confirmed, data is saved as a CSV file
        including velocity components (v_x, v_y), yaw rate (omega), and steering angle (delta).
        """
        # Prompt user for confirmation to export data
        user_input = input("\033[33m[WARN] Press 'Y' and then ENTER to export data as CSV, or press ENTER to continue without dumping: \033[0m")
        if user_input.lower() == 'y':
            data_dir = os.path.join(self.package_path, 'data', self.racecar_version)
            if not os.path.exists(data_dir):
                os.makedirs(data_dir)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_file = os.path.join(data_dir, f'{self.racecar_version}_sys_id_data_{timestamp}.csv')
            
            # Write data to CSV file    
            with open(csv_file, mode='w') as file:
                writer = csv.writer(file)
                writer.writerow(['v_x', 'v_y', 'omega', 'delta'])
                for row in self.data:
                    writer.writerow(row)  # Each row contains v_x, v_y, omega, delta
            rospy.loginfo(f"DATA HAS BEEN EXPORTED TO: {csv_file}")

    def odom_cb(self, msg):
        self.current_state[0] = msg.twist.twist.linear.x
        self.current_state[1] = msg.twist.twist.linear.y
        # self.current_state[2] = msg.twist.twist.angular.z
        
    def ackermann_cb(self, msg):
        self.current_state[3] = msg.drive.steering_angle
        
    def imu_cb(self, msg):
        self.current_state[2] = -msg.angular_velocity.z
        self.current_state[4] = -msg.linear_acceleration.x
        # self.current_state[4] = 0.0


    def collect_data(self):
        """
        Collects data during simulation.

        Adds the current state to the data array and updates the counter.
        Closes the progress bar and logs a message if data collection is complete.
        """
        # if self.current_state[0] > 1.0: # Only collect data when the car is moving
        if self.current_state[0] > 1.5: # Only collect data when the car is moving

            self.data = np.roll(self.data, -1, axis=0)
            self.data[-1] = self.current_state
            self.counter += 1
            self.pbar.update(1)
        if self.counter == self.timesteps + 1:
            self.pbar.close()
            rospy.loginfo("Data collection completed.")
            
    def run_nn_train(self):
        """
        Initiates training of the neural network using collected data.
        """
        rospy.loginfo("Training neural network...")
        nn_train(self.data, self.racecar_version, self.save_LUT_name, self.plot_model)
        
    def loop(self):
        """
        Main loop for data collection, training, and exporting.

        This loop continuously collects data until completion, then runs neural network training
        and exports the collected data as CSV before shutting down the node.
        """
        self.pbar = tqdm(total=self.timesteps, desc='Collecting data', ascii=True)
        while not rospy.is_shutdown():
            self.collect_data()
            if self.counter == self.timesteps + 1:
                self.run_nn_train()
                self.export_data_as_csv()
                
                rospy.signal_shutdown("Training completed. Shutting down...")
            self.loop_rate.sleep()
if __name__ == '__main__':
    sys_id = On_Track_Sys_Id()
    sys_id.loop()