#!/usr/bin/env python3

import rospy
from f110_msgs.msg import LapData, WpntArray
from std_msgs.msg import Float32
from std_msgs.msg import Empty as EmptyMsg

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker

from std_srvs.srv import EmptyResponse
from std_srvs.srv import Empty as EmptySrv
from collections import deque
import numpy as np

import subprocess # save map
from rospkg import RosPack
from datetime import datetime
import os

class LapAnalyser:
    def __init__(self):

        # Wait for state machine to start to figure out where to place the visualization message
        self.vis_pos = Pose
        msg: Marker = rospy.wait_for_message("/state_marker", Marker, timeout=None)
        if msg is not None:
            self.vis_pos = msg.pose
        self.vis_pos.position.z += 1.5  # appear on top of the state marker
        self.vis_pos.position.y += 2.0
        rospy.loginfo(f"LapAnalyser will be centered at {self.vis_pos.position.x}, {self.vis_pos.position.y}, {self.vis_pos.position.z}")

        # stuff for min distance to track boundary
        self.wp_flag = False
        self.car_distance_to_boundary = []
        self.global_lateral_waypoints = None
        rospy.loginfo("[LapAnalyser] Waiting for /global_waypoints topic")
        rospy.wait_for_message('/global_waypoints', WpntArray)
        rospy.loginfo("[LapAnalyser] Ready to go")
        rospy.Subscriber('/global_waypoints', WpntArray, self.waypoints_cb) # TODO maybe add wait for topic/timeout?

        while self.global_lateral_waypoints is None:
            print("[Lap Analyzer] Waiting for global lateral waypoints")
            rospy.sleep(0.1)

        self.latest_odom = None
        self.odom_points = []  # to store current odom (x,y) along with d and dist_to_boundary

        # publishes once when a lap is completed
        self.lap_data_pub = rospy.Publisher('lap_data', LapData, queue_size=10)
        self.min_car_distance_to_boundary_pub = rospy.Publisher('min_car_distance_to_boundary', Float32, queue_size=10) # publishes every time a new car position is received
        self.lap_start_time = rospy.Time.now()
        self.last_s = 0
        self.accumulated_error = 0
        self.max_error = 0
        self.n_datapoints = 0
        self.lap_count = -1
        self.global_waypoints_len = 0

        self.NUM_LAPS_ANALYSED = 10
        '''The number of laps to analyse and compute statistics for'''
        self.lap_time_acc = deque(maxlen=self.NUM_LAPS_ANALYSED)
        self.lat_err_acc = deque(maxlen=self.NUM_LAPS_ANALYSED)
        self.max_lat_err_acc = deque(maxlen=self.NUM_LAPS_ANALYSED)

        self.LOC_METHOD = rospy.get_param("~loc_algo", default="slam")

        # Start Line init
        self.start_line = 0

        # Reset start line
        rospy.Subscriber('update_start_line', Float32, self.update_start_line_cb)

        rospy.Subscriber('/car_state/odom_frenet', Odometry, self.frenet_odom_cb)  # car odom in frenet frame

        rospy.Subscriber('/lap_analyser/start', EmptyMsg, self.start_log_cb)

        # New subscriber for x,y coordinate odom (assumed available)
        rospy.Subscriber('/car_state/odom', Odometry, self.odom_xy_cb)

        # Publish stuff to RViz
        self.lap_data_vis = rospy.Publisher('lap_data_vis', Marker, queue_size=5)

        # New publisher for odom trajectory and other markers
        self.lap_marker_pub = rospy.Publisher('lap_marker', Marker, queue_size=5)

        # Open up logfile
        self.logfile_name = f"lap_analyzer_{datetime.now().strftime('%d%m_%H%M')}.txt"
        self.logfile_par = os.path.join(RosPack().get_path('lap_analyser'), 'data')
        self.logfile_dir = os.path.join(self.logfile_par, self.logfile_name)
        if not os.path.exists(self.logfile_par):
            os.makedirs(self.logfile_par)
        with open(self.logfile_dir, 'w') as f:
            f.write(f"Laps done on " + datetime.now().strftime('%d %b %H:%M:%S') + '\n')


    def waypoints_cb(self, data: WpntArray):
        """
        Callback function of /global_waypoints subscriber.

        Parameters
        ----------
        data
            Data received from /global_waypoints topic
        """
        if not self.wp_flag:
            # Store original waypoint array
            self.global_lateral_waypoints = np.array([
                [w.s_m, w.d_right, w.d_left] for w in data.wpnts
            ]) 
            self.global_waypoints_len = 0.1 * int(len(self.global_lateral_waypoints))
            self.wp_flag = True
        else:
            pass

    def odom_xy_cb(self, msg):
        # Callback for x,y coordinate odom; simply store the latest message
        self.latest_odom = msg

    # def frenet_odom_cb(self, msg):
    #     if not self.wp_flag:
    #         return

    #     current_s = msg.pose.pose.position.x
    #     current_d = msg.pose.pose.position.y
    #     if self.check_for_finish_line_pass(current_s):
    #         if (self.lap_count == -1):
    #             self.lap_start_time = rospy.Time.now()
    #             rospy.loginfo("LapAnalyser: started first lap")
    #             self.lap_count = 0
    #         else:
    #             self.lap_count += 1
    #             self.publish_lap_info()

    #             if self.lap_count >= 2:  
    #                 self.publish_min_distance()
    #                 # Publish the stored odom trajectory as markers on the separate publisher
    #                 self.publish_odom_marker()

    #             # Reset stored odom trajectory after publishing
    #             self.odom_points = []
    #             self.car_distance_to_boundary = []
    #             self.lap_start_time = rospy.Time.now()
    #             self.max_error = abs(current_d)
    #             self.accumulated_error = abs(current_d)
    #             self.n_datapoints = 1

    #             # Compute and publish statistics. Perhaps publish to a file?
    #             if self.lap_count > 0 and self.lap_count % self.NUM_LAPS_ANALYSED == 0:
    #                 lap_time_str = f"Lap time over the past {self.NUM_LAPS_ANALYSED} laps: Mean: {np.mean(self.lap_time_acc):.4f}, Std: {np.std(self.lap_time_acc):.4f}"
    #                 avg_err_str = f"Avg Lat Error over the past {self.NUM_LAPS_ANALYSED} laps: Mean: {np.mean(self.lat_err_acc):.4f}, Std: {np.std(self.lat_err_acc):.4f}"
    #                 max_err_str = f"Max Lat Error over the past {self.NUM_LAPS_ANALYSED} laps: Mean: {np.mean(self.max_lat_err_acc):.4f}, Std: {np.std(self.max_lat_err_acc):.4f}"
    #                 rospy.logwarn(lap_time_str)
    #                 rospy.logwarn(avg_err_str)
    #                 rospy.logwarn(max_err_str)

    #                 with open(self.logfile_dir, 'a') as f:
    #                     f.write(lap_time_str+'\n')
    #                     f.write(avg_err_str+'\n')
    #                     f.write(max_err_str+'\n')

    #                 # // This was to check for map shift during SE/Loc/Sensor experiments. Not needed during the race.
    #                 # if self.LOC_METHOD == "slam":
    #                 #     # Create map folder
    #                 #     self.map_name = f"map_{datetime.now().strftime('%d%m_%H%M')}"
    #                 #     self.map_dir = os.path.join(RosPack().get_path('lap_analyser'), 'maps', self.map_name)
    #                 #     os.makedirs(self.map_dir)
    #                 #     rospy.loginfo(f"LapAnalyser: Saving map to {self.map_dir}")

    #                 #     subprocess.run(f"rosrun map_server map_saver -f {self.map_name} map:=/map_new", cwd=f"{self.map_dir}", shell=True)
    #     else:
    #         self.accumulated_error += abs(current_d)
    #         self.n_datapoints += 1
    #         if self.max_error < abs(current_d):
    #             self.max_error = abs(current_d)
    #     self.last_s = current_s

    #     # search for closest s value: s values of global waypoints do not match the s values of car position exactly
    #     s_ref_line_values = np.array(self.global_lateral_waypoints)[:, 0]
    #     index_of_interest = np.argmin(np.abs(s_ref_line_values - current_s)) # index where s car state value is closest to s ref line

    #     d_right = self.global_lateral_waypoints[index_of_interest, 1] # [w.s_m, w.d_right, w.d_left] 
    #     d_left = self.global_lateral_waypoints[index_of_interest, 2]

    #     dist_to_bound = self.get_distance_to_boundary(current_d, d_left, d_right)
    #     self.car_distance_to_boundary.append(dist_to_bound)  

    #     # Continuously store the current odom (x,y) along with d and dist_to_boundary if available
    #     if self.latest_odom is not None:
    #         x = self.latest_odom.pose.pose.position.x
    #         y = self.latest_odom.pose.pose.position.y
    #         self.odom_points.append({'x': x, 'y': y, 'd': current_d, 'dist_to_boundary': dist_to_bound})


    def frenet_odom_cb(self, msg):
        if not self.wp_flag:
            return
        
        if (self.lap_count == -1):
            self.lap_start_time = rospy.Time.now()
            rospy.loginfo("LapAnalyser: started first lap")
            self.lap_count = 0
        
        current_s = msg.pose.pose.position.x
        current_d = msg.pose.pose.position.y
        
        if self.check_for_finish_line_pass(current_s):

            self.lap_count += 1
            self.publish_lap_info()

            if self.lap_count >= 1:  
                self.publish_min_distance()
                # Publish the stored odom trajectory as markers on the separate publisher
                self.publish_odom_marker()

            # Reset stored odom trajectory after publishing
            self.odom_points = []
            self.car_distance_to_boundary = []
            self.lap_start_time = rospy.Time.now()
            self.max_error = abs(current_d)
            self.accumulated_error = abs(current_d)
            self.n_datapoints = 1

            # Compute and publish statistics. Perhaps publish to a file?
            if self.lap_count > 0 and self.lap_count % self.NUM_LAPS_ANALYSED == 0:
                lap_time_str = f"Lap time over the past {self.NUM_LAPS_ANALYSED} laps: Mean: {np.mean(self.lap_time_acc):.4f}, Std: {np.std(self.lap_time_acc):.4f}"
                avg_err_str = f"Avg Lat Error over the past {self.NUM_LAPS_ANALYSED} laps: Mean: {np.mean(self.lat_err_acc):.4f}, Std: {np.std(self.lat_err_acc):.4f}"
                max_err_str = f"Max Lat Error over the past {self.NUM_LAPS_ANALYSED} laps: Mean: {np.mean(self.max_lat_err_acc):.4f}, Std: {np.std(self.max_lat_err_acc):.4f}"
                rospy.logwarn(lap_time_str)
                rospy.logwarn(avg_err_str)
                rospy.logwarn(max_err_str)

                with open(self.logfile_dir, 'a') as f:
                    f.write(lap_time_str+'\n')
                    f.write(avg_err_str+'\n')
                    f.write(max_err_str+'\n')

                # // This was to check for map shift during SE/Loc/Sensor experiments. Not needed during the race.
                # if self.LOC_METHOD == "slam":
                #     # Create map folder
                #     self.map_name = f"map_{datetime.now().strftime('%d%m_%H%M')}"
                #     self.map_dir = os.path.join(RosPack().get_path('lap_analyser'), 'maps', self.map_name)
                #     os.makedirs(self.map_dir)
                #     rospy.loginfo(f"LapAnalyser: Saving map to {self.map_dir}")

                #     subprocess.run(f"rosrun map_server map_saver -f {self.map_name} map:=/map_new", cwd=f"{self.map_dir}", shell=True)

        else:
            self.accumulated_error += abs(current_d)
            self.n_datapoints += 1
            if self.max_error < abs(current_d):
                self.max_error = abs(current_d)
        self.last_s = current_s

        # search for closest s value: s values of global waypoints do not match the s values of car position exactly
        s_ref_line_values = np.array(self.global_lateral_waypoints)[:, 0]
        index_of_interest = np.argmin(np.abs(s_ref_line_values - current_s)) # index where s car state value is closest to s ref line

        d_right = self.global_lateral_waypoints[index_of_interest, 1] # [w.s_m, w.d_right, w.d_left] 
        d_left = self.global_lateral_waypoints[index_of_interest, 2]

        dist_to_bound = self.get_distance_to_boundary(current_d, d_left, d_right)
        self.car_distance_to_boundary.append(dist_to_bound)  

        # Continuously store the current odom (x,y) along with d and dist_to_boundary if available
        if self.latest_odom is not None:
            x = self.latest_odom.pose.pose.position.x
            y = self.latest_odom.pose.pose.position.y
            self.odom_points.append({'x': x, 'y': y, 'd': current_d, 'dist_to_boundary': dist_to_bound})

    def start_log_cb(self, _):
        '''Start logging. Reset all metrics.'''
        rospy.loginfo(
            f"LapAnalyser: Start logging statistics for {self.NUM_LAPS_ANALYSED} laps.")
        self.accumulated_error = 0
        self.max_error = 0
        self.lap_count = -1
        self.n_datapoints = 0

    # def check_for_finish_line_pass(self, current_s):
    #     # detect wrapping of the track, should happen exactly once per round
    #     if (self.last_s - current_s) > 1.0:
    #         return True
    #     else:
    #         return False

        # ? Future extension: would be cool to check for sector times...

    def check_for_finish_line_pass(self, current_s):
        # last_s에서 current_s로 넘어갈 때 출발선(self.start_line)을 지났는지 판단
        current_s_from_start = current_s - self.start_line
        last_s_from_start = self.last_s - self.start_line

        if current_s_from_start < 0:
            current_s_from_start += self.global_waypoints_len

        if last_s_from_start < 0:
            last_s_from_start += self.global_waypoints_len

        if (last_s_from_start - current_s_from_start) > 1.0:
            return True
        else :
            return False

    def update_start_line_cb(self, msg):
        # update start point to current pose s
        if msg.data > 0:
            self.start_line = msg.data
            rospy.loginfo(f"LapAnalyser: Updated start line to current position s={self.start_line:.2f}")
            
            self.reset_lap_stats()
            
        else:
            rospy.logwarn("LapAnalyser: Cannot update start line. Frenet odometry data not available yet.")
        return EmptyResponse()


    def reset_lap_stats(self):
        '''Reset lap data when start point has been updated'''
        rospy.loginfo("LapAnalyser: Resetting lap count and statistics.")
        self.lap_count = -1
        self.lap_time_acc.clear()
        self.lat_err_acc.clear()
        self.max_lat_err_acc.clear()
        self.accumulated_error = 0
        self.max_error = 0
        self.n_datapoints = 0
        self.lap_start_time = rospy.Time.now()
        self.odom_points.clear()
        self.car_distance_to_boundary.clear()


    def publish_lap_info(self):
        msg = LapData()
        msg.lap_time = (rospy.Time.now() - self.lap_start_time).to_sec()
        rospy.loginfo(
            f"LapAnalyser: completed lap #{self.lap_count} in {msg.lap_time}")

        with open(self.logfile_dir, 'a') as f:
            f.write(f"Lap #{self.lap_count}: {msg.lap_time:.4f}" + '\n')

        msg.header.stamp = rospy.Time.now()
        msg.lap_count = self.lap_count
        msg.average_lateral_error_to_global_waypoints = self.accumulated_error / self.n_datapoints
        msg.max_lateral_error_to_global_waypoints = self.max_error
        self.lap_data_pub.publish(msg)

        # append to deques for statistics
        self.lap_time_acc.append(msg.lap_time)
        self.lat_err_acc.append(msg.average_lateral_error_to_global_waypoints)
        self.max_lat_err_acc.append(msg.max_lateral_error_to_global_waypoints)

        mark = Marker()
        mark.header.stamp = rospy.Time.now()
        mark.header.frame_id = 'map'
        mark.id = 0
        mark.ns = 'lap_info'
        mark.type = Marker.TEXT_VIEW_FACING
        mark.action = Marker.ADD
        mark.pose = self.vis_pos
        mark.scale.x = 0.0
        mark.scale.y = 0.0
        mark.scale.z = 0.5  # Upper case A
        mark.color.a = 1.0
        mark.color.r = 0.2
        mark.color.g = 0.2
        mark.color.b = 0.2
        mark.text = f"Lap {self.lap_count:02d} {msg.lap_time:.3f}s"
        self.lap_data_vis.publish(mark)

    def publish_min_distance(self):
        self.min_car_distance_to_boundary = np.min(self.car_distance_to_boundary)
        self.min_car_distance_to_boundary_pub.publish(self.min_car_distance_to_boundary)

    def publish_odom_marker(self):
        # Publish the trajectory marker for the current lap using stored odom points
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.ns = 'lap_trajectory'
        marker.id = 10
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 1.0
        for pt in self.odom_points:
            p = Point()
            p.x = pt['x']
            p.y = pt['y']
            p.z = 0.0
            marker.points.append(p)
        self.lap_marker_pub.publish(marker)

        # Find and publish marker for min distance to boundary point
        if len(self.odom_points) > 0:
            min_idx = np.argmin([pt['dist_to_boundary'] for pt in self.odom_points])
            min_pt = self.odom_points[min_idx]
            marker_min = Marker()
            marker_min.header.stamp = rospy.Time.now()
            marker_min.header.frame_id = 'map'
            marker_min.ns = 'lap_min_dist'
            marker_min.id = 11
            marker_min.type = Marker.SPHERE
            marker_min.action = Marker.ADD
            marker_min.pose.position.x = min_pt['x']
            marker_min.pose.position.y = min_pt['y']
            marker_min.pose.position.z = 0.0
            marker_min.scale.x = 0.3
            marker_min.scale.y = 0.3
            marker_min.scale.z = 0.3
            marker_min.color.r = 1.0
            marker_min.color.g = 0.0
            marker_min.color.b = 0.0
            marker_min.color.a = 1.0
            self.lap_marker_pub.publish(marker_min)

            # Publish text marker for min boundary point
            marker_min_text = Marker()
            marker_min_text.header.stamp = rospy.Time.now()
            marker_min_text.header.frame_id = 'map'
            marker_min_text.ns = 'lap_min_dist_text'
            marker_min_text.id = 13
            marker_min_text.type = Marker.TEXT_VIEW_FACING
            marker_min_text.action = Marker.ADD
            # Position text slightly above the sphere marker
            marker_min_text.pose.position.x = min_pt['x']
            marker_min_text.pose.position.y = min_pt['y'] + 0.5
            marker_min_text.pose.position.z = 0.0
            marker_min_text.scale.z = 0.5  # text height
            marker_min_text.color.r = 0.2
            marker_min_text.color.g = 0.2
            marker_min_text.color.b = 0.2
            marker_min_text.color.a = 1.0
            marker_min_text.text = f"Min boundary: {min_pt['dist_to_boundary']:.2f}m"
            self.lap_marker_pub.publish(marker_min_text)

            # Find and publish marker for max lateral error (max abs(d)) point
            max_idx = np.argmax([abs(pt['d']) for pt in self.odom_points])
            max_pt = self.odom_points[max_idx]
            marker_max = Marker()
            marker_max.header.stamp = rospy.Time.now()
            marker_max.header.frame_id = 'map'
            marker_max.ns = 'lap_max_d'
            marker_max.id = 12
            marker_max.type = Marker.SPHERE
            marker_max.action = Marker.ADD
            marker_max.pose.position.x = max_pt['x']
            marker_max.pose.position.y = max_pt['y']
            marker_max.pose.position.z = 0.0
            marker_max.scale.x = 0.3
            marker_max.scale.y = 0.3
            marker_max.scale.z = 0.3
            marker_max.color.r = 0.0
            marker_max.color.g = 1.0
            marker_max.color.b = 0.0
            marker_max.color.a = 1.0
            self.lap_marker_pub.publish(marker_max)

            # Publish text marker for max lateral error point
            marker_max_text = Marker()
            marker_max_text.header.stamp = rospy.Time.now()
            marker_max_text.header.frame_id = 'map'
            marker_max_text.ns = 'lap_max_d_text'
            marker_max_text.id = 14
            marker_max_text.type = Marker.TEXT_VIEW_FACING
            marker_max_text.action = Marker.ADD
            marker_max_text.pose.position.x = max_pt['x']
            marker_max_text.pose.position.y = max_pt['y'] + 0.5
            marker_max_text.pose.position.z = 0.0
            marker_max_text.scale.z = 0.5
            marker_max_text.color.r = 0.2
            marker_max_text.color.g = 0.2
            marker_max_text.color.b = 0.2
            marker_max_text.color.a = 1.0
            marker_max_text.text = f"Max d: {max_pt['d']:.2f}m"
            self.lap_marker_pub.publish(marker_max_text)

    def get_distance_to_boundary(self, current_d, d_left, d_right):
        """
        comment this function    
        ----------
        Input: 
            current_d: lateral distance to reference line
            d_left: distance from ref. line to left track boundary
            d_right: distance from ref. line to right track boundary
        Output:
            distance: critical distance to track boundary (whichever is smaller, to the right or left)  
        """
        # calculate distance from car to boundary
        car_dist_to_bound_left = d_left - current_d                 
        car_dist_to_bound_right = d_right + current_d  

        # select whichever distance is smaller (to the right or left)
        if car_dist_to_bound_left > car_dist_to_bound_right: # car is closer to right boundary
            return car_dist_to_bound_right
        else:
            return car_dist_to_bound_left


if __name__ == '__main__':
    rospy.init_node('lap_analyser')
    analyser = LapAnalyser()
    while not rospy.is_shutdown():
        rospy.spin()
