#!/usr/bin/env python3

import rospy
import tf
import time
from frenet_converter.frenet_converter import FrenetConverter
from frenet_conversion.srv import Glob2FrenetArr, Frenet2GlobArr

from f110_msgs.msg import WpntArray
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point, Point32
from std_msgs.msg import Float32
from bisect import bisect_left
from nav_msgs.msg import Odometry
import math
import copy
import numpy as np
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.msg import Config

from visualization_msgs.msg import Marker,MarkerArray

from f110_msgs.msg import ObstacleArray
from f110_msgs.msg import Obstacle as ObstacleMessage

def normalize_s(x,track_length):
        x = x % (track_length)
        if x > track_length/2:
            x -= track_length
        return x


class Obstacle :
    """
    This class implements the properties of the obstacles
    """
    current_id = 0
    def __init__(self,x,y,size,theta) -> None:
        self.center_x = x
        self.center_y = y
        self.size = size
        self.id = None
        self.theta = theta

class Detect :
    """
    This class implements a ROS node that detects obstacles on the track

    It subscribes to the following topics:
        - `/scan`: Publishes the lidar scans
        - `/global_waypoints`: Publishes the global waypoints
        - `/odom_frenet`: Publishes the car state in frenet frame


    The node publishes the following topics:
        - `/breakpoints_markers`: Publishes the breakpoint markers of the obstacles
        - `/detect_bound`: Publishes the detect boundaries
        - `/raw_obstacles`: Publishes the detected obstacles
        - `/obstacles_markers_new`: Publishes the markers of the detected obstacles
    """
    def __init__(self) -> None:
        """
        Initialize the node, subscribe to topics, and create publishers and service proxies
        """
        self.converter = None

        # --- Node properties ---
        rospy.init_node('StaticDynamic', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self.measuring = rospy.get_param("/measure", False)
        self.from_bag = rospy.get_param("/from_bag", False)

        # --- Publisher ---
        self.breakpoints_markers_pub = rospy.Publisher('/detect/breakpoints_markers', MarkerArray, queue_size=5)
        self.boundaries_pub = rospy.Publisher('/detect/detect_bound', Marker, queue_size=5)
        self.obstacles_msg_pub = rospy.Publisher('/detect/raw_obstacles', ObstacleArray, queue_size=5)
        self.obstacles_marker_pub = rospy.Publisher('/detect/obstacles_markers_new', MarkerArray, queue_size=5)
        if self.measuring:
            self.latency_pub = rospy.Publisher('/detect/latency', Float32, queue_size=5)
            self.on_track_points_pub = rospy.Publisher('/detect/on_track_points', PointCloud2, queue_size=5)
            
        rospy.wait_for_service("convert_glob2frenet_service")
        self.glob2frenet = rospy.ServiceProxy("convert_glob2frenetarr_service", Glob2FrenetArr)
        
        # --- Tunable params ---
        self.rate = rospy.get_param("/perception/rate_detect")
        self.lambda_angle = rospy.get_param("/perception/lambda_deg")*math.pi/180
        self.sigma = rospy.get_param("/perception/sigma")
        self.min_size_m = rospy.get_param("/perception/min_size_m", 0.2)
        self.min_2_points_dist = rospy.get_param("/perception/min_2_points_dist") # prevent zero division

        # --- dyn params sub ---
        self.min_size_n = 10
        self.max_size_m = 0.5
        self.max_viewing_distance = 9
        self.boundaries_inflation = 0.1

        # --- variables ---

        # track variables
        self.waypoints = None
        self.biggest_d = None
        self.smallest_d = None
        self.s_array = None
        self.d_right_array = None
        self.d_left_array = None
        self.track_length = None

        # ego car s position
        self.car_s = 0
        self.T = None
        self.R = None
        # raw scans from the lidar
        self.scans = None

        self.current_stamp = None
        self.tracked_obstacles = []

        self.tf_listener = tf.TransformListener()
        self.path_needs_update = False
        # --- Subscribers ---
        rospy.Subscriber('/scan', LaserScan, self.laserCb)
        rospy.Subscriber('/global_waypoints', WpntArray, self.pathCb)
        rospy.Subscriber('/car_state/odom_frenet', Odometry, self.carStateCb)
        if not self.from_bag:
            rospy.Subscriber("/dyn_perception/parameter_updates", Config, self.dyn_param_cb)
        while(self.waypoints is None):
            rospy.sleep(0.1)
            print("[Opponent Detection]: waiting ...")

        self.converter = self.initialize_converter()

    def shutdown(self):
        rospy.logwarn('Opponent Detection is shutdown')

    def initialize_converter(self):
        """
        Initialize the FrenetConverter object
        """

        # Initialize the FrenetConverter object
        converter = FrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1])
        rospy.loginfo("[Opponent Detection]: initialized FrenetConverter object")

        return converter
    
    # --- Callback functions ---

    def laserCb(self, msg):
        self.scans = msg

    def pathCb(self, data):
        # Initial calls: initialize the converter
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in data.wpnts])
        # Second call: create the boundaries arrays
        if (self.s_array is None or self.path_needs_update) and self.converter is not None:
            rospy.loginfo('[Opponent Detection]: received global path')
            waypoint_array = data.wpnts
            points=[]
            self.s_array = []
            self.d_right_array = []
            self.d_left_array = []
            for waypoint in waypoint_array:
                self.s_array.append(waypoint.s_m)
                self.d_right_array.append(waypoint.d_right-self.boundaries_inflation)
                self.d_left_array.append(waypoint.d_left-self.boundaries_inflation)
                resp = self.converter.get_cartesian(waypoint.s_m,-waypoint.d_right+self.boundaries_inflation)
                points.append(Point(resp[0],resp[1],0))
                resp = self.converter.get_cartesian(waypoint.s_m,waypoint.d_left-self.boundaries_inflation)
                points.append(Point(resp[0],resp[1],0))
            self.smallest_d = min(self.d_right_array+self.d_left_array)
            self.biggest_d = max(self.d_right_array+self.d_left_array)
            self.track_length = data.wpnts[-1].s_m

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.id = 0
            marker.type = marker.SPHERE_LIST
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.a = 1
            marker.color.g = 0.
            marker.color.r = 1.
            marker.color.b = 0.
            marker.points = points

            self.boundaries_pub.publish(marker)
        self.path_needs_update = False

    def carStateCb(self, data):
        self.car_s = data.pose.pose.position.x
    
    def dyn_param_cb(self, params: Config):
        self.min_size_n = rospy.get_param('dyn_perception/min_size_n', 10)
        self.max_size_m = rospy.get_param('dyn_perception/max_size_m', 0.5)
        self.max_viewing_distance = rospy.get_param('dyn_perception/max_viewing_distance', 9)
        self.boundaries_inflation = rospy.get_param('dyn_perception/boundaries_inflation', 0.1)

        self.path_needs_update = True
        param_list = [self.min_size_n, self.max_size_m, self.max_viewing_distance]
        print(f'[Opponent Detection]: New dyn reconf values recieved: Min size {param_list[0]} [laser points], Max size {param_list[1]} [m], max viewing dist {param_list[2]} [m]')

    # --- Functions ---

    def clearmarkers(self) -> list:
        marker = Marker()
        marker.action = 3
        return [marker]

    def laserPointOnTrack (self, s, d) -> bool:
        if normalize_s(s-self.car_s,self.track_length)>self.max_viewing_distance:
            return False
        if abs(d) >= self.biggest_d:
            return False
        if abs(d) <= self.smallest_d:
            return True
        idx = bisect_left(self.s_array, s)
        if idx:
            idx -= 1
        if d <= -self.d_right_array[idx] or d >= self.d_left_array[idx]:
            return False
        return True
    
    def publish_filtered_pointcloud(self, on_track_points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        
        pc_msg = pc2.create_cloud_xyz32(header, on_track_points)        
        self.on_track_points_pub.publish(pc_msg)

    def clustering(self, scan_msg):
        """
        Converts the lidar scans to a 2D PointCloud and segments them into objects
        """

        # --- initialisation of some sutility parameters ---
        l = self.lambda_angle
        d_phi = scan_msg.angle_increment
        sigma = self.sigma

        # --- transform the scan ranges to a cloud point ---
        self.current_stamp = rospy.Time.now()
        try:
            lct = self.tf_listener.getLatestCommonTime("map", "laser")
        except:
            rospy.logerr("[Opponent Detection]: lookup Tranform between map and laser not possible")
            empty = []
            return empty
        trans, quat = self.tf_listener.lookupTransform('/map', '/laser', lct)
        self.T = np.array(trans)
        self.R = tf.transformations.quaternion_matrix(quat)

        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
        x_lf = (scan_msg.ranges * np.cos(angles)).flatten()
        y_lf = (scan_msg.ranges * np.sin(angles)).flatten()
        z_lf = (-self.T[2] * np.ones(len(scan_msg.ranges))).flatten()
        xyz_lf = np.vstack((x_lf, y_lf, z_lf, np.ones(len(scan_msg.ranges))))

        H_l2m = self.R
        H_l2m[:3, -1] = self.T

        xyz_map = H_l2m @ xyz_lf

        cloudPoints_list = []
        for i in range(xyz_map.shape[1]):
            pt = (xyz_map[0,i], xyz_map[1,i])
            cloudPoints_list.append(pt)
            
        cloudPoints_list = np.array(cloudPoints_list)
        
        s_points, d_points = self.converter.get_frenet(cloudPoints_list[:, 0], cloudPoints_list[:, 1])

        div_const = np.sin(d_phi) / np.sin(l - d_phi)

        objects_pointcloud_list = []
        on_track_points = []
        prev_on_track = False
        
        for i in range(1, len(cloudPoints_list)):
            if self.laserPointOnTrack(s_points[i], d_points[i]):
                curr_point = (cloudPoints_list[i,0], cloudPoints_list[i,1])
                if not prev_on_track:
                    objects_pointcloud_list.append([curr_point])
                else:
                    curr_range = scan_msg.ranges[i]
                    d_max = curr_range * div_const + 3 * sigma

                    # Distance between points does not change in map frame or laser frame.
                    dist_to_next_point = np.linalg.norm(xyz_lf[:2, i] - xyz_lf[:2, i - 1])
                    
                    if dist_to_next_point < d_max:
                        objects_pointcloud_list[-1].append(curr_point)
                    else:
                        objects_pointcloud_list.append([curr_point])
                prev_on_track = True
                if self.measuring:
                    on_track_points.append([cloudPoints_list[i,0], cloudPoints_list[i,1], 0.0])
            else:
                prev_on_track = False

        if self.measuring:
            self.publish_filtered_pointcloud(on_track_points)

        remove_array=[]
        for idx, object in enumerate(objects_pointcloud_list):
            if len(object) < self.min_size_n:
                remove_array.append(object)
                continue

        for object in remove_array:
            objects_pointcloud_list.remove(object)

        markers_array = []
        for idx,object in enumerate(objects_pointcloud_list):
            #first element
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.current_stamp
            marker.id = idx*10
            marker.type = marker.SPHERE
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.a = 0.5
            marker.color.g = 1.
            marker.color.r = 0.
            marker.color.b = idx/len(objects_pointcloud_list)
            marker.pose.position.x = object[0][0]
            marker.pose.position.y = object[0][1]
            marker.pose.orientation.w = 1
            markers_array.append(marker)

            #last element
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.current_stamp
            marker.id = idx*10+2
            marker.type = marker.SPHERE
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.a = 0.5
            marker.color.g = 1.
            marker.color.r = 0.
            marker.color.b = idx/len(objects_pointcloud_list)
            marker.pose.position.x = object[-1][0]
            marker.pose.position.y = object[-1][1]
            marker.pose.orientation.w = 1
            markers_array.append(marker)

        self.breakpoints_markers_pub.publish(self.clearmarkers())
        self.breakpoints_markers_pub.publish(markers_array)
        return objects_pointcloud_list

    def fitting_l_shape (self, objects_pointcloud_list):
        current_obstacle_array =[]
        min_dist = self.min_2_points_dist
        for obstacle in objects_pointcloud_list:

            # --- fit a rectangle to the data points ---
            theta = np.linspace(0,np.pi/2-np.pi/180,90)
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)
            
            # projection
            distance1 = np.dot(obstacle,[cos_theta,sin_theta]) # [cos_theta,sin_theta] unit vec1 (1,0) -> (0,1)
            distance2 = np.dot(obstacle,[-sin_theta,cos_theta]) # [-sin_theta,cos_theta] unit vec2 (0,1) -> (-1,0)
            
            # Translate to Origin
            D10 = -distance1 + np.amax(distance1,axis=0)
            D11 = distance1 - np.amin(distance1,axis=0)
            D20 = -distance2 + np.amax(distance2,axis=0)
            D21 = distance2 - np.amin(distance2,axis=0)
            
            # Flip so that the dense region is moved to the origin
            min_array = np.argmin([np.linalg.norm(D10,axis=0),np.linalg.norm(D11,axis=0)],axis=0)
            D10 = np.transpose(D10)
            D11 = np.transpose(D11)
            D10[min_array==1]=D11[min_array==1]
            D10 = np.transpose(D10)
            # Flip so that the dense region is moved to the origin
            min_array = np.argmin([np.linalg.norm(D20,axis=0),np.linalg.norm(D21,axis=0)],axis=0)
            D20 = np.transpose(D20)
            D21 = np.transpose(D21)
            D20[min_array==1]=D21[min_array==1]
            D20 = np.transpose(D20)
            
            # project to x=y
            D = np.minimum(D10,D20)
            D[D<min_dist]=min_dist
            
            # find Optimized theta
            theta_opt = np.argmax(np.sum(np.reciprocal(D),axis=0))*np.pi/180

            # --------------------------------------------
            # extract the center of the obstacle assuming
            # that it is actually a square obstacle
            # --------------------------------------------
            distances1 = np.dot(obstacle,[np.cos(theta_opt),np.sin(theta_opt)])
            distances2 = np.dot(obstacle,[-np.sin(theta_opt),np.cos(theta_opt)])
            max_dist1 = np.max(distances1)
            min_dist1 = np.min(distances1)
            max_dist2 = np.max(distances2)
            min_dist2 = np.min(distances2)
            
            ##############################################################
            ##### with car_pose information
            x_rot= np.dot(self.T[0:2],[np.cos(theta_opt),np.sin(theta_opt)])
            y_rot= np.dot(self.T[0:2],[-np.sin(theta_opt),np.cos(theta_opt)])
            my_pos = np.array([x_rot, y_rot])
            corner_UR = np.array([max_dist1, max_dist2])
            corner_LR = np.array([max_dist1, min_dist2])
            corner_UL = np.array([min_dist1, max_dist2])
            corner_LL = np.array([min_dist1, min_dist2])
            corners = [corner_UR, corner_LR, corner_UL, corner_LL]
            distances_to_my_pos = [np.linalg.norm(c - my_pos) for c in corners]
            closest_index = np.argmin(distances_to_my_pos)
            chosen_corner = corners[closest_index]
            width = max(max_dist1 - min_dist1, self.min_size_m)
            height = max(max_dist2 - min_dist2, self.min_size_m)
            rect_size = max(width, height)
            center_x = None
            if closest_index == 0:
                center_x = chosen_corner + np.array([-1,-1])*rect_size/2
            elif closest_index == 1:
                center_x = chosen_corner + np.array([-1,1])*rect_size/2
            elif closest_index == 2:
                center_x = chosen_corner + np.array([1,-1])*rect_size/2
            elif closest_index == 3:
                center_x = chosen_corner + np.array([1,1])*rect_size/2
            
            corrected_center_x = np.array([np.cos(theta_opt) * center_x[0] - np.sin(theta_opt) * center_x[1],
                                np.sin(theta_opt) * center_x[0] + np.cos(theta_opt) * center_x[1]])
            
            current_obstacle_array.append(Obstacle(corrected_center_x[0],corrected_center_x[1],rect_size,theta_opt))

        return current_obstacle_array
    
    def checkObstacles(self, current_obstacles):
        """
        Delete obstacles that are too big
        """

        remove_list = []
        self.tracked_obstacles.clear()
        for obs in current_obstacles:
            if(obs.size > self.max_size_m):
                remove_list.append(obs)

        for obs in remove_list:
            current_obstacles.remove(obs)
        
        for idx, curr_obs in enumerate(current_obstacles):
            curr_obs.id = idx
            self.tracked_obstacles.append(curr_obs)

    def publishObstaclesMessage (self):
        obstacles_array_message = ObstacleArray()
        obstacles_array_message.header.stamp = self.current_stamp
        obstacles_array_message.header.frame_id = "map"

        x_center = []
        y_center = []
        for obstacle in self.tracked_obstacles:
            x_center.append(obstacle.center_x)
            y_center.append(obstacle.center_y)

        s_points, d_points = self.converter.get_frenet(np.array(x_center), np.array(y_center))

        for idx, obstacle in enumerate(self.tracked_obstacles):
            s = s_points[idx]
            d = d_points[idx]

            obsMsg = ObstacleMessage()
            obsMsg.id = obstacle.id
            obsMsg.s_start = s-obstacle.size/2
            obsMsg.s_end = s+obstacle.size/2
            obsMsg.d_left = d+obstacle.size/2
            obsMsg.d_right = d-obstacle.size/2
            obsMsg.s_center = s
            obsMsg.d_center = d
            obsMsg.size = obstacle.size

            obstacles_array_message.obstacles.append(obsMsg)
        self.obstacles_msg_pub.publish(obstacles_array_message)

    def publishObstaclesMarkers(self):
        markers_array = []
        for obs in self.tracked_obstacles:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.current_stamp
            marker.id = obs.id
            marker.type = marker.CUBE
            marker.scale.x = obs.size
            marker.scale.y = obs.size
            marker.scale.z = obs.size
            marker.color.a = 0.8
            marker.color.g = 0.
            marker.color.r = 1.
            marker.color.b = 0.
            marker.pose.position.x = obs.center_x
            marker.pose.position.y = obs.center_y
            q = quaternion_from_euler(0, 0, obs.theta)
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            markers_array.append(marker)
        self.obstacles_marker_pub.publish(self.clearmarkers())
        self.obstacles_marker_pub.publish(markers_array)
        Obstacle.current_id = 0

    def timer_callback(self, event):
        if self.measuring:
            start_time = time.perf_counter()
        # Clustering
        scans_copy = copy.deepcopy(self.scans)
        objects_pointcloud_list= self.clustering(scans_copy)
        
        # L-Shape fitting
        current_obstacles = self.fitting_l_shape(objects_pointcloud_list)
        self.checkObstacles(current_obstacles)
        if self.measuring:
            end_time = time.perf_counter()
            latency = 1/(end_time - start_time)
            self.latency_pub.publish(latency)
        self.publishObstaclesMessage()
        self.publishObstaclesMarkers()
        

    def main (self):
        rospy.loginfo('[Opponent Detection]: Waiting for global wpnts')
        rospy.wait_for_message('/global_waypoints', WpntArray)
        rospy.loginfo('[Opponent Detection]: Ready')
        
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.timer_callback)
        rospy.spin()


if __name__ == '__main__':
    detect = Detect()
    detect.main()