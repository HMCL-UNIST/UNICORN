#!/usr/bin/env python3

import os
import sys
import subprocess
import copy

import csv
import yaml
import math
import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from skimage.morphology import skeletonize
from skimage.segmentation import watershed

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

# Make the vendored `global_racetrajectory_optimization` subtree importable as a
# top-level package (it uses absolute imports internally, e.g.
# `from global_racetrajectory_optimization import ...`). The subtree lives inside
# this python package directory, so adding this directory to sys.path makes it
# resolve both for the node and for the library's own internal imports.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer
from global_racetrajectory_optimization import helper_funcs_glob
import trajectory_planning_helpers as tph

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from f110_msgs.msg import Wpnt, WpntArray
from transforms3d.euler import quat2euler
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool, Float32

# To write global waypoints
from .readwrite_global_waypoints import write_global_waypoints


class GlobalPlanner(Node):
    """
    Global planner node
    """

    def __init__(self):
        super().__init__('global_planner_node')

        # Declare parameters (formerly rospy.get_param). Defaults mirror the
        # UNICORN ROS1 launch/yaml conventions; values should be overridden via
        # a params file at launch.
        self.declare_parameter('racecar_version', 'NUC2')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('test_on_car', False)
        self.declare_parameter('safety_width', 0.8)
        self.declare_parameter('safety_width_sp', 0.8)
        self.declare_parameter('occupancy_grid_threshold', 50)
        self.declare_parameter('show_plots', False)
        self.declare_parameter('create_map', True)
        self.declare_parameter('create_global_path', True)
        self.declare_parameter('map', '')
        self.declare_parameter('map_dir', '')
        self.declare_parameter('reverse_mapping', False)
        self.declare_parameter('required_laps', 1)
        self.declare_parameter('enable_mintime', False)

        self.racecar_version = self.get_parameter('racecar_version').value  # NUCX

        self.input_path = os.path.join(
            get_package_share_directory('stack_master'), 'config', self.racecar_version)
        self.rate = self.get_parameter('rate').value
        self.test_on_car = self.get_parameter('test_on_car').value
        self.current_key = ''

        self.safety_width = self.get_parameter('safety_width').value
        self.safety_width_sp = self.get_parameter('safety_width_sp').value
        self.enable_mintime = self.get_parameter('enable_mintime').value
        self.occupancy_grid_threshold = self.get_parameter('occupancy_grid_threshold').value

        self.show_plots = self.get_parameter('show_plots').value  # show no plots if False

        self.create_map = self.get_parameter('create_map').value
        create_global_path = self.get_parameter('create_global_path').value

        if self.create_map and create_global_path:
            self.map_editor = False
            self.map_editor_mapping = False
        elif self.create_map and not create_global_path:
            self.map_editor = True
            self.map_editor_mapping = True
        else:
            self.map_editor = True
            self.map_editor_mapping = False

        self.map_name = self.get_parameter('map').value
        self.map_dir = self._resolve_source_map_dir(self.get_parameter('map_dir').value)
        self.reverse_mapping = self.get_parameter('reverse_mapping').value
        self.watershed = True  # use watershed algorithm

        # map variables
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin = Pose()
        self.map_occupancy_grid = None

        self.current_position = None
        self.initial_position = None

        self.map_ready = False
        self.est_lap_time = 0.0

        # load map maps from yaml file if we do not want to create a map
        if not self.create_map:
            with open(os.path.join(self.map_dir, self.map_name + '.yaml')) as f:
                data = yaml.safe_load(f)
                # only need resolution and origin for waypoints
                self.map_resolution = data['resolution']
                self.map_origin.position.x = data['origin'][0]
                self.map_origin.position.y = data['origin'][1]

        # variables to check how many laps were completed
        self.just_once = False
        self.was_at_init_pos = True
        self.x_max_diff = 0.5  # meter
        self.y_max_diff = 0.5  # meter
        self.theta_max_diff = math.pi / 2  # rad
        self.lap_count = 0
        self.required_laps = self.get_parameter('required_laps').value
        if not self.test_on_car or self.map_editor:
            self.required_laps = 0

        # for comparing driven lap length with calculated centerline length
        self.cent_driven = None
        self.cent_length_done = False

        # all required subscribers
        # nav2 map_server latches /map (TRANSIENT_LOCAL); match it or we never
        # receive the one-shot map and _wait_for_map() blocks forever.
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)
        # self.create_subscription(PoseStamped, '/car_state/pose', self.pose_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/base_link_pose_with_cov', self.pose_cb, 10)

        # publisher for local planner
        self.wpnt_global_iqp_pub = self.create_publisher(WpntArray, 'global_waypoints', 10)
        self.wpnt_center_pub = self.create_publisher(WpntArray, 'centerline_waypoints', 10)
        # publisher only for visualization
        self.vis_wpnt_cent_pub = self.create_publisher(MarkerArray, 'centerline_waypoints/markers', 10)
        self.vis_wpnt_global_iqp_pub = self.create_publisher(MarkerArray, 'global_waypoints/markers', 10)
        self.vis_track_bnds = self.create_publisher(MarkerArray, 'trackbounds/markers', 10)
        # shortest path
        self.wpnt_global_sp_pub = self.create_publisher(WpntArray, 'global_waypoints/shortest_path', 10)
        self.vis_wpnt_global_sp_pub = self.create_publisher(
            MarkerArray, 'global_waypoints/shortest_path/markers', 10)
        # publish map infos
        self.map_info_pub = self.create_publisher(String, 'map_infos', 10)
        self.map_info_str = ''
        # publish bool if map is ready for gb optimizer
        self.map_ready_pub = self.create_publisher(Bool, 'map_ready', 10)
        # for l1_param_optimizer
        self.est_lap_time_pub = self.create_publisher(Float32, 'estimated_lap_time', 10)

    def _resolve_source_map_dir(self, map_dir):
        """Make outputs land in the SOURCE maps/<map>/ folder, not the install copy.

        Launch passes the installed share path (find-pkg-share). With colcon
        --symlink-install the files inside it (e.g. <map>.yaml) are symlinks back
        to src, so following one to its real location yields the source folder -
        independent of the repo/workspace name. Falls back to the given path if
        nothing resolvable is found (e.g. a non-symlink install).
        """
        if not map_dir:
            return map_dir
        for probe in (self.map_name + '.yaml', self.map_name + '.png', self.map_name + '.pgm'):
            p = os.path.join(map_dir, probe)
            if os.path.islink(p) or os.path.exists(p):
                real_dir = os.path.dirname(os.path.realpath(p))
                if real_dir != os.path.realpath(map_dir):
                    self.get_logger().info(
                        f'[GB Planner]: resolved source map dir -> {real_dir}')
                return real_dir
        return map_dir

    def map_cb(self, data):
        """
        Callback function of /map subscriber.

        Parameters
        ----------
        data
            Data received from /map topic
        """
        # Data should not be overwritten if we use map maps from yaml file
        if self.create_map:
            self.map_width = data.info.width  # uint32, [cells]
            self.map_height = data.info.height  # uint32, [cells]
            self.map_resolution = data.info.resolution  # float32, [m/cell]
            self.map_origin = data.info.origin
            self.map_occupancy_grid = data.data  # int8[]

    def pose_cb(self, data):
        """
        Callback function of /tracked_pose subscriber.

        Parameters
        ----------
        data
            Data received from /tracked_pose topic
        """
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        # transforms3d expects (w, x, y, z); returns (roll, pitch, yaw)
        theta = quat2euler([data.pose.pose.orientation.w, data.pose.pose.orientation.x,
                            data.pose.pose.orientation.y, data.pose.pose.orientation.z])[2]

        if self.current_position is None:
            self.initial_position = [x, y, theta]
        self.current_position = [x, y, theta]

        if self.lap_count == 0:
            if self.cent_driven is None:
                self.cent_driven = np.array([self.current_position])
            else:
                self.cent_driven = np.append(self.cent_driven, [self.current_position], axis=0)

    def _wait_for_map(self):
        """Block (spinning) until a message has been received on /map."""
        while rclpy.ok() and self.map_occupancy_grid is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def global_plan_loop(self):
        rate_period = 1.0 / self.rate if self.rate else 0.05

        # If in map_editor mode while not mapping (i.e. no need for .pbstream) we only compute the gb from the img and exit
        if self.map_editor and not self.map_editor_mapping:
            # create_map=false: the plan is built from the saved .png + .yaml
            # (resolution/origin below), NOT from a live /map occupancy grid -
            # map_cb only fills map_occupancy_grid when create_map is true, so
            # waiting on it here would block forever.
            with open(os.path.join(self.map_dir, self.map_name + '.yaml')) as f:
                data = yaml.safe_load(f)
                # only need resolution and origin for waypoints
                self.map_resolution = data['resolution']
                self.map_origin.position.x = data['origin'][0]
                self.map_origin.position.y = data['origin'][1]
                self.map_origin.position.z = data['origin'][2]
                # self.initial_position = [self.map_origin.position.x, self.map_origin.position.y, 0]
                self.initial_position = [0, 0, 0]
            # compute global trajectory from img only and return
            self.compute_global_trajectory(cent_length=0.0, save_map=False, save_pf_copy=False)

            self.get_logger().info('[GB Planner]: Successfully created map! Killing nodes...')
            return

        # waiting for position
        else:
            while rclpy.ok() and (self.current_position is None or self.initial_position is None
                                  or (self.map_occupancy_grid is None and self.create_map)):
                rclpy.spin_once(self, timeout_sec=rate_period)
        self.get_logger().info('[GB Planner]: Global planner ready!')
        while rclpy.ok():
            # process any pending callbacks (pose/map updates)
            rclpy.spin_once(self, timeout_sec=rate_period)
            # If mapping for map_editor mode we only need the img, yaml and the .pbstream
            # We don't care about the global trajectory
            if self.map_editor_mapping and self.create_map:
                # listen to keyboard input
                self.get_logger().warn("[GB Planner]: Press 'y' when the map looks good enough")
                while True:
                    # waiting for y
                    self.current_key = input().lower().strip()
                    if self.current_key == 'y':
                        self.initial_position = self.current_position  # use position after mapping is done
                        self.just_once = True
                        # Save map png and yaml in map_dir, but it will break before computing the global trajectory
                        self.compute_global_trajectory(cent_length=0.0, save_map=True, save_pf_copy=True)
                        pb_dir = os.path.join(self.map_dir, self.map_name + '.pbstream')
                        script_dir = os.path.join(
                            get_package_share_directory('gb_optimizer'), 'scripts', 'finish_map.sh')
                        subprocess.Popen(args=[script_dir, pb_dir], shell=False)
            # Normal mode
            else:
                is_at_init_pos = self.at_init_pos_check()
                # check if car is now at start position and wasn't before --> means we completed a lap
                if is_at_init_pos and not self.was_at_init_pos:
                    self.was_at_init_pos = True
                    self.lap_count += 1
                    self.get_logger().info("[GB Planner]:" + f"Laps completed {self.lap_count}")

                elif not is_at_init_pos:
                    self.was_at_init_pos = False

                # calculate global trajectory only once after a certain number of completed laps
                if not self.just_once and self.lap_count == self.required_laps:
                    if self.required_laps != 0:
                        # calculate length of driven path in first lap for an approx length of the centerline
                        cent_length = np.sum(np.sqrt(np.sum(np.power(np.diff(self.cent_driven[:, :2], axis=0), 2), axis=1)))

                        cent_len_str = "[GB Planner]:" + f"Approximate centerline length: {round(cent_length, 4)}m; "
                        self.get_logger().info(cent_len_str)
                        self.map_info_str += cent_len_str
                    else:
                        cent_length = 0

                    # listen to keyboard input
                    self.get_logger().warn("[GB Planner]: Press 'y' when the map looks good enough")
                    map_ready_msg = Bool()
                    map_ready_msg.data = True
                    self.map_ready_pub.publish(map_ready_msg)
                    while True:
                        # waiting for y
                        self.current_key = input().lower().strip()
                        if self.current_key == 'y':
                            self.initial_position = self.current_position  # use position after mapping is done
                            self.just_once = True
                            if self.compute_global_trajectory(cent_length=cent_length, save_map=True, save_pf_copy=True):
                                self.get_logger().info('[GB Planner]: Successfully computed waypoints!')

                                pb_dir = os.path.join(self.map_dir, self.map_name + '.pbstream')

                                script_dir = os.path.join(
                                    get_package_share_directory('gb_optimizer'), 'scripts', 'finish_map.sh')
                                if self.test_on_car:
                                    subprocess.Popen(args=[script_dir, pb_dir], shell=False)
                                break
                            else:
                                self.get_logger().warn('[GB Planner]: Was unable to compute waypoints in compute_global_trajectory!')
                                self.current_key = ''
                                break
                        else:
                            self.current_key = ''
                            break

    def at_init_pos_check(self) -> bool:
        """
        Check if the current position is similar to initial position.

        Returns
        -------
        at_init_pos : bool
            True if current position is similar to initial position
        """
        # absolute values of difference between current and initial position
        x_diff = math.fabs(self.current_position[0] - self.initial_position[0])
        y_diff = math.fabs(self.current_position[1] - self.initial_position[1])

        # The smallest distance between the two angles is using this
        theta_diff0 = math.fabs(self.current_position[2] - self.initial_position[2])
        theta_diff1 = 2*np.pi-theta_diff0
        theta_diff = min(theta_diff0, theta_diff1)

        at_init_pos = (x_diff < self.x_max_diff) and (y_diff < self.y_max_diff) and (theta_diff < self.theta_max_diff)
        return at_init_pos

    def compute_global_trajectory(self, cent_length, save_map: bool, save_pf_copy: bool) -> bool:
        """
        Compute the global optimized trajectory of a map.

        Calculate the centerline of the track and compute global optimized trajectory with minimum curvature
        optimization.
        Publish the markers and waypoints of the global optimized trajectory.
        A waypoint has the following form: [s_m, x_m, y_m, d_right, d_left, psi_rad, vx_mps, ax_mps2]

        Parameters
        ----------
        cent_length
            Approximate length of the centerline

        save_map
            Whether or not to save png and yaml files under map_name

        save_pf_copy
            Whether or not to save a duplicate copy of the map for PF usage

        Returns
        -------
        bool
            True if successfully computed the global waypoints
        """
        ################################################################################################################
        # Create a filtered black and white image of the map
        ################################################################################################################
        if self.create_map:
            # get right shape for occupancy grid map
            og_map = np.int8(self.map_occupancy_grid).reshape(self.map_height, self.map_width)
            # mark unknown (-1) as occupied (100)
            og_map = np.where(og_map == -1, 100, og_map)

            # binarised map
            bw = np.where(og_map < self.occupancy_grid_threshold, 255, 0)
            bw = np.uint8(bw)

            # Also write a duplicate copy that won't be overwritten by future map_editor shenanigans
            if save_pf_copy:
                img_path = os.path.join(self.map_dir, 'pf_map.png')
                flip_open = cv2.flip(bw, 0)
                cv2.imwrite(img_path, flip_open)

                dict_map = {'image': 'pf_map.png',
                            'resolution': self.map_resolution,
                            'origin': [self.map_origin.position.x, self.map_origin.position.y, 0],
                            'negate': 0,
                            'occupied_thresh': 0.65,
                            'free_thresh': 0.196}

                with open(os.path.join(self.map_dir, "pf_map.yaml"), 'w') as file:
                    _ = yaml.dump(dict_map, file, default_flow_style=False)

            # Filtering with morphological opening
            kernel1 = np.ones((9, 9), np.uint8)
            opening = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel1, iterations=2)

            if self.map_editor:
                if self.show_plots:
                    plt.imshow(opening, cmap='gray', origin='lower')
                    plt.show()

                # write image as png and a yaml file in the folder
                if save_map:
                    img_path = os.path.join(self.map_dir, self.map_name + '.png')
                    flip_open = cv2.flip(opening, 0)
                    cv2.imwrite(img_path, flip_open)

                    dict_map = {'image': self.map_name + '.png',
                                'resolution': self.map_resolution,
                                'origin': [self.map_origin.position.x, self.map_origin.position.y, 0],
                                'negate': 0,
                                'occupied_thresh': 0.65,
                                'free_thresh': 0.196}

                    with open(os.path.join(self.map_dir, self.map_name + ".yaml"), 'w') as file:
                        _ = yaml.dump(dict_map, file, default_flow_style=False)

                ros_info_string = '[GB Planner]:' + f'PNG and YAML file created and saved in the {self.map_dir} folder'
                self.get_logger().info(ros_info_string)
                return True
        else:
            img_path = os.path.join(self.map_dir, self.map_name + '.png')
            bw = cv2.flip(cv2.imread(img_path, 0), 0)

            # Filtering with morphological opening
            kernel1 = np.ones((9, 9), np.uint8)
            opening = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel1, iterations=1)

        # get morphological skeleton of the map
        skeleton = skeletonize(opening, method='lee')

        # ! For debugging
        # origin='lower' flips the display about the x-axis so the map shows
        # right-side-up (viz only; the underlying arrays / waypoints are unchanged).
        f, (ax0, ax1) = plt.subplots(2, 1)
        ax0.imshow(opening, cmap='gray', origin='lower')
        ax1.imshow(skeleton, cmap='gray', origin='lower')
        plt.show()

        ################################################################################################################
        # Extract centerline from filtered occupancy grid map
        ################################################################################################################
        try:
            centerline = self.extract_centerline(skeleton=skeleton, cent_length=cent_length)

            print(centerline.shape)
        except IOError:
            self.just_once = False
            if self.map_editor:
                self.get_logger().warn('[GB Planner]: No closed contours found! Check the edited image...')
            else:
                self.get_logger().warn('[GB Planner]: No closed contours found! Keep driving...')
            return False
        except ValueError:
            self.get_logger().warn("[GB Planner]: Couldn't find a closed contour with similar length as driven path!")
            self.get_logger().info('[GB Planner]: Maybe missed a lap completion...')
            self.get_logger().info('[GB Planner]: Will try again in one lap, so drive at least one more lap!')
            self.just_once = False
            self.cent_length_done = False
            self.was_at_init_pos = True
            self.initial_position = self.current_position
            self.lap_count = 0
            self.required_laps = 1
            self.cent_driven = [self.initial_position]
            return False

        centerline_smooth = self.smooth_centerline(centerline)

        # convert centerline from cells to meters
        centerline_meter = np.zeros(np.shape(centerline_smooth))
        centerline_meter[:, 0] = centerline_smooth[:, 0] * self.map_resolution + self.map_origin.position.x
        centerline_meter[:, 1] = centerline_smooth[:, 1] * self.map_resolution + self.map_origin.position.y

        # interpolate centerline to 0.1m stepsize: less computation needed later for distance to track bounds
        centerline_meter = np.column_stack((centerline_meter, np.zeros((centerline_meter.shape[0], 2))))
        centerline_meter_int = helper_funcs_glob.src.interp_track.interp_track(reftrack=centerline_meter,
                                                                               stepsize_approx=0.1)[:, :2]

        # get distance to initial position for every point on centerline
        cent_distance = np.sqrt(np.power(centerline_meter_int[:, 0] - self.initial_position[0], 2)
                                + np.power(centerline_meter_int[:, 1] - self.initial_position[1], 2))

        min_dist_ind = np.argmin(cent_distance)

        cent_direction = np.angle([complex(centerline_meter_int[min_dist_ind, 0] -
                                           centerline_meter_int[min_dist_ind - 1, 0],
                                           centerline_meter_int[min_dist_ind, 1] -
                                           centerline_meter_int[min_dist_ind - 1, 1])])

        # if self.show_plots and not self.map_editor:
        if self.show_plots:
            print("Direction of the centerline: ", cent_direction[0])
            print("Direction of the initial car position: ", self.initial_position[2])
            plt.plot(centerline_meter_int[:, 0], centerline_meter_int[:, 1], 'ko', label='Centerline interpolated')
            plt.plot(centerline_meter_int[min_dist_ind - 1, 0], centerline_meter_int[min_dist_ind - 1, 1], 'ro',
                     label='First point')
            plt.plot(centerline_meter_int[min_dist_ind, 0], centerline_meter_int[min_dist_ind, 1], 'bo',
                     label='Second point')
            plt.legend()
            plt.show()

        # flip centerline if directions don't match
        # cent_direction is np.angle([...]) -> a length-1 array; pass the scalar
        # element (newer numpy won't coerce a 1-elem array in math.fabs).
        if not self.compare_direction(cent_direction[0], self.initial_position[2]):
            centerline_smooth = np.flip(centerline_smooth, axis=0)
            centerline_meter_int = np.flip(centerline_meter_int, axis=0)

        # Flip again if necessary
        if self.reverse_mapping:
            centerline_smooth = np.flip(centerline_smooth, axis=0)
            centerline_meter_int = np.flip(centerline_meter_int, axis=0)
            self.get_logger().info('[GB Planner]: Centerline flipped')

        # extract track bounds
        if self.watershed:
            try:
                bound_r_water, bound_l_water = self.extract_track_bounds(centerline_smooth, opening, save_img=(not self.map_editor))
                dist_transform = None
                self.get_logger().info('[GB Planner]: Using watershed for track bound extraction...')
            except IOError:
                self.get_logger().warn('[GB Planner]: More than two track bounds detected with watershed algorithm')
                self.get_logger().info('[GB Planner]: Trying with simple distance transform...')
                self.watershed = False
                bound_r_water = None
                bound_l_water = None
                dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
        else:
            self.get_logger().info('[GB Planner]: Using distance transform for track bound extraction...')
            bound_r_water = None
            bound_l_water = None
            dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)

        ################################################################################################################
        # Compute global trajectory with mincurv_iqp optimization
        ################################################################################################################
        cent_with_dist = self.add_dist_to_cent(centerline_smooth=centerline_smooth,
                                               centerline_meter=centerline_meter_int,
                                               dist_transform=dist_transform,
                                               bound_r=bound_r_water,
                                               bound_l=bound_l_water,
                                               reverse=self.reverse_mapping)

        # Write centerline in a csv file and get a marker array of it
        centerline_waypoints, centerline_markers = self.write_centerline(cent_with_dist)

        # Add curvature and angle to centerline waypoints
        centerline_coords = np.array([
            [coord.x_m, coord.y_m] for coord in centerline_waypoints.wpnts
        ])

        psi_centerline, kappa_centerline = tph.calc_head_curv_num.\
            calc_head_curv_num(
                path=centerline_coords,
                el_lengths=0.1*np.ones(len(centerline_coords)-1),
                is_closed=False,
                stepsize_curv_preview=5.0,
                stepsize_curv_review=5.0
            )
        for i, (psi, kappa) in enumerate(zip(psi_centerline, kappa_centerline)):
            centerline_waypoints.wpnts[i].s_m = i*0.1
            centerline_waypoints.wpnts[i].psi_rad = psi + np.pi/2  # pi/2 added because trajectory_planning_helpers package assumes north to be zero psi
            centerline_waypoints.wpnts[i].kappa_radpm = kappa

        # publish the centerline markers
        self.vis_wpnt_cent_pub.publish(centerline_markers)

        main_opt_type = 'mintime' if self.enable_mintime else 'mincurv_iqp'
        self.get_logger().info(f'[GB Planner]: Start Global Trajectory optimization ({main_opt_type})...')
        global_trajectory_iqp, bound_r_iqp, bound_l_iqp, est_t_iqp = trajectory_optimizer(input_path=self.input_path,
                                                                                          track_name='map_centerline',
                                                                                          curv_opt_type=main_opt_type,
                                                                                          safety_width=self.safety_width,
                                                                                          plot=(self.show_plots and not self.map_editor))

        self.map_info_str += f'IQP estimated lap time: {round(est_t_iqp, 4)}s; '
        self.map_info_str += f'IQP maximum speed: {round(np.amax(global_trajectory_iqp[:, 5]), 4)}m/s; '

        # do not use bounds of optimizer if the one's from the watershed algorithm are available
        if self.watershed:
            bound_r_iqp = bound_r_water
            bound_l_iqp = bound_l_water

        bounds_markers = self.publish_track_bounds(bound_r_iqp, bound_l_iqp, reverse=False)

        d_right_iqp, d_left_iqp = self.dist_to_bounds(trajectory=global_trajectory_iqp,
                                                      bound_r=bound_r_iqp,
                                                      bound_l=bound_l_iqp,
                                                      centerline=centerline_meter_int,
                                                      reverse=self.reverse_mapping)

        global_traj_wpnts_iqp, global_traj_markers_iqp = self.create_wpnts_markers(trajectory=global_trajectory_iqp,
                                                                                   d_right=d_right_iqp,
                                                                                   d_left=d_left_iqp)

        # publish global trajectory markers and waypoints
        self.wpnt_center_pub.publish(centerline_waypoints)
        self.wpnt_global_iqp_pub.publish(global_traj_wpnts_iqp)
        self.vis_wpnt_global_iqp_pub.publish(global_traj_markers_iqp)
        self.get_logger().info('[GB Planner]: Done with iterative minimum curvature optimization')
        self.get_logger().info('[GB Planner]: Lap Completed now publishing global waypoints')

        ################################################################################################################
        # Compute global trajectory with shortest path optimization
        ################################################################################################################

        self.get_logger().info('[GB Planner]: Start reverse Global Trajectory optimization with shortest path...')

        self.get_logger().info('[GB Planner]: Start Global Trajectory optimization with iterative minimum curvature for overtaking...')
        global_trajectory_iqp_ot, *_ = trajectory_optimizer(input_path=self.input_path,
                                                            track_name='map_centerline',
                                                            curv_opt_type='mincurv_iqp',
                                                            safety_width=self.safety_width_sp,
                                                            plot=(self.show_plots and not self.map_editor))

        # use new iqp path as centerline
        new_cent_with_dist = self.add_dist_to_cent(centerline_smooth=global_trajectory_iqp_ot[:, 1:3],
                                                   centerline_meter=global_trajectory_iqp_ot[:, 1:3],
                                                   dist_transform=None,
                                                   bound_r=bound_r_water,
                                                   bound_l=bound_l_water,
                                                   reverse=self.reverse_mapping)

        _, new_centerline_markers = self.write_centerline(new_cent_with_dist, sp_bool=True)

        # Use the TRUE centerline (map_centerline) as the shortest-path reference,
        # not the IQP line (map_centerline_2). The IQP line already hugs the inside
        # of corners, so measuring d_right/d_left from it gives a lopsided width:
        # the outer-bound distance is large, which lets the shortest-path QP shift
        # the path up to ~0.8 m past the outer wall (~15% of points went outside on
        # the f map). The centerline has balanced left/right widths, keeping the
        # solution inside the track. (map_centerline_2 is "a bit faster but cuts
        # corners more" per the original note -- at the cost of leaving the track.)
        global_trajectory_sp, bound_r_sp, bound_l_sp, est_t_sp = trajectory_optimizer(input_path=self.input_path,
                                                                                      track_name='map_centerline',
                                                                                      curv_opt_type='shortest_path',
                                                                                      safety_width=self.safety_width_sp,
                                                                                      plot=(self.show_plots and not self.map_editor))

        self.est_lap_time = est_t_sp  # variable which will be published and used in l1_param_optimizer
        est_lap_time_msg = Float32()
        est_lap_time_msg.data = float(self.est_lap_time)
        self.est_lap_time_pub.publish(est_lap_time_msg)

        self.map_info_str += f'SP estimated lap time: {round(est_t_sp, 4)}s; '
        self.map_info_str += f'SP maximum speed: {round(np.amax(global_trajectory_sp[:, 5]), 4)}m/s; '

        # do not use bounds of optimizer if the one's from the watershed algorithm are available
        if self.watershed:
            bound_r_sp = bound_r_water
            bound_l_sp = bound_l_water

        d_right_sp, d_left_sp = self.dist_to_bounds(trajectory=global_trajectory_sp,
                                                    bound_r=bound_r_sp,
                                                    bound_l=bound_l_sp,
                                                    centerline=centerline_meter_int,
                                                    reverse=self.reverse_mapping)

        global_traj_wpnts_sp, global_traj_markers_sp = self.create_wpnts_markers(trajectory=global_trajectory_sp,
                                                                                 d_right=d_right_sp,
                                                                                 d_left=d_left_sp,
                                                                                 second_traj=True)

        # publish global trajectory markers and waypoints
        self.vis_wpnt_global_sp_pub.publish(global_traj_markers_sp)
        self.wpnt_global_sp_pub.publish(global_traj_wpnts_sp)
        map_info_msg = String()
        map_info_msg.data = self.map_info_str
        self.map_info_pub.publish(map_info_msg)
        self.get_logger().info('[GB Planner]: Done with shortest path optimization')
        self.get_logger().info('[GB Planner]: Lap Completed now publishing shortest path global waypoints')

        # Save info into a JSON file
        write_global_waypoints(
            self.map_name,
            self.map_info_str,
            self.est_lap_time,
            centerline_markers,
            centerline_waypoints,
            global_traj_markers_iqp,
            global_traj_wpnts_iqp,
            global_traj_markers_sp,
            global_traj_wpnts_sp,
            bounds_markers,
            map_editor_bool=self.map_editor,
        )
        return True

    def extract_centerline(self, skeleton, cent_length: float) -> np.ndarray:
        """
        Extract the centerline out of the skeletonized binary image.

        This is done by finding closed contours and comparing these contours to the approximate centerline
        length (which is known because of the driven path).

        Parameters
        ----------
        skeleton
            The skeleton of the binarised and filtered map
        cent_length : float
            Expected approximate centerline length

        Returns
        -------
        centerline : np.ndarray
            The centerline in form [[x1,y1],...] and in cells not meters

        Raises
        ------
        IOError
            If no closed contour is found
        ValueError
            If all found contours do not have a similar line length as the centerline (can only happen if
            {self.test_on_car} is True)
        """
        # get contours from skeleton
        # skimage.skeletonize returns a bool array; OpenCV >=4.6 rejects bool in
        # findContours, so cast to the uint8 (0/255) image it expects.
        if skeleton.dtype == bool:
            skeleton = skeleton.astype(np.uint8) * 255
        contours, hierarchy = cv2.findContours(skeleton, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

        # save all closed contours
        closed_contours = []
        for i, elem in enumerate(contours):
            opened = hierarchy[0][i][2] < 0 and hierarchy[0][i][3] < 0
            if not opened:
                closed_contours.append(elem)

        # if we have no closed contour, we can't calculate a global trajectory
        if len(closed_contours) == 0:
            raise IOError("No closed contours")

        # calculate the line length of every contour to get the real centerline
        line_lengths = [math.inf] * len(closed_contours)
        for i, cont in enumerate(closed_contours):
            line_length = 0
            for k, pnt in enumerate(cont):
                line_length += np.sqrt((pnt[0][0] - cont[k - 1][0][0]) ** 2 +
                                       (pnt[0][1] - cont[k - 1][0][1]) ** 2)

            line_length *= self.map_resolution  # length in meters not cells
            # line length should be around the length of the centerline otherwise keep length infinity
            if self.test_on_car and math.fabs(cent_length / line_length - 1.0) < 0.15:
                line_lengths[i] = line_length
            if not self.test_on_car or self.map_editor:
                line_lengths[i] = line_length

        # take the shortest line
        min_line_length = min(line_lengths)

        if min_line_length == math.inf:
            raise ValueError("Only invalid closed contour line lengths")

        min_length_index = line_lengths.index(min_line_length)
        # print(line_lengths)
        smallest_contour = np.array(closed_contours[min_length_index]).flatten()

        # reshape smallest_contours from the shape [x1,y1,x2,y2,...] to [[x1,y1],[x2,y2],...]
        # this will be the centerline
        len_reshape = int(len(smallest_contour) / 2)
        centerline = smallest_contour.reshape(len_reshape, 2)

        return centerline

    @staticmethod
    def smooth_centerline(centerline: np.ndarray) -> np.ndarray:
        """
        Smooth the centerline with a Savitzky-Golay filter.

        Notes
        -----
        The savgol filter doesn't ensure a smooth transition at the end and beginning of the centerline. That's why
        we apply a savgol filter to the centerline with start and end points on the other half of the track.
        Afterwards, we take the results of the second smoothed centerline for the beginning and end of the
        first centerline to get an overall smooth centerline

        Parameters
        ----------
        centerline : np.ndarray
            Unsmoothed centerline

        Returns
        -------
        centerline_smooth : np.ndarray
            Smooth centerline
        """
        centerline_length = len(centerline)

        if centerline_length > 2000:
            filter_length = int(centerline_length / 200) * 10 + 1
        elif centerline_length > 1000:
            filter_length = 81
        elif centerline_length > 500:
            filter_length = 41
        else:
            filter_length = 21
        centerline_smooth = savgol_filter(centerline, filter_length, 3, axis=0)

        # cen_len is half the length of the centerline
        cen_len = int(len(centerline) / 2)
        centerline2 = np.append(centerline[cen_len:], centerline[0:cen_len], axis=0)
        centerline_smooth2 = savgol_filter(centerline2, filter_length, 3, axis=0)

        # take points from second (smoothed) centerline for first centerline
        centerline_smooth[0:filter_length] = centerline_smooth2[cen_len:(cen_len + filter_length)]
        centerline_smooth[-filter_length:] = centerline_smooth2[(cen_len - filter_length):cen_len]

        return centerline_smooth

    def extract_track_bounds(self, centerline: np.ndarray, filtered_bw: np.ndarray, save_img=False):
        """
        Extract the boundaries of the track.

        Use the watershed algorithm with the centerline as marker to extract the boundaries of the filtered black
        and white image of the map.

        Parameters
        ----------
        centerline : np.ndarray
            The centerline of the track (in cells not meters)
        filtered_bw : np.ndarray
            Filtered black and white image of the track
        save_img : bool
            Only saves sim map when specifically asked for because the function is called in reverse too
        Returns
        -------
        bound_right, bound_left : tuple[np.ndarray, np.ndarray]
            Points of the track bounds right and left in meters

        Raises
        ------
        IOError
            If there were more (or less) than two track bounds found
        """
        # create a black and white image of the centerline
        cent_img = np.zeros((filtered_bw.shape[0], filtered_bw.shape[1]), dtype=np.uint8)
        cv2.drawContours(cent_img, [centerline.astype(int)], 0, 255, 2, cv2.LINE_8)

        # create markers for watershed algorithm
        _, cent_markers = cv2.connectedComponents(cent_img)

        # apply watershed algorithm to get only the track (without any lidar beams outside the track)
        dist_transform = cv2.distanceTransform(filtered_bw, cv2.DIST_L2, 5)
        labels = watershed(-dist_transform, cent_markers, mask=filtered_bw)

        closed_contours = []

        for label in np.unique(labels):
            if label == 0:
                continue

            # Create a mask, the mask should be the track
            mask = np.zeros(filtered_bw.shape, dtype="uint8")
            mask[labels == label] = 255

            if self.show_plots and not self.map_editor:
                plt.imshow(mask, cmap='gray')
                plt.show()

            if self.create_map and save_img:
                self.get_logger().info('[GB Planner]: Creating map for simulation...')
                image_name = self.map_name + ".png"
                dict_map = {'image': image_name,
                            'resolution': self.map_resolution,
                            'origin': [self.map_origin.position.x, self.map_origin.position.y, 0],
                            'negate': 0,
                            'occupied_thresh': 0.65,
                            'free_thresh': 0.196}
                # We need to copy the mask because altering it crashes the further excecution for saving it in the right orientation in the later course
                mask_copy = copy.deepcopy(mask)
                mask_copy = cv2.flip(mask_copy, 0)
                cv2.imwrite(os.path.join(self.map_dir, image_name), mask_copy)
                try:
                    with open(os.path.join(self.map_dir, self.map_name + ".yaml"), 'w') as file:
                        documents = yaml.dump(dict_map, file, default_flow_style=False)
                except IOError as exp:
                    self.get_logger().warn('[GB Planner]: Could not create a yaml file for simulator map')
                    print(exp)
                else:
                    sim_info_str = '[GB Planner]:' + f' Map for simulation done! Stored in {self.map_dir}'
                    self.get_logger().info(sim_info_str)
                    save_img = False

            # Find contours
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

            # save all closed contours
            for i, cont in enumerate(contours):
                opened = hierarchy[0][i][2] < 0 and hierarchy[0][i][3] < 0
                if not opened:
                    closed_contours.append(cont)

            # there must not be more (or less) than two closed contour
            if len(closed_contours) != 2:
                raise IOError("More than two track bounds detected! Check input")
            # draw the boundary into the centerline image
            cv2.drawContours(cent_img, closed_contours, 0, 255, 4)
            cv2.drawContours(cent_img, closed_contours, 1, 255, 4)

        # the longest closed contour is the outer boundary
        bound_long = max(closed_contours, key=len)
        bound_long = np.array(bound_long).flatten()

        # reshape from the shape [x1,y1,x2,y2,...] to [[x1,y1],[x2,y2],...]
        len_reshape = int(len(bound_long) / 2)
        bound_long = bound_long.reshape(len_reshape, 2)
        # convert to meter
        bound_long_meter = np.zeros(np.shape(bound_long))
        bound_long_meter[:, 0] = bound_long[:, 0] * self.map_resolution + self.map_origin.position.x
        bound_long_meter[:, 1] = bound_long[:, 1] * self.map_resolution + self.map_origin.position.y

        # inner boundary is the shorter one
        bound_short = min(closed_contours, key=len)
        bound_short = np.array(bound_short).flatten()

        # reshape from the shape [x1,y1,x2,y2,...] to [[x1,y1],[x2,y2],...]
        len_reshape = int(len(bound_short) / 2)
        bound_short = bound_short.reshape(len_reshape, 2)
        # convert to meter
        bound_short_meter = np.zeros(np.shape(bound_short))
        bound_short_meter[:, 0] = bound_short[:, 0] * self.map_resolution + self.map_origin.position.x
        bound_short_meter[:, 1] = bound_short[:, 1] * self.map_resolution + self.map_origin.position.y

        # Decide which boundary is left/right purely from the (already direction-aligned) centerline
        # geometry, NOT from the initial car pose or the OpenCV contour storage order (both were
        # unreliable and flipped left/right on some maps). Convention: with the track traversed
        # along the centerline direction, +d (frenet left) is the [-sin(psi), cos(psi)] normal, so a
        # boundary point lying on the +normal side of its nearest centerline point is the LEFT bound.
        cent_meter = np.zeros((centerline.shape[0], 2))
        cent_meter[:, 0] = centerline[:, 0] * self.map_resolution + self.map_origin.position.x
        cent_meter[:, 1] = centerline[:, 1] * self.map_resolution + self.map_origin.position.y
        # centerline tangent (direction of travel) at each centerline point
        cent_tangent = np.gradient(cent_meter, axis=0)

        def _mean_left_signed(bound_meter):
            # for each boundary point, project (bound - nearest_cent) onto the left normal
            signs = np.zeros(len(bound_meter))
            for i, p in enumerate(bound_meter):
                j = np.argmin((cent_meter[:, 0] - p[0]) ** 2 + (cent_meter[:, 1] - p[1]) ** 2)
                tx, ty = cent_tangent[j]
                n = math.hypot(tx, ty)
                if n < 1e-9:
                    continue
                # left normal of travel direction = [-ty, tx] / |t|
                nx, ny = -ty / n, tx / n
                signs[i] = (p[0] - cent_meter[j, 0]) * nx + (p[1] - cent_meter[j, 1]) * ny
            return np.mean(signs)

        # if bound_long sits on the +normal (left) side on average, it is the left boundary
        if _mean_left_signed(bound_long_meter) > 0:
            bound_left = bound_long_meter
            bound_right = bound_short_meter
        else:
            bound_right = bound_long_meter
            bound_left = bound_short_meter

        # if self.show_plots and not self.map_editor:
        if self.show_plots:
            plt.imshow(cent_img, cmap='gray')
            fig1, ax1 = plt.subplots()
            ax1.plot(bound_right[:, 0], bound_right[:, 1], 'b', label='Right bound')
            ax1.plot(bound_left[:, 0], bound_left[:, 1], 'g', label='Left bound')
            ax1.plot(centerline[:, 0] * self.map_resolution + self.map_origin.position.x,
                     centerline[:, 1] * self.map_resolution + self.map_origin.position.y, 'r', label='Centerline')
            ax1.legend()
            plt.show()

        return bound_right, bound_left

    def dist_to_bounds(self, trajectory: np.ndarray, bound_r, bound_l, centerline: np.ndarray, reverse=False):
        """
        Calculate the distance to track bounds for every point on a trajectory.

        Parameters
        ----------
        trajectory : np.ndarray
            A trajectory in form [s_m, x_m, y_m, psi_rad, vx_mps, ax_mps2] or [x_m, y_m]
        bound_r
            Points in meters of boundary right
        bound_l
            Points in meters of boundary left
        centerline : np.ndarray
            Centerline only needed if global trajectory is given and plot of it is wanted

        Returns
        -------
        dists_right, dists_left : tuple[np.ndarray, np.ndarray]
            Distances to the right and left track boundaries for every waypoint
        """
        # check format of trajectory
        if len(trajectory[0]) > 2:
            help_trajectory = trajectory[:, 1:3]
            trajectory_str = "Global Trajectory"
        else:
            help_trajectory = trajectory
            trajectory_str = "Centerline"

        # interpolate track bounds
        bound_r_tmp = np.column_stack((bound_r, np.zeros((bound_r.shape[0], 2))))
        bound_l_tmp = np.column_stack((bound_l, np.zeros((bound_l.shape[0], 2))))

        bound_r_int = helper_funcs_glob.src.interp_track.interp_track(reftrack=bound_r_tmp,
                                                                      stepsize_approx=0.1)
        bound_l_int = helper_funcs_glob.src.interp_track.interp_track(reftrack=bound_l_tmp,
                                                                      stepsize_approx=0.1)

        # find the closest points of the track bounds to global trajectory waypoints
        n_wpnt = len(help_trajectory)
        dists_right = np.zeros(n_wpnt)  # contains (min) distances between waypoints and right bound
        dists_left = np.zeros(n_wpnt)  # contains (min) distances between waypoints and left bound

        for i, wpnt in enumerate(help_trajectory):
            dists_bound_right = np.sqrt(np.power(bound_r_int[:, 0] - wpnt[0], 2)
                                        + np.power(bound_r_int[:, 1] - wpnt[1], 2))
            dists_right[i] = np.amin(dists_bound_right)

            dists_bound_left = np.sqrt(np.power(bound_l_int[:, 0] - wpnt[0], 2)
                                       + np.power(bound_l_int[:, 1] - wpnt[1], 2))
            dists_left[i] = np.amin(dists_bound_left)

        if self.show_plots and trajectory_str == "Global Trajectory":
            width_veh_real = 0.3
            normvec_normalized_opt = tph.calc_normal_vectors.calc_normal_vectors(trajectory[:, 3])

            veh_bound1_virt = trajectory[:, 1:3] + normvec_normalized_opt * self.safety_width / 2
            veh_bound2_virt = trajectory[:, 1:3] - normvec_normalized_opt * self.safety_width / 2

            veh_bound1_real = trajectory[:, 1:3] + normvec_normalized_opt * width_veh_real / 2
            veh_bound2_real = trajectory[:, 1:3] - normvec_normalized_opt * width_veh_real / 2

            # plot track including optimized path
            fig, ax = plt.subplots()

            ax.plot(bound_r[:, 0], bound_r[:, 1], "k-", linewidth=1.0, label="Track bounds")
            ax.plot(bound_l[:, 0], bound_l[:, 1], "k-", linewidth=1.0)
            ax.plot(centerline[:, 0], centerline[:, 1], "k--", linewidth=1.0, label='Centerline')

            ax.plot(veh_bound1_virt[:, 0], veh_bound1_virt[:, 1], "b", linewidth=0.5, label="Vehicle width with safety")
            ax.plot(veh_bound2_virt[:, 0], veh_bound2_virt[:, 1], "b", linewidth=0.5)
            ax.plot(veh_bound1_real[:, 0], veh_bound1_real[:, 1], "c", linewidth=0.5, label="Real vehicle width")
            ax.plot(veh_bound2_real[:, 0], veh_bound2_real[:, 1], "c", linewidth=0.5)

            ax.plot(trajectory[:, 1], trajectory[:, 2], 'tab:orange', linewidth=2.0, label="Global trajectory")

            plt.grid()
            ax1 = plt.gca()

            point1_arrow = np.array([trajectory[0, 1], trajectory[0, 2]])
            point2_arrow = np.array([trajectory[5, 1], trajectory[5, 2]])
            vec_arrow = (point2_arrow - point1_arrow)
            ax1.arrow(point1_arrow[0], point1_arrow[1], vec_arrow[0], vec_arrow[1], width=0.05,
                      head_width=0.3, head_length=0.3, fc='g', ec='g')

            ax.set_aspect("equal", "datalim")
            plt.xlabel("x-distance from origin [m]")
            plt.ylabel("y-distance from origin [m]")
            plt.title("Global trajectory with vehicle width")
            plt.legend()

            plt.show()
        # NOTE: no reverse-swap here. Left/right is now decided in extract_track_bounds from the
        # (already reverse-aware) centerline direction, so bound_r/bound_l passed in already carry
        # the correct sides for both forward and reversed mapping. Swapping again would double-flip.
        return dists_right, dists_left

    def add_dist_to_cent(self, centerline_smooth: np.ndarray,
                         centerline_meter: np.ndarray, dist_transform=None,
                         bound_r: np.ndarray = None, bound_l: np.ndarray = None, reverse=False) -> np.ndarray:
        """
        Add distance to track bounds to the centerline points.

        Parameters
        ----------
        centerline_smooth : np.ndarray
            Smooth centerline in cells (not meters)
        centerline_meter : np.ndarray
            Smooth centerline in meters (not cells)
        dist_transform : Any, default=None
            Euclidean distance transform of the filtered black and white image
        bound_r : np.ndarray, default=None
            Points of the right track bound in meters
        bound_l : np.ndarray, default=None
            Points of the left track bound in meters

        Returns
        -------
        centerline_comp : np.ndarray
            Complete centerline with distance to right and left track bounds for every point
        """
        centerline_comp = np.zeros((len(centerline_meter), 4))

        if dist_transform is not None:
            width_track_right = dist_transform[centerline_smooth[:, 1].astype(int),
                                               centerline_smooth[:, 0].astype(int)] * self.map_resolution
            if len(width_track_right) != len(centerline_meter):
                width_track_right = np.interp(np.arange(0, len(centerline_meter)), np.arange(0, len(width_track_right)),
                                              width_track_right)
            width_track_left = width_track_right
        elif bound_r is not None and bound_l is not None:
            width_track_right, width_track_left = self.dist_to_bounds(centerline_meter, bound_r, bound_l,
                                                                      centerline=centerline_meter,
                                                                      reverse=reverse)
        else:
            raise IOError("No closed contours found...")

        centerline_comp[:, 0] = centerline_meter[:, 0]
        centerline_comp[:, 1] = centerline_meter[:, 1]
        centerline_comp[:, 2] = width_track_right
        centerline_comp[:, 3] = width_track_left
        return centerline_comp

    @staticmethod
    def write_centerline(centerline: np.ndarray, sp_bool: bool = False) -> MarkerArray:
        """
        Create a csv file with centerline maps.

        The centerline maps includes position and width to the track bounds in meter, so that it can be used in the
        global trajectory optimizer. The file has the following format: x_m, y_m, w_tr_right_m, w_tr_left_m .

        Parameters
        ----------
        centerline : np.ndarray
            The centerline in form [x_m, y_m, w_tr_right_m, w_tr_left_m]
        sp_bool : bool, default=False
            Used for shortest path optimization when another centerline csv should be created

        Returns
        -------
        centerline_markers : MarkerArray
            Centerline as a MarkerArray which can be published
        """
        centerline_markers = MarkerArray()
        centerline_wpnts = WpntArray()

        # Intermediate track file consumed by trajectory_optimizer (track_name +
        # '.csv'). Written to /tmp so it doesn't litter the launch cwd; the
        # optimizer resolves the same /tmp path from track_name.
        cent_name = 'map_centerline' if not sp_bool else 'map_centerline_2'
        cent_str = os.path.join('/tmp', cent_name + '.csv')
        with open(cent_str, 'w', newline='') as file:
            writer = csv.writer(file)
            id_cnt = 0  # for marker id

            for row in centerline:
                x_m = row[0]
                y_m = row[1]
                width_tr_right_m = row[2]
                width_tr_left_m = row[3]
                writer.writerow([x_m, y_m, width_tr_right_m, width_tr_left_m])

                cent_marker = Marker()
                cent_marker.header.frame_id = 'map'
                cent_marker.type = cent_marker.SPHERE
                cent_marker.scale.x = 0.05
                cent_marker.scale.y = 0.05
                cent_marker.scale.z = 0.05
                cent_marker.color.a = 1.0
                cent_marker.color.b = 1.0

                cent_marker.id = id_cnt
                cent_marker.pose.position.x = x_m
                cent_marker.pose.position.y = y_m
                cent_marker.pose.orientation.w = 1.0
                centerline_markers.markers.append(cent_marker)

                wpnt = Wpnt()
                wpnt.id = id_cnt
                wpnt.x_m = x_m
                wpnt.d_right = width_tr_right_m
                wpnt.d_left = width_tr_left_m
                wpnt.y_m = y_m
                centerline_wpnts.wpnts.append(wpnt)

                id_cnt += 1

        return centerline_wpnts, centerline_markers

    def publish_track_bounds(self, bound_r, bound_l, reverse: bool = False) -> MarkerArray:
        bounds_markers = MarkerArray()
        id_cnt = 0
        if reverse:
            bound_l_real = bound_r.copy()
            bound_r_real = bound_l.copy()
            publisher = self.vis_track_bnds_reverse
        else:
            bound_l_real = bound_l
            bound_r_real = bound_r
            publisher = self.vis_track_bnds

        for i, pnt_r in enumerate(bound_r_real):
            bnd_r_mrk = Marker()
            bnd_r_mrk.header.frame_id = 'map'
            bnd_r_mrk.type = bnd_r_mrk.SPHERE
            bnd_r_mrk.scale.x = 0.05
            bnd_r_mrk.scale.y = 0.05
            bnd_r_mrk.scale.z = 0.05
            bnd_r_mrk.color.a = 1.0
            bnd_r_mrk.color.b = 0.5
            bnd_r_mrk.color.r = 0.5

            bnd_r_mrk.id = id_cnt
            id_cnt += 1
            bnd_r_mrk.pose.position.x = pnt_r[0]
            bnd_r_mrk.pose.position.y = pnt_r[1]
            bnd_r_mrk.pose.orientation.w = 1.0
            bounds_markers.markers.append(bnd_r_mrk)

        for i, pnt_l in enumerate(bound_l_real):
            bnd_l_mrk = Marker()
            bnd_l_mrk.header.frame_id = 'map'
            bnd_l_mrk.type = bnd_l_mrk.SPHERE
            bnd_l_mrk.scale.x = 0.05
            bnd_l_mrk.scale.y = 0.05
            bnd_l_mrk.scale.z = 0.05
            bnd_l_mrk.color.a = 1.0
            bnd_l_mrk.color.r = 0.5
            bnd_l_mrk.color.g = 1.0

            bnd_l_mrk.id = id_cnt
            id_cnt += 1
            bnd_l_mrk.pose.position.x = pnt_l[0]
            bnd_l_mrk.pose.position.y = pnt_l[1]
            bnd_l_mrk.pose.orientation.w = 1.0
            bounds_markers.markers.append(bnd_l_mrk)

        publisher.publish(bounds_markers)

        return bounds_markers

    def create_wpnts_markers(self, trajectory: np.ndarray, d_right: np.ndarray, d_left: np.ndarray,
                             second_traj: bool = False):
        """
        Create and return a waypoint array and a marker array.

        Parameters
        ----------
        trajectory : np.ndarray
            A trajectory with waypoints in form [s_m, x_m, y_m, psi_rad, vx_mps, ax_mps2]
        d_right : np.ndarray
            Distances to the right track bounds for every waypoint in {trajectory}
        d_left : np.ndarray
            Distances to the left track bounds for every waypoint in {trajectory}
        second_traj : bool, default=False
            Display second trajectory with a different color than first, better for visualization

        Returns
        -------
        global_wpnts, global_markers : tuple[WpntArray, MarkerArray]
            A waypoint array and a marker array with all points of {trajectory}
        """
        max_vx_mps = max(trajectory[:, 5])
        speed_string = "[GB Planner]: Max speed: " + str(max_vx_mps)
        self.get_logger().info(speed_string)

        global_wpnts = WpntArray()
        global_markers = MarkerArray()

        for i, pnt in enumerate(trajectory):
            global_wpnt = Wpnt()
            global_wpnt.id = i
            global_wpnt.s_m = pnt[0]
            global_wpnt.x_m = pnt[1]
            global_wpnt.d_right = d_right[i]
            global_wpnt.d_left = d_left[i]
            global_wpnt.y_m = pnt[2]
            global_wpnt.psi_rad = self.conv_psi(pnt[3])
            global_wpnt.kappa_radpm = pnt[4]
            global_wpnt.vx_mps = pnt[5]
            global_wpnt.ax_mps2 = pnt[6]

            global_wpnts.wpnts.append(global_wpnt)

            global_marker = Marker()
            global_marker.header.frame_id = 'map'
            global_marker.type = global_marker.CYLINDER
            global_marker.scale.x = 0.1
            global_marker.scale.y = 0.1
            global_marker.scale.z = global_wpnt.vx_mps / max_vx_mps
            global_marker.color.a = 1.0
            global_marker.color.r = 1.0
            global_marker.color.g = 1.0 if second_traj else 0.0

            global_marker.id = i
            global_marker.pose.position.x = pnt[1]
            global_marker.pose.position.y = pnt[2]
            global_marker.pose.position.z = global_wpnt.vx_mps / max_vx_mps / 2
            global_marker.pose.orientation.w = 1.0
            global_markers.markers.append(global_marker)

        return global_wpnts, global_markers

    @staticmethod
    def conv_psi(psi: float) -> float:
        """
        Convert psi angles where 0 is at the y-axis into psi angles where 0 is at the x-axis.

        Parameters
        ----------
        psi : float
            angle (in rad) to convert

        Returns
        -------
        new_psi : float
            converted angle
        """
        new_psi = psi + math.pi / 2

        if new_psi > math.pi:
            new_psi = new_psi - 2 * math.pi

        return new_psi

    @staticmethod
    def compare_direction(alpha: float, beta: float) -> bool:
        """
        Compare the direction of two points and check if they point in the same direction.

        Parameters
        ----------
        alpha : float
            direction angle in rad
        beta : float
            direction angle in rad

        Returns
        -------
        bool
            True if alpha and beta point in the same direction
        """
        # callers sometimes pass np.angle([...]) results (length-1 arrays);
        # coerce to scalars so math.fabs works under newer numpy.
        alpha = float(np.asarray(alpha).reshape(-1)[0])
        beta = float(np.asarray(beta).reshape(-1)[0])
        delta_theta = math.fabs(alpha - beta)

        if delta_theta > math.pi:
            delta_theta = 2 * math.pi - delta_theta

        return delta_theta < math.pi / 2


def main(args=None):
    rclpy.init(args=args)
    planner = GlobalPlanner()
    try:
        planner.global_plan_loop()
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
