#!/usr/bin/env python3

import rospy
import numpy as np


from std_msgs.msg import String, Float32, Bool
from f110_msgs.msg import WpntArray, Wpnt
from visualization_msgs.msg import MarkerArray, Marker, InteractiveMarkerFeedback, InteractiveMarkerControl, InteractiveMarker
from geometry_msgs.msg import Pose, Point
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from scipy.spatial import cKDTree  # 최근접 이웃 탐색을 위한 KDTree
from scipy.signal import savgol_filter


# the `.` tells Python to look in the current directory at runtime
from readwrite_global_waypoints import read_global_waypoints, write_global_waypoints
from global_trajectory_tuner_helpers import straighten_1d, straighten_2d, Vel_Offset, Vel_Set, Vel_Weight, \
    entire_traj_translation, entire_traj_rotation, sampleCubicSplinesWithDerivative, cal_slope, cal_yaw
from pop_up_display import PopUpDisplay
from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer
from global_racetrajectory_optimization import helper_funcs_glob
import trajectory_planning_helpers as tph
    
    
class GlobalTrajTuner(object):
    """
    Node for publishing the global waypoints/markers and track bounds markers frequently after they have been calculated
    """
    def __init__(self):
        self.glb_markers = None
        self.glb_wpnts = None
        self.modified_glb_markers = None
        self.modified_glb_wpnts = None
        self.wpnts_data = None
        self.wpnts_xyz_data = None
        self.track_bounds = None
        self.sampling_step = 5
        self.glb_wpnts_pub = rospy.Publisher('global_waypoints', WpntArray, queue_size=10)
        self.glb_markers_pub = rospy.Publisher('global_waypoints/markers', MarkerArray, queue_size=10)
        self.update_map_pub = rospy.Publisher('update_map', Bool, queue_size=10)
        
        self.server = InteractiveMarkerServer("track_info_interactive")
        self.menu_handler = MenuHandler()
        self.pop_up_display = PopUpDisplay()

        self.main_menu_list = ["pose_smooth","vel_smooth","Anchor1","Anchor2","Pos_Straighten","Vel_Straighten","Vel_Set","int_idx","Control-Z","Update", "Save","entire_translation","entire_rotation"]
        self.sub_menu_list = ["10","20","30","idx"]
        # onoff_list = ["Lookahead","Vel"]
        
        self.map_name = rospy.get_param("~map")
        self.map_path = rospy.get_param("~map_dir")

        for main in self.main_menu_list:
            self.menu_handler.insert( main , callback=self.process_feedback)
        for i in range(2):
            for sub in self.sub_menu_list:
                self.menu_handler.insert( sub , parent=i+1, callback=self.process_feedback)
            
        self.anchor1 = None
        self.anchor2 = None
        # Read info from json file if it is provided, so everything is always published
        # self.map_path = self.get_parameter('map_path').get_parameter_value().string_value

        if self.map_name:
            
            rospy.loginfo(f"Reading parameters from {self.map_name}")
            try:
                (
                    self.map_infos, self.est_lap_time, self.centerline_markers,
                    self.centerline_wpnts, self.glb_markers, self.glb_wpnts,
                    self.glb_sp_markers, self.glb_sp_wpnts, self.track_bounds
                ) = read_global_waypoints(self.map_name)
                
                rospy.logwarn(f"{self.map_infos.data}")
            except FileNotFoundError:
                rospy.logwarn(f"{self.map_path} param not found. Not publishing")
        else:
            rospy.loginfo(f"global_trajectory_publisher did not find any self.map_path param {self.map_path}")
        self.wpnts_data = self.wpnts_to_ndarray(self.glb_wpnts)
        self.bounds_r, self.bounds_l = self.trackbounds_to_ndarray(self.track_bounds)        
        self.bounds_r = self.smooth_centerline(self.bounds_r)
        self.bounds_l = self.smooth_centerline(self.bounds_l)
        self.sub1_wpnts_data = np.copy(self.wpnts_data)
        self.sub2_wpnts_data = np.copy(self.wpnts_data)
        self.sub3_wpnts_data = np.copy(self.wpnts_data)
        self.sub4_wpnts_data = np.copy(self.wpnts_data)
        self.sub5_wpnts_data = np.copy(self.wpnts_data)
        
        self.max_id = len(self.wpnts_data)-1
        self.track_len = len(self.wpnts_data)
        self.wpnts_xyz_data = np.column_stack((self.wpnts_data[:,3:5], self.wpnts_data[:,7]))
        self.initialize_interactive_markers(self.wpnts_data)
        
        # Publish at 1 Hz
        # self.create_timer(1.0, self.timer_cb)
        
    def publish_update_map(self):
        bool_msg = Bool()
        self.update_map_pub.publish(bool_msg)


    def publish_global_traj(self):
        max_vx_mps = max(self.wpnts_data[:, 9])

        global_wpnts = WpntArray()
        global_markers = MarkerArray()

        for i, pnt in enumerate(self.wpnts_data):
            global_wpnt = Wpnt()
            global_wpnt.id = int(pnt[0])
            global_wpnt.s_m = pnt[1]
            global_wpnt.d_m = pnt[2]
            global_wpnt.x_m = pnt[3]
            global_wpnt.y_m = pnt[4]
            global_wpnt.d_right = pnt[5]
            global_wpnt.d_left = pnt[6]
            global_wpnt.psi_rad = pnt[7]
            global_wpnt.kappa_radpm = pnt[8]
            global_wpnt.vx_mps = pnt[9]
            global_wpnt.ax_mps2 = pnt[10]

            global_wpnts.wpnts.append(global_wpnt)

            global_marker = Marker()
            global_marker.header.frame_id = 'map'
            global_marker.type = global_marker.CYLINDER
            global_marker.scale.x = 0.1
            global_marker.scale.y = 0.1
            global_marker.scale.z = global_wpnt.vx_mps / max_vx_mps
            global_marker.color.a = 1.0
            global_marker.color.r = 1.0
            global_marker.color.g = 1.0

            global_marker.id = i
            global_marker.pose.position.x = global_wpnt.x_m
            global_marker.pose.position.y = global_wpnt.y_m
            global_marker.pose.position.z = global_wpnt.vx_mps / max_vx_mps / 2
            global_marker.pose.orientation.w = 1.0
            global_markers.markers.append(global_marker)

        self.glb_markers = global_markers
        self.glb_wpnts = global_wpnts
        self.glb_wpnts_pub.publish(global_wpnts)
        self.glb_markers_pub.publish(global_markers)
                
    def save_global_traj(self):
        write_global_waypoints(
            self.map_name,
            self.map_infos.data,
            self.est_lap_time.data,
            self.centerline_markers,
            self.centerline_wpnts,
            self.glb_markers,
            self.glb_wpnts,
            self.glb_sp_markers,
            self.glb_sp_wpnts,
            self.track_bounds
        )   
        rospy.loginfo(f"Successfully write to the {self.map_name}")

        
    def calculate_yaw_from_xy(self, xy_data):
        """
        Calculate yaw angles from a series of XY positions.

        Parameters:
            xy_data (numpy.ndarray): A 2D array with shape (N, 2), where each row is [x, y].

        Returns:
            numpy.ndarray: A 1D array of yaw angles with shape (N,).
                        The last yaw is set to match the second-to-last yaw for consistency.
        """
        if len(xy_data) < 2:
            raise ValueError("At least two waypoints are required to calculate yaw.")

        # Calculate differences in x and y
        dx = np.diff(xy_data[:, 0], append=xy_data[-1, 0])
        dy = np.diff(xy_data[:, 1], append=xy_data[-1, 1])

        # Compute yaw angles using arctan2
        yaw = np.arctan2(dy, dx)

        # Set the last yaw to match the second-to-last yaw for continuity
        yaw[-1] = yaw[-2]

        return yaw

    def update_yaw(self):
        """
        Update the yaw values for the waypoints in self.wpnts_data based on their XY positions.
        """
        if self.wpnts_data is None or len(self.wpnts_data) == 0:
            rospy.logwarning("No waypoints data available to update yaw.")
            return

        xy_data = self.wpnts_data[:, 3:5]  # Extract XY positions
        yaw_angles = self.calculate_yaw_from_xy(xy_data)  # Calculate yaw angles

        # Update yaw values in wpnts_data
        self.wpnts_data[:, 7] = yaw_angles
        rospy.loginfo("Yaw values updated based on XY positions.")

    def update_d(self):
        # Extract data
        xy_data = self.wpnts_data[:, 3:5]  # X, Y positions
        yaw_data = self.wpnts_data[:, 7]   # Yaw angles

        # Store updated values
        d_right = np.full(len(xy_data), np.inf)  # Initialize with large values
        d_left = np.full(len(xy_data), np.inf)   # Initialize with large values

        # Convert yaw angles to radians
        yaw_rad = yaw_data

        # Compute normal vectors (yaw - 90° for right, yaw + 90° for left)
        normal_right = np.column_stack((np.cos(yaw_rad - np.pi/2), np.sin(yaw_rad - np.pi/2)))  # (N, 2)
        normal_left = np.column_stack((np.cos(yaw_rad + np.pi/2), np.sin(yaw_rad + np.pi/2)))   # (N, 2)

        # Iterate over all waypoints
        for i, (pos, yaw, norm_r, norm_l) in enumerate(zip(xy_data, yaw_rad, normal_right, normal_left)):

            # Compute vectors from waypoint to all boundary points
            vecs_to_bounds_r = self.bounds_r - pos  # (M, 2)
            vecs_to_bounds_l = self.bounds_l - pos  # (M, 2)

            # Normalize boundary vectors
            norms_r = np.linalg.norm(vecs_to_bounds_r, axis=1, keepdims=True)
            norms_l = np.linalg.norm(vecs_to_bounds_l, axis=1, keepdims=True)
            unit_vecs_r = vecs_to_bounds_r / norms_r
            unit_vecs_l = vecs_to_bounds_l / norms_l

            # Compute cosine similarity (dot product with normal vector)
            cos_sim_r = np.einsum('ij,j->i', unit_vecs_r, norm_r)
            cos_sim_l = np.einsum('ij,j->i', unit_vecs_l, norm_l)

            # Convert cosine similarity to angle (in degrees)
            angles_r = np.degrees(np.arccos(np.clip(cos_sim_r, -1, 1)))
            angles_l = np.degrees(np.arccos(np.clip(cos_sim_l, -1, 1)))

            # Find boundary points within ±10° range
            valid_r = np.abs(angles_r) <= 5
            valid_l = np.abs(angles_l) <= 5

            # Compute distances for valid boundary points
            if np.any(valid_r):
                min_idx_r = np.argmin(norms_r[valid_r])  # Index of closest valid boundary point
                d_right[i] = norms_r[valid_r][min_idx_r][0]  # Store the minimum distance

            if np.any(valid_l):
                min_idx_l = np.argmin(norms_l[valid_l])  # Index of closest valid boundary point
                d_left[i] = norms_l[valid_l][min_idx_l][0]  # Store the minimum distance

        # Store results in the original array
        self.wpnts_data[:, 5] = d_right
        self.wpnts_data[:, 6] = d_left

        rospy.loginfo("Updated d_right and d_left for all waypoints.")
        
    def update_psi_kappa(self):
        
        xy = self.wpnts_data[:, 3:5]
        
        # interpolate centerline to 0.1m stepsize: less computation needed later for distance to track bounds
        xy = np.column_stack((xy, np.zeros((xy.shape[0], 2))))

        xy_int = helper_funcs_glob.src.interp_track.interp_track(reftrack=xy, stepsize_approx=0.1)[:, :2]

        psi, kappa = tph.calc_head_curv_num.\
            calc_head_curv_num(
                path=xy_int,
                el_lengths=0.1*np.ones(len(xy_int)-1),
                is_closed=False
            )
            
        # 최근접 이웃 탐색을 위한 KDTree 생성
        tree = cKDTree(self.wpnts_data[:, 3:5])  # 기존 x, y 데이터를 KDTree에 저장

        # 새로운 웨이포인트에 대해 최근접 기존 웨이포인트 찾기
        _, nearest_indices = tree.query(xy_int)  # 최근접 인덱스 찾기
        
        new_wpnts_data = []
        for i in range(len(xy_int)):
            wpnt_data = [
                i,
                0.1*i,
                0.,
                xy_int[i,0],
                xy_int[i,1],
                0.,
                0.,
                psi[i] + np.pi/2,
                kappa[i],
                self.wpnts_data[nearest_indices[i], 9],
                self.wpnts_data[nearest_indices[i], 10]
            ]
            new_wpnts_data.append(wpnt_data)
        # Convert the list to a numpy array
        self.wpnts_data =  np.array(new_wpnts_data)

    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        """
        Process feedback from Interactive Markers.

        Parameters
        ----------
        feedback : InteractiveMarkerFeedback
            Feedback message from the Interactive Marker.
        """
        m_name = feedback.marker_name
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id <= len(self.main_menu_list):
                selected_menu = self.main_menu_list[(feedback.menu_entry_id-1)]
                if selected_menu == "Anchor1":
                    position = self.server.get(m_name).pose.position
                    self.anchor1 = [int(m_name), position.x, position.y, position.z]
                    rospy.logwarn(f"{selected_menu} is {self.anchor1[0]} waypoint")
                elif selected_menu == "Anchor2":
                    position = self.server.get(m_name).pose.position
                    self.anchor2 = [int(m_name), position.x, position.y, position.z]
                    rospy.logwarn(f"{selected_menu} is {self.anchor2[0]} waypoint")
                elif selected_menu == "Vel_Straighten":
                    self.push_back_waypoints()
                    self.wpnts_data[:,9] = straighten_1d(self.anchor1, self.anchor2, self.wpnts_data[:,9])
                    self.initialize_interactive_markers(self.wpnts_data)
                    rospy.logwarn(f"Finish straightening velocity in z-axis!!")
                elif selected_menu == "Pos_Straighten":
                    self.push_back_waypoints()
                    self.wpnts_data[:,3:5] = straighten_2d(self.anchor1, self.anchor2, self.wpnts_data[:,3:5])
                    self.update_yaw()
                    self.initialize_interactive_markers(self.wpnts_data)
                    rospy.logwarn(f"Finish straightening position in xy-plane!!")             
                elif selected_menu == "Vel_Set":
                    self.push_back_waypoints()
                    self.pop_up_display.show_input_dialog("Vel_Set")
                    user_input = self.pop_up_display.user_input
                    self.wpnts_data[:,9] = Vel_Set(self.anchor1, self.anchor2, user_input,self.wpnts_data[:,9])
                    self.initialize_interactive_markers(self.wpnts_data)
                    rospy.logwarn(f"Finish setting velocity {user_input} m/s!!")             
                elif selected_menu == "int_idx":
                    self.pop_up_display.INTEGER=True
                    self.pop_up_display.show_input_dialog("int_dix")
                    if self.pop_up_display.user_input is None:
                        rospy.logwarn(f"Something Wrong INDEX!!")             
                    else:
                        self.sampling_step = self.pop_up_display.user_input
                        self.initialize_interactive_markers(self.wpnts_data)
                        self.pop_up_display.user_input = None
                        self.pop_up_display.INTEGER=False
                elif selected_menu == "Control-Z":
                    self.pull_forward_waypoints()
                    self.initialize_interactive_markers(self.wpnts_data)
                elif selected_menu == "entire_translation":
                    self.push_back_waypoints()
                    position = self.server.get(m_name).pose.position
                    reference = [int(m_name), position.x, position.y, position.z]
                    self.wpnts_data[:,3:5] = entire_traj_translation(reference, self.wpnts_data[:.3:5])
                    self.initialize_interactive_markers(self.wpnts_data)
                elif selected_menu == "entire_rotation":
                    self.push_back_waypoints()
                    self.wpnts_data[:,3:5] = entire_traj_rotation(self.anchor1, self.anchor2, self.wpnts_data[:,3:5])
                    self.update_yaw()
                    self.initialize_interactive_markers(self.wpnts_data)
                elif selected_menu == "Update":
                    self.publish_global_traj()
                    self.publish_update_map()
                elif selected_menu == "Save":
                    self.publish_global_traj()
                    self.save_global_traj()       
                else:
                    rospy.logwarn(f"Something Wrong!!")             
                
            elif feedback.menu_entry_id > len(self.main_menu_list) and feedback.menu_entry_id <=len(self.main_menu_list)+2*len(self.sub_menu_list):
                smooth_var = 10
                a=(feedback.menu_entry_id - len(self.main_menu_list)-1)//len(self.sub_menu_list)
                b=(feedback.menu_entry_id - len(self.main_menu_list)-1)%len(self.sub_menu_list)
                if b==3:
                    self.pop_up_display.show_input_dialog("Smooth length")
                    smooth_var = int(self.pop_up_display.user_input)
                    
                c=[10,20,30,smooth_var]
                
                self.push_back_waypoints()
                position = self.server.get(m_name).pose.position
                reference = [int(m_name), position.x, position.y, position.z]
                if a==0:
                    xy_yaw = np.column_stack((self.wpnts_data[:, 3:5], self.wpnts_data[:, 7]))
                    updated_xy_yaw = sampleCubicSplinesWithDerivative(reference, xy_yaw, c[b],"Pose" ,1.0 )
                    self.wpnts_data[:, 3:5] = updated_xy_yaw[:,0:2]
                    self.wpnts_data[:, 7] = updated_xy_yaw[:,2]
                    self.update_psi_kappa()
                    self.update_d()

                    
                    # self.update_yaw()
                    rospy.logwarn(f"Pose spline {c[b]}!!")             
                elif a==1:
                    self.wpnts_data[:,9] = sampleCubicSplinesWithDerivative(reference, self.wpnts_data[:,9],c[b],"Vel" ,1.0)
                    rospy.logwarn(f"vel spline {c[b]}!!")             
                self.initialize_interactive_markers(self.wpnts_data)
        return
    
    

    def create_marker_control(self, interaction_marker, interaction_mode, name, w,x,y,z):
        track_marker_control = InteractiveMarkerControl()
        track_marker_control.always_visible = True
        track_marker_control.name = name
        track_marker_control.orientation.w = w
        track_marker_control.orientation.x = x
        track_marker_control.orientation.y = y
        track_marker_control.orientation.z = z
        track_marker_control.interaction_mode = interaction_mode
        interaction_marker.controls.append(track_marker_control) 

    def smooth_centerline(self, centerline: np.ndarray) -> np.ndarray:
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
        # centerline_smooth = centerline
        # smooth centerline with a Savitzky Golay filter
        # filter_length = 20
        centerline_length = len(centerline)
        # print("Number of centerline points: ", centerline_length)

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
    
    def initialize_interactive_markers(self, waypoints_data):
        """
        Add waypoints from `waypoints_data` to the interactive marker server.

        Assumes `waypoints_data` is a list or ndarray where each row contains:
        [x, y, z, orientation_x, orientation_y, orientation_z, orientation_w].
        """
        if waypoints_data is None or len(waypoints_data) == 0:
            rospy.logwarning("No waypoints data available to add interactive markers.")
            return

        # rospy.loginfo("Adding waypoints to interactive markers...")

        for idx, wpnt in enumerate(waypoints_data):
            # Extract waypoint data
            x, y, vx_mps = wpnt[3], wpnt[4], wpnt[9] 
            orientation_x, orientation_y, orientation_z, orientation_w = 0.0, 0.0, np.sin(wpnt[7] / 2.0), np.cos(wpnt[7] / 2.0)

            # Create an Interactive Marker
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = "map"
            int_marker.name = f"{idx}"
            int_marker.scale = 0.5

            # Set the position and orientation
            int_marker.pose.position.x = x
            int_marker.pose.position.y = y
            int_marker.pose.position.z = vx_mps
            int_marker.pose.orientation.x = orientation_x
            int_marker.pose.orientation.y = orientation_y
            int_marker.pose.orientation.z = orientation_z
            int_marker.pose.orientation.w = orientation_w

            # Add a visualization marker
            marker = Marker()
            marker.type = Marker.SPHERE
            marker.scale.x = 0.1  # Reduced size (smaller sphere)
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0  # Red component for purple
            marker.color.g = 0.0  # Green component for purple
            marker.color.b = 1.0  # Blue component for purple
            marker.color.a = 1.0  # Fully opaque

            marker_control = InteractiveMarkerControl()
            marker_control.always_visible = True
            marker_control.markers.append(marker)
            int_marker.controls.append(marker_control)
            if idx%self.sampling_step == 0:
                # int_marker.description = f"idx : {idx}\n\n v : {vx_mps:.2f}"
                int_marker.description = f"v : {vx_mps:.2f}"
                # Add movement controls
                # self.create_marker_control(int_marker, InteractiveMarkerControl.MOVE_AXIS, "move_x", 1., 1., 0., 0.)
                # self.create_marker_control(int_marker, InteractiveMarkerControl.MOVE_AXIS, "move_y", 1., 0., 0., 1.)
                self.create_marker_control(int_marker, InteractiveMarkerControl.MOVE_AXIS, "move_z", 1., 0., 1., 0.)


            # Insert the marker into the server
            self.server.insert(int_marker)
            self.server.setCallback(int_marker.name, self.process_feedback)
            self.menu_handler.apply( self.server, int_marker.name )

            # --- 양쪽 (d_right, d_left) 마커 추가 ---
            # for side, d_offset in [("right", wpnt[5]), ("left", -wpnt[6])]:
            #     side_marker = InteractiveMarker()
            #     side_marker.header.frame_id = "map"
            #     side_marker.name = f"{idx}_{side}"
            #     side_marker.scale = 0.5

            #     # d_right, d_left를 이용하여 마커 위치 계산
            #     side_x = x + d_offset * np.cos(wpnt[7] - np.pi / 2)  # 왼쪽/오른쪽 이동
            #     side_y = y + d_offset * np.sin(wpnt[7] - np.pi / 2)

            #     side_marker.pose.position.x = side_x
            #     side_marker.pose.position.y = side_y
            #     side_marker.pose.position.z = 0
            #     side_marker.pose.orientation.x = orientation_x
            #     side_marker.pose.orientation.y = orientation_y
            #     side_marker.pose.orientation.z = orientation_z
            #     side_marker.pose.orientation.w = orientation_w

            #     # 시각화 마커 (초록색)
            #     side_vis_marker = Marker()
            #     side_vis_marker.type = Marker.SPHERE
            #     side_vis_marker.scale.x = 0.1
            #     side_vis_marker.scale.y = 0.1
            #     side_vis_marker.scale.z = 0.1
            #     side_vis_marker.color.r = 0.0
            #     side_vis_marker.color.g = 1.0  # Green
            #     side_vis_marker.color.b = 0.0
            #     side_vis_marker.color.a = 1.0

            #     side_marker_control = InteractiveMarkerControl()
            #     side_marker_control.always_visible = True
            #     side_marker_control.markers.append(side_vis_marker)
            #     side_marker.controls.append(side_marker_control)

            #     # Y축 이동만 가능하도록 설정
            #     # self.create_marker_control(side_marker, InteractiveMarkerControl.MOVE_AXIS, f"move_y_{side}", 1., 0., 0., 1.)

            #     # 서버에 추가
            #     self.server.insert(side_marker)
            #     self.server.setCallback(side_marker.name, self.process_feedback)
            #     self.menu_handler.apply(self.server, side_marker.name)
            
        # Apply changes to the server
        self.server.applyChanges()
        rospy.loginfo(f"Initialize {len(waypoints_data)} waypoints as interactive markers.")
        
    def push_back_waypoints(self):
        self.sub5_wpnts_data = np.copy(self.sub4_wpnts_data)
        self.sub4_wpnts_data = np.copy(self.sub3_wpnts_data)
        self.sub3_wpnts_data = np.copy(self.sub2_wpnts_data)
        self.sub2_wpnts_data = np.copy(self.sub1_wpnts_data)
        self.sub1_wpnts_data = np.copy(self.wpnts_data)

    def pull_forward_waypoints(self):
        self.sub4_wpnts_data = np.copy(self.sub5_wpnts_data)
        self.sub3_wpnts_data = np.copy(self.sub4_wpnts_data)
        self.sub2_wpnts_data = np.copy(self.sub3_wpnts_data)
        self.sub1_wpnts_data = np.copy(self.sub2_wpnts_data)
        self.wpnts_data = np.copy(self.sub1_wpnts_data)
        
    def wpnts_to_ndarray(self, wpnt_array):
        """
        Convert a WpntArray to a numpy ndarray.

        Parameters
        ----------
        wpnt_array : WpntArray
            The input waypoint array containing waypoints.

        Returns
        -------
        ndarray : np.ndarray
            A numpy array with waypoints in the form
            [s_m, x_m, y_m, psi_rad, vx_mps, ax_mps2, d_right, d_left].
        """
        # Initialize a list to hold waypoint data
        wpnt_list = []

        for wpnt in wpnt_array.wpnts:
            wpnt_data = [
                wpnt.id,
                wpnt.s_m,
                wpnt.d_m,
                wpnt.x_m,
                wpnt.y_m,
                wpnt.d_right,
                wpnt.d_left,
                wpnt.psi_rad,
                wpnt.kappa_radpm,
                wpnt.vx_mps,
                wpnt.ax_mps2
            ]
            wpnt_list.append(wpnt_data)
        # Convert the list to a numpy array
        return np.array(wpnt_list)
    
    def trackbounds_to_ndarray(self, track_bound):
        bounds_r = []
        bounds_l = []

        for marker in track_bound.markers:
            if marker.color.g == 1.0:
                bounds_l.append([marker.pose.position.x, marker.pose.position.y])
            elif marker.color.b == 0.5:
                bounds_r.append([marker.pose.position.x, marker.pose.position.y])
        # Convert the list to a numpy array
        return np.array(bounds_r), np.array(bounds_l) 

def main(args=None):
    rospy.init_node('global_traj_tuner_node')
    tuner = GlobalTrajTuner()
    rospy.spin()
    
if __name__ == '__main__':
    main()
