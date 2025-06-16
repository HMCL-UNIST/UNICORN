#!/usr/bin/env python3

import rospy
from rospkg import RosPack
import os
from geometry_msgs.msg import PointStamped
from cartographer_ros_msgs.srv import GetTrajectoryStates
import subprocess

class SaveCartographerMap():
    def __init__(self):
        rospy.init_node('savemap', anonymous=True)

        rospy.on_shutdown(self.shutdown)

        # '/home/icra_crew/catkin_ws/src/f1tenth_system/racecar/racecar/config/common/slam'
        self.CONFIG_DIR = rospy.get_param('set_slam_pose_node/config_dir')
        self.CONFIG_BASE = rospy.get_param('set_slam_pose_node/config_base')  # 'localization.lua'

        self.map = rospy.get_param('~map')
        
        self.map_path = os.path.join(RosPack().get_path('stack_master'), 'maps', self.map, self.map + '.pbstream')

        # A variable to hold the initial pose of the robot to be set by the user in RViz
        self.initial_pose = PointStamped()
        self.ready = False

        # Get the initial pose from the user
        rospy.loginfo("Click the Publish Point button in RViz to save the cartographer...")

        #rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        rospy.Subscriber('/clicked_point', PointStamped, self.save_map)

        # 0 is the one we saved during mapping, so in loc only we start trajectory 1
        self.trajectory_num = 1

        while not rospy.is_shutdown():
            rospy.sleep(1)

    def get_next_trajectory_id(self):
        rospy.wait_for_service('/get_trajectory_states')
        try:
            get_trajectory_states = rospy.ServiceProxy('/get_trajectory_states', GetTrajectoryStates)
            response = get_trajectory_states()
            trajectory_ids = response.trajectory_states.trajectory_id
            if trajectory_ids:
                return trajectory_ids[-1] #max(trajectory_ids) + 1
            else:
                return 1  # 아무 것도 없으면 기본값 1로 시작
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return 1

    def save_map(self, msg):

        self.trajectory_num = self.get_next_trajectory_id()
        
        s = 'rosservice call /finish_trajectory ' + str(self.trajectory_num)
        process = subprocess.Popen(
            s, stdout=subprocess.PIPE, stderr=None, shell=True)
        output = process.communicate()
        # print(output[0])

        base_path = os.path.join(RosPack().get_path('stack_master'), 'maps', self.map)
        file_path = os.path.join(base_path, self.map + '.pbstream')
        version = 2
        print(os.path.exists(file_path))
        while os.path.exists(file_path):
            file_path = os.path.join(base_path, f"{self.map}_v{version}.pbstream")
            version += 1
        # print()
        # s = 'rosservice call /write_state "{filename: "' 
        # s = s + self.map_path + '", include_unfinished_submaps: "true"}"' # '$1', include_unfinished_submaps: "true"}"'
        # s = s + '"' + self.map_path + '", include_unfinished_submaps: "true"}"' 
        write_state_cmd = f'rosservice call /write_state "{{filename: \\"{file_path}\\", include_unfinished_submaps: true}}"'

        print("Calling:", write_state_cmd)
        process = subprocess.Popen(write_state_cmd, stdout=subprocess.PIPE, stderr=None, shell=True)
        output = process.communicate()

        # print(output[0])
        # self.trajectory_num += 1

    def shutdown(self):
        rospy.loginfo("Stopping setinitpose...")


if __name__ == '__main__':
    try:
        SaveCartographerMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Localize finished.")
