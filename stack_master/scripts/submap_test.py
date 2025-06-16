#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from cartographer_ros_msgs.msg import SubmapList

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
        self.name = "submap_count"
        rospy.init_node(self.name, anonymous=True)


        rospy.Subscriber("/submap_list", SubmapList, self.submap_callback)

    def submap_callback(self, msg):
        num = 0

        for sub in msg.submap:  
            if sub.trajectory_id == 0:
                num += 1

        print(num)
        print(f"total submap : {len(msg.submap)}")






if __name__ == '__main__':
    simple_mux = SimpleMuxNode()
    rospy.spin()
