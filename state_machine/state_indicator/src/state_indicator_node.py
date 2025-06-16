#!/usr/bin/env python3

import rospy

from visualization_msgs.msg import Marker
from blink1.msg import Blink1msg

BL_FADE      = 1  # fade to the RGB color, t is the time of fading in ms
BL_ON        = 2  # turn on to the RGB color, t is ignored
BL_BLINK     = 3  # blink the RGB color, t is the period in ms
BL_RANDBLINK = 4  # blink at random RGB colors, t is the period in ms


class StateIndicatorNode:
    """
    This class implements a ROS node that handles the terminal utilities of the state indicator.
    """

    def __init__(self):
        """
        Initialize the node, subscribe to topics, create publishers
        """

        # Initialize the node
        self.name = "state_indicator_node"
        rospy.init_node(self.name, anonymous=True)

        # Subscribe to the topics
        rospy.Subscriber('/state_marker', Marker, self.state_callback)
        # Publish the topics
        self.blink_pub = rospy.Publisher('blink1/blink', Blink1msg, queue_size=10)

    #############
    # CALLBACKS #
    #############

    def state_callback(self, state_msg):
        """
        Callback function for the state of the racecar. 

        Args:
        - state_msg (String): The received message containing the state.
        """
        blink_msg = Blink1msg()
        blink_msg.function = BL_ON # BL_FADE, BL_ON, BL_RANDBLINK, BL_BLINK
        blink_msg.t= 10 # miliseconds
        blink_msg.r= int(255 * state_msg.color.r)
        blink_msg.g= int(255 * state_msg.color.g)
        blink_msg.b= int(255 * state_msg.color.b)        
        self.blink_pub.publish(blink_msg)


if __name__ == '__main__':
    state_indicator = StateIndicatorNode()
    rospy.spin()
    # state_indicator.run()

