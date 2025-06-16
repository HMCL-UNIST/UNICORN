#!/usr/bin/env python3

from rospkg import RosPack
import rospy
import subprocess, os, copy

import csv
import yaml
import math
import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from skimage.morphology import skeletonize
from skimage.segmentation import watershed

from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer
from global_racetrajectory_optimization import helper_funcs_glob
import trajectory_planning_helpers as tph

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped
from f110_msgs.msg import Wpnt, WpntArray
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool, Float32

# To write global waypoints
from readwrite_global_waypoints import write_global_waypoints



if __name__ == "__main__":
    planner = GlobalPlanner()
    planner.global_plan_loop()


