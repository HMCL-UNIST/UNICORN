#!/usr/bin/env python3

import rospy
import os
import numpy as np
import json
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped    
from std_msgs.msg import Float64    
import trajectory_planning_helpers as tph
from rospkg import RosPack
from f110_msgs.msg import ObstacleArray, OTWpntArray, WpntArray, Wpnt
# from calc_vel_profile import calc_vel_profile
from vel_planner.vel_planner import calc_vel_profile

from copy import deepcopy
import configparser

class VelocityPlanner:

    def __init__(self):
        """
        Initialize the node, subscribe to topics, create publishers and set up member variables.
        """

        self.racecar_version = rospy.get_param("/racecar_version")

        # Velocity Planning
        parser = configparser.ConfigParser()
        self.pars = {}
        if not parser.read(os.path.join(RosPack().get_path('stack_master'), 'config', self.racecar_version, 'racecar_f110.ini')):
            raise ValueError('Specified config file does not exist or is empty!')
        self.pars["veh_params"] = json.loads(parser.get('GENERAL_OPTIONS', 'veh_params'))
        self.pars["vel_calc_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'vel_calc_opts'))
        ggv_path = os.path.join(RosPack().get_path('stack_master'), 'config', self.racecar_version, "veh_dyn_info", "ggv.csv")
        ax_max_path = os.path.join(RosPack().get_path('stack_master'), 'config', self.racecar_version, "veh_dyn_info", "ax_max_machines.csv")
        b_ax_max_path = os.path.join(RosPack().get_path('stack_master'), 'config', self.racecar_version, "veh_dyn_info", "b_ax_max_machines.csv")
        self.ggv, self.ax_max_machines = tph.import_veh_dyn_info.\
            import_veh_dyn_info(ggv_import_path=ggv_path,
                                ax_max_machines_import_path=ax_max_path)
            
        _, self.b_ax_max_machines = tph.import_veh_dyn_info.\
            import_veh_dyn_info(ggv_import_path=ggv_path,
                                ax_max_machines_import_path=b_ax_max_path)

                                
        self.v_max = self.pars["veh_params"]["v_max"]
        self.drag_coeff = self.pars["veh_params"]["dragcoeff"]
        self.m_veh = self.pars["veh_params"]["mass"]
        self.filt_window = self.pars["vel_calc_opts"]["vel_profile_conv_filt_window"]
        self.dyn_model_exp = self.pars["vel_calc_opts"]["dyn_model_exp"] 



        self.v_max = 12.0
        self.ax_max_motor = 3.9
        self.ax_max_brake = 7.0
        self.dyn_model_exp = 1.0

        self.a_y_max = 5.0
        self.a_x_max = 7.0

        self.ggv[:,1] = self.a_x_max
        self.ggv[:,2] = self.a_y_max
        self.ax_max_machines[:,1] = self.ax_max_motor
        self.b_ax_max_machines[:,1] = self.ax_max_brake

        # Subscribe to the topics
        self.glb_wpnts_pub = rospy.Publisher('/global_waypoints', WpntArray, queue_size=10)

        rospy.Subscriber("/global_waypoints", WpntArray, self.wpnts_callback)

    def wpnts_callback(self, msg):
        wpnts = msg.wpnts


        kappa = np.array([wp.kappa_radpm for wp in wpnts])
        el_lengths = 0.1 * np.ones(len(kappa))
        # rospy.logwarn(f"{self.dyn_model_exp}")

        vx_profile = calc_vel_profile(ggv=self.ggv,
                                      ax_max_machines=self.ax_max_machines,
                                      b_ax_max_machines=self.b_ax_max_machines,
                                      v_max=self.v_max,
                                      kappa=kappa,
                                      el_lengths=el_lengths,
                                      closed=True,
                                      filt_window=self.filt_window,
                                      dyn_model_exp=self.dyn_model_exp,
                                      drag_coeff=self.drag_coeff,
                                      m_veh=self.m_veh)


        for i in range(len(vx_profile)):
            wpnts[i].vx_mps = vx_profile[i]
            
        vx_profile_opt_cl = np.append(vx_profile, vx_profile[0])

        ax_profile = tph.calc_ax_profile.calc_ax_profile(vx_profile=vx_profile_opt_cl,
                                                            el_lengths=el_lengths,
                                                            eq_length_output=False)


        for i in range(len(ax_profile)):
            wpnts[i].ax_mps2 = ax_profile[i]
        
        msg.wpnts = wpnts
        print("NEW Vel Profliie Pub")
        self.glb_wpnts_pub.publish(msg)


        


if __name__ == '__main__':
    simple_mux = VelocityPlanner()
    rospy.init_node("global_velplanner")
    rospy.spin()
