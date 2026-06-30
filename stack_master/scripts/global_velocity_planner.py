#!/usr/bin/env python3
"""Global velocity (re)planner / tuning script.

ROS2 port of the ros1 stack_master/scripts/global_velocity_planner.py.

Subscribes to /global_waypoints, recomputes the velocity + acceleration profile
from the track curvature with the limits hardcoded below, and republishes it.
Edit the values in __init__ to tune. With save_csv:=true the chosen limits are
also written back to config/<racecar_version>/veh_dyn_info/*.csv; otherwise only
the published topic is updated.
"""

import os
import json
import configparser

import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import trajectory_planning_helpers as tph
from vel_planner.vel_planner import calc_vel_profile
from f110_msgs.msg import WpntArray


class VelocityPlanner(Node):
    def __init__(self):
        super().__init__('global_velplanner')

        self.declare_parameter('racecar_version', 'SIM')
        self.declare_parameter('save_csv', False)
        self.racecar_version = self.get_parameter('racecar_version').value
        self.save_csv = self.get_parameter('save_csv').value

        config_dir = os.path.join(
            get_package_share_directory('stack_master'), 'config', self.racecar_version)
        self.veh_dyn_dir = os.path.join(config_dir, 'veh_dyn_info')

        # Velocity Planning
        parser = configparser.ConfigParser()
        self.pars = {}
        if not parser.read(os.path.join(config_dir, 'racecar_f110.ini')):
            raise ValueError('Specified config file does not exist or is empty!')
        self.pars["veh_params"] = json.loads(parser.get('GENERAL_OPTIONS', 'veh_params'))
        self.pars["vel_calc_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'vel_calc_opts'))

        ggv_path = os.path.join(self.veh_dyn_dir, 'ggv.csv')
        ax_max_path = os.path.join(self.veh_dyn_dir, 'ax_max_machines.csv')
        b_ax_max_path = os.path.join(self.veh_dyn_dir, 'b_ax_max_machines.csv')
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

        # ---- tune here: hardcoded limits override the ini/csv values ----
        self.v_max = 12.0
        self.ax_max_motor = 6.0
        self.ax_max_brake = 8.0
        self.dyn_model_exp = 1.0

        self.a_y_max = 6.0
        self.a_x_max = 8.0

        self.ggv[:, 1] = self.a_x_max
        self.ggv[:, 2] = self.a_y_max
        self.ax_max_machines[:, 1] = self.ax_max_motor
        self.b_ax_max_machines[:, 1] = self.ax_max_brake

        if self.save_csv:
            self._save_csvs()

        self.glb_wpnts_pub = self.create_publisher(WpntArray, '/global_waypoints', 10)
        self.create_subscription(WpntArray, '/global_waypoints', self.wpnts_callback, 10)
        self._publishing = False  # guard against the self-subscription feedback loop

    def _save_csvs(self):
        """Write the tuned limits back to the veh_dyn_info csvs."""
        np.savetxt(os.path.join(self.veh_dyn_dir, 'ggv.csv'), self.ggv,
                   fmt=['%.1f', '%.2f', '%.2f'], delimiter=',',
                   header='v_mps,ax_max_mps2,ay_max_mps2', comments='# ')
        np.savetxt(os.path.join(self.veh_dyn_dir, 'ax_max_machines.csv'), self.ax_max_machines,
                   fmt=['%.2f', '%.2f'], delimiter=',',
                   header='v_mps, ax_max_machines_mps2', comments='#')
        np.savetxt(os.path.join(self.veh_dyn_dir, 'b_ax_max_machines.csv'), self.b_ax_max_machines,
                   fmt=['%.2f', '%.2f'], delimiter=',',
                   header='v_mps, ax_max_machines_mps2', comments='#')
        self.get_logger().info(f"saved tuned veh_dyn_info csvs to {self.veh_dyn_dir}")

    def wpnts_callback(self, msg):
        if self._publishing:   # ignore our own republished message
            return
        wpnts = msg.wpnts

        kappa = np.array([wp.kappa_radpm for wp in wpnts])
        el_lengths = 0.1 * np.ones(len(kappa))

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
            wpnts[i].vx_mps = float(vx_profile[i])

        vx_profile_opt_cl = np.append(vx_profile, vx_profile[0])
        ax_profile = tph.calc_ax_profile.calc_ax_profile(vx_profile=vx_profile_opt_cl,
                                                         el_lengths=el_lengths,
                                                         eq_length_output=False)
        for i in range(len(ax_profile)):
            wpnts[i].ax_mps2 = float(ax_profile[i])

        msg.wpnts = wpnts
        self._publishing = True
        self.glb_wpnts_pub.publish(msg)
        self._publishing = False
        self.get_logger().info("NEW Vel Profile Pub")


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
