#!/usr/bin/env python3
"""Compute the vesc_imu_rot -> vesc_imu mounting rpy from gravity.

Place the car on level ground and keep it STILL. The node averages the IMU
specific force over `duration` seconds; at rest that vector is gravity's
reaction (+9.81 "up") expressed in the IMU frame, which pins down the mounting
roll and pitch. Yaw rotates about gravity and is therefore UNOBSERVABLE from
the accelerometer — it is kept from the existing static_transforms.yaml (or
the `yaw` parameter if set).

Math: static_tf_publisher applies rpy as R = Rz(y)*Ry(p)*Rx(r) (child->parent).
At rest we need R * u = e_z with u the normalized mean specific force in the
IMU frame. Since Rz(y)*e_z = e_z, yaw drops out and:

    roll  = atan2(u_y, u_z)                    (full range, handles upside-down)
    pitch = atan2(-u_x, sqrt(u_y^2 + u_z^2))   (in [-90, 90] deg)

The result is PRINTED as a ready-to-paste YAML block for
stack_master/config/CAR/static_transforms.yaml — this tool does not write any
file.

Usage (car stationary on level ground):

    ros2 run stack_master imu_mount_calibration.py
    ros2 run stack_master imu_mount_calibration.py --ros-args -p duration:=5.0 -p yaw:=0.0
"""
import math

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import yaml

G = 9.80665


class ImuMountCalibration(Node):
    def __init__(self):
        super().__init__('imu_mount_calibration')
        self.declare_parameter('imu_topic', '/vesc/sensors/imu')
        self.declare_parameter('duration', 2.0)
        # NaN = keep the yaw currently in static_transforms.yaml (yaw is not
        # observable from gravity; it encodes the a/b/c/d mounting direction).
        self.declare_parameter('yaw', math.nan)

        self.duration = self.get_parameter('duration').value
        self.samples = []
        self.t_first = None

        topic = self.get_parameter('imu_topic').value
        self.sub = self.create_subscription(Imu, topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'keep the car STILL on level ground — collecting {self.duration:.1f}s '
            f'of {topic} ...')

    def cb(self, msg: Imu):
        now = self.get_clock().now()
        if self.t_first is None:
            self.t_first = now
        a = msg.linear_acceleration
        self.samples.append((a.x, a.y, a.z))
        if (now - self.t_first).nanoseconds * 1e-9 >= self.duration:
            self.finish()

    def current_yaw(self):
        yaw = self.get_parameter('yaw').value
        if not math.isnan(yaw):
            return yaw, 'parameter'
        cfg = (get_package_share_directory('stack_master')
               + '/config/CAR/static_transforms.yaml')
        try:
            with open(cfg) as f:
                params = yaml.safe_load(f)['/**']['ros__parameters']
            return float(params['vesc_imu_rot_to_vesc_imu']['rpy'][2]), cfg
        except (OSError, KeyError, TypeError) as e:
            self.get_logger().warn(f'could not read yaw from {cfg} ({e}) — using 0.0')
            return 0.0, 'fallback'

    def finish(self):
        self.destroy_subscription(self.sub)
        acc = np.array(self.samples)
        mean = acc.mean(axis=0)
        std = acc.std(axis=0)
        norm = float(np.linalg.norm(mean))

        n = len(self.samples)
        self.get_logger().info(f'{n} samples   mean=[{mean[0]:+.3f} {mean[1]:+.3f} '
                               f'{mean[2]:+.3f}] m/s^2   |mean|={norm:.3f}')

        ok = True
        if n < 20:
            self.get_logger().warn(f'only {n} samples — is the IMU publishing?')
            ok = False
        if abs(norm - G) > 0.5:
            self.get_logger().warn(
                f'|mean|={norm:.2f} differs from g={G:.2f} by more than 0.5 m/s^2 '
                '— car moving, vibrating, or wrong units?')
            ok = False
        if float(std.max()) > 0.3:
            self.get_logger().warn(
                f'per-axis std {std.round(3).tolist()} is high — car not still?')
            ok = False

        u = mean / norm
        roll = math.atan2(u[1], u[2])
        pitch = math.atan2(-u[0], math.hypot(u[1], u[2]))
        yaw, yaw_src = self.current_yaw()

        tilt = math.degrees(math.acos(np.clip(u[2], -1.0, 1.0)))
        self.get_logger().info(
            f'gravity tilt vs IMU z-axis: {tilt:.2f} deg   '
            f'roll={math.degrees(roll):+.2f} deg  pitch={math.degrees(pitch):+.2f} deg  '
            f'yaw kept from {yaw_src}: {math.degrees(yaw):+.1f} deg')

        # sanity: applying the result must map the measurement onto +z
        cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        up = ry @ rx @ u
        self.get_logger().info(f'check: calibrated gravity direction in vesc_imu_rot = '
                               f'[{up[0]:+.4f} {up[1]:+.4f} {up[2]:+.4f}] (want [0 0 1])')

        print('\n# paste into stack_master/config/CAR/static_transforms.yaml'
              + ('' if ok else '   # WARNING: see log above, result may be unreliable'))
        print('    vesc_imu_rot_to_vesc_imu:')
        print('      parent: vesc_imu_rot')
        print('      child: vesc_imu')
        print('      xyz: [0.0, 0.0, 0.0]')
        print(f'      rpy: [{roll:.7f}, {pitch:.7f}, {yaw:.7f}]'
              f'  # roll/pitch from gravity cal, yaw = mounting direction\n')
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = ImuMountCalibration()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
