#!/usr/bin/env python3
"""Publish a configurable set of STATIC transforms from parameters.

Replaces hand-written `static_transform_publisher` nodes whose values were
hard-coded in the launch file, so the transforms can differ per vehicle
(config/SIM vs config/CAR) by editing a YAML — no launch change needed.

YAML schema (loaded via <param from=".../static_transforms.yaml">):

    /**:
      ros__parameters:
        transforms: [base_link_to_laser, base_link_to_vesc_imu_rot, vesc_imu_rot_to_vesc_imu]
        base_link_to_laser:
          parent: base_link
          child:  laser
          xyz:    [0.27, 0.0, 0.11]   # metres
          rpy:    [0.0, 0.0, 0.0]     # roll, pitch, yaw [rad]
        ...

An empty (or missing) `transforms` list publishes nothing — handy for SIM,
where these frames come from the gym URDF / aren't used.
"""
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations


class StaticTfPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher',
                         automatically_declare_parameters_from_overrides=True)
        self.br = StaticTransformBroadcaster(self)

        names = self.get_parameter('transforms').value if self.has_parameter('transforms') else []
        if not names:
            self.get_logger().warn('no "transforms" list in params — nothing to publish')
            return

        tfs = []
        stamp = self.get_clock().now().to_msg()
        for n in names:
            need = (f'{n}.parent', f'{n}.child', f'{n}.xyz', f'{n}.rpy')
            if not all(self.has_parameter(p) for p in need):
                self.get_logger().error(f'transform "{n}" missing parent/child/xyz/rpy — skipped')
                continue
            parent = str(self.get_parameter(f'{n}.parent').value)
            child = str(self.get_parameter(f'{n}.child').value)
            xyz = self.get_parameter(f'{n}.xyz').value
            rpy = self.get_parameter(f'{n}.rpy').value

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = float(xyz[0])
            t.transform.translation.y = float(xyz[1])
            t.transform.translation.z = float(xyz[2])
            q = tf_transformations.quaternion_from_euler(float(rpy[0]), float(rpy[1]), float(rpy[2]))
            t.transform.rotation.x = float(q[0])
            t.transform.rotation.y = float(q[1])
            t.transform.rotation.z = float(q[2])
            t.transform.rotation.w = float(q[3])
            tfs.append(t)
            self.get_logger().info(f'  {parent} -> {child}  xyz={list(xyz)} rpy={list(rpy)}')

        if tfs:
            self.br.sendTransform(tfs)
            self.get_logger().info(f'published {len(tfs)} static transform(s)')


def main(args=None):
    rclpy.init(args=args)
    node = StaticTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
