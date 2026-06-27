#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from copy import deepcopy

from controller.estop import EStop


class SimpleMuxNode(Node):

    def __init__(self):
        super().__init__('simple_mux')

        self.declare_parameter('out_topic',                      'low_level/ackermann_cmd_mux/output')
        self.declare_parameter('in_topic',                       'high_level/ackermann_cmd')
        self.declare_parameter('joy_topic',                      '/joy')
        self.declare_parameter('keyboard_joy_topic',             '/joy_keyboard')
        self.declare_parameter('ego_control_topic',              '/ego/use_keyboard')
        self.declare_parameter('scan_topic',                     '/scan')
        self.declare_parameter('odom_topic',                     '/vesc/odom')
        self.declare_parameter('rate_hz',                        50.0)
        self.declare_parameter('joy_max_speed',                  4.0)
        self.declare_parameter('joy_max_steer',                  0.4)
        self.declare_parameter('joy_freshness_threshold',        1.0)
        self.declare_parameter('servo_min',                      0.15)
        self.declare_parameter('servo_max',                      0.85)
        self.declare_parameter('steering_angle_to_servo_offset', 0.5)
        self.declare_parameter('steering_angle_to_servo_gain',  -1.2135)
        self.declare_parameter('use_estop',  False)
        self.declare_parameter('sim',  False)
        # initial human-drive source; the pitwall panel can flip it at runtime
        # via ego_control_topic. False = joy, True = keyboard (/joy_keyboard).
        self.declare_parameter('use_keyboard', False)
        p = lambda name: self.get_parameter(name).value

        out_topic          = p('out_topic')
        in_topic           = p('in_topic')
        joy_topic          = p('joy_topic')
        keyboard_joy_topic = p('keyboard_joy_topic')
        ego_control_topic  = p('ego_control_topic')
        scan_topic         = p('scan_topic')
        odom_topic         = p('odom_topic')

        self.use_estop = p('use_estop')
        self.max_speed               = p('joy_max_speed')
        self.max_steer               = p('joy_max_steer')
        self.joy_freshness_threshold = p('joy_freshness_threshold')

        servo_offset = p('steering_angle_to_servo_offset')
        servo_gain   = p('steering_angle_to_servo_gain')
        self.servo_max_abs = min(
            abs((p('servo_max') - servo_offset) / servo_gain),
            abs((p('servo_min') - servo_offset) / servo_gain),
        )
        

        self.current_host = 'autodrive' if p('sim') else None
        self.human_drive  = None
        self.autodrive    = None
        self.scan         = None
        self.odom         = None
        # Ego human-drive input source: initial value from the use_keyboard param,
        # then overridable at runtime by the pitwall panel via ego_control_topic.
        # False = joy (physical pad on joy_topic), True = keyboard (keyboard_joy_topic).
        self.use_keyboard = p('use_keyboard')

        self.create_subscription(AckermannDriveStamped, in_topic,           self._drive_cb,         10)
        self.create_subscription(Joy,                   joy_topic,          self._joy_cb,           10)
        self.create_subscription(Joy,                   keyboard_joy_topic, self._joy_keyboard_cb,  10)
        self.create_subscription(Bool,                  ego_control_topic,  self._ego_control_cb,   10)
        if self.use_estop:
            self.estop = EStop(self)

            self.create_subscription(LaserScan, scan_topic, self._scan_cb, qos_profile_sensor_data)
            self.create_subscription(Odometry,  odom_topic, self._odom_cb, 10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, out_topic, 10)
        self.create_timer(1.0 / p('rate_hz'), self._loop)

    def _scan_cb(self, msg): self.scan = msg
    def _odom_cb(self, msg): self.odom = msg
    def _drive_cb(self, msg): self.autodrive = msg

    def _is_fresh(self, msg):
        if msg is None:
            return False
        dt = (self.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        return abs(dt) < self.joy_freshness_threshold

    def _clip(self, msg):
        out = deepcopy(msg)
        out.drive.steering_angle = max(-self.servo_max_abs, min(self.servo_max_abs, out.drive.steering_angle))
        return out

    def _loop(self):
        zero = AckermannDriveStamped()
        zero.header.stamp = self.get_clock().now().to_msg()

        if self.current_host is None:
            return
        elif self.current_host == 'autodrive' and self._is_fresh(self.autodrive):            # out = self._clip(self.autodrive)
            out = deepcopy(self.autodrive)
        elif self.current_host == 'humandrive' and self._is_fresh(self.human_drive):
            # out = self._clip(self.human_drive)
            out = deepcopy(self.human_drive)
        else:
            out = zero

        if self.use_estop:
            out = self.estop.should_stop(self.scan, self.odom, out)

        self.drive_pub.publish(out)

    def _ego_control_cb(self, msg):
        self.use_keyboard = bool(msg.data)

    def _joy_cb(self, msg):
        # physical pad: honoured only while joy is the selected source
        if not self.use_keyboard:
            self._handle_joy(msg)

    def _joy_keyboard_cb(self, msg):
        # keyboard_joy_node: honoured only while keyboard is the selected source
        if self.use_keyboard:
            self._handle_joy(msg)

    def _handle_joy(self, msg):
        use_human = msg.buttons[4] if len(msg.buttons) > 4 else False
        use_auto  = msg.buttons[5] if len(msg.buttons) > 5 else False

        if use_human:
            drive = AckermannDriveStamped()
            drive.header.stamp = self.get_clock().now().to_msg()
            drive.drive.steering_angle = msg.axes[3] * self.max_steer if len(msg.axes) > 3 else 0.0
            drive.drive.speed          = msg.axes[1] * self.max_speed  if len(msg.axes) > 1 else 0.0
            self.human_drive   = drive
            self.current_host  = 'humandrive'
        elif use_auto:
            self.current_host = 'autodrive'


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
