"""Read a rosbag2 (.mcap) and extract the time series needed for VESC odometry
calibration and localization evaluation.

All topics are placed on a single, common timeline using the message *log time*
(the recorder clock), which is consistent across topics because the bag was
recorded in real time on one machine.  Times are returned in seconds, offset so
the first message in the bag is t = 0.

No ROS installation is required: ``mcap_ros2`` decodes every message (including
the custom ``vesc_msgs``) straight from the schema embedded in the .mcap file.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from mcap_ros2.reader import read_ros2_messages


# ---- topics we care about -------------------------------------------------
GT_POSE_TOPIC = "/tracked_pose"            # cartographer output -> ground truth
GT_ODOM_TOPIC = "/tracked_pose/odom"
VESC_ODOM_TOPIC = "/vesc/odom"             # wheel dead-reckoning (free-running)
CAR_STATE_TOPIC = "/car_state/odom"        # EKF output (carto-locked, reference)
CORE_TOPIC = "/vesc/sensors/core"          # eRPM, motor current, ...
SERVO_TOPIC = "/vesc/sensors/servo_position_command"
IMU_TOPIC = "/vesc/sensors/imu_vesc"       # VESC-native AHRS: ypr (deg), gyro, accel
CMD_SPEED_TOPIC = "/vesc/commands/motor/speed"
CMD_SERVO_TOPIC = "/vesc/commands/servo/position"
ACKERMANN_TOPIC = "/vesc/ackermann_cmd"   # drive.speed [m/s], drive.steering_angle [rad]

ALL_TOPICS = [
    GT_POSE_TOPIC, GT_ODOM_TOPIC, VESC_ODOM_TOPIC, CAR_STATE_TOPIC,
    CORE_TOPIC, SERVO_TOPIC, IMU_TOPIC, CMD_SPEED_TOPIC, CMD_SERVO_TOPIC,
    ACKERMANN_TOPIC,
]


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about +Z) from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass
class Series:
    """A scalar (or pose) time series; every field is a 1-D numpy array."""
    t: np.ndarray = field(default_factory=lambda: np.empty(0))

    def __len__(self):
        return len(self.t)


@dataclass
class BagData:
    t0_ns: int
    duration: float
    # ground truth (cartographer tracked_pose, map frame)
    gt: Series = field(default_factory=Series)
    # raw VESC wheel odometry (odom frame, free running)
    vo: Series = field(default_factory=Series)
    # EKF / car_state (map frame, carto-locked -> reference only)
    cs: Series = field(default_factory=Series)
    # VESC core telemetry
    core: Series = field(default_factory=Series)
    # servo position command (steering)
    servo: Series = field(default_factory=Series)
    # AHRS IMU
    imu: Series = field(default_factory=Series)
    # high-level commands
    cmd_speed: Series = field(default_factory=Series)
    cmd_servo: Series = field(default_factory=Series)
    # ackermann command (steering_angle [rad], speed [m/s]) -- clean control reference
    ack: Series = field(default_factory=Series)


def read_bag(path: str, t_start: float | None = None, t_end: float | None = None) -> BagData:
    """Read ``path`` (an .mcap file) and return a :class:`BagData`.

    ``t_start`` / ``t_end`` (seconds, relative to bag start) optionally crop the
    record, e.g. to drop a stationary warm-up at the beginning.
    """
    buf: dict[str, dict[str, list]] = {t: {} for t in ALL_TOPICS}

    def push(topic, key, val):
        buf[topic].setdefault(key, []).append(val)

    t0_ns = None
    last_ns = None
    for m in read_ros2_messages(path, topics=ALL_TOPICS):
        ns = m.log_time_ns
        if t0_ns is None:
            t0_ns = ns
        last_ns = ns
        t = (ns - t0_ns) * 1e-9
        topic = m.channel.topic
        msg = m.ros_msg
        push(topic, "t", t)

        if topic == GT_POSE_TOPIC:
            p = msg.pose
            push(topic, "x", p.position.x)
            push(topic, "y", p.position.y)
            q = p.orientation
            push(topic, "yaw", yaw_from_quat(q.x, q.y, q.z, q.w))
        elif topic in (VESC_ODOM_TOPIC, CAR_STATE_TOPIC, GT_ODOM_TOPIC):
            p = msg.pose.pose
            push(topic, "x", p.position.x)
            push(topic, "y", p.position.y)
            q = p.orientation
            push(topic, "yaw", yaw_from_quat(q.x, q.y, q.z, q.w))
            push(topic, "v", msg.twist.twist.linear.x)
            push(topic, "wz", msg.twist.twist.angular.z)
        elif topic == CORE_TOPIC:
            s = msg.state
            push(topic, "erpm", s.speed)
            push(topic, "current", s.current_motor)
            push(topic, "duty", s.duty_cycle)
            push(topic, "disp", float(s.displacement))
        elif topic == SERVO_TOPIC:
            push(topic, "servo", msg.data)
        elif topic == IMU_TOPIC:
            im = msg.imu
            # VescImu reports ypr in DEGREES and angular_velocity in DEG/S
            # (the sibling /vesc/sensors/imu carries the same data in rad & rad/s).
            push(topic, "yaw", math.radians(im.ypr.z))        # AHRS heading -> rad
            push(topic, "gyro_z", math.radians(im.angular_velocity.z))  # -> rad/s
            push(topic, "acc_x", im.linear_acceleration.x)
            push(topic, "acc_y", im.linear_acceleration.y)
        elif topic == CMD_SPEED_TOPIC:
            push(topic, "cmd_speed", msg.data)
        elif topic == CMD_SERVO_TOPIC:
            push(topic, "cmd_servo", msg.data)
        elif topic == ACKERMANN_TOPIC:
            push(topic, "ack_steer", msg.drive.steering_angle)
            push(topic, "ack_speed", msg.drive.speed)

    duration = (last_ns - t0_ns) * 1e-9 if last_ns else 0.0

    def to_series(topic, fields):
        d = buf[topic]
        s = Series()
        s.t = np.asarray(d.get("t", []), dtype=float)
        for f in fields:
            setattr(s, f, np.asarray(d.get(f, []), dtype=float))
        return s

    data = BagData(t0_ns=t0_ns or 0, duration=duration)
    data.gt = to_series(GT_POSE_TOPIC, ["x", "y", "yaw"])
    data.vo = to_series(VESC_ODOM_TOPIC, ["x", "y", "yaw", "v", "wz"])
    data.cs = to_series(CAR_STATE_TOPIC, ["x", "y", "yaw", "v", "wz"])
    data.core = to_series(CORE_TOPIC, ["erpm", "current", "duty", "disp"])
    data.servo = to_series(SERVO_TOPIC, ["servo"])
    data.imu = to_series(IMU_TOPIC, ["yaw", "gyro_z", "acc_x", "acc_y"])
    data.cmd_speed = to_series(CMD_SPEED_TOPIC, ["cmd_speed"])
    data.cmd_servo = to_series(CMD_SERVO_TOPIC, ["cmd_servo"])
    data.ack = to_series(ACKERMANN_TOPIC, ["ack_steer", "ack_speed"])

    if t_start is not None or t_end is not None:
        _crop(data, t_start, t_end)
    return data


def _crop(data: BagData, t_start, t_end):
    lo = -np.inf if t_start is None else t_start
    hi = np.inf if t_end is None else t_end
    for name in ("gt", "vo", "cs", "core", "servo", "imu", "cmd_speed", "cmd_servo", "ack"):
        s: Series = getattr(data, name)
        if len(s) == 0:
            continue
        mask = (s.t >= lo) & (s.t <= hi)
        for attr, val in list(vars(s).items()):
            if isinstance(val, np.ndarray) and val.shape == mask.shape:
                setattr(s, attr, val[mask])
