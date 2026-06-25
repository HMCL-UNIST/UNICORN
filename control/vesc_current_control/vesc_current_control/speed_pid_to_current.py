#!/usr/bin/env python3
"""speed_pid_to_current_node.

기존 ackermann_to_vesc (속도→eRPM, VESC 내부 RPM PID 사용) 를 대체하는
ROS-단 속도 PID → 전류 변환 노드.

  /ackermann_cmd (.drive.speed)            ─┐
                                            ├─► PID ─► /commands/motor/current (A)
  /sensors/core (state.speed=eRPM)         ─┘
  /ackermann_cmd (.drive.steering_angle)   ───► servo ─► /commands/servo/position

설계 메모
  - 실측속도[m/s] = (speed_sign*state.speed - speed_to_erpm_offset) / speed_to_erpm_gain
    speed_sign 기본 -1.0 → vesc_to_odom.cpp 의 `-state->state.speed` 와 부호 일치.
  - 감속: current_min 을 음수로 두어 음전류(회생제동) 허용 (사용자 결정).
  - PID 는 control_rate[Hz] 타이머로 고정 주기 실행 (cmd/feedback 콜백 rate 와 디커플).
  - anti-windup: 적분항 clamp + 출력 포화 시 back-calculation.
  - 안전: cmd_timeout 안에 ackermann_cmd 가 없으면 전류 0 (또는 brake) 출력.
  - current_sign: VESC 전류부호↔진행방향은 결선/펌웨어 의존 → 벤치 확인 후 플립용.
"""
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from rcl_interfaces.msg import SetParametersResult


def clip(x, lo, hi):
    return max(lo, min(hi, x))


class SpeedPidToCurrent(Node):
    def __init__(self):
        super().__init__('speed_pid_to_current_node')

        # ── 변환/부호 ──
        self.gain = self.declare_parameter('speed_to_erpm_gain', 3423.0).value
        self.offset = self.declare_parameter('speed_to_erpm_offset', 0.0).value
        self.speed_sign = self.declare_parameter('speed_sign', 1.0).value  # HW실측 2026-06-16: +current=전진 시 state.speed 양수

        # ── PID gains ──
        self.kp = self.declare_parameter('kp', 8.0).value
        self.ki = self.declare_parameter('ki', 20.0).value
        self.kd = self.declare_parameter('kd', 0.0).value

        # ── 전류 출력 한계 [A] (current_min 음수 = 회생제동) ──
        # vesc_driver 캡(bench_real 기본 90/-55)과 동일하게 맞춤 — PID 가 드라이버
        # 풀레인지 사용. 펌웨어 모터±100/회생-60 안쪽. 첫 테스트는 launch arg 로 낮춰 시작.
        self.current_max = self.declare_parameter('current_max', 90.0).value
        self.current_min = self.declare_parameter('current_min', -55.0).value
        self.current_sign = self.declare_parameter('current_sign', 1.0).value

        # ── 적분 anti-windup 한계 (전류 단위) ── 캡 상향에 맞춰 50 (고속 유지전류 확보)
        self.integral_max = self.declare_parameter('integral_max', 50.0).value
        self.enabled = self.declare_parameter('enabled', True).value

        # ── 제어 주기 / 안전 ──
        self.rate = self.declare_parameter('control_rate', 100.0).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.3).value
        # 목표가 이 속도[m/s] 미만이고 실측도 미만이면 PID 정지 후 0 전류
        self.stop_speed = self.declare_parameter('stop_speed_threshold', 0.05).value
        # ★폭주 안전가드: 실측속도 |meas| 가 이 값[m/s] 초과면 전류 0 (0=비활성).
        #   부호 어긋남 등으로 PID 가 전류를 max 로 밀어 휠이 폭주하는 것 차단.
        self.max_abs_speed = self.declare_parameter('max_abs_speed', 0.0).value
        # ★정지 제동: 목표 0인데 아직 구르면 이 brake 전류[A] 로 강제동 (0=비활성=회생만).
        self.stop_brake_current = self.declare_parameter('stop_brake_current', 0.0).value

        # ── 조향(servo) — 기존 ackermann_to_vesc 와 동일 변환 흡수 ──
        self.steer_gain = self.declare_parameter('steering_angle_to_servo_gain', 0.5135).value
        self.steer_offset = self.declare_parameter('steering_angle_to_servo_offset', 0.445).value
        self.servo_max = self.declare_parameter('servo_max', 0.85).value
        self.servo_min = self.declare_parameter('servo_min', 0.15).value

        # ── state ──
        self.target_speed = 0.0
        self.target_steer = 0.0
        self.meas_speed = 0.0
        self.integral = 0.0
        self.last_meas = 0.0
        self._runaway_latched = False   # 폭주 가드 latch
        self.last_cmd_time = None
        self.have_feedback = False

        # ── I/O ──
        self.cur_pub = self.create_publisher(Float64, 'commands/motor/current', 10)
        self.brake_pub = self.create_publisher(Float64, 'commands/motor/brake', 10)
        self.servo_pub = self.create_publisher(Float64, 'commands/servo/position', 10)
        self.create_subscription(AckermannDriveStamped, 'ackermann_cmd',
                                 self.cmd_cb, 10)
        self.create_subscription(VescStateStamped, 'sensors/core',
                                 self.state_cb, 10)

        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self.control_loop)

        # ── 런타임 파라미터 변경 콜백 (GUI 슬라이더 / ros2 param set 으로 라이브 튜닝) ──
        self._live = {'kp', 'ki', 'kd', 'current_max', 'current_min',
                      'current_sign', 'integral_max', 'speed_sign', 'max_abs_speed',
                      'stop_brake_current'}
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f'speed_pid_to_current up: kp={self.kp} ki={self.ki} kd={self.kd} '
            f'I[{self.current_min},{self.current_max}]A gain={self.gain} '
            f'sign(spd={self.speed_sign},cur={self.current_sign}) rate={self.rate}Hz '
            f'stop_brake={self.stop_brake_current}A')

    # ── 런타임 파라미터 변경 → 내부 상태 즉시 반영 ──
    def _on_set_params(self, params):
        attr = {'kp': 'kp', 'ki': 'ki', 'kd': 'kd',
                'current_max': 'current_max', 'current_min': 'current_min',
                'current_sign': 'current_sign', 'integral_max': 'integral_max',
                'speed_sign': 'speed_sign', 'max_abs_speed': 'max_abs_speed',
                'stop_brake_current': 'stop_brake_current'}
        for p in params:
            if p.name == 'enabled':
                new_en = bool(p.value)
                if not new_en:
                    self.integral = 0.0
                    self.cur_pub.publish(Float64(data=0.0))
                self.enabled = new_en
            elif p.name in self._live:
                setattr(self, attr[p.name], float(p.value))
        return SetParametersResult(successful=True)

    # ── callbacks ──
    def cmd_cb(self, msg: AckermannDriveStamped):
        self.target_speed = msg.drive.speed
        self.target_steer = msg.drive.steering_angle
        self.last_cmd_time = self.get_clock().now()
        # 조향은 cmd 들어올 때마다 즉시 반영 (PID 와 무관)
        servo = clip(self.steer_gain * self.target_steer + self.steer_offset,
                     self.servo_min, self.servo_max)
        self.servo_pub.publish(Float64(data=servo))

    def state_cb(self, msg: VescStateStamped):
        self.meas_speed = (self.speed_sign * msg.state.speed - self.offset) / self.gain
        self.have_feedback = True

    # ── 100Hz PID ──
    def control_loop(self):
        now = self.get_clock().now()

        # CURRENT 모드 등에서 GUI 가 PID 를 끄면(enabled=False) 발행 중단(명령 충돌 방지)
        if not self.enabled:
            return

        # 안전: cmd timeout → 정지
        timed_out = (self.last_cmd_time is None or
                     (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout)
        if timed_out or not self.have_feedback:
            self.integral = 0.0
            self.cur_pub.publish(Float64(data=0.0))
            return

        # ★폭주 안전가드 (latch): |실측속도| 가 한계 초과 → 전류 0 으로 래치.
        #   목표를 0(정지/E-STOP)으로 내릴 때까지 0 유지 → chatter 없이 안전 정지.
        if self._runaway_latched:
            self.integral = 0.0
            self.cur_pub.publish(Float64(data=0.0))
            if abs(self.target_speed) < self.stop_speed:
                self._runaway_latched = False
                self.get_logger().warn('RUNAWAY GUARD 해제 (목표 0). speed_sign 확인 후 재시도.')
            return
        if self.max_abs_speed > 0.0 and abs(self.meas_speed) > self.max_abs_speed:
            self._runaway_latched = True
            self.integral = 0.0
            self.cur_pub.publish(Float64(data=0.0))
            self.get_logger().warn(
                f'RUNAWAY GUARD 발동: |meas|={abs(self.meas_speed):.1f} > '
                f'{self.max_abs_speed:.1f} m/s → 전류 0 LATCH. speed_sign 어긋남 의심! '
                f'CURRENT 모드로 방향 확인 후 speed_sign 뒤집을 것. (목표 0 으로 내리면 해제)')
            return

        # ★정지 제동(opt-in): 목표 0인데 아직 구르면 brake 로 강제동 (회생보다 강하고 저속서도 잡힘)
        if (self.stop_brake_current > 0.0 and abs(self.target_speed) < self.stop_speed
                and abs(self.meas_speed) >= self.stop_speed):
            self.integral = 0.0
            self.last_meas = self.meas_speed
            self.brake_pub.publish(Float64(data=self.stop_brake_current))
            return
        # 완전 정지 의도 → PID 끄고 0 전류 (브레이크는 별도 정책)
        if abs(self.target_speed) < self.stop_speed and abs(self.meas_speed) < self.stop_speed:
            self.integral = 0.0
            self.last_meas = self.meas_speed
            self.cur_pub.publish(Float64(data=0.0))
            return

        error = self.target_speed - self.meas_speed

        # P + I + D(on measurement, setpoint kick 방지)
        p = self.kp * error
        self.integral += error * self.dt
        self.integral = clip(self.integral, -self.integral_max / max(self.ki, 1e-9),
                             self.integral_max / max(self.ki, 1e-9))
        i = self.ki * self.integral
        d = -self.kd * (self.meas_speed - self.last_meas) / self.dt
        self.last_meas = self.meas_speed

        raw = self.current_sign * (p + i + d)
        cur = clip(raw, self.current_min, self.current_max)

        # back-calculation anti-windup: 포화분만큼 적분 되감기
        if self.ki > 1e-9 and raw != cur:
            self.integral += (cur - raw) / self.ki * self.current_sign

        self.cur_pub.publish(Float64(data=cur))


def main(args=None):
    rclpy.init(args=args)
    node = SpeedPidToCurrent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cur_pub.publish(Float64(data=0.0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
