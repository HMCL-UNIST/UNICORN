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
  - feedforward: 목표속도 기반. vel_ff_gain·static_ff(항력/정지마찰 상쇄)
    + accel_ff_gain(목표속도 미분, opt-in).
    ⚠️ 무결성 감사(2026-08-06, docs/USAGE.md) 결과 아래 두 가지가 확인됨:
      · accel_ff_gain 은 **지금 목표의 미분** = 인과적 → P 항과 시점 동일, preview 정보 0.
        실측상 오히려 느려짐(0.72→1.10s). **켜지 말 것.** 진짜 preview 는
        /local_waypoints(vx_mps, ax_mps2) 또는 MPCC 예측 호라이즌에서 와야 함.
      · 이 차(3.5kg)는 항력 @5m/s = 0.33A vs 관성 @5m/s² = 16.8A → **관성이 51배**.
        vel_ff_gain(항력항) 튜닝은 실익이 거의 없음. 재설계 방향은
        I_ff = (m*a_preview + dragcoeff*v^2)/1.0414[N/A] 모델기반(튜닝 불필요).
    FF 의 검증된 이점은 "가속 한계 증가"가 아니라 "같은 노이즈 예산에서 추종 2배"이며,
    쓰려면 ki 를 낮춰야 함(FF+큰 ki = 이중보상 → 오버슈트 25%).
  - anti-windup: 적분항 clamp + conditional integration(포화 심화 방향이면 적분 동결).
    FF 가 포화시켜도 적분을 오염 안 시킴(back-calc 대비 FF+PI 안정).
  - 안전: cmd_timeout 안에 ackermann_cmd 가 없으면 전류 0 (또는 brake) 출력.
  - current_sign: VESC 전류부호↔진행방향은 결선/펌웨어 의존 → 벤치 확인 후 플립용.
"""
import math

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
        # kp 는 "가속 구간 속도오차 → 전류"의 직접 지렛대.
        # 캡을 채우려면 대략 kp ≈ current_max / (가속구간 예상오차[m/s]).
        # 기본 12 는 오차 ~3m/s 에서 ~36A → 벤치서 캡/노이즈 보고 상향.
        self.kp = self.declare_parameter('kp', 12.0).value
        self.ki = self.declare_parameter('ki', 20.0).value
        self.kd = self.declare_parameter('kd', 0.0).value

        # ── Feedforward (목표속도 기반, planner 가속도 불필요) ──
        # ① 속도 FF: 그 속도 유지에 필요한 전류를 미리 명령 → 적분이 항력에
        #    매달릴 필요 없음(정상상태 lag↓). I_ff = vel_ff_gain*v_target + static_ff.
        #    vel_ff_gain[A/(m/s)], static_ff[A]=정지마찰 breakaway. 벤치서 유지전류↔속도로 캘리브.
        self.vel_ff_gain = self.declare_parameter('vel_ff_gain', 0.0).value
        self.static_ff = self.declare_parameter('static_ff', 0.0).value
        # ③ 가속 FF(opt-in, 기본 0=off): 목표속도의 (필터된) 변화율 → 전류.
        #    I_ff += accel_ff_gain * d(v_target)/dt. 목표 스텝 스파이크 방지용 LPF(accel_ff_tau[s]).
        #    accel_ff_gain[A/(m/s^2)]. 스텝응답 전류↔가속 기울기의 역수로 캘리브.
        self.accel_ff_gain = self.declare_parameter('accel_ff_gain', 0.0).value
        self.accel_ff_tau = self.declare_parameter('accel_ff_tau', 0.1).value

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
        self.last_target = 0.0          # 가속 FF 용 목표속도 미분
        self.tgt_dot = 0.0              # 필터된 d(v_target)/dt
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
                      'stop_brake_current',
                      'vel_ff_gain', 'static_ff', 'accel_ff_gain', 'accel_ff_tau'}
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f'speed_pid_to_current up: kp={self.kp} ki={self.ki} kd={self.kd} '
            f'I[{self.current_min},{self.current_max}]A gain={self.gain} '
            f'sign(spd={self.speed_sign},cur={self.current_sign}) rate={self.rate}Hz '
            f'stop_brake={self.stop_brake_current}A '
            f'ff(vel={self.vel_ff_gain},static={self.static_ff},accel={self.accel_ff_gain})')

    # ── 런타임 파라미터 변경 → 내부 상태 즉시 반영 ──
    def _on_set_params(self, params):
        attr = {'kp': 'kp', 'ki': 'ki', 'kd': 'kd',
                'current_max': 'current_max', 'current_min': 'current_min',
                'current_sign': 'current_sign', 'integral_max': 'integral_max',
                'speed_sign': 'speed_sign', 'max_abs_speed': 'max_abs_speed',
                'stop_brake_current': 'stop_brake_current',
                'vel_ff_gain': 'vel_ff_gain', 'static_ff': 'static_ff',
                'accel_ff_gain': 'accel_ff_gain', 'accel_ff_tau': 'accel_ff_tau'}
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
            self.tgt_dot = 0.0
            self.last_target = self.target_speed
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

        # D(on measurement, setpoint kick 방지)
        d = -self.kd * (self.meas_speed - self.last_meas) / self.dt
        self.last_meas = self.meas_speed

        # ── Feedforward (목표속도 기반) ──
        # ① 속도 FF: 그 속도 유지 전류 미리. static_ff 는 |목표|≥stop_speed 일 때만
        #    (정지/감속 목표 0 에 전진 전류가 새는 것 방지).
        ff = self.vel_ff_gain * self.target_speed
        if abs(self.target_speed) >= self.stop_speed:
            ff += math.copysign(self.static_ff, self.target_speed)
        # ③ 가속 FF(opt-in): 목표속도 변화율(LPF) → 전류.
        if self.accel_ff_gain != 0.0:
            raw_dot = (self.target_speed - self.last_target) / self.dt
            alpha = self.dt / (max(self.accel_ff_tau, 1e-6) + self.dt)
            self.tgt_dot += alpha * (raw_dot - self.tgt_dot)
            ff += self.accel_ff_gain * self.tgt_dot
        self.last_target = self.target_speed

        # 출력(직전 적분값 사용) = current_sign * (P + I + D + FF)
        p = self.kp * error
        i = self.ki * self.integral
        raw = self.current_sign * (p + i + d + ff)
        cur = clip(raw, self.current_min, self.current_max)

        # anti-windup: conditional integration.
        #  출력이 포화 중이고 적분을 더하면 포화가 깊어지는 방향이면 적분 동결.
        #  (back-calc 와 달리 FF 가 포화시켜도 적분을 음으로 덤프하지 않음 → FF+PI 안정)
        push = self.current_sign * error   # error 가 출력을 미는 방향
        if not ((raw > self.current_max and push > 0) or
                (raw < self.current_min and push < 0)):
            self.integral += error * self.dt
            self.integral = clip(self.integral, -self.integral_max / max(self.ki, 1e-9),
                                 self.integral_max / max(self.ki, 1e-9))

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
