#!/usr/bin/env python3
"""VESC 전류제어 1차 벤치 테스트 콘솔.

터미널에서 숫자를 입력하면 모터에 명령이 들어가고, 실측 속도/전류가
실시간으로 표시됩니다. 바퀴를 들고(휠 공회전) 또는 안전하게 고정한 상태에서
사용하세요.

명령(입력 후 Enter):
  c <A>     전류 직접 주입 [A]  → /commands/motor/current  (PID 노드 끄고 사용)
  v <m/s>   목표 속도 setpoint  → /ackermann_cmd            (PID 노드 켜고 사용)
  s <deg>   조향각 [deg] (속도모드에서 같이 publish)
  x  또는 빈 Enter   즉시 0 (E-STOP)
  q         종료 (0 출력 후 정리)

권장 절차:
  1) [부호확인] PID 노드 끄고 이 콘솔만 실행 →  c 2   입력.
     바퀴가 "전진" 방향으로 도는지 확인.  뒤로 돌면 speed_pid.yaml 의
     current_sign 을 -1.0 으로.  (그리고 실측속도 부호도 같이 확인)
  2) [폐루프] PID 노드 켜고(speed_pid_current.launch.py) →  v 1.0  입력.
     실측속도가 1.0 으로 수렴하고 지령전류가 안정되는지 확인.
  3) 끝나면  x  로 0, 그다음  q.

안전:
  - 50Hz 로 계속 publish → 콘솔이 죽으면 PID 노드의 cmd_timeout 이 전류를 0 으로.
  - --max-current / --max-speed 로 입력 상한 클램프 (기본 전류 10A, 속도 2 m/s).
"""
import argparse
import sys
import threading

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped

GAIN_DEFAULT = 3423.0
SPEED_SIGN = -1.0  # 실측: m/s = SPEED_SIGN*state.speed/gain  (vesc_to_odom 과 동일)


class BenchConsole(Node):
    def __init__(self, gain, max_cur, max_spd):
        super().__init__('vesc_bench_console')
        self.gain = gain
        self.max_cur = max_cur
        self.max_spd = max_spd

        self.mode = 'idle'        # 'idle' | 'current' | 'speed'
        self.cur_cmd = 0.0        # 직접 전류 [A]
        self.spd_cmd = 0.0        # 목표 속도 [m/s]
        self.steer_cmd = 0.0      # 조향 [rad]

        # 텔레메트리
        self.meas_speed = 0.0
        self.meas_current = 0.0
        self.cmd_current_seen = 0.0

        self.cur_pub = self.create_publisher(Float64, 'commands/motor/current', 10)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        self.create_subscription(VescStateStamped, 'sensors/core', self.state_cb, 10)
        self.create_subscription(Float64, 'commands/motor/current', self.cur_seen_cb, 10)

        self.create_timer(0.02, self.pub_loop)     # 50 Hz 명령 publish
        self.create_timer(0.2, self.status_loop)    # 5 Hz 상태표시

    # ── telemetry ──
    def state_cb(self, m):
        self.meas_speed = SPEED_SIGN * m.state.speed / self.gain
        self.meas_current = m.state.current_motor

    def cur_seen_cb(self, m):
        self.cmd_current_seen = m.data

    # ── publish ──
    def pub_loop(self):
        if self.mode == 'current':
            self.cur_pub.publish(Float64(data=self.cur_cmd))
        elif self.mode == 'speed':
            c = AckermannDriveStamped()
            c.drive.speed = self.spd_cmd
            c.drive.steering_angle = self.steer_cmd
            self.cmd_pub.publish(c)
        # idle: 아무것도 안 보냄 (PID 노드 cmd_timeout 이 0 으로 처리)

    def status_loop(self):
        if self.mode == 'current':
            tgt = f'I_cmd={self.cur_cmd:+.1f}A'
        elif self.mode == 'speed':
            tgt = f'v_cmd={self.spd_cmd:+.2f}m/s steer={self.steer_cmd:+.2f}rad'
        else:
            tgt = 'IDLE'
        line = (f'[{self.mode:>7}] {tgt:<34} | meas: '
                f'v={self.meas_speed:+.2f}m/s  I_cmd_bus={self.cmd_current_seen:+.1f}A  '
                f'I_motor={self.meas_current:+.1f}A')
        sys.stdout.write('\r' + line + '   ')
        sys.stdout.flush()

    # ── 입력 처리 (별도 스레드에서 호출) ──
    def process_cmd(self, text):
        t = text.strip()
        if t == '' or t == 'x':
            self.mode = 'idle'; self.cur_cmd = 0.0; self.spd_cmd = 0.0
            self.cur_pub.publish(Float64(data=0.0))
            print('\n  >> E-STOP / idle (0)')
            return True
        if t == 'q':
            return False
        parts = t.split()
        try:
            if parts[0] == 'c':
                v = max(-self.max_cur, min(self.max_cur, float(parts[1])))
                self.mode = 'current'; self.cur_cmd = v
                print(f'\n  >> 전류 직접 {v:+.1f}A (PID 노드 꺼져 있어야 함)')
            elif parts[0] == 'v':
                v = max(-self.max_spd, min(self.max_spd, float(parts[1])))
                self.mode = 'speed'; self.spd_cmd = v
                print(f'\n  >> 속도 setpoint {v:+.2f}m/s (PID 노드 켜져 있어야 함)')
            elif parts[0] == 's':
                import math
                self.steer_cmd = float(parts[1]) * math.pi / 180.0
                print(f'\n  >> 조향 {parts[1]}deg')
            else:
                print(f'\n  ?? 모르는 명령: {t}')
        except (IndexError, ValueError):
            print(f'\n  ?? 형식 오류: "{t}"  (예: c 2 / v 1.0 / s 10 / x / q)')
        return True


def input_thread(node, stop):
    print(__doc__)
    print('명령 대기 (c <A> / v <m/s> / s <deg> / x / q):')
    while not stop.is_set():
        try:
            text = input()
        except EOFError:
            break
        if not node.process_cmd(text):
            break
    stop.set()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gain', type=float, default=GAIN_DEFAULT,
                    help='speed_to_erpm_gain (실측 환산용, 기본 3423)')
    ap.add_argument('--max-current', type=float, default=10.0,
                    help='직접 전류 입력 상한 [A] (기본 10)')
    ap.add_argument('--max-speed', type=float, default=10.0,
                    help='속도 setpoint 입력 상한 [m/s] (기본 2)')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = BenchConsole(args.gain, args.max_current, args.max_speed)
    stop = threading.Event()
    th = threading.Thread(target=input_thread, args=(node, stop), daemon=True)
    th.start()
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.cur_pub.publish(Float64(data=0.0))
        rclpy.spin_once(node, timeout_sec=0.1)
        print('\n정지(0) 후 종료.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
