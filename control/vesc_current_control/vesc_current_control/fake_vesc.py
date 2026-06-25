#!/usr/bin/env python3
"""가짜 VESC 플랜트 (HW 없이 제어 로직 테스트용).

/commands/motor/current 를 받아 단순 1차 종방향 모델로 속도를 적분하고,
/sensors/core (VescStateStamped) 로 eRPM·모터전류를 publish 한다. 이걸로
speed_pid_to_current + bench_gui 의 폐루프를 HW 없이 돌려볼 수 있다.

⚠️ 실제 모터/ESC/차량 동역학이 아니라 toy plant 다. 토픽 배선·PID 수식·
회생(음전류) 부호·GUI 검증용이며, 게인/거동은 실차와 다르다. 실차 튜닝은
벤치(휠 공중)에서 다시 해야 한다.

plant: dv/dt = k_acc * current - drag * v
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped


class FakeVesc(Node):
    def __init__(self):
        super().__init__('fake_vesc')
        self.gain = self.declare_parameter('speed_to_erpm_gain', 3423.0).value
        self.k_acc = self.declare_parameter('k_acc', 0.15).value   # 전류[A] → 가속 계수
        self.drag = self.declare_parameter('drag', 0.5).value      # 속도 비례 마찰
        self.dt = self.declare_parameter('dt', 0.02).value
        self.cur = 0.0
        self.spd = 0.0
        self.create_subscription(Float64, 'commands/motor/current',
                                 lambda m: setattr(self, 'cur', m.data), 10)
        self.pub = self.create_publisher(VescStateStamped, 'sensors/core', 10)
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'fake_vesc plant up (toy model, NOT real dynamics): '
            f'k_acc={self.k_acc} drag={self.drag} gain={self.gain}')

    def tick(self):
        self.spd += (self.k_acc * self.cur - self.drag * self.spd) * self.dt
        m = VescStateStamped()
        # m/s → eRPM. HW 실측(2026-06-16) 규약: 전진 시 state.speed 양수 → +gain
        # (노드 speed_sign=+1 과 일관. 부호 뒤집으면 fake 폐루프가 발산함)
        m.state.speed = self.gain * self.spd
        m.state.current_motor = self.cur
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FakeVesc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
