"""HW 없이 fake 테스트: fake_vesc 플랜트 + speed_pid_to_current 동시 기동.

bench_gui 는 인터랙티브라 따로 실행:
  ros2 launch vesc_current_control fake_test.launch.py     # 이 launch
  ros2 run   vesc_current_control bench_gui                 # 별도 터미널

GUI 에서 SPEED 모드 → step 버튼/슬라이더로 목표속도 주면, fake_vesc 가
폐루프를 돌려 그래프에 속도/전류 응답이 그려진다. (toy plant — 실차 아님)
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vesc_current_control', executable='fake_vesc',
            name='fake_vesc', output='screen',
        ),
        Node(
            package='vesc_current_control', executable='speed_pid_to_current',
            name='speed_pid_to_current_node', output='screen',
            parameters=[{
                'current_max': 40.0,
                'current_min': -20.0,   # 음전류(회생제동) 허용
                'kp': 28.0, 'ki': 55.0, 'kd': 0.0,   # fake plant 기준 튜닝값
                'speed_sign': 1.0,          # 실차/ fake_vesc 규약(전진=+eRPM)
                'max_abs_speed': 8.0,       # sim 발산 안전망
            }],
        ),
    ])
