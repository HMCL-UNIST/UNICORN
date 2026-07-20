"""실차 벤치 (휠 공중) — 진짜 VESC 로 전류제어 테스트.

vesc_driver(시리얼) + speed_pid_to_current 만 띄운다. ackermann_to_vesc(속도→eRPM,
충돌) / fake_vesc 는 안 띄움. 조작은 bench_gui 로.

  터미널1:  ros2 launch vesc_current_control bench_real.launch.py
  터미널2:  ros2 run   vesc_current_control bench_gui

⚠️ 안전 / 미확인 (실차 처음 전원인가 전 반드시 확인):
  - **바퀴를 들고(휠 공중)** 시작. CURRENT 모드 +2A 로 회전 방향부터 확인.
  - 게인/전류 기본값은 일부러 보수적(작게). GUI 로 올려가며 튜닝.
  - 회생제동(음전류)을 실제 모터까지 보내려면 vesc_driver 의 current_min 이
    음수여야 함 → 여기서 driver_current_min 인자로 override (기본 vesc_config 는 0).
  - 이 launch 는 HW 없이 검증 못함 → 차에서 처음 돌릴 때 토픽/거동 확인할 것.

인자(override 예): ros2 launch ... bench_real.launch.py kp:=3.0 pid_current_max:=10.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def farg(name):
    """launch arg 를 float 파라미터로 강제 변환 (정수 입력도 DOUBLE 로 — 타입거부 방지)."""
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    vesc_config = LaunchConfiguration('vesc_config')
    drv_imin = farg('driver_current_min')
    drv_imax = farg('driver_current_max')
    pid_imax = farg('pid_current_max')
    pid_imin = farg('pid_current_min')
    gain = farg('speed_to_erpm_gain')
    kp = farg('kp')
    ki = farg('ki')
    kd = farg('kd')
    speed_sign = farg('speed_sign')
    current_sign = farg('current_sign')
    max_abs_speed = farg('max_abs_speed')

    return LaunchDescription([
        # vesc_config: 시리얼 포트/servo/gain 등 vesc_driver 파라미터.
        # 기본은 이 패키지 동봉 사본(자립) — 실차 스택 것을 쓰려면 vesc_config:=<경로>.
        DeclareLaunchArgument(
            'vesc_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('vesc_current_control'), 'config', 'vesc_config.yaml']),
            description='vesc_driver 용 vesc_config.yaml 경로 (포트/servo/limit). '
                        '기본=패키지 동봉본. 실차 스택 것 쓰려면 인자로 지정.'),
        # 드라이버 전류 한계 — 회생 위해 min 음수. 레이스용 상향(펌웨어 모터±100/회생-60 안쪽).
        DeclareLaunchArgument('driver_current_min', default_value='-55.0',
                              description='vesc_driver current_min [A]. 음수=회생(펌웨어 -60 안쪽).'),
        DeclareLaunchArgument('driver_current_max', default_value='90.0',
                              description='vesc_driver current_max [A] (펌웨어 100 안쪽).'),
        # PID 노드 출력 전류 한계 = 드라이버 캡과 동일(90/-55) → PID 가 드라이버 풀레인지 사용.
        # ★첫 실차 테스트는 작게: pid_current_max:=20 처럼 낮춰 시작 후 올릴 것.
        DeclareLaunchArgument('pid_current_max', default_value='90.0'),
        DeclareLaunchArgument('pid_current_min', default_value='-55.0'),
        DeclareLaunchArgument('speed_to_erpm_gain', default_value='3423.0'),
        # 보수적 시작 게인 (fake plant 의 28/55 와 무관 — 실차는 낮게 시작해 올림)
        DeclareLaunchArgument('kp', default_value='3.0'),
        DeclareLaunchArgument('ki', default_value='6.0'),
        DeclareLaunchArgument('kd', default_value='0.0'),
        # ★부호 — 벤치에서 CURRENT 모드로 방향/피드백 확인 후 맞출 것 (어긋나면 폭주!)
        DeclareLaunchArgument('speed_sign', default_value='1.0',
                              description='실측속도 부호. 앞으로 도는데 meas 가 음수면 +1.0 으로.'),
        DeclareLaunchArgument('current_sign', default_value='1.0',
                              description='전류부호↔진행방향. +전류가 후진이면 -1.0.'),
        # ★폭주 안전가드: |실측속도| 가 이 값 초과면 전류 0 (부호 어긋남 시 휠 폭주 차단)
        DeclareLaunchArgument('max_abs_speed', default_value='12.0',
                              description='실측속도 절대값 한계[m/s]. 초과 시 전류 0(latch). '
                                          '레이스 최고속 위로 둘 것(부호어긋남/발산만 차단). 0=비활성.'),

        # ── 진짜 VESC 드라이버 (시리얼). current_min/max override 로 회생 허용 ──
        Node(
            package='vesc_driver', executable='vesc_driver_node',
            name='vesc_driver_node', output='screen',
            parameters=[vesc_config, {
                'current_min': drv_imin,
                'current_max': drv_imax,
            }],
        ),
        # ── 우리 속도 PID → 전류 노드 ──
        Node(
            package='vesc_current_control', executable='speed_pid_to_current',
            name='speed_pid_to_current_node', output='screen',
            parameters=[{
                'kp': kp, 'ki': ki, 'kd': kd,
                'current_max': pid_imax, 'current_min': pid_imin,
                'speed_to_erpm_gain': gain,
                'speed_sign': speed_sign,
                'current_sign': current_sign,
                'max_abs_speed': max_abs_speed,   # 폭주 가드
                # servo 변환 (vesc_config 와 동일값)
                'steering_angle_to_servo_gain': 0.5135,
                'steering_angle_to_servo_offset': 0.445,
                'servo_max': 0.85, 'servo_min': 0.15,
            }],
        ),
    ])
