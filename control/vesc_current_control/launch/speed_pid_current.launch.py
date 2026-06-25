"""속도 PID → 전류 제어 변환 노드 단독 launch.

ackermann_to_vesc_node(속도→eRPM) 대신 띄움. vesc_driver / vesc_to_odom /
simple_mux 등 나머지는 기존 low_level_mac 그대로 사용.

사용:
  # ackermann_to_vesc 끄고 이걸 띄움 (토픽이 같아 동시 실행 금지!)
  ros2 launch vesc_current_control speed_pid_current.launch.py
  # 레이스 튜닝 — 캡/게인/부호/가드를 인자로 바로 조정 (config 편집 불필요):
  ros2 launch vesc_current_control speed_pid_current.launch.py \
       pid_current_max:=60 pid_current_min:=-40 kp:=6 ki:=15 max_abs_speed:=12

주의: ackermann_to_vesc_node 와 동시에 띄우면 /commands/motor/speed 와
/commands/motor/current 가 둘 다 나가 VESC 거동이 충돌합니다. 반드시 택일.

파라미터 우선순위: pid_config(YAML) 로드 후 아래 인자들이 override.
숫자 인자는 정수로 줘도(예: :=55) float 으로 강제 변환됨(ParameterValue).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def farg(name):
    """launch arg 를 float 파라미터로 강제 변환 (정수 입력도 DOUBLE 로)."""
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    cfg = LaunchConfiguration('pid_config')
    return LaunchDescription([
        DeclareLaunchArgument(
            'pid_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('vesc_current_control'), 'config', 'speed_pid.yaml',
            ]),
            description='Speed-PID-to-current config YAML (베이스).'),
        # ── 레이스에서 자주 만지는 값들 (YAML override) ──
        DeclareLaunchArgument('kp', default_value='8.0'),
        DeclareLaunchArgument('ki', default_value='20.0'),
        DeclareLaunchArgument('kd', default_value='0.0'),
        DeclareLaunchArgument('pid_current_max', default_value='90.0',
                              description='PID 출력 전류 상한[A] = 드라이버 캡. 첫 테스트는 낮춰 시작.'),
        DeclareLaunchArgument('pid_current_min', default_value='-55.0',
                              description='PID 출력 전류 하한[A](음수=회생) = 드라이버 캡.'),
        DeclareLaunchArgument('speed_sign', default_value='1.0',
                              description='실측속도 부호(HW 실측 +1.0). 어긋나면 -1.0.'),
        DeclareLaunchArgument('current_sign', default_value='1.0'),
        DeclareLaunchArgument('max_abs_speed', default_value='12.0',
                              description='폭주 가드[m/s]. 레이스 최고속 위로.'),
        DeclareLaunchArgument('speed_to_erpm_gain', default_value='3423.0'),
        Node(
            package='vesc_current_control',
            executable='speed_pid_to_current',
            name='speed_pid_to_current_node',
            output='screen',
            parameters=[cfg, {
                'kp': farg('kp'), 'ki': farg('ki'), 'kd': farg('kd'),
                'current_max': farg('pid_current_max'),
                'current_min': farg('pid_current_min'),
                'speed_sign': farg('speed_sign'),
                'current_sign': farg('current_sign'),
                'max_abs_speed': farg('max_abs_speed'),
                'speed_to_erpm_gain': farg('speed_to_erpm_gain'),
            }],
        ),
    ])
