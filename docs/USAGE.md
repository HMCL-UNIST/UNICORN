# vesc_current_control 사용/통합 가이드

ackermann_to_vesc(속도→eRPM, VESC 내부 RPM PID) 대신, **목표속도를 PID로 전류[A]로 변환**해
`/commands/motor/current` 로 발행하는 제어 노드 + 벤치 툴.

## 패키지 구성
| 실행파일 | 용도 |
|---|---|
| `speed_pid_to_current` | 속도 PID → 전류 (핵심 노드). `/ackermann_cmd` 구독 → `/commands/motor/current` 발행 |
| `bench_gui` | GUI 벤치 (슬라이더로 전류/속도, 실시간 그래프) — Qt(cocoa) |
| `bench_console` | 텍스트 벤치 (SSH로 바로, GUI 불필요) |
| `fake_vesc` | 가짜 VESC (HW 없이 폐루프 검증) |

| 런치 | 띄우는 것 |
|---|---|
| `bench_real.launch.py` | vesc_driver(시리얼) + speed_pid_to_current (실차 벤치) |
| `speed_pid_current.launch.py` | speed_pid_to_current 단독 (기존 low_level에 얹어 ackermann_to_vesc 대체) |
| `fake_test.launch.py` | fake_vesc + speed_pid (HW 없이) |

## 실행 방법

### 1) 실차 벤치 (휠 공중) — driver + PID
```bash
# 환경 진입 (Mac unicorn / IFAC ros_env 등 워크스페이스 source 후)
ros2 launch vesc_current_control bench_real.launch.py   # T1: driver + PID
ros2 run   vesc_current_control bench_gui                # T2: GUI
#   또는 GUI 대신 SSH 텍스트:
ros2 run   vesc_current_control bench_console
```

### 2) 기존 스택에 PID만 얹기 (ackermann_to_vesc 대체)
```bash
ros2 launch vesc_current_control speed_pid_current.launch.py
# ⚠️ ackermann_to_vesc 와 동시 실행 금지 — 둘 다 /commands/motor/* 로 나가 충돌. 택일.
```

### 3) HW 없이 fake 시뮬
```bash
ros2 launch vesc_current_control fake_test.launch.py
```

### 4) 노드만 직접 (params 필수)
```bash
ros2 run vesc_current_control speed_pid_to_current --ros-args \
  -r __node:=speed_pid_to_current_node \
  --params-file <pkg>/config/speed_pid.yaml
```

## 파라미터 (config/speed_pid.yaml)
| 파라미터 | 의미 | 실차 확정값 |
|---|---|---|
| `speed_to_erpm_gain` | eRPM↔m/s 환산 (m/s = sign·state.speed/gain) | **3423.0** |
| `speed_sign` | 피드백 부호 | **1.0** (HW실측: +current=전진 시 state.speed 양수) |
| `current_sign` | 전류↔진행방향 부호 | **1.0** (+current=전진) |
| `kp/ki/kd` | PID 게인 | 8/20/0 (시작값; 가속 강화 시 kp↑+kd로 오버슈트 감쇠) |
| `current_max/min` | PID 출력 전류한계 [A] | 90 / -55 (드라이버·펌웨어 안쪽) |
| `max_abs_speed` | 폭주 안전가드 — |meas| 초과 시 전류 0 LATCH (0=비활성) | latch |
| `enabled` | False면 발행 중단 (CURRENT 직접제어 시 GUI가 자동 OFF) | true |
| `stop_brake_current` | **정지제동** — 목표 0인데 구르면 brake[A] (0=비활성, 회생만) | opt-in |
| `cmd_timeout` | ackermann_cmd 끊기면 전류 0 [s] | 0.3 |

## VESC 한계 (vesc_config.yaml / 펌웨어 mcconf)
- 드라이버 `current_max/min`: bench_real 에서 override (회생 위해 min 음수). fallback은 vesc_config.
- 펌웨어 한계: `l_current_max=100`(모터 가속), `l_current_min=-100`, `l_in_current_min=-60`(배터리 회생), `l_abs_current_max=150`.
- **회생 부족 시**: 드라이버 current_min을 음수로 + PID current_min. 단 배터리 -60 한계.
- **저속/정지 제동 강화**: `stop_brake_current`(brake 명령, 배터리 회생한계 무관, 저속서도 강함).
- **가속 강화**: GUI accel max(=PID current_max)를 100까지 ↑. 그 이상은 펌웨어 l_current_max 상향(발열주의).

## 제어 메커니즘 비교
| | 회생(음전류) | brake 명령 |
|---|---|---|
| 한계 | 배터리 -60A | 모터 abs 150A |
| 저속 제동 | 약함(역기전력↓) | 강함 |
| 에너지 | 배터리 회수 | 모터 소모 |

## 통합 (unicorn-racing-stack)
- 팀 레포에 `control/vesc_current_control` 로 추가 PR: HMCL-UNIST/unicorn-racing-stack#7
- 의존: `vesc_msgs`(sensor/vesc), `ackermann_msgs`, `rclpy`, `python_qt_binding`
- ⚠️ 유니콘 스택 vesc는 **`/vesc/` 네임스페이스** — PP 통합 시 remap 필요
  (예: `-r commands/motor/current:=/vesc/commands/motor/current`, `-r sensors/core:=/vesc/sensors/core`)

## 플랫폼 메모
- Mac(RoboStack conda): `source install/setup.zsh`. 시리얼 포트 `/dev/cu.usbmodem*` (tty. 아님 → Resource busy).
- Linux: `source install/setup.bash`. 시리얼 포트 보통 `/dev/ttyACM0`.
- vesc_driver 직접 실행 시 `--ros-args -r __node:=vesc_driver_node --params-file ...` (params 키가 node명).

## 검증 (실차 VESC fw6.5, 휠 공중)
- RAW: +3A→전진, state.speed 양수 → speed_sign/current_sign=1.0 확정
- 폐루프 SPEED 1.0/1.5 m/s → setpoint 수렴, 폭주 없음
- stop_brake 로 정지 제동 강화 확인
