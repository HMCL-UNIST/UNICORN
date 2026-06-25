# vesc_current_control

VESC 속도 PID → 전류 변환 노드 + 벤치 테스트 툴 (ROS2 Jazzy).

기존 `ackermann_to_vesc`(속도→eRPM, VESC 내부 RPM PID) 대신, ROS 단에서
목표 속도를 PID로 받아 **전류[A]** 로 변환해 `/commands/motor/current` 로 보낸다.

## 맥북 fake 테스트 (HW 없이 — 빠른 시작)

```bash
# 1) ROS2 워크스페이스 src 안에 클론
cd ~/<your_ws>/src
git clone https://github.com/heedo7425/vesc_current_control.git

# 2) 빌드
cd ~/<your_ws>
colcon build --packages-select vesc_current_control --symlink-install
source install/setup.bash

# 3) DDS 격리 (각 터미널에서)
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# 4) 실행 — 터미널 2개
ros2 launch vesc_current_control fake_test.launch.py   # 터미널1: fake plant + PID 노드
ros2 run   vesc_current_control bench_gui              # 터미널2: GUI
```

GUI 에서 `SPEED` 모드 선택 → step 버튼(0/2.5/5/7.5/10) 또는 슬라이더로 목표속도를
주면, 하단 그래프에 **목표·실측 속도 + 전류** 응답이 실시간으로 그려진다.
kp/ki/kd 슬라이더로 라이브 튜닝, `E-STOP` 으로 즉시 0.

> ⚠️ `fake_vesc` 는 toy plant(실제 모터/차량 아님). 로직·GUI·회생부호 확인용이며,
> PID 게인은 실차(벤치)에서 다시 잡아야 한다.

## 실차 벤치 (휠 공중 — 진짜 VESC)

```bash
# vesc_driver(시리얼) + speed_pid_to_current 동시 기동. ackermann_to_vesc 안 띄움.
ros2 launch vesc_current_control bench_real.launch.py     # 터미널1
ros2 run   vesc_current_control bench_gui                 # 터미널2
```

전제(맥/실차 워크스페이스에 있어야 함): `vesc_driver`(시리얼 드라이버), `vesc_msgs`,
시리얼 포트 연결(vesc_config.yaml 의 `port`). 게인/전류 한계는 **보수적 기본값**
(kp3/ki6, PID 전류 ±12/−8A, 드라이버 −15~30A) — GUI 로 올려가며 튜닝.

순서:
1. **바퀴 들고** launch → `bench_gui` `CURRENT` 모드 `+2A` → **전진 방향 확인**.
   뒤로 돌면 launch 의 `current_sign:=-1.0` (또는 speed_pid.yaml).
2. `SPEED` 0.5~1 m/s 저속 폐루프 → 그래프로 전류 추종 확인, kp/ki 튜닝.
3. 안정되면 속도/전류 한계 올림.

> ⚠️ 회생제동(음전류)이 실제 모터까지 가려면 vesc_driver `current_min` 이 음수여야 함.
> bench_real.launch.py 가 `driver_current_min:=-15.0` 으로 override 함(기본 vesc_config 는 0=잘림).

## 노드 / 실행

| executable | 설명 |
|---|---|
| `speed_pid_to_current` | `/ackermann_cmd`(목표속도) + `/sensors/core`(실측) → 100Hz PID → `/commands/motor/current`. 조향 servo 변환 포함. |
| `bench_gui` | 슬라이더 GUI: 전류/속도(±5 m/s)/조향 + **PID 게인(kp/ki/kd) 라이브 튜닝** + E-STOP + 실시간 텔레메트리. python_qt_binding. 게인 슬라이더는 `speed_pid_to_current_node` 에 파라미터로 즉시 반영(`add_on_set_parameters_callback`). |
| `bench_console` | 동일 기능 터미널 버전. |

```bash
colcon build --packages-select vesc_current_control --symlink-install
source install/setup.bash

ros2 launch vesc_current_control speed_pid_current.launch.py   # PID 노드
ros2 run vesc_current_control bench_gui                        # GUI
```

## 설정 — `config/speed_pid.yaml`

- PID gain (`kp/ki/kd`), 전류 한계(`current_max/min`, 음수=회생제동), `current_sign`(전류부호↔진행방향, 벤치 확인 후 조정), `speed_to_erpm_gain`(기본 3423).

## 의존성

`rclpy`, `std_msgs`, `ackermann_msgs`, `vesc_msgs`, `python_qt_binding`.
HW(serial/vesc_driver) 의존 없음 — 어디서나 빌드/테스트 가능.

## 실차 미확인 사항

- `current_sign`: 결선 의존. 벤치에서 `bench_gui` CURRENT 모드 `+2A` 로 전진 방향 확인 후 필요시 `-1.0`.
- `vesc_config.yaml` 의 driver `current_min` 이 0 이면 음전류(회생제동) 잘림 → 음수로 풀어야 함.
- PID gain 실차 튜닝 필요.
