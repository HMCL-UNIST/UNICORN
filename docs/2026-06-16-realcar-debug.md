# 실차 디버깅 기록 — SPEED 모드 폭주 부호 교정 & GUI CURRENT 자동 비활성 (2026-06-16)

대상: 실차(Mac Mini, RoboStack ROS 2 Jazzy, VESC 시리얼). 휠 공중 벤치.

> 이 작업은 `a657b47`(폭주 안전가드 latch + speed_sign/current_sign param·launch 노출) **위에 얹은 합본**이다.
> latch 가드는 그대로 유지(안전망)하고, 여기서는 HW 실측으로 **speed_sign 기본값을 교정**하고
> GUI CURRENT 모드 충돌 방지(`enabled`)를 추가한다.

## 1. 증상
`bench_real.launch.py`(vesc_driver + speed_pid_to_current)로 SPEED 모드 → 전류가 max 에 박혀
안 줄고(폭주성), 바퀴 급가속. CURRENT 직접 모드는 정상.

## 2. 진단 (HW 실측, CURRENT 모드 +4 A)
- 토픽 일치(`/commands/motor/current`, root), 텔레메트리 정상(`/sensors/core` 50Hz, voltage 15.5, fault 0).
- **근본 원인 = 피드백 부호**. +current → 바퀴 **전진**, 그때 `state.speed` = **양수(+)**.
  `meas_speed = speed_sign*state.speed/gain` 에서 `speed_sign=-1.0` 이면 전진인데 음수로 읽혀
  `error=target-meas` 가 발산 → 적분 windup → 전류 포화.
- HW 사실: `+current=전진`, 전진 시 `state.speed` 양수, `speed_to_erpm_gain=3423`.
  (`vesc_config.yaml` 의 `duty_cycle_max/min=0.0` → duty 지령은 클램프되어 안 돎. 부호테스트는 CURRENT 모드로.)

## 3. 수정 (a657b47 위에)
| 파일 | 변경 |
|---|---|
| `config/speed_pid.yaml` | `speed_sign` 기본 `-1.0 → 1.0` (HW 교정) |
| `launch/bench_real.launch.py` | `speed_sign` launch arg 기본 `-1.0 → 1.0` |
| `speed_pid_to_current.py` | `speed_sign` declare 기본 `-1.0 → 1.0`; `enabled` 파라미터 추가(False면 control_loop 발행 중단, 끌 때 0 1회) |
| `bench_gui.py` | `set_enabled()` + 모드전환 자동 토글(CURRENT→off); 표시용 `SPEED_SIGN -1.0 → 1.0` |

- **유지**: a657b47 의 폭주 안전가드(`max_abs_speed` latch). 부호를 고쳐 폭주 자체가 사라지지만,
  latch 는 부호 재오설정/이상 시 안전망으로 그대로 둠.
- **`enabled`**(신규): 기본 True. GUI CURRENT 모드 진입 시 PID 를 꺼서 `/commands/motor/current` 충돌 방지
  (PID 가 0 을 100Hz 로 쏘며 직접주입을 덮어쓰던 문제). `ros2 param set /speed_pid_to_current_node enabled false` 로 수동 가능.

## 4. 검증
SPEED 1.0/1.5 m/s → setpoint 수렴(폭주 해소), 바퀴 정상. ±1.5~2% 미세 ripple(무부하 한계진동, 튜닝 선택).
1.5 지령 시 평균 ~1.59(약 6% 상회) — 정밀 추종 필요 시 `speed_to_erpm_gain` 캘리브레이션.

## 5. 실행 (휠 공중)
```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source install/setup.bash    # (mac: setup.zsh)
ros2 launch vesc_current_control bench_real.launch.py
ros2 run vesc_current_control bench_gui   # CURRENT/SPEED 전환 시 enabled 자동 토글
ros2 topic pub /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 1.0, steering_angle: 0.0}}" -r 20
```

## 6. 참고: VESC 내부 파라미터 추출 (VESC Tool CLI, 헤드리스)
```bash
APP="/Applications/VESC Tool.app/Contents/MacOS/VESC Tool"
"$APP" --offscreen --vescPort /dev/cu.usbmodem3041 --getMcConf mcconf.xml
"$APP" --offscreen --vescPort /dev/cu.usbmodem3041 --getAppConf appconf.xml
```
포트는 `/dev/cu.*`(tty. 아님 → Resource busy), ROS vesc 노드 전부 종료 후.
실측 주요값: motor_type=2(FOC), l_current_max=100, l_in_current_min=-60, l_abs_current_max=150, l_max_erpm=100000.
