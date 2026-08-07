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
| `kp/ki/kd` | PID 게인 | 12/20/0 (kp≈current_max/가속구간오차; kd로 오버슈트 감쇠) |
| `vel_ff_gain` | 속도 FF — 유지전류 [A/(m/s)] (0=off). ⚠️ 이 차에선 항력이 관성의 1/51 → 실익 거의 없음 | 0 (off) |
| `static_ff` | 정지마찰 FF — 목표≠0 시 breakaway [A] (0=off) | 0 (off) |
| `accel_ff_gain` | ⛔ **사용 금지** — 목표속도의 인과적 미분이라 preview 없음, 실측상 오히려 느려짐 | **0 (고정)** |
| `accel_ff_tau` | `accel_ff_gain` 전용 LPF [s] (위 항목 미사용이므로 무의미) | 0.1 |
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

## ⭐ 전류제어의 진짜 근거 — VESC 속도모드의 숨은 가속 상한
실차 `mcconf-2026-06-16.xml` 에 **속도모드에만 걸리는 제한**이 두 개 있다.
전류모드는 VESC 속도 PID 자체를 우회하므로 둘 다 적용되지 않는다.

```
s_pid_ramp_erpms_s = 25000   ← 속도 setpoint 슬루 제한 = 가속 상한
s_pid_min_erpm     = 900     ← 이하에서 속도제어 불가 (저속 불감대)
```

실측 모터상수로 유도한 각 벽 (지면추력 1.0414 N/A, 질량 4kg 기준):

| 벽 | 값 | 속도모드 | 전류모드 |
|---|---|---|---|
| 토크 한계 (100A) | ~26 m/s² | 안 걸림 | 안 걸림 |
| **속도모드 램프** | **5.42 m/s²** (gain 4614) / 7.30 (gain 3423) | **걸림** | **없음** |
| 접지 한계 | μ×9.81 (미측정) | 걸림 | 걸림 |
| 저속 불감대 | 0.195 m/s (gain 4614) | **있음** | 없음 |

**토크는 남아돈다. 실제 병목은 속도모드 램프일 가능성이 크다.**

### 판정 기준 (측정으로 끝낼 수 있음)
> **전류제어가 가속에서 이득을 보려면 μ > 25000/(gain × 9.81)**
> unicorn(gain 4614): **μ > 0.552** / IFAC(gain 3423): **μ > 0.744**

0→5 m/s 도달시간 (unicorn, 램프 5.42 m/s² 기준):

| μ | 속도모드 | 전류모드 | 이득 |
|---|---|---|---|
| 0.5 | 1.019s | 1.019s | **0%** (램프가 접지 위 → 안 걸림) |
| 0.6 | 0.923s | 0.849s | 7.9% |
| 0.7 | 0.923s | 0.728s | 21.1% |
| 0.8 | 0.923s | 0.637s | **31.0%** |
| 0.9 | 0.923s | 0.566s | 38.6% |

μ 가 판정선 아래면 **가속 이득은 정확히 0** 이고, 남는 근거는 저속 불감대 제거·
regen 명시 제어·traction 제어 여지뿐이다. 노면이 저그립 우레탄이면 μ 가 판정선
근처일 수 있어 **반드시 측정해야 한다.**

### 실차 측정 절차 (10분, 2회 주행)
1. **접지한계 μ** — PID 끄고 전류모드 개루프 60A 고정, 직선에서 eRPM 로그.
   초기 가속도가 `1.0414×60/m` 보다 낮게 포화하면 그 값이 μg → μ 확정.
2. **램프 확인** — 기존 속도모드(ackermann_to_vesc)로 0→5 m/s 스텝, eRPM 로그.
   가속도가 5.42 m/s²(gain 4614)에서 평평하게 잘리면 램프 확인.
   1번의 μg 보다 낮은 값에서 잘리면 확정적.

### ⚠️ 먼저 배제해야 할 대안 (null hypothesis)
램프가 병목으로 확인되면, **VESC Tool 에서 `s_pid_ramp_erpms_s` 를 올리는 것만으로**
속도모드 그대로 개선될 수 있다(코드 0줄). `s_pid_min_erpm` 하향도 마찬가지.
**이 대안을 시도하지 않고 전류제어를 "가속 때문에" 정당화하면 안 된다.**
전류제어의 고유 근거는 속도 setpoint 로 표현 불가능한 것 — traction 제어(슬립 기반
전류 캡), 명시적 regen 성형, MPC 종방향 **힘/가속도** 입력 — 이지 "더 빠른 가속" 이 아니다.

## Feedforward — 무결성 감사 결과 (2026-08-06) ⚠️ 이전 주장 철회

이 절의 이전 내용("FF 로 rise90 1.30s→0.76s, ~40%↓, 초기가속 확연 강화")은
**대조군 없는 교란된 비교**였다. 재감사에서 다음이 확인되어 **철회**한다.

### 철회 1 — "FF 로 40% 빨라짐" 은 재현되지 않음
같은 fake 플랜트(`k_acc=0.15,drag=0.5`) 재현 결과:

| 조건 | kp12 단독 | +`vel_ff_gain=3.33` |
|---|---|---|
| 캡 30A, 0→5 m/s | 1.46s | 1.44s (**개선 1%**) |
| 캡 90A, 0→5 m/s | 0.72s | 0.56s (22%) |

문서의 40% 는 어느 조건에서도 안 나온다. 더 중요한 건 **대조군이 빠져 있었다**는 것:

```
velFF=8.0, kp=12 → rise 0.52s, peak 30A
FF=0,     kp=60 → rise 0.52s, peak 30A     ← 완전 동일
```

**FF 없이 kp 만 올려도 같은 결과.** rise 만 보고 전류를 안 찍으면
"FF 가 빠르다"와 "전류를 더 썼다"가 구분되지 않는다. **FF 성능 주장에는
반드시 (a) 피크전류·포화시간 동시 기록, (b) 전류를 맞춘 고게인 대조군이 필요하다.**

포화 영역에서는 애초에 제어법이 아무것도 못 바꾼다 — 이론 하한(항상 Imax 개루프)
0.46s 에 모든 설정이 0.52~0.72s 로 몰린다. **가감속을 정하는 것은 제어법이 아니라 전류 캡.**

### 철회 2 — `accel_ff_gain` 은 preview 가 없다 (설계 결함)
```python
raw_dot = (v_target - last_target)/dt   # 지금 목표의 미분 = 인과적(causal)
```
목표가 바뀌는 **그 순간** 반응하는데 오차도 같은 순간 생긴다 → **P 항과 시점이 동일**,
추가되는 미래 정보 **0**. 실측도 일치하여 **오히려 느려진다**:

```
accFF=0 → 0.72s      accFF=5 → 0.92s      accFF=20 → 1.10s
```
(포화 중 conditional integration 이 적분을 얼리는데, FF 가 LPF 로 감쇠하면 받쳐줄 적분이 없음)

→ **`accel_ff_gain` 은 켜지 말 것.** 진짜 preview 는 `/local_waypoints`·`/global_waypoints`
(`f110_msgs/Wpnt.vx_mps`, `.ax_mps2`) 또는 MPCC 예측 호라이즌에서 와야 한다. 아래 "재설계" 참조.

### 철회 3 — 이 차에서 `vel_ff_gain`(항력항)은 무시해도 되는 크기
실측 모터상수에서 유도한 지면추력 **1.0414 N/A**
(`kt = 1.5×7극쌍×0.001372Wb = 0.014406 Nm/A`, 기어 3, 휠반경 41.5mm)와
스택 차량 파라미터(`mass=3.5kg, dragcoeff=0.0136`) 기준:

| 항 | 크기 | 전류 환산 |
|---|---|---|
| 항력 @5 m/s | 0.34 N | **0.33 A** (무부하전류 1A 보다 작음) |
| 관성 @5 m/s² | 17.5 N | **16.8 A** |

**관성항이 항력항의 51배.** 즉 이전 캘리브 절차(정속 유지전류↔속도 기울기로
`vel_ff_gain` 튜닝)는 **51배 작은 항을 튜닝하는 것**이라 실익이 없다 → 절차 폐기.

### 그럼 FF 가 언제 의미 있나 (검증된 유일한 근거)
노이즈(0.05 m/s)·지연(60ms)을 넣고 **제어노력(전류 채터링)을 맞춘** 공정비교:

| 채터링 | FF 있음 | FF 없음 |
|---|---|---|
| 0.75 A | **0.82s**, OS 4.5% | 4.16s, 정상상태 −7.6%(도달 실패) |
| 1.89 A | **0.46s**, OS 0.9% | 0.88s, −6.3% |
| 3.78 A | — | 0.40s, OS 0.1% |

**같은 노이즈 증폭 예산에서 FF 가 2배 빠르다.** 이것이 FF 의 진짜 이점이며,
"가속 한계를 늘린다"가 아니라 "낮은 게인으로도 추종이 된다"이다.

단 이 이점을 쓰려면 **ki 를 반드시 낮춰야 한다.** FF 와 적분이 이중보상하면
오버슈트 25.5% 가 난다(현재 권장 `velFF + ki=20` 이 이 상태). 검증된 최선 조합:
`vel_ff_gain=(물리값) / ki=2 / kp=30`.

> anti-windup 은 conditional integration — FF 가 전류를 포화시켜도 적분이 오염되지 않아 FF+PI 가 안정적으로 동작(back-calc 는 FF 포화 시 적분을 음으로 덤프해 정체 유발).

### 재설계 방향 — 모델기반 preview FF (튜닝 불필요)
경험적 게인 튜닝 대신 물리로 바로 계산한다:
```
I_ff = (m·a_preview + dragcoeff·v²) / 1.0414 [N/A]  + I_noload
  a_preview : 전방 vx 프로파일의 v·dv/ds, 또는 Wpnt.ax_mps2, 또는 MPCC 예측 호라이즌
  m, dragcoeff : 스택 veh_params (3.5kg, 0.0136)
  1.0414 N/A  : 실측 mcconf 모터상수에서 유도
```
`vel_ff_gain`/`accel_ff_gain` 캘리브레이션 자체가 사라지고, 지배적인 관성항에
**진짜 미래 정보**를 적용하게 된다.

### 아직 안 잰 것 (성능 주장 전 필수)
- 기존 벤치 검증(2026-06-16)은 **휠 공중(무부하)** — 접지도 항력도 없어
  μ 도 `vel_ff_gain` 도 그 벤치에서는 **원리적으로 측정 불가**. 부호·폭주 없음 확인이지
  성능 검증이 아니다.
- fake 플랜트는 접지한계·모터 전류 슬루가 없고, 위 노이즈/지연 값은 **가정**이다.
- `/sensors/core` 는 **50Hz** 인데 제어루프는 **100Hz** → 측정이 한 틱 걸러 그대로라
  `d = -kd*(meas-last_meas)/dt` 가 한 틱 걸러 0 이 된다. 현재 `kd=0` 이라 잠복 상태이나
  **D 를 켜면 50Hz 채터링이 난다.** 켤 거면 `control_rate` 를 50 으로 맞추거나
  측정 갱신 시에만 D 를 갱신할 것.

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
