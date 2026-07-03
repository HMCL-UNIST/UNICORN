# VESC Calibration — 변경 내역 (push 대상)

외부 odom(cartographer/kiss/localization)을 기준으로 VESC 속도/조향 매핑을
캘리브레이션하는 노드와, gain을 **실시간으로 바꾸고 저장**하는 기능을 추가한다.

## 포함 파일 (10)

### 캘리브레이션 기능
| 파일 | 내용 |
|---|---|
| `stack_master/scripts/vesc_calibration_node.py` | **신규** — 캘리브레이션 노드 |
| `stack_master/launch/vesc_calibration.launch.xml` | **신규** — localization + 캘리 노드 + rqt |
| `stack_master/CMakeLists.txt` | `vesc_calibration_node.py` install 추가 |
| `stack_master/package.xml` | `std_srvs`, `pitwall` 의존성 추가 |

### VESC 실시간 파라미터 콜백
| 파일 | 내용 |
|---|---|
| `sensor/vesc/vesc_ackermann/src/ackermann_to_vesc.cpp` | `add_on_set_parameters_callback` — gain 재시작 없이 반영 |
| `sensor/vesc/vesc_ackermann/include/vesc_ackermann/ackermann_to_vesc.hpp` | 위 선언 |
| `sensor/vesc/vesc_ackermann/src/vesc_to_odom.cpp` | 동일 콜백 (odom 경로) |
| `sensor/vesc/vesc_ackermann/include/vesc_ackermann/vesc_to_odom.hpp` | 위 선언 |

### 부수 작업 (Task 1)
| 파일 | 내용 |
|---|---|
| `stack_master/launch/low_level.launch.xml` | `keyboard_teleop` 기본 false, hokuyo(urg) 드라이버 기본 off (`hokuyo` arg) |
| `stack_master/launch/localization.launch.xml` | `keyboard_teleop` 기본 false |

## 제외 (이 push에 넣지 않음)
- `stack_master/config/CAR/vehicle_config.yaml` — 실차 튜닝값(`steering_angle_to_servo_offset`),
  차량 개체값이라 제외.
- `stack_master/maps/ifac/ifac_ground_lidar.yaml` — 별개 작업(GLIM ground crop).
- `state_estimation/kiss_icp_localization` — 서브모듈 포인터(`-dirty`), 무관.

## 핵심 설계

- **명령 경로**: 노드가 `/vesc/high_level/ackermann_cmd`(mux autodrive 입력)에 발행.
  simple_mux가 조이스틱 autodrive/humandrive로 시작·비상정지 중재.
- **측정(ground truth)**: `/car_state/odom` (외부 localization).
  - 선속도 = `twist.linear` 우선, 없으면 **pose 변위/시간** 폴백.
  - yaw rate = **pose heading 기울기** `d(yaw)/dt` (kiss는 `twist.angular.z`를
    안 채워서 0이 되므로 twist 대신 pose 사용). → drift 정상 측정.
- **분류**: cmd_steer ≈ 0 → 직진(속도 gain + servo 중립 offset), ≠ 0 → 회전(servo gain).
- **결과**: 한 번 실행 = 독립 결과. pitwall 패널에 `명령/기대/실제/바꿀 config/RAISE·LOWER`만
  표시(구체값은 콘솔). 누적 없음.
- **두 노드 일관성**: `vesc/**` 와일드카드로 `ackermann_to_vesc`(명령)와
  `vesc_to_odom`(odom)가 같은 gain을 공유. apply/save가 **둘 다 동기화**.

## 사용법

```bash
# 차 쪽
ros2 launch stack_master vesc_calibration.launch.xml map:=<MAP> localization:=kiss
# 로컬 GUI (SSH면 로컬에서)
rqt   # Dynamic Reconfigure → 필터 "cmd" 로 명령값만 표시
```

- rqt에서 `cmd_speed`/`cmd_steer` 설정 → 조이스틱 **AUTODRIVE** 버튼으로 그 1점 주행.
- 패널의 RAISE/LOWER 보고 `/vesc/ackermann_to_vesc_node`의 gain을 rqt로 조정(실시간 반영).
- 만족하면 rqt에서 **`save_config` 토글 True** (또는 `~/save` 서비스) →
  `vehicle_config.yaml`에 저장 + 두 vesc 노드 동기화.

### 서비스
| 서비스 | 동작 |
|---|---|
| `~/stop` | 즉시 정지 |
| `~/reset` | 직전 결과 + viz path 초기화 |
| `~/apply` | 직전 캘리 결과를 두 vesc 노드에 실시간 push |
| `~/save` | 현재 live gain을 `vehicle_config.yaml`에 저장 + 두 노드 동기화 |

### Path 시각화
autodrive 세션 동안 발행 (세션 시작 시 초기화, 각자 시작 pose 기준 상대좌표):
- `/vesc_calibration/expected_path` — `/vesc/odom` 데드레커닝(기대 경로)
- `/vesc_calibration/estimated_path` — `/car_state/odom` localization(실제 경로)

RViz에서 Path 디스플레이 두 개(위 토픽) 추가해 보면, 두 경로의 벌어짐이 캘리 오차.

## 빌드
```bash
source src/unicorn-racing-stack/unicorn.sh
cbuild --packages-select vesc_ackermann stack_master pitwall
```
symlink-install이라 Python 노드는 재빌드 없이 노드 재시작만으로 반영됨.
