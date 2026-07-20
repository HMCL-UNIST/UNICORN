# VESC 내부 설정 스냅샷 (실차, 2026-06-16)

실차 VESC 펌웨어 config 를 VESC Tool CLI 로 추출한 **참조용 스냅샷**(읽기용, 빌드/런타임과 무관).

추출:
```bash
APP="/Applications/VESC Tool.app/Contents/MacOS/VESC Tool"
"$APP" --offscreen --vescPort /dev/cu.usbmodem3041 --getMcConf mcconf-2026-06-16.xml   # 모터 설정
"$APP" --offscreen --vescPort /dev/cu.usbmodem3041 --getAppConf appconf-2026-06-16.xml  # 앱 설정
```
(포트는 `/dev/cu.*` — tty. 쓰면 Resource busy. ROS vesc 노드 전부 종료 후. Linux 면 보통 `/dev/ttyACM0`.)

되돌려 쓰기: `--setMcConf <xml>` / `--setAppConf <xml>`.

## 우리 전류제어와 직결되는 XML-only 값 (이 차 실측)
- `l_current_max=100 / l_current_min=-100` — 모터 전류 하드캡 (펌웨어). 우리 PID current_max(40) 는 안쪽.
- `l_in_current_min=-60` — **배터리 회생(음전류) 한계** = 회생제동의 실제 천장.
- `l_abs_current_max=150` — 절대 전류캡.
- `l_battery_cut_start=10 / cut_end=8` — 전압 컷오프(처지면 힘 빠짐).
- `l_temp_fet/motor_start=85 / end=100` — 온도 디레이팅.
- `l_max_duty=0.95`, `l_max_erpm=100000`.
- `foc_current_kp=0.0089 / foc_current_ki=12.12` — VESC **내부 전류루프 PID**(우리 speed PID 와 별개 안쪽 루프).
- `motor_type=2`(FOC), `foc_sensor_mode=2`. `foc_motor_r/l/flux_linkage` 등은 모터 detection 산출값 — 손편집 금지.
- appconf: `app_to_use=3`, `permanent_uart_enabled=1`, `timeout_msec=1000`.

이 값들은 ROS(bench_gui/param)에서 못 바꾸고 **VESC Tool(GUI 또는 XML)** 로만 변경 가능.
