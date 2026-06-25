#!/usr/bin/env python3
"""VESC 전류제어 벤치 GUI.

bench_console 의 GUI 버전. 슬라이더로 전류/속도를 넣고 큰 E-STOP 버튼으로
즉시 정지. 실측 속도/전류가 실시간 표시됩니다. rqt 가 쓰는 python_qt_binding
을 사용하므로 mac 실차에서도 동일하게 동작합니다.

실행:
  python3 bench_gui.py                         # 기본 상한 10A / 10 m/s
  python3 bench_gui.py --max-current 8 --max-speed 3 --kp 6 --ki 15

모드:
  CURRENT (직접)  : 슬라이더 전류[A] → /commands/motor/current  (PID 노드 끄고)
  SPEED   (PID)   : 슬라이더 속도[m/s] → /ackermann_cmd          (PID 노드 켜고)
  IDLE / E-STOP   : 전류 0
"""
import argparse
import signal
import sys
import math

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped

from python_qt_binding.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QSlider, QDoubleSpinBox, QLabel, QPushButton, QRadioButton, QButtonGroup,
    QScrollArea, QSizePolicy,
)
from python_qt_binding.QtCore import Qt, QTimer, QPointF
from python_qt_binding.QtGui import QFont, QPainter, QColor, QPen

from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

import time
from collections import deque

PID_NODE = 'speed_pid_to_current_node'
PLOT_WINDOW = 10.0   # 그래프 시간창 [s]
GAIN_DEFAULT = 3423.0
SPEED_SIGN = 1.0  # m/s = SPEED_SIGN*state.speed/gain (HW 실측 2026-06-16: +current=전진 시 state.speed 양수)


class Bridge(Node):
    """ROS 통신 + 명령 상태 보관."""
    def __init__(self, gain):
        super().__init__('vesc_bench_gui')
        self.gain = gain
        self.mode = 'idle'        # 'idle' | 'current' | 'speed'
        self.cur_cmd = 0.0
        self.spd_cmd = 0.0
        self.steer_cmd = 0.0      # rad
        self.meas_speed = 0.0
        self.meas_current = 0.0
        self.cmd_current_seen = 0.0

        self.cur_pub = self.create_publisher(Float64, 'commands/motor/current', 10)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        self.create_subscription(VescStateStamped, 'sensors/core', self._state_cb, 10)
        self.create_subscription(Float64, 'commands/motor/current', self._cur_cb, 10)

        # PID 노드 파라미터 클라이언트 (kp/ki/kd 라이브 튜닝)
        self.pclient = AsyncParameterClient(self, PID_NODE)
        self.pid_online = False

    def set_gains(self, kp, ki, kd):
        """PID 노드에 게인 비동기 set. 노드 미기동이면 조용히 skip."""
        if not self.pclient.services_are_ready():
            self.pid_online = False
            return
        self.pid_online = True
        self.pclient.set_parameters([
            Parameter('kp', Parameter.Type.DOUBLE, float(kp)),
            Parameter('ki', Parameter.Type.DOUBLE, float(ki)),
            Parameter('kd', Parameter.Type.DOUBLE, float(kd)),
        ])

    def set_enabled(self, flag):
        """CURRENT 모드 진입 시 PID 노드를 꺼서(enabled=False) /commands/motor/current 충돌 방지."""
        if not self.pclient.services_are_ready():
            return
        self.pclient.set_parameters([
            Parameter('enabled', Parameter.Type.BOOL, bool(flag)),
        ])

    def set_caps(self, cmax, cmin):
        """PID 노드 전류 캡(가속/회생) 라이브 set."""
        if not self.pclient.services_are_ready():
            return
        self.pclient.set_parameters([
            Parameter('current_max', Parameter.Type.DOUBLE, float(cmax)),
            Parameter('current_min', Parameter.Type.DOUBLE, float(cmin)),
        ])

    def fetch_tunables(self):
        """노드의 kp/ki/kd + current_max/min 을 비동기로 읽어옴 (연결 시 1회, 노드=source of truth).
        준비 안 됐으면 None."""
        if not self.pclient.services_are_ready():
            self.pid_online = False
            return None
        self.pid_online = True
        return self.pclient.get_parameters(['kp', 'ki', 'kd', 'current_max', 'current_min'])

    def _state_cb(self, m):
        self.meas_speed = SPEED_SIGN * m.state.speed / self.gain
        self.meas_current = m.state.current_motor

    def _cur_cb(self, m):
        self.cmd_current_seen = m.data

    def publish(self):
        if not rclpy.ok():
            return
        if self.mode == 'current':
            self.cur_pub.publish(Float64(data=self.cur_cmd))
        elif self.mode == 'speed':
            c = AckermannDriveStamped()
            c.drive.speed = self.spd_cmd
            c.drive.steering_angle = self.steer_cmd
            self.cmd_pub.publish(c)

    def estop(self):
        self.mode = 'idle'; self.cur_cmd = 0.0; self.spd_cmd = 0.0
        self.cur_pub.publish(Float64(data=0.0))


def labeled_slider(lo, hi, step, suffix, decimals):
    """슬라이더 + 스핀박스 (양방향 동기). 반환: (widget, get, set, set_enabled)."""
    box = QHBoxLayout()
    factor = round(1.0 / step)
    sld = QSlider(Qt.Horizontal)
    sld.setMinimum(int(round(lo * factor)))
    sld.setMaximum(int(round(hi * factor)))
    sld.setValue(0)
    spin = QDoubleSpinBox()
    spin.setRange(lo, hi); spin.setSingleStep(step); spin.setDecimals(decimals)
    spin.setSuffix(' ' + suffix); spin.setValue(0.0)

    guard = {'busy': False}
    def on_slider(v):
        if guard['busy']:
            return
        guard['busy'] = True; spin.setValue(v / factor); guard['busy'] = False
    def on_spin(v):
        if guard['busy']:
            return
        guard['busy'] = True; sld.setValue(int(round(v * factor))); guard['busy'] = False
    sld.valueChanged.connect(on_slider)
    spin.valueChanged.connect(on_spin)

    box.addWidget(sld, 4); box.addWidget(spin, 1)
    w = QWidget(); w.setLayout(box)
    def set_enabled(en):
        sld.setEnabled(en); spin.setEnabled(en)
    def set_val(x):
        guard['busy'] = True
        sld.setValue(int(round(x * factor))); spin.setValue(x)
        guard['busy'] = False
    return w, spin.value, set_val, set_enabled


class StripChart(QWidget):
    """롤링 시계열 그래프: 목표속도/실측속도(m/s, 좌축) + 지령전류(A, 우축).

    의존성 없이 QPainter 로 직접 그림 (pyqtgraph/matplotlib 불필요).
    """
    def __init__(self, v_range, i_range):
        super().__init__()
        self.setMinimumHeight(150)
        self.setMinimumWidth(440)   # 우측 넓은 그래프
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.v_range = max(0.5, v_range)   # 속도 좌축 ±range
        self.i_range = max(5.0, i_range)   # 전류 우축 ±range
        self.data = deque()                # (t, target_v, meas_v, cmd_i)
        self.t0 = time.monotonic()

    def push(self, target_v, meas_v, cmd_i):
        t = time.monotonic() - self.t0
        self.data.append((t, target_v, meas_v, cmd_i))
        while self.data and t - self.data[0][0] > PLOT_WINDOW:
            self.data.popleft()
        # 전류 우축 자동확장
        ai = abs(cmd_i)
        if ai > self.i_range:
            self.i_range = ai * 1.15
        self.update()

    def paintEvent(self, _ev):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()
        ml, mr, mt, mb = 44, 44, 14, 18
        x0, x1 = ml, W - mr
        y0, y1 = mt, H - mb
        midy = (y0 + y1) / 2.0
        qp.fillRect(self.rect(), QColor(24, 26, 33))
        # 0 기준선
        qp.setPen(QPen(QColor(80, 84, 96), 1))
        qp.drawLine(int(x0), int(midy), int(x1), int(midy))
        qp.setPen(QPen(QColor(120, 124, 136), 1))
        qp.drawText(2, int(midy) + 4, '0')
        # 축 라벨
        qp.setPen(QColor(110, 200, 110))
        qp.drawText(2, y0 + 8, f'{self.v_range:.1f}')
        qp.drawText(2, y1, f'-{self.v_range:.1f}')
        qp.setPen(QColor(235, 150, 70))
        qp.drawText(W - mr + 4, y0 + 8, f'{self.i_range:.0f}A')
        qp.drawText(W - mr + 4, y1, f'-{self.i_range:.0f}')

        if len(self.data) < 2:
            return
        tnow = self.data[-1][0]
        tmin = tnow - PLOT_WINDOW

        def xof(t):
            return x0 + (t - tmin) / PLOT_WINDOW * (x1 - x0)

        def yof(val, rng):
            v = max(-rng, min(rng, val))
            return midy - (v / rng) * (y1 - y0) / 2.0

        def draw(series_idx, rng, color, width):
            qp.setPen(QPen(color, width))
            pts = [QPointF(xof(d[0]), yof(d[series_idx], rng)) for d in self.data]
            for a, b in zip(pts, pts[1:]):
                qp.drawLine(a, b)

        draw(1, self.v_range, QColor(90, 170, 255), 1.5)   # target v (파랑)
        draw(2, self.v_range, QColor(90, 220, 110), 2.0)   # meas v (초록)
        draw(3, self.i_range, QColor(235, 150, 70), 2.0)   # current (주황)
        # 범례
        qp.setPen(QColor(90, 170, 255)); qp.drawText(x0 + 6, y0 + 10, 'target v')
        qp.setPen(QColor(90, 220, 110)); qp.drawText(x0 + 70, y0 + 10, 'meas v')
        qp.setPen(QColor(235, 150, 70)); qp.drawText(x0 + 130, y0 + 10, 'current')


class BenchGui(QWidget):
    def __init__(self, bridge, max_cur, max_spd, gains=(8.0, 20.0, 0.0)):
        super().__init__()
        self.b = bridge
        self.setWindowTitle('VESC Current Control — Bench')
        self.setMinimumWidth(860)
        root = QVBoxLayout(self)

        # ── 모드 선택 (상단 전체폭) ──
        mode_box = QGroupBox('Mode')
        ml = QHBoxLayout(mode_box)
        self.rb_idle = QRadioButton('IDLE')
        self.rb_cur = QRadioButton('CURRENT (direct)')
        self.rb_spd = QRadioButton('SPEED (PID)')
        self.rb_idle.setChecked(True)
        grp = QButtonGroup(self)
        for rb in (self.rb_idle, self.rb_cur, self.rb_spd):
            grp.addButton(rb); ml.addWidget(rb)
        self.rb_idle.toggled.connect(self._mode_changed)
        self.rb_cur.toggled.connect(self._mode_changed)
        self.rb_spd.toggled.connect(self._mode_changed)
        root.addWidget(mode_box)

        # ── 본문: 좌=컨트롤(2 하위컬럼, 높이 억제), 우=응답그래프(남는 폭 전부, 넓게) ──
        cols = QHBoxLayout()
        la = QVBoxLayout(); lb = QVBoxLayout()   # 컨트롤 좌/우 하위컬럼
        ctrl = QHBoxLayout(); ctrl.addLayout(la); ctrl.addLayout(lb)
        cols.addLayout(ctrl, 0)
        root.addLayout(cols)

        # [좌] 전류 직접 명령 슬라이더
        cg = QGroupBox(f'Current command  [±{max_cur:.0f} A]')
        cl = QVBoxLayout(cg)
        self.cur_w, self.cur_get, self.cur_set, self.cur_en = \
            labeled_slider(-max_cur, max_cur, 0.5, 'A', 1)
        cl.addWidget(self.cur_w)
        la.addWidget(cg)

        # [좌] 속도 setpoint + 조향 + step
        sg = QGroupBox(f'Speed setpoint  [±{max_spd:.1f} m/s]')
        sl = QVBoxLayout(sg)
        self.spd_w, self.spd_get, self.spd_set, self.spd_en = \
            labeled_slider(-max_spd, max_spd, 0.05, 'm/s', 2)
        sl.addWidget(self.spd_w)
        st_row = QHBoxLayout()
        st_row.addWidget(QLabel('Steering'))
        self.steer_w, self.steer_get, self.steer_set, self.steer_en = \
            labeled_slider(-30, 30, 1.0, 'deg', 0)
        st_row.addWidget(self.steer_w)
        st_wrap = QWidget(); st_wrap.setLayout(st_row)
        sl.addWidget(st_wrap)
        step_row = QHBoxLayout()
        step_row.addWidget(QLabel('step →'))
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            v = round(max_spd * frac, 2)
            btn = QPushButton(f'{v:g}')
            btn.clicked.connect(lambda _checked=False, val=v: self._step_speed(val))
            step_row.addWidget(btn)
        step_wrap = QWidget(); step_wrap.setLayout(step_row)
        sl.addWidget(step_wrap)
        la.addWidget(sg)

        # [좌] 텔레메트리
        tg = QGroupBox('Telemetry')
        gl = QGridLayout(tg)
        self.lbl_mode = QLabel('IDLE')
        self.lbl_tgt = QLabel('-')
        self.lbl_vmeas = QLabel('-')
        self.lbl_icmd = QLabel('-')
        self.lbl_imot = QLabel('-')
        big = QFont(); big.setPointSize(13); big.setBold(True)
        for w in (self.lbl_mode, self.lbl_tgt, self.lbl_vmeas, self.lbl_icmd, self.lbl_imot):
            w.setFont(big)
        gl.addWidget(QLabel('mode'), 0, 0);          gl.addWidget(self.lbl_mode, 0, 1)
        gl.addWidget(QLabel('target'), 1, 0);        gl.addWidget(self.lbl_tgt, 1, 1)
        gl.addWidget(QLabel('meas speed'), 2, 0);    gl.addWidget(self.lbl_vmeas, 2, 1)
        gl.addWidget(QLabel('cmd current (bus)'), 3, 0); gl.addWidget(self.lbl_icmd, 3, 1)
        gl.addWidget(QLabel('motor current'), 4, 0); gl.addWidget(self.lbl_imot, 4, 1)
        la.addWidget(tg)

        # [우] PID gains (라이브)
        pg = QGroupBox('PID gains (live)')
        pl = QGridLayout(pg)
        kp0, ki0, kd0 = gains
        self.kp_w, self.kp_get, self.kp_set, _ = labeled_slider(0, 50, 0.5, '', 1)
        self.ki_w, self.ki_get, self.ki_set, _ = labeled_slider(0, 100, 1.0, '', 1)
        self.kd_w, self.kd_get, self.kd_set, _ = labeled_slider(0, 5, 0.1, '', 2)
        self.kp_set(kp0); self.ki_set(ki0); self.kd_set(kd0)
        pl.addWidget(QLabel('kp'), 0, 0); pl.addWidget(self.kp_w, 0, 1)
        pl.addWidget(QLabel('ki'), 1, 0); pl.addWidget(self.ki_w, 1, 1)
        pl.addWidget(QLabel('kd'), 2, 0); pl.addWidget(self.kd_w, 2, 1)
        self.lbl_pid = QLabel('PID node: (대기)')
        pl.addWidget(self.lbl_pid, 3, 0, 1, 2)
        lb.addWidget(pg)

        # 전류 캡 (라이브) — 가속/회생 실시간 조정
        capg = QGroupBox('Current limits (live)')
        capl = QGridLayout(capg)
        self.cmax_w, self.cmax_get, self.cmax_set, _ = labeled_slider(0, 100, 1.0, 'A', 0)
        self.cmin_w, self.cmin_get, self.cmin_set, _ = labeled_slider(-60, 0, 1.0, 'A', 0)
        self.cmax_set(90.0); self.cmin_set(-55.0)   # 연결 시 노드값으로 동기됨
        capl.addWidget(QLabel('accel max (+)'), 0, 0); capl.addWidget(self.cmax_w, 0, 1)
        capl.addWidget(QLabel('regen min (−)'), 1, 0); capl.addWidget(self.cmin_w, 1, 1)
        lb.addWidget(capg)

        # ── E-STOP (전체폭) ──
        self.estop_btn = QPushButton('E-STOP  (0)')
        self.estop_btn.setMinimumHeight(48)
        f = QFont(); f.setPointSize(16); f.setBold(True)
        self.estop_btn.setFont(f)
        self.estop_btn.setStyleSheet(
            'QPushButton{background:#c0392b;color:white;border-radius:6px;}'
            'QPushButton:pressed{background:#e74c3c;}')
        self.estop_btn.clicked.connect(self._estop)
        lb.addWidget(self.estop_btn)
        lb.addStretch(1); la.addStretch(1)   # 두 컬럼 위로 정렬

        # ── 응답 그래프 (우측, 남는 폭 전부 — 넓게) ──
        chart_box = QGroupBox(f'Response  (최근 {PLOT_WINDOW:.0f}s)')
        cbl = QVBoxLayout(chart_box)
        self.chart = StripChart(v_range=max_spd, i_range=max_cur)
        cbl.addWidget(self.chart)
        cols.addWidget(chart_box, 1)     # stretch=1 → 우측 전체 차지(넓은 그래프)

        self._mode_changed()

        # ── 타이머: ROS spin+publish 50Hz, UI 갱신 10Hz ──
        self.t_ros = QTimer(self); self.t_ros.timeout.connect(self._ros_tick)
        self.t_ros.start(20)
        self.t_ui = QTimer(self); self.t_ui.timeout.connect(self._ui_tick)
        self.t_ui.start(100)
        # 게인 동기 1Hz: 연결 시 노드값을 읽어 슬라이더에 1회 반영(노드=source of
        # truth) → 그 다음부터 슬라이더값을 노드에 push (사용자 튜닝이 노드를 바꿈)
        self._gain_synced = False
        self._gain_future = None
        self.t_gain = QTimer(self); self.t_gain.timeout.connect(self._gain_tick)
        self.t_gain.start(1000)

    def _gain_tick(self):
        if not rclpy.ok():
            return
        if not self._gain_synced:
            # 1) 연결되면 노드의 현재 kp/ki/kd + 전류캡 을 읽어와 슬라이더에 반영
            if self._gain_future is None:
                self._gain_future = self.b.fetch_tunables()
            elif self._gain_future.done():
                try:
                    vals = self._gain_future.result().values
                    self.kp_set(vals[0].double_value)
                    self.ki_set(vals[1].double_value)
                    self.kd_set(vals[2].double_value)
                    self.cmax_set(vals[3].double_value)
                    self.cmin_set(vals[4].double_value)
                except Exception:
                    pass
                self._gain_synced = True
            return
        # 2) 동기 후엔 슬라이더값(게인+전류캡)을 노드에 push (사용자가 움직이면 반영)
        self.b.set_gains(self.kp_get(), self.ki_get(), self.kd_get())
        self.b.set_caps(self.cmax_get(), self.cmin_get())

    def _step_speed(self, val):
        """프리셋 버튼: SPEED 모드로 전환하고 목표속도를 즉시 val 로 (깔끔한 step)."""
        self.rb_spd.setChecked(True)
        self.spd_set(val)

    def _mode_changed(self, *_):
        if self.rb_cur.isChecked():
            self.b.mode = 'current'
        elif self.rb_spd.isChecked():
            self.b.mode = 'speed'
        else:
            self.b.mode = 'idle'
        self.cur_en(self.b.mode == 'current')
        self.spd_en(self.b.mode == 'speed')
        self.steer_en(self.b.mode == 'speed')
        self.b.set_enabled(self.b.mode != 'current')

    def _estop(self):
        self.rb_idle.setChecked(True)
        self.cur_set(0.0); self.spd_set(0.0)
        self.b.estop()

    def _ros_tick(self):
        if not rclpy.ok():        # 종료 레이스 가드 (Ctrl-C 후 타이머 1회 더 발화)
            return
        # 슬라이더 → 명령 반영
        if self.b.mode == 'current':
            self.b.cur_cmd = self.cur_get()
        elif self.b.mode == 'speed':
            self.b.spd_cmd = self.spd_get()
            self.b.steer_cmd = self.steer_get() * math.pi / 180.0
        rclpy.spin_once(self.b, timeout_sec=0.0)
        self.b.publish()

    def _ui_tick(self):
        self.lbl_mode.setText(self.b.mode.upper())
        if self.b.mode == 'current':
            self.lbl_tgt.setText(f'{self.b.cur_cmd:+.1f} A')
        elif self.b.mode == 'speed':
            self.lbl_tgt.setText(f'{self.b.spd_cmd:+.2f} m/s   {math.degrees(self.b.steer_cmd):+.0f}°')
        else:
            self.lbl_tgt.setText('—')
        self.lbl_vmeas.setText(f'{self.b.meas_speed:+.2f} m/s')
        self.lbl_icmd.setText(f'{self.b.cmd_current_seen:+.1f} A')
        self.lbl_imot.setText(f'{self.b.meas_current:+.1f} A')
        if self.b.pid_online:
            self.lbl_pid.setText(f'PID node: ● online  '
                                 f'(kp={self.kp_get():.1f} ki={self.ki_get():.1f} kd={self.kd_get():.2f})')
        else:
            self.lbl_pid.setText('PID node: ○ offline (speed_pid_to_current 미기동 — 게인 전송 대기)')
        # 그래프 갱신: 목표속도(모드에 따라) / 실측속도 / 지령전류
        tv = self.b.spd_cmd if self.b.mode == 'speed' else 0.0
        self.chart.push(tv, self.b.meas_speed, self.b.cmd_current_seen)

    def closeEvent(self, ev):
        self.b.estop()
        ev.accept()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gain', type=float, default=GAIN_DEFAULT)
    ap.add_argument('--max-current', type=float, default=10.0)
    ap.add_argument('--max-speed', type=float, default=10.0)
    ap.add_argument('--kp', type=float, default=8.0)
    ap.add_argument('--ki', type=float, default=20.0)
    ap.add_argument('--kd', type=float, default=0.0)
    ap.add_argument('--selftest', action='store_true', help='1.5초 후 자동 종료(검증용)')
    args, _ = ap.parse_known_args()

    # rclpy 의 SIGINT 핸들러 비활성화 → Ctrl-C 를 우리가 받아 Qt 를 깨끗이 종료
    # (rclpy 핸들러가 컨텍스트를 먼저 파괴하고 Qt 타이머가 죽은 컨텍스트에 publish →
    #  RCLError/Abort 하던 레이스 제거). 종료는 finally 에서 순서대로.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        rclpy.init()
    bridge = Bridge(args.gain)
    app = QApplication(sys.argv)
    gui = BenchGui(bridge, args.max_current, args.max_speed, gains=(args.kp, args.ki, args.kd))

    # 작은 화면(13" 등)에서 창이 잘리지 않게 스크롤영역으로 감싸고 가용화면에 맞춤
    scroll = QScrollArea()
    scroll.setWidget(gui)
    scroll.setWidgetResizable(True)
    scroll.setWindowTitle('VESC Current Control — Bench')
    avail = app.primaryScreen().availableGeometry()
    want_h = gui.sizeHint().height() + 4
    want_w = gui.sizeHint().width() + 24
    scroll.resize(min(want_w, avail.width() - 40),
                  min(want_h, avail.height() - 60))
    scroll.show()
    app.aboutToQuit.connect(bridge.estop)   # 창 닫힘 시 0 (gui.closeEvent 대체)

    # Ctrl-C(SIGINT) → Qt 종료. Qt 이벤트루프 중엔 파이썬이 시그널을 못 받으므로
    # 빈 heartbeat 타이머로 인터프리터가 주기적으로 깨어 시그널을 처리하게 함.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    hb = QTimer(); hb.timeout.connect(lambda: None); hb.start(200)

    if args.selftest:
        QTimer.singleShot(1500, app.quit)
    try:
        app.exec_() if hasattr(app, 'exec_') else app.exec()
    finally:
        # 타이머 먼저 정지 → 죽은 컨텍스트 접근 차단
        for t in (gui.t_ros, gui.t_ui, gui.t_gain):
            t.stop()
        bridge.estop()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
