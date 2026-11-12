import sys
import time
import math
from collections import deque

import pygame
from pymavlink import mavutil

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox, QDoubleSpinBox, QGroupBox, QTabWidget
)
from PyQt5.QtCore import QTimer, Qt, QUrl, QPointF, QRectF
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPolygonF, QFont
from PyQt5.QtWebEngineWidgets import QWebEngineView
import pyqtgraph as pg


# =====================================================================
# CUSTOM INSTRUMENT WIDGETS (NavBall & Compass)
# =====================================================================
class NavBallWidget(QWidget):
    """Artificial Horizon / Attitude Indicator for Roll & Pitch (radians)."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(110, 110)
        self.roll = 0.0   # radians
        self.pitch = 0.0  # radians

    def set_orientation(self, roll_rad: float, pitch_rad: float):
        self.roll = roll_rad
        self.pitch = pitch_rad
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        radius = min(w, h) / 2.0 - 4

        # Clip to circular instrument bezel
        painter.save()
        path = pg.QtGui.QPainterPath()
        path.addEllipse(QPointF(cx, cy), radius, radius)
        painter.setClipPath(path)

        # Rotate & translate horizon by roll and pitch
        painter.translate(cx, cy)
        painter.rotate(-math.degrees(self.roll))
        pitch_offset = (math.degrees(self.pitch) / 30.0) * radius

        # Sky (Upper) & Water/Ground (Lower)
        painter.fillRect(QRectF(-radius * 2, -radius * 2 + pitch_offset, radius * 4, radius * 2), QColor("#313244"))
        painter.fillRect(QRectF(-radius * 2, pitch_offset, radius * 4, radius * 2), QColor("#181825"))

        # Horizon line
        painter.setPen(QPen(QColor("#89B4FA"), 2))
        painter.drawLine(QPointF(-radius, pitch_offset), QPointF(radius, pitch_offset))
        painter.restore()

        # Center crosshair (fixed aircraft/ROV reference)
        painter.setPen(QPen(QColor("#F9E2AF"), 2))
        painter.drawLine(QPointF(cx - 18, cy), QPointF(cx - 5, cy))
        painter.drawLine(QPointF(cx + 5, cy), QPointF(cx + 18, cy))
        painter.drawEllipse(QPointF(cx, cy), 2, 2)

        # Outer Bezel
        painter.setPen(QPen(QColor("#45475A"), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(cx, cy), radius, radius)


class CompassWidget(QWidget):
    """Heading Indicator Dial (0-360 degrees)."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(110, 110)
        self.heading = 0.0  # degrees

    def set_heading(self, heading_deg: float):
        self.heading = heading_deg % 360.0
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        radius = min(w, h) / 2.0 - 4

        # Dial Background
        painter.setPen(QPen(QColor("#45475A"), 2))
        painter.setBrush(QBrush(QColor("#11111B")))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        # Cardinal Labels
        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        painter.setPen(QColor("#CDD6F4"))
        painter.drawText(QRectF(cx - 10, cy - radius + 2, 20, 14), Qt.AlignCenter, "N")
        painter.drawText(QRectF(cx - 10, cy + radius - 16, 20, 14), Qt.AlignCenter, "S")
        painter.drawText(QRectF(cx + radius - 16, cy - 7, 14, 14), Qt.AlignCenter, "E")
        painter.drawText(QRectF(cx - radius + 2, cy - 7, 14, 14), Qt.AlignCenter, "W")

        # Heading Needle
        painter.save()
        painter.translate(cx, cy)
        painter.rotate(self.heading)

        north_poly = QPolygonF([QPointF(0, -radius + 16), QPointF(-5, 0), QPointF(5, 0)])
        painter.setBrush(QBrush(QColor("#F38BA8")))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(north_poly)

        south_poly = QPolygonF([QPointF(0, radius - 16), QPointF(-5, 0), QPointF(5, 0)])
        painter.setBrush(QBrush(QColor("#89B4FA")))
        painter.drawPolygon(south_poly)
        painter.restore()


# =====================================================================
# MAIN ROV DASHBOARD (Replicated UI + UDP PyMavlink)
# =====================================================================
class ROVDashboard(QWidget):
    def __init__(self, telemetry_port=14550, command_port=14551):
        super().__init__()
        self.connected_cmd = False
        self.connected_telem = False

        # UDP PyMavlink Sockets (14550 = Telemetry In, 14551 = Commands Out)
        self.mav_in = mavutil.mavlink_connection(f'udpin:127.0.0.1:{telemetry_port}')
        self.mav_out = mavutil.mavlink_connection(f'udpout:127.0.0.1:{command_port}', source_system=255)

        # Target Command State (Mirrored in Control Inputs Tab)
        self.target_arm = False
        self.target_mode_idx = 0
        self.mode_names = {0: "MANUAL", 1: "DEPTH HOLD", 2: "HEADING HOLD"}
        self.servo1_pwm = 1500
        self.servo2_pwm = 1500

        # Telemetry Plot Buffers
        self.start_time = time.time()
        self.last_rx_time = 0.0
        self.time_history = deque(maxlen=300)
        self.depth_history = deque(maxlen=300)
        self.target_depth_history = deque(maxlen=300)
        self.map_ready = False

        # Pygame Xbox Joystick + Keyboard Fallback
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        self.keys_pressed = set()

        self.initUI()

        # 30 Hz Non-Blocking Loop for UDP Telemetry + Xbox/Keyboard Polling
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_udp_loop)
        self.timer.start(33)

    def initUI(self):
        self.setWindowTitle("ROV Master Dashboard")
        self.resize(1200, 820)

        self.setStyleSheet("""
            QWidget { background-color: #1E1E2E; color: #CDD6F4; font-family: 'Segoe UI', Arial, sans-serif; }
            QTabWidget::pane { border: 1px solid #45475A; border-radius: 4px; }
            QTabBar::tab { background: #313244; padding: 8px 20px; margin-right: 2px; border-top-left-radius: 4px; border-top-right-radius: 4px; }
            QTabBar::tab:selected { background: #89B4FA; color: #1E1E2E; font-weight: bold; }
            QGroupBox { border: 1px solid #45475A; border-radius: 6px; margin-top: 15px; font-weight: bold; padding-top: 15px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; color: #89B4FA; }
            QPushButton { background-color: #313244; border: 1px solid #45475A; padding: 6px 12px; border-radius: 4px; font-weight: bold; color: #CDD6F4; }
            QPushButton:hover { background-color: #45475A; }
            QComboBox, QDoubleSpinBox { background-color: #11111B; border: 1px solid #45475A; padding: 4px; border-radius: 4px; color: #CDD6F4; }
            .ValueLabel { font-weight: bold; color: #A6E3A1; font-size: 14px; }
            .StatusLabel { font-weight: bold; font-size: 14px; padding: 5px; border-radius: 4px; }
            .VideoLabel { background-color: #11111B; border: 2px solid #45475A; border-radius: 6px; }
        """)

        # Main Layout: Left = Visuals (Depth Plot + OSM Map), Right = Data Tabs
        main_layout = QHBoxLayout()

        # --- LEFT PANEL: DEPTH PLOT & OPENSTREETMAP VIEW ---
        video_layout = QVBoxLayout()

        plot_grp = QGroupBox("Live Depth vs. Time Plot")
        plot_lyt = QVBoxLayout()
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#11111B')
        self.plot_widget.setLabel('left', 'Depth', units='m')
        self.plot_widget.setLabel('bottom', 'Time', units='s')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.25)
        self.plot_widget.getViewBox().invertY(True)
        self.depth_curve = self.plot_widget.plot(pen=pg.mkPen('#89B4FA', width=2.5), name="Actual Depth")
        self.target_curve = self.plot_widget.plot(pen=pg.mkPen('#F38BA8', width=1.5, style=Qt.DashLine), name="Target Depth")
        plot_lyt.addWidget(self.plot_widget)
        plot_grp.setLayout(plot_lyt)

        map_grp = QGroupBox("OpenStreetMap Live ROV Position")
        map_lyt = QVBoxLayout()
        self.web_view = QWebEngineView()
        self.web_view.setMinimumSize(600, 320)
        self.init_osm_map()
        map_lyt.addWidget(self.web_view)
        map_grp.setLayout(map_lyt)

        video_layout.addWidget(plot_grp, stretch=1)
        video_layout.addWidget(map_grp, stretch=1)

        # --- RIGHT PANEL: TABS ---
        data_layout = QVBoxLayout()
        self.lbl_status = QLabel("Status: Waiting for Data...")
        self.lbl_status.setProperty("class", "StatusLabel")
        self.lbl_status.setStyleSheet("color: #F9E2AF; background-color: #313244;")
        self.lbl_status.setAlignment(Qt.AlignCenter)
        data_layout.addWidget(self.lbl_status)

        tabs = QTabWidget()
        self.tab_commands = QWidget()
        self.tab_telemetry = QWidget()
        tabs.addTab(self.tab_commands, "Control Inputs")
        tabs.addTab(self.tab_telemetry, "Telemetry & Sensors")

        self.setup_commands_tab()
        self.setup_telemetry_tab()

        data_layout.addWidget(tabs)

        main_layout.addLayout(video_layout, stretch=3)
        main_layout.addLayout(data_layout, stretch=2)
        self.setLayout(main_layout)

    def create_value_label(self, text, color="#A6E3A1"):
        label = QLabel(text)
        label.setProperty("class", "ValueLabel")
        label.setStyleSheet(f"font-weight: bold; font-size: 14px; color: {color};")
        return label

    def init_osm_map(self):
        html = """
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8"/>
            <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
            <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
            <style>html, body, #map { height: 100%; margin: 0; background: #11111B; }</style>
        </head>
        <body>
            <div id="map"></div>
            <script>
                var map = L.map('map').setView([19.9975, 85.8245], 18);
                L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                    maxZoom: 19,
                    attribution: '© OpenStreetMap'
                }).addTo(map);

                var rovMarker = L.circleMarker([19.9975, 85.8245], {
                    radius: 8, fillColor: "#89B4FA", color: "#1E1E2E", weight: 2, fillOpacity: 0.95
                }).addTo(map);
                var pathLine = L.polyline([], {color: '#A6E3A1', weight: 2}).addTo(map);

                function updateROVPosition(lat, lon) {
                    var latlng = [lat, lon];
                    rovMarker.setLatLng(latlng);
                    pathLine.addLatLng(latlng);
                }
            </script>
        </body>
        </html>
        """
        self.web_view.setHtml(html, QUrl("https://localhost"))
        self.web_view.loadFinished.connect(lambda ok: setattr(self, 'map_ready', ok))

    def setup_commands_tab(self):
        layout = QVBoxLayout()

        # 1. Interactive Operator Controls (Sends UDP MAVLink Commands)
        op_grp = QGroupBox("Operator Controls (UDP Out)")
        op_lyt = QGridLayout()

        self.btn_arm = QPushButton("ARM")
        self.btn_arm.setStyleSheet("background-color: #A6E3A1; color: #1E1E2E;")
        self.btn_arm.clicked.connect(lambda: self.send_arm_cmd(True))

        self.btn_disarm = QPushButton("DISARM")
        self.btn_disarm.setStyleSheet("background-color: #F38BA8; color: #1E1E2E;")
        self.btn_disarm.clicked.connect(lambda: self.send_arm_cmd(False))

        self.combo_mode = QComboBox()
        self.combo_mode.addItems(["MANUAL", "DEPTH HOLD", "HEADING HOLD"])
        self.combo_mode.currentIndexChanged.connect(self.send_mode_cmd)

        self.spin_depth = QDoubleSpinBox()
        self.spin_depth.setRange(0.0, 100.0)
        self.spin_depth.setSingleStep(0.5)
        self.spin_depth.setValue(2.0)
        btn_depth = QPushButton("Set Depth (m)")
        btn_depth.clicked.connect(self.send_depth_setpoint)

        self.spin_heading = QDoubleSpinBox()
        self.spin_heading.setRange(0.0, 359.9)
        self.spin_heading.setSingleStep(5.0)
        self.spin_heading.setValue(90.0)
        btn_heading = QPushButton("Set Heading (°)")
        btn_heading.clicked.connect(self.send_heading_setpoint)

        op_lyt.addWidget(self.btn_arm, 0, 0)
        op_lyt.addWidget(self.btn_disarm, 0, 1)
        op_lyt.addWidget(QLabel("Flight Mode:"), 1, 0)
        op_lyt.addWidget(self.combo_mode, 1, 1)
        op_lyt.addWidget(self.spin_depth, 2, 0)
        op_lyt.addWidget(btn_depth, 2, 1)
        op_lyt.addWidget(self.spin_heading, 3, 0)
        op_lyt.addWidget(btn_heading, 3, 1)
        op_grp.setLayout(op_lyt)

        # 2. Your Target State Group
        status_grp = QGroupBox("Target State")
        status_lyt = QGridLayout()
        self.cmd_arm = self.create_value_label("DISARMED", "#F38BA8")
        self.cmd_mode = self.create_value_label("MANUAL", "#F9E2AF")
        status_lyt.addWidget(QLabel("Arm Cmd:"), 0, 0)
        status_lyt.addWidget(self.cmd_arm, 0, 1, Qt.AlignRight)
        status_lyt.addWidget(QLabel("Mode Cmd:"), 1, 0)
        status_lyt.addWidget(self.cmd_mode, 1, 1, Qt.AlignRight)
        status_grp.setLayout(status_lyt)

        # 3. Your Target Movement (PWM) Group
        move_grp = QGroupBox("Target Movement (PWM)")
        move_lyt = QGridLayout()
        self.cmd_fwd = self.create_value_label("1500")
        self.cmd_lat = self.create_value_label("1500")
        self.cmd_thr = self.create_value_label("1500")
        self.cmd_pch = self.create_value_label("1500")
        self.cmd_rol = self.create_value_label("1500")
        self.cmd_yaw = self.create_value_label("1500")
        move_lyt.addWidget(QLabel("Forward:"), 0, 0); move_lyt.addWidget(self.cmd_fwd, 0, 1, Qt.AlignRight)
        move_lyt.addWidget(QLabel("Lateral:"), 1, 0); move_lyt.addWidget(self.cmd_lat, 1, 1, Qt.AlignRight)
        move_lyt.addWidget(QLabel("Thrust:"), 2, 0);  move_lyt.addWidget(self.cmd_thr, 2, 1, Qt.AlignRight)
        move_lyt.addWidget(QLabel("Pitch:"), 0, 2);   move_lyt.addWidget(self.cmd_pch, 0, 3, Qt.AlignRight)
        move_lyt.addWidget(QLabel("Roll:"), 1, 2);    move_lyt.addWidget(self.cmd_rol, 1, 3, Qt.AlignRight)
        move_lyt.addWidget(QLabel("Yaw:"), 2, 2);     move_lyt.addWidget(self.cmd_yaw, 2, 3, Qt.AlignRight)
        move_grp.setLayout(move_lyt)

        # 4. Your Target Servos Group
        servo_grp = QGroupBox("Target Servos")
        servo_lyt = QGridLayout()
        self.cmd_srv1 = self.create_value_label("1500")
        self.cmd_srv2 = self.create_value_label("1500")
        servo_lyt.addWidget(QLabel("Servo 1:"), 0, 0); servo_lyt.addWidget(self.cmd_srv1, 0, 1, Qt.AlignRight)
        servo_lyt.addWidget(QLabel("Servo 2:"), 1, 0); servo_lyt.addWidget(self.cmd_srv2, 1, 1, Qt.AlignRight)
        servo_grp.setLayout(servo_lyt)

        layout.addWidget(op_grp)
        layout.addWidget(status_grp)
        layout.addWidget(move_grp)
        layout.addWidget(servo_grp)
        layout.addStretch()
        self.tab_commands.setLayout(layout)

    def setup_telemetry_tab(self):
        layout = QVBoxLayout()

        # 1. System Vitals
        sys_grp = QGroupBox("System Vitals")
        sys_lyt = QGridLayout()
        self.tlm_arm = self.create_value_label("DISARMED", "#F38BA8")
        self.tlm_mode = self.create_value_label("MANUAL", "#89B4FA")
        self.tlm_bat = self.create_value_label("0.0 V")
        self.tlm_time = self.create_value_label("0.0 s")
        sys_lyt.addWidget(QLabel("Actual Arm:"), 0, 0); sys_lyt.addWidget(self.tlm_arm, 0, 1, Qt.AlignRight)
        sys_lyt.addWidget(QLabel("Battery:"), 1, 0);    sys_lyt.addWidget(self.tlm_bat, 1, 1, Qt.AlignRight)
        sys_lyt.addWidget(QLabel("Uptime:"), 0, 2);     sys_lyt.addWidget(self.tlm_time, 0, 3, Qt.AlignRight)
        sys_lyt.addWidget(QLabel("Active Mode:"), 1, 2); sys_lyt.addWidget(self.tlm_mode, 1, 3, Qt.AlignRight)
        sys_grp.setLayout(sys_lyt)

        # 2. Environment (Added Live Depth alongside Pressures)
        env_grp = QGroupBox("Environment")
        env_lyt = QGridLayout()
        self.tlm_depth = self.create_value_label("0.00 m", "#89B4FA")
        self.tlm_in_press = self.create_value_label("1013.2 hPa")
        self.tlm_out_press = self.create_value_label("1013.2 hPa")
        env_lyt.addWidget(QLabel("Depth:"), 0, 0);         env_lyt.addWidget(self.tlm_depth, 0, 1, Qt.AlignLeft)
        env_lyt.addWidget(QLabel("Int. Pressure:"), 1, 0); env_lyt.addWidget(self.tlm_in_press, 1, 1, Qt.AlignLeft)
        env_lyt.addWidget(QLabel("Ext. Pressure:"), 1, 2); env_lyt.addWidget(self.tlm_out_press, 1, 3, Qt.AlignLeft)
        env_grp.setLayout(env_lyt)

        # 3. Spatial Orientation + NavBall + Compass
        ori_grp = QGroupBox("Spatial Orientation")
        ori_main_lyt = QHBoxLayout()
        ori_text_lyt = QGridLayout()
        self.tlm_roll = self.create_value_label("0.0")
        self.tlm_pitch = self.create_value_label("0.0")
        self.tlm_yaw = self.create_value_label("0.0")
        self.tlm_rspeed = self.create_value_label("0.0")
        self.tlm_pspeed = self.create_value_label("0.0")
        self.tlm_yspeed = self.create_value_label("0.0")
        ori_text_lyt.addWidget(QLabel("Roll (rad):"), 0, 0);  ori_text_lyt.addWidget(self.tlm_roll, 0, 1, Qt.AlignRight)
        ori_text_lyt.addWidget(QLabel("Pitch (rad):"), 1, 0); ori_text_lyt.addWidget(self.tlm_pitch, 1, 1, Qt.AlignRight)
        ori_text_lyt.addWidget(QLabel("Yaw/Head:"), 2, 0);    ori_text_lyt.addWidget(self.tlm_yaw, 2, 1, Qt.AlignRight)
        ori_text_lyt.addWidget(QLabel("Roll Spd:"), 3, 0);    ori_text_lyt.addWidget(self.tlm_rspeed, 3, 1, Qt.AlignRight)
        ori_text_lyt.addWidget(QLabel("Pitch Spd:"), 4, 0);   ori_text_lyt.addWidget(self.tlm_pspeed, 4, 1, Qt.AlignRight)
        ori_text_lyt.addWidget(QLabel("Yaw Spd:"), 5, 0);     ori_text_lyt.addWidget(self.tlm_yspeed, 5, 1, Qt.AlignRight)

        self.navball_widget = NavBallWidget()
        self.compass_widget = CompassWidget()
        ori_main_lyt.addLayout(ori_text_lyt)
        ori_main_lyt.addStretch()
        ori_main_lyt.addWidget(self.navball_widget)
        ori_main_lyt.addWidget(self.compass_widget)
        ori_grp.setLayout(ori_main_lyt)

        # 4. Actual Thruster PWMs (M1 - M8)
        thr_grp = QGroupBox("Actual Thruster PWMs")
        thr_lyt = QGridLayout()
        self.tlm_pwms = [self.create_value_label("1500") for _ in range(8)]
        for i in range(4):
            thr_lyt.addWidget(QLabel(f"M{i+1}:"), i, 0)
            thr_lyt.addWidget(self.tlm_pwms[i], i, 1, Qt.AlignRight)
            thr_lyt.addWidget(QLabel(f"M{i+5}:"), i, 2)
            thr_lyt.addWidget(self.tlm_pwms[i+4], i, 3, Qt.AlignRight)
        thr_grp.setLayout(thr_lyt)

        layout.addWidget(sys_grp)
        layout.addWidget(env_grp)
        layout.addWidget(ori_grp)
        layout.addWidget(thr_grp)
        layout.addStretch()
        self.tab_telemetry.setLayout(layout)

    def check_connection(self):
        if self.connected_cmd or self.connected_telem:
            self.lbl_status.setText("Status: LIVE DATA CONNECTED")
            self.lbl_status.setStyleSheet("color: #1E1E2E; background-color: #A6E3A1; font-weight: bold; padding: 5px; border-radius: 4px;")

    # ---------------- MAVLINK UDP SENDERS ----------------
    def send_arm_cmd(self, arm: bool):
        self.target_arm = arm
        self.mav_out.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1.0 if arm else 0.0, 0, 0, 0, 0, 0, 0
        )

    def send_mode_cmd(self, mode_idx: int):
        self.target_mode_idx = mode_idx
        self.mav_out.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            float(mode_idx), 0, 0, 0, 0, 0, 0
        )
        self.send_depth_setpoint()
        self.send_heading_setpoint()

    def send_depth_setpoint(self):
        self.mav_out.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT, 0,
            float(self.spin_depth.value()), 0, 0, 0, 0, 0, 0
        )

    def send_heading_setpoint(self):
        self.mav_out.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
            float(self.spin_heading.value()), 0, 0, 0, 0, 0, 0
        )

    # ---------------- XBOX JOYSTICK & KEYBOARD POLLING ----------------
    def keyPressEvent(self, event):
        self.keys_pressed.add(event.key())

    def keyReleaseEvent(self, event):
        self.keys_pressed.discard(event.key())

    def apply_deadzone(self, val, dz=0.12):
        return 0.0 if abs(val) < dz else float(val)

    def poll_and_send_manual_control(self):
        pygame.event.pump()
        fwd, lat, thr, yaw = 0.0, 0.0, 0.0, 0.0

        if self.joystick and self.joystick.get_init():
            num_axes = self.joystick.get_numaxes()
            if num_axes >= 4:
                lat = self.apply_deadzone(self.joystick.get_axis(0))
                fwd = self.apply_deadzone(-self.joystick.get_axis(1))
                yaw = self.apply_deadzone(self.joystick.get_axis(2) if num_axes == 4 else self.joystick.get_axis(3))
                thr = self.apply_deadzone(self.joystick.get_axis(3) if num_axes == 4 else self.joystick.get_axis(4))

        # Keyboard Fallback (WASD + Arrow Keys)
        if Qt.Key_W in self.keys_pressed: fwd = 0.75
        if Qt.Key_S in self.keys_pressed: fwd = -0.75
        if Qt.Key_D in self.keys_pressed: lat = 0.75
        if Qt.Key_A in self.keys_pressed: lat = -0.75
        if Qt.Key_Down in self.keys_pressed: thr = 0.75   # Dive
        if Qt.Key_Up in self.keys_pressed: thr = -0.75    # Surface
        if Qt.Key_Right in self.keys_pressed: yaw = 0.65
        if Qt.Key_Left in self.keys_pressed: yaw = -0.65

        # Convert normalized [-1, 1] to standard BlueROV2 PWM [1100, 1900] for UI display
        cmd_dict = {
            'arm': self.target_arm,
            'mode': self.mode_names.get(self.target_mode_idx, "MANUAL"),
            'forward': int(1500 + fwd * 400),
            'lateral': int(1500 + lat * 400),
            'thrust': int(1500 + thr * 400),
            'pitch': 1500,
            'roll': 1500,
            'yaw': int(1500 + yaw * 400),
            'servo1': self.servo1_pwm,
            'servo2': self.servo2_pwm
        }
        self.update_cmd_ui(cmd_dict)

        # Only transmit MANUAL_CONTROL over UDP when sticks/keys are non-zero
        # (So the Pixhawk Simulator's 1-second safety watchdog triggers when released)
        if any(abs(v) > 0.01 for v in (fwd, lat, thr, yaw)):
            self.mav_out.mav.manual_control_send(
                1,
                int(fwd * 1000),
                int(lat * 1000),
                int(thr * 1000),
                int(yaw * 1000),
                0
            )

    def update_cmd_ui(self, msg: dict):
        self.connected_cmd = True
        if msg['arm']:
            self.cmd_arm.setText("ARMED")
            self.cmd_arm.setStyleSheet("color: #A6E3A1; font-weight: bold; font-size: 14px;")
        else:
            self.cmd_arm.setText("DISARMED")
            self.cmd_arm.setStyleSheet("color: #F38BA8; font-weight: bold; font-size: 14px;")

        self.cmd_mode.setText(str(msg['mode']))
        self.cmd_fwd.setText(str(msg['forward']))
        self.cmd_lat.setText(str(msg['lateral']))
        self.cmd_thr.setText(str(msg['thrust']))
        self.cmd_pch.setText(str(msg['pitch']))
        self.cmd_rol.setText(str(msg['roll']))
        self.cmd_yaw.setText(str(msg['yaw']))
        self.cmd_srv1.setText(str(msg['servo1']))
        self.cmd_srv2.setText(str(msg['servo2']))

    # ---------------- UDP TELEMETRY RECEIVER ----------------
    def update_udp_loop(self):
        self.poll_and_send_manual_control()

        while True:
            msg = self.mav_in.recv_match(blocking=False)
            if not msg:
                break

            self.connected_telem = True
            self.last_rx_time = time.time()
            self.check_connection()

            mtype = msg.get_type()

            if mtype == 'HEARTBEAT':
                is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if is_armed:
                    self.tlm_arm.setText("ARMED")
                    self.tlm_arm.setStyleSheet("color: #A6E3A1; font-weight: bold; font-size: 14px;")
                else:
                    self.tlm_arm.setText("DISARMED")
                    self.tlm_arm.setStyleSheet("color: #F38BA8; font-weight: bold; font-size: 14px;")

                self.tlm_mode.setText(self.mode_names.get(msg.custom_mode, "MANUAL"))

            elif mtype == 'SYS_STATUS':
                bat_v = msg.voltage_battery / 1000.0
                self.tlm_bat.setText(f"{bat_v:.2f} V")
                if 1.0 < bat_v < 14.0:
                    self.tlm_bat.setStyleSheet("color: #F38BA8; font-weight: bold; font-size: 14px;")
                else:
                    self.tlm_bat.setStyleSheet("color: #A6E3A1; font-weight: bold; font-size: 14px;")

            elif mtype == 'ATTITUDE':
                uptime = time.time() - self.start_time
                self.tlm_time.setText(f"{uptime:.1f} s")

                self.tlm_roll.setText(f"{msg.roll:.3f}")
                self.tlm_pitch.setText(f"{msg.pitch:.3f}")
                yaw_deg = math.degrees(msg.yaw) % 360.0
                self.tlm_yaw.setText(f"{yaw_deg:.1f}°")

                self.tlm_rspeed.setText(f"{msg.rollspeed:.3f}")
                self.tlm_pspeed.setText(f"{msg.pitchspeed:.3f}")
                self.tlm_yspeed.setText(f"{msg.yawspeed:.3f}")

                self.navball_widget.set_orientation(float(msg.roll), float(msg.pitch))
                self.compass_widget.set_heading(yaw_deg)

            elif mtype == 'VFR_HUD':
                depth = -msg.alt
                self.tlm_depth.setText(f"{depth:.2f} m")

                # Calculate hydrostatic external pressure from depth (1013.25 hPa surface + ~98.06 hPa/m)
                ext_press = 1013.25 + (depth * 98.0665)
                self.tlm_in_press.setText("1013.2 hPa")
                self.tlm_out_press.setText(f"{ext_press:.1f} hPa")

                t_now = time.time() - self.start_time
                self.time_history.append(t_now)
                self.depth_history.append(depth)
                self.target_depth_history.append(self.spin_depth.value() if self.target_mode_idx == 1 else depth)

                self.depth_curve.setData(list(self.time_history), list(self.depth_history))
                self.target_curve.setData(list(self.time_history), list(self.target_depth_history))

            elif mtype == 'SERVO_OUTPUT_RAW':
                # Extract M1-M8 Thruster PWMs directly from MAVLink SERVO_OUTPUT_RAW
                pwms = [
                    msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw,
                    msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw
                ]
                for i in range(8):
                    self.tlm_pwms[i].setText(f"{int(pwms[i])}")

            elif mtype == 'GLOBAL_POSITION_INT':
                if self.map_ready:
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    self.web_view.page().runJavaScript(f"updateROVPosition({lat}, {lon});")

        # Connection Timeout Watchdog
        if self.last_rx_time > 0 and (time.time() - self.last_rx_time > 2.0):
            self.lbl_status.setText("Status: TELEMETRY LINK LOST")
            self.lbl_status.setStyleSheet("color: #1E1E2E; background-color: #F38BA8; font-weight: bold; padding: 5px; border-radius: 4px;")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ROVDashboard()
    window.show()
    sys.exit(app.exec_())