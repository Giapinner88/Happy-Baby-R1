"""Tuning Main Window - Performance & PD Tuning GUI"""
import sys
import math
import numpy as np

from .qt_compat import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QStatusBar, QMenuBar, QMenu, QToolBar,
    QLabel, QFrame, QMessageBox, QTabWidget, Qt, QTimer, QCoreApplication, QEvent, QKeySequence, QAction
)

from .theme import Theme
from .joint_panel import JointPanel
from .tuning_panel import TuningPanel
from .telemetry_graph import TelemetryGraph
from .control_panel import ControlPanel

from utils.joint_names import JOINT_IDX
from utils.safe_limits import SAFE_LIMITS_DEG
from bridge.udp_client import DEFAULT_KP, DEFAULT_KD


class TuningMainWindow(QMainWindow):
    """Main window for R1 PD Tuning GUI"""

    def __init__(self, udp_client=None, interface="auto"):
        super().__init__()
        self.udp_client = udp_client
        self.interface = interface

        self.selected_joint = 24
        self.motors_enabled = False
        self.speed_dps = 5.0
        self.step_deg = 3.0

        self._setup_ui()
        self._setup_menus()
        self._setup_toolbar()
        self._setup_statusbar()

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._update_ui)
        self.update_timer.start(50)

        self.keys_pressed = set()
        QCoreApplication.instance().installEventFilter(self)

    def _setup_ui(self):
        self.setWindowTitle("R1 Performance & Tuning GUI")
        self.setMinimumSize(1450, 980)
        self.setStyleSheet(Theme.get_stylesheet())

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        # Tabs for Functionality
        self.tabs = QTabWidget()
        
        # Tab 1: Position Control (Standard)
        self.panel_pos = JointPanel()
        self.panel_pos.jointSelected.connect(self._on_joint_selected)
        self.tabs.addTab(self.panel_pos, "Position Control")

        # Tab 2: PD Tuning
        self.panel_tune = TuningPanel(DEFAULT_KP, DEFAULT_KD)
        self.panel_tune.jointSelected.connect(self._on_joint_selected)
        self.panel_tune.kpChanged.connect(self._on_kp_changed)
        self.panel_tune.kdChanged.connect(self._on_kd_changed)
        self.tabs.addTab(self.panel_tune, "PD Tuning (Kp/Kd)")

        main_layout.addWidget(self.tabs, 3)

        # Right column: Graph + Controls
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        self.telemetry_graph = TelemetryGraph()
        right_layout.addWidget(self.telemetry_graph, 2)
        
        self.control_panel = ControlPanel()
        self.control_panel.enableMotors.connect(self._on_enable_motors)
        self.control_panel.disableMotors.connect(self._on_disable_motors)
        self.control_panel.setSpeed.connect(self._on_speed_change)
        self.control_panel.setStep.connect(self._on_step_change)
        
        right_layout.addWidget(self.control_panel, 1)
        
        main_layout.addWidget(right_widget, 1)
        central.setFocusPolicy(Qt.StrongFocus)

    def _setup_menus(self):
        menubar = self.menuBar()
        file_menu = menubar.addMenu("&File")
        exit_action = QAction("E&xit", self)
        exit_action.setShortcut(QKeySequence.Quit)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        robot_menu = menubar.addMenu("&Robot")
        enable_action = QAction("&Enable Motors", self)
        enable_action.setShortcut(QKeySequence("Enter"))
        enable_action.triggered.connect(self._on_enable_motors)
        robot_menu.addAction(enable_action)

        stop_action = QAction("&Emergency Stop", self)
        stop_action.setShortcut(QKeySequence("Space"))
        stop_action.triggered.connect(self._on_disable_motors)
        robot_menu.addAction(stop_action)

    def _setup_toolbar(self):
        toolbar = QToolBar("Main")
        self.addToolBar(toolbar)
        self.status_icon = QLabel("●")
        self.status_icon.setStyleSheet(f"color: {Theme.ALERT.name()}; font-size: 16px;")
        toolbar.addWidget(self.status_icon)
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: #6B7686;")
        toolbar.addWidget(self.status_label)
        toolbar.addSeparator()

        enable_btn = QAction("Enable Motors", self)
        enable_btn.triggered.connect(self._on_enable_motors)
        toolbar.addAction(enable_btn)
        stop_btn = QAction("Emergency Stop", self)
        stop_btn.triggered.connect(self._on_disable_motors)
        toolbar.addAction(stop_btn)

    def _setup_statusbar(self):
        self.statusbar = QStatusBar()
        self.setStatusBar(self.statusbar)
        self.battery_label = QLabel("Battery: --%")
        self.statusbar.addPermanentWidget(self.battery_label)

    def _update_ui(self):
        if not self.udp_client: return
        state = self.udp_client.get_state()
        if not state: return

        self.status_icon.setStyleSheet(f"color: {Theme.VAL_REAL.name()}; font-size: 16px;")
        self.status_label.setText("Connected")
        self.battery_label.setText(f"Battery: {state.soc}%")

        self.panel_pos.update_all_joints(state.motor_states, self.udp_client.target_q)
        
        sdk_idx = JOINT_IDX[self.selected_joint]
        ms = state.motor_states[sdk_idx]
        self.telemetry_graph.update_telemetry(
            self.selected_joint, ms.dq, 0.0, ms.tau, ms.temp
        )

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Space: self._on_disable_motors()
        elif key in (Qt.Key_Return, Qt.Key_E): self._on_enable_motors()
        elif key == Qt.Key_D and self.motors_enabled: self._adjust_angle(self.selected_joint, self.step_deg)
        elif key == Qt.Key_A and self.motors_enabled: self._adjust_angle(self.selected_joint, -self.step_deg)
        elif key == Qt.Key_W and self.motors_enabled: self._adjust_angle(self.selected_joint, self.speed_dps * 0.05)
        elif key == Qt.Key_S and self.motors_enabled: self._adjust_angle(self.selected_joint, -self.speed_dps * 0.05)
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat(): return
        key = event.key()
        if key in (Qt.Key_W, Qt.Key_S) and self.motors_enabled:
            self._snap_target_to_actual(self.selected_joint)
        super().keyReleaseEvent(event)

    def _adjust_angle(self, joint_id, delta_deg):
        if not self.udp_client or not self.motors_enabled or self.udp_client.target_q is None: return
        lim_min, lim_max = SAFE_LIMITS_DEG[joint_id]
        current = math.degrees(self.udp_client.target_q[joint_id])
        new_val = max(lim_min, min(lim_max, current + delta_deg))
        self.udp_client.set_target_angle(joint_id, math.radians(new_val))

    def _snap_target_to_actual(self, joint_id):
        if not self.udp_client or not self.motors_enabled: return
        state = self.udp_client.get_state()
        if state:
            phys_q = state.motor_states[JOINT_IDX[joint_id]].q
            self.udp_client.set_target_angle(joint_id, phys_q)

    def _on_joint_selected(self, joint_id: int):
        self.selected_joint = joint_id
        self.panel_pos.select_joint(joint_id)
        self.panel_tune.select_joint(joint_id)
        self.control_panel.set_selected_joint(joint_id)
        self.telemetry_graph.clear_history()
        self.statusbar.showMessage(f"Selected joint: {joint_id}")
        
    def _on_kp_changed(self, joint_id, kp):
        if self.udp_client:
            self.udp_client.set_target_kp(joint_id, kp)
            
    def _on_kd_changed(self, joint_id, kd):
        if self.udp_client:
            self.udp_client.set_target_kd(joint_id, kd)

    def _on_enable_motors(self):
        if self.udp_client and self.udp_client.enable_motors():
            self.motors_enabled = True
            self.control_panel.set_enabled_state(True)
            self.statusbar.showMessage("Motors enabled")

    def _on_disable_motors(self):
        if self.udp_client: self.udp_client.disable_motors()
        self.motors_enabled = False
        self.control_panel.set_enabled_state(False)
        self.statusbar.showMessage("Motors disabled")

    def _on_speed_change(self, speed): self.speed_dps = speed
    def _on_step_change(self, step): self.step_deg = step
    
    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress and event.key() == Qt.Key_Space:
            self._on_disable_motors()
            return True
        return super().eventFilter(obj, event)

    def closeEvent(self, event):
        if self.udp_client: self.udp_client.disable_motors()
        event.accept()
