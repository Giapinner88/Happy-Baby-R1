"""Main Window - Professional R1 Joint Tuner GUI"""
import sys
import math
import numpy as np

from .qt_compat import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QStatusBar, QMenuBar, QMenu, QToolBar,
    QLabel, QFrame, QMessageBox,
    Qt, QTimer, Signal, QEvent, QCoreApplication,
    QAction, QKeySequence, QFont, QIcon
)

from .theme import Theme
from .joint_panel import JointPanel
from .imu_panel import IMUPanel
from .telemetry_graph import TelemetryGraph
from .control_panel import ControlPanel

from utils.joint_names import JOINT_IDX
from utils.safe_limits import SAFE_LIMITS_DEG


class MainWindow(QMainWindow):
    """Main window for R1 Professional Joint Tuner GUI"""

    def __init__(self, udp_client=None, interface="auto", is_dashboard=False):
        super().__init__()
        self.udp_client = udp_client
        self.interface = interface
        self.is_dashboard = is_dashboard

        # State
        self.selected_joint = 24  # Default to head_pitch
        self.motors_enabled = False
        self.monitor_mode = False
        self.speed_dps = 5.0
        self.step_deg = 3.0
        self.movement_cmd = 0.0
        self.last_ws_time = 0.0

        self._setup_ui()
        self._setup_menus()
        self._setup_toolbar()
        self._setup_statusbar()

        # Update timer
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._update_ui)
        self.update_timer.start(50)  # 20 Hz UI update

        # Keyboard state
        self.keys_pressed = set()

        # Global event filter to capture Space key globally
        QCoreApplication.instance().installEventFilter(self)

    def _setup_ui(self):
        """Setup main UI layout"""
        self.setWindowTitle("R1 Joint Tuner - Professional GUI")
        self.setMinimumSize(1450, 980)
        self.setStyleSheet(Theme.get_stylesheet())

        # Central widget
        central = QWidget()
        self.setCentralWidget(central)

        # Main layout: 2 columns
        main_layout = QHBoxLayout(central)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # Left column: Joint panel only (5 columns, horizontally aligned)
        self.joint_panel = JointPanel()
        self.joint_panel.jointSelected.connect(self._on_joint_selected)
        main_layout.addWidget(self.joint_panel, 4)

        # Right column: IMU + Graph + Controls
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setSpacing(10)

        self.imu_panel = IMUPanel()
        right_layout.addWidget(self.imu_panel)

        self.telemetry_graph = TelemetryGraph(is_dashboard=self.is_dashboard)
        right_layout.addWidget(self.telemetry_graph, 2)

        self.control_panel = ControlPanel()
        self.control_panel.enableMotors.connect(self._on_enable_motors)
        self.control_panel.disableMotors.connect(self._on_disable_motors)
        self.control_panel.toggleMonitor.connect(self._on_toggle_monitor)
        self.control_panel.setSpeed.connect(self._on_speed_change)
        self.control_panel.setStep.connect(self._on_step_change)
        right_layout.addWidget(self.control_panel, 2)

        main_layout.addWidget(right_widget, 2)

        # Keyboard focus
        central.setFocusPolicy(Qt.StrongFocus)

    def _setup_menus(self):
        """Setup menu bar"""
        menubar = self.menuBar()

        # File menu
        file_menu = menubar.addMenu("&File")

        exit_action = QAction("E&xit", self)
        exit_action.setShortcut(QKeySequence.Quit)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # Robot menu
        robot_menu = menubar.addMenu("&Robot")

        enable_action = QAction("&Enable Motors", self)
        enable_action.setShortcut(QKeySequence("Enter"))
        enable_action.triggered.connect(self._on_enable_motors)
        robot_menu.addAction(enable_action)

        stop_action = QAction("&Emergency Stop", self)
        stop_action.setShortcut(QKeySequence("Space"))
        stop_action.triggered.connect(self._on_disable_motors)
        robot_menu.addAction(stop_action)

        robot_menu.addSeparator()

        reset_action = QAction("&Reset to Default", self)
        reset_action.triggered.connect(self._on_reset)
        robot_menu.addAction(reset_action)

        # View menu
        view_menu = menubar.addMenu("&View")

        fullscreen_action = QAction("&Full Screen", self)
        fullscreen_action.setShortcut(QKeySequence("F11"))
        fullscreen_action.triggered.connect(self._toggle_fullscreen)
        view_menu.addAction(fullscreen_action)

        # Help menu
        help_menu = menubar.addMenu("&Help")

        about_action = QAction("&About", self)
        about_action.triggered.connect(self._show_about)
        help_menu.addAction(about_action)

    def _setup_toolbar(self):
        """Setup toolbar"""
        toolbar = QToolBar("Main Toolbar")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        # Connection status
        self.status_icon = QLabel("●")
        self.status_icon.setStyleSheet(f"color: {Theme.ALERT.name()}; font-size: 16px;")
        toolbar.addWidget(self.status_icon)

        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: #6B7686;")
        toolbar.addWidget(self.status_label)

        toolbar.addSeparator()

        # Enable/Disable buttons
        enable_btn = QAction("Enable Motors", self)
        enable_btn.setShortcut(QKeySequence("Enter"))
        enable_btn.triggered.connect(self._on_enable_motors)
        toolbar.addAction(enable_btn)

        stop_btn = QAction("Emergency Stop", self)
        stop_btn.setShortcut(QKeySequence("Space"))
        stop_btn.triggered.connect(self._on_disable_motors)
        toolbar.addAction(stop_btn)

        self.monitor_toolbar_label = QLabel(" ")
        self.monitor_toolbar_label.setStyleSheet(f"color: {Theme.VAL_WARNING.name()}; font-weight: bold; margin-left: 20px;")
        toolbar.addWidget(self.monitor_toolbar_label)

        toolbar.addSeparator()

        # Reset button
        reset_btn = QAction("Reset", self)
        reset_btn.triggered.connect(self._on_reset)
        toolbar.addAction(reset_btn)

    def _setup_statusbar(self):
        """Setup status bar"""
        self.statusbar = QStatusBar()
        self.setStatusBar(self.statusbar)

        self.statusbar.showMessage("Ready")

        # Add permanent widgets
        self.battery_label = QLabel("Battery: --%")
        self.battery_label.setStyleSheet("color: #6B7686; font-weight: bold; margin-right: 15px;")
        self.statusbar.addPermanentWidget(self.battery_label)

        self.fps_label = QLabel("FPS: --")
        self.fps_label.setStyleSheet("margin-right: 15px;")
        self.statusbar.addPermanentWidget(self.fps_label)

        self.interface_label = QLabel(f"Interface: {self.interface}")
        self.statusbar.addPermanentWidget(self.interface_label)

    def _update_ui(self):
        """Update UI from robot state"""
        if self.udp_client is None:
            return

        # Calculate received FPS
        import time
        current_time = time.time()
        with self.udp_client.state_lock:
            count = self.udp_client.packet_count
        
        if not hasattr(self, '_last_fps_time'):
            self._last_fps_time = current_time
            self._last_packet_count = count
        else:
            dt = current_time - self._last_fps_time
            if dt >= 1.0:
                fps = (count - self._last_packet_count) / dt
                self.fps_label.setText(f"FPS: {int(fps)}")
                self._last_fps_time = current_time
                self._last_packet_count = count

        state = self.udp_client.get_state()
        if state is None:
            self.status_icon.setStyleSheet(f"color: {Theme.ALERT.name()}; font-size: 16px;")
            self.status_label.setText("Waiting for data...")
            self.battery_label.setText("Battery: --%")
            self.battery_label.setStyleSheet("color: #6B7686; font-weight: bold; margin-right: 15px;")
            return

        # Update connection status
        self.status_icon.setStyleSheet(f"color: {Theme.VAL_REAL.name()}; font-size: 16px;")
        self.status_label.setText("Connected")

        # Update battery level in status bar
        soc = state.soc
        if soc >= 40:
            color = Theme.VAL_REAL.name()
            weight = "normal"
        elif soc >= 20:
            color = Theme.VAL_WARNING.name()
            weight = "normal"
        else:
            color = Theme.ALERT.name()
            weight = "bold"
        self.battery_label.setStyleSheet(f"color: {color}; font-weight: {weight}; margin-right: 15px;")
        self.battery_label.setText(f"Battery: {soc}%")

        # Update joint panel
        self.joint_panel.update_all_joints(state.motor_states, self.udp_client.target_q)

        # Update IMU panel
        self.imu_panel.update_imu(state.imu_roll, state.imu_pitch, state.imu_yaw, state.imu_gyro)

        # Update telemetry graphs
        self.telemetry_graph.update_telemetry(self.selected_joint, state)

        # Handle continuous movement (W/S) robustly against X11 auto-repeat issues
        import time
        if time.time() - self.last_ws_time > 0.15:
            self.movement_cmd = 0.0
            
        if self.movement_cmd != 0.0 and self.motors_enabled and not self.monitor_mode:
            self._adjust_angle(self.selected_joint, self.movement_cmd * 0.05)  # 20Hz = 0.05s

    def keyPressEvent(self, event):
        """Handle key press events"""
        key = event.key()
        self.keys_pressed.add(key)

        # Navigation
        if key == Qt.Key_Up:
            self._navigate_joint('up')
        elif key == Qt.Key_Down:
            self._navigate_joint('down')
        elif key == Qt.Key_Left:
            self._navigate_joint('left')
        elif key == Qt.Key_Right:
            self._navigate_joint('right')

        # Motor control
        elif key == Qt.Key_Space:
            self._on_disable_motors()
        elif (key == Qt.Key_Return or key == Qt.Key_E) and not self.monitor_mode:
            self._on_enable_motors()
        elif key == Qt.Key_M:
            self.control_panel._on_toggle_monitor()

        # Speed settings
        elif key == Qt.Key_1:
            self._on_speed_change(3.0)
        elif key == Qt.Key_2:
            self._on_speed_change(5.0)
        elif key == Qt.Key_3:
            self._on_speed_change(10.0)
        elif key == Qt.Key_4:
            self._on_speed_change(15.0)

        # Step size settings
        elif key == Qt.Key_5:
            self._on_step_change(1.0)
        elif key == Qt.Key_6:
            self._on_step_change(2.0)
        elif key == Qt.Key_7:
            self._on_step_change(3.0)
        elif key == Qt.Key_8:
            self._on_step_change(4.0)

        # Step settings (A/D - single step)
        elif key == Qt.Key_D and self.motors_enabled:
            self._adjust_angle(self.selected_joint, self.step_deg)
        elif key == Qt.Key_A and self.motors_enabled:
            self._adjust_angle(self.selected_joint, -self.step_deg)

        # Continuous (W/S - hold)
        import time
        if key == Qt.Key_W:
            self.movement_cmd = self.speed_dps
            self.last_ws_time = time.time()
        elif key == Qt.Key_S:
            self.movement_cmd = -self.speed_dps
            self.last_ws_time = time.time()

        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        """Handle key release events"""
        if event.isAutoRepeat():
            super().keyReleaseEvent(event)
            return

        key = event.key()
        self.keys_pressed.discard(key)
        
        # We intentionally ignore W/S releases here to fix X11 SSH auto-repeat flutter.
        # It is handled automatically by the timeout in _update_ui.
            
        super().keyReleaseEvent(event)

    def _navigate_joint(self, direction):
        """Navigate 2D grid based on arrow keys"""
        # Grid structure matching the QGridLayout columns definition:
        # Left Arm -> Left Leg -> Head & Waist -> Right Leg -> Right Arm
        grid = [
            [14, 15, 16, 17, 18],     # Col 0: LEFT ARM
            [0, 1, 2, 3, 4, 5],        # Col 1: LEFT LEG
            [24, 25, 12, 13],          # Col 2: HEAD & WAIST
            [6, 7, 8, 9, 10, 11],      # Col 3: RIGHT LEG
            [19, 20, 21, 22, 23]       # Col 4: RIGHT ARM
        ]
        
        # Find current pos
        current_col = 0
        current_row = 0
        found = False
        
        for col_idx, col_joints in enumerate(grid):
            if self.selected_joint in col_joints:
                current_col = col_idx
                current_row = col_joints.index(self.selected_joint)
                found = True
                break
                
        if not found:
            return
            
        if direction == 'up':
            new_row = current_row - 1
            if new_row < 0:
                new_row = len(grid[current_col]) - 1
            new_joint = grid[current_col][new_row]
        elif direction == 'down':
            new_row = current_row + 1
            if new_row >= len(grid[current_col]):
                new_row = 0
            new_joint = grid[current_col][new_row]
        elif direction == 'left':
            new_col = current_col - 1
            if new_col < 0:
                new_col = len(grid) - 1
            new_row = min(current_row, len(grid[new_col]) - 1)
            new_joint = grid[new_col][new_row]
        elif direction == 'right':
            new_col = current_col + 1
            if new_col >= len(grid):
                new_col = 0
            new_row = min(current_row, len(grid[new_col]) - 1)
            new_joint = grid[new_col][new_row]
            
        self._on_joint_selected(new_joint)

    def _adjust_angle(self, joint_id, delta_deg):
        """Adjust joint angle by delta degrees"""
        if self.udp_client is None or not self.motors_enabled:
            return

        if self.udp_client.target_q is None:
            return

        lim_min, lim_max = SAFE_LIMITS_DEG[joint_id]
        current = math.degrees(self.udp_client.target_q[joint_id])
        new_val = current + delta_deg
        new_val = max(lim_min, min(lim_max, new_val))

        self.udp_client.set_target_angle(joint_id, math.radians(new_val))

    def _snap_target_to_actual(self, joint_id):
        """Snap target angle to the current physical angle to stop movement immediately"""
        if self.udp_client is None or not self.motors_enabled:
            return

        state = self.udp_client.get_state()
        if state is not None:
            sdk_idx = JOINT_IDX[joint_id]
            phys_q = state.motor_states[sdk_idx].q
            self.udp_client.set_target_angle(joint_id, phys_q)

    def _on_joint_selected(self, joint_id):
        """Handle joint selection"""
        self.selected_joint = joint_id
        self.joint_panel.select_joint(joint_id)
        self.control_panel.set_selected_joint(joint_id)
        self.statusbar.showMessage(f"Selected joint: {joint_id}")

    def _on_enable_motors(self):
        """Enable motors"""
        if self.udp_client is None:
            return

        if self.udp_client.enable_motors():
            self.motors_enabled = True
            self.control_panel.set_enabled_state(True)
            self.statusbar.showMessage("Motors enabled - posture locked")
            print("🟢 Motors Enabled!")

    def _on_disable_motors(self):
        """Disable motors (emergency stop)"""
        if self.udp_client:
            self.udp_client.disable_motors()
        self.motors_enabled = False
        self.control_panel.set_enabled_state(False)
        self.statusbar.showMessage("EMERGENCY STOP - Motors disabled")
        print("🛑 Emergency Stop!")

    def _on_toggle_monitor(self, enabled: bool):
        """Toggle monitor mode"""
        self.monitor_mode = enabled
        if enabled:
            self._on_disable_motors()  # Stop any active control safely
            self.statusbar.showMessage("MONITOR ONLY MODE - Control Disabled")
            self.monitor_toolbar_label.setText("MONITOR ONLY MODE")
            print("👁️  Monitor Mode ON!")
        else:
            self.statusbar.showMessage("Monitor Mode Disabled - Control Ready")
            self.monitor_toolbar_label.setText(" ")
            print("👁️  Monitor Mode OFF!")

        if self.udp_client:
            self.udp_client.set_monitor_mode(enabled)
        
        self.control_panel.set_monitor_mode_state(enabled)

    def _on_speed_change(self, speed):
        """Handle speed change"""
        self.speed_dps = speed
        self.control_panel.speed_dps = speed
        self.control_panel._update_speed_btns_style()

    def _on_step_change(self, step):
        """Handle step change"""
        self.step_deg = step
        self.control_panel.step_deg = step
        self.control_panel._update_step_btns_style()

    def _on_reset(self):
        """Reset to default position"""
        if self.udp_client and self.udp_client.target_q is not None:
            self.udp_client.target_q.fill(0.0)
            self.statusbar.showMessage("Reset to default position")

    def _toggle_fullscreen(self):
        """Toggle fullscreen mode"""
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def _show_about(self):
        """Show about dialog"""
        QMessageBox.about(
            self,
            "About R1 Joint Tuner",
            "<h3>R1 Professional Joint Tuner GUI</h3>"
            "<p>Version 1.0</p>"
            "<p>A professional GUI for tuning R1 robot joints.</p>"
            "<p>Built with PySide6</p>"
        )

    def eventFilter(self, obj, event):
        """Global event filter to capture Space key globally"""
        if event.type() == QEvent.KeyPress:
            if event.key() == Qt.Key_Space:
                self._on_disable_motors()
                return True
        return super().eventFilter(obj, event)

    def closeEvent(self, event):
        """Handle window close"""
        if self.udp_client:
            self.udp_client.disable_motors()
        event.accept()
