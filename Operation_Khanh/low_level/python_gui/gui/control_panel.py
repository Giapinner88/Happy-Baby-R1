from .qt_compat import (
    Qt, Signal, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QFrame, QSlider, QFont, QIcon
)

from .theme import Theme


class ControlPanel(QWidget):
    """Motor control panel with speed/step settings"""

    # Signals
    enableMotors = Signal()
    disableMotors = Signal()
    adjustAngle = Signal(int, float)  # joint_id, delta_deg
    setSpeed = Signal(float)
    setStep = Signal(float)
    toggleMonitor = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.selected_joint = 0
        self.speed_dps = 5.0
        self.speed_dps = 5.0
        self.step_deg = 3.0
        self.is_monitor_mode = False
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(12)

        # Title
        title = QLabel("MOTOR CONTROL")
        title.setFont(QFont(Theme.FONT_FAMILY, 12, QFont.Bold))
        title.setStyleSheet(f"color: {Theme.ACCENT.name()}; background: transparent;")
        layout.addWidget(title)

        # Main control buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(15)

        # Enable Button
        self.btn_enable = QPushButton("ENABLE (Enter/E)")
        self.btn_enable.setFont(QFont(Theme.FONT_FAMILY, 11, QFont.Bold))
        self.btn_enable.setFixedHeight(45)
        self.btn_enable.setStyleSheet(f"""
            QPushButton {{
                background-color: {Theme.ACCENT.name()};
                color: #0B0E14;
                border: 2px solid {Theme.ACCENT_DARK.name()};
                border-radius: 6px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: {Theme.ACCENT_DARK.name()};
            }}
            QPushButton:disabled {{
                background-color: {Theme.BG_BUTTON.name()};
                color: {Theme.TXT_DIM.name()};
                border-color: {Theme.BDR_DEFAULT.name()};
            }}
        """)
        self.btn_enable.clicked.connect(self._on_enable)
        btn_layout.addWidget(self.btn_enable)

        # Disable Button (Emergency Stop)
        self.btn_stop = QPushButton("STOP (Space)")
        self.btn_stop.setFont(QFont(Theme.FONT_FAMILY, 11, QFont.Bold))
        self.btn_stop.setFixedHeight(45)
        self.btn_stop.setStyleSheet(f"""
            QPushButton {{
                background-color: {Theme.ALERT.name()};
                color: white;
                border: 2px solid {Theme.ALERT_DARK.name()};
                border-radius: 6px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: {Theme.ALERT_DARK.name()};
            }}
        """)
        self.btn_stop.clicked.connect(self._on_disable)
        btn_layout.addWidget(self.btn_stop)

        layout.addLayout(btn_layout)

        # Monitor Mode Button
        self.btn_monitor = QPushButton("MONITOR MODE: OFF (M)")
        self.btn_monitor.setFont(QFont(Theme.FONT_FAMILY, 11, QFont.Bold))
        self.btn_monitor.setFixedHeight(45)
        self.btn_monitor.setStyleSheet(f"""
            QPushButton {{
                background-color: {Theme.BG_BUTTON.name()};
                color: {Theme.TXT_TITLE.name()};
                border: 2px solid {Theme.BDR_DEFAULT.name()};
                border-radius: 6px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: {Theme.BDR_HOVER.name()};
            }}
        """)
        self.btn_monitor.clicked.connect(self._on_toggle_monitor)
        layout.addWidget(self.btn_monitor)

        # Speed control
        speed_frame = QFrame()
        speed_frame.setObjectName("card")
        speed_frame.setStyleSheet(f"""
            QFrame#card {{
                background-color: {Theme.BG_CARD.name()};
                border: 1px solid {Theme.BDR_DEFAULT.name()};
                border-radius: 6px;
                padding: 10px;
            }}
        """)

        speed_layout = QVBoxLayout(speed_frame)
        speed_layout.setSpacing(8)

        speed_header = QLabel("CONTINUOUS SPEED (W/S keys, 1-4)")
        speed_header.setFont(QFont(Theme.FONT_FAMILY, 9, QFont.Bold))
        speed_header.setStyleSheet(f"color: {Theme.TXT_LABEL.name()};")
        speed_layout.addWidget(speed_header)

        speed_btn_layout = QHBoxLayout()
        speed_btn_layout.setSpacing(8)

        self.speed_btns = []
        speed_values = [3.0, 5.0, 10.0, 15.0]
        for i, val in enumerate(speed_values):
            btn = QPushButton(f"{val}°/s")
            btn.setFont(QFont(Theme.MONO_FONT, 9))
            btn.setFixedHeight(30)
            btn.setCheckable(True)
            btn.setChecked(val == self.speed_dps)
            btn.clicked.connect(lambda checked, v=val: self._on_speed_change(v))
            self.speed_btns.append(btn)
            speed_btn_layout.addWidget(btn)

        self._update_speed_btns_style()
        speed_layout.addLayout(speed_btn_layout)
        layout.addWidget(speed_frame)

        # Step control
        step_frame = QFrame()
        step_frame.setObjectName("card")
        step_frame.setStyleSheet(f"""
            QFrame#card {{
                background-color: {Theme.BG_CARD.name()};
                border: 1px solid {Theme.BDR_DEFAULT.name()};
                border-radius: 6px;
                padding: 10px;
            }}
        """)

        step_layout = QVBoxLayout(step_frame)
        step_layout.setSpacing(8)

        step_header = QLabel("STEP SIZE (A/D keys, 5-8)")
        step_header.setFont(QFont(Theme.FONT_FAMILY, 9, QFont.Bold))
        step_header.setStyleSheet(f"color: {Theme.TXT_LABEL.name()};")
        step_layout.addWidget(step_header)

        step_btn_layout = QHBoxLayout()
        step_btn_layout.setSpacing(8)

        self.step_btns = []
        step_values = [1.0, 2.0, 3.0, 4.0]
        for i, val in enumerate(step_values):
            btn = QPushButton(f"{val}°")
            btn.setFont(QFont(Theme.MONO_FONT, 9))
            btn.setFixedHeight(30)
            btn.setCheckable(True)
            btn.setChecked(val == self.step_deg)
            btn.clicked.connect(lambda checked, v=val: self._on_step_change(v))
            self.step_btns.append(btn)
            step_btn_layout.addWidget(btn)

        self._update_step_btns_style()
        step_layout.addLayout(step_btn_layout)
        layout.addWidget(step_frame)

        # Update button texts with hints
        speed_header.setText("CONTINUOUS (W/S) - 1-4: 3/5/10/15°/s")
        step_header.setText("STEP (A/D) - 5-8: 1/2/3/4°")

        layout.addStretch()

    def _update_speed_btns_style(self):
        """Update speed button styles"""
        for btn, val in zip(self.speed_btns, [3.0, 5.0, 10.0, 15.0]):
            if abs(val - self.speed_dps) < 0.1:
                btn.setStyleSheet(f"""
                    QPushButton {{
                        background-color: {Theme.VAL_REAL.name()};
                        color: {Theme.BG_MAIN.name()};
                        border: 1px solid {Theme.VAL_REAL.name()};
                        border-radius: 4px;
                        font-weight: bold;
                    }}
                """)
            else:
                btn.setStyleSheet(f"""
                    QPushButton {{
                        background-color: {Theme.BG_BUTTON.name()};
                        color: {Theme.TXT_TITLE.name()};
                        border: 1px solid {Theme.BDR_DEFAULT.name()};
                        border-radius: 4px;
                    }}
                    QPushButton:hover {{
                        background-color: {Theme.BDR_HOVER.name()};
                    }}
                """)

    def _update_step_btns_style(self):
        """Update step button styles"""
        for btn, val in zip(self.step_btns, [1.0, 2.0, 3.0, 4.0]):
            if abs(val - self.step_deg) < 0.1:
                btn.setStyleSheet(f"""
                    QPushButton {{
                        background-color: {Theme.VAL_REAL.name()};
                        color: {Theme.BG_MAIN.name()};
                        border: 1px solid {Theme.VAL_REAL.name()};
                        border-radius: 4px;
                        font-weight: bold;
                    }}
                """)
            else:
                btn.setStyleSheet(f"""
                    QPushButton {{
                        background-color: {Theme.BG_BUTTON.name()};
                        color: {Theme.TXT_TITLE.name()};
                        border: 1px solid {Theme.BDR_DEFAULT.name()};
                        border-radius: 4px;
                    }}
                    QPushButton:hover {{
                        background-color: {Theme.BDR_HOVER.name()};
                    }}
                """)

    def _on_enable(self):
        self.enableMotors.emit()

    def _on_disable(self):
        self.disableMotors.emit()

    def _on_toggle_monitor(self):
        self.is_monitor_mode = not self.is_monitor_mode
        self._update_monitor_btn_style()
        self.toggleMonitor.emit(self.is_monitor_mode)

    def _update_monitor_btn_style(self):
        if self.is_monitor_mode:
            self.btn_monitor.setText("MONITOR MODE: ON (M)")
            self.btn_monitor.setStyleSheet(f"""
                QPushButton {{
                    background-color: {Theme.VAL_WARNING.name()};
                    color: #0B0E14;
                    border: 2px solid {Theme.VAL_WARNING.name()};
                    border-radius: 6px;
                    font-weight: bold;
                }}
            """)
        else:
            self.btn_monitor.setText("MONITOR MODE: OFF (M)")
            self.btn_monitor.setStyleSheet(f"""
                QPushButton {{
                    background-color: {Theme.BG_BUTTON.name()};
                    color: {Theme.TXT_TITLE.name()};
                    border: 2px solid {Theme.BDR_DEFAULT.name()};
                    border-radius: 6px;
                    font-weight: bold;
                }}
                QPushButton:hover {{
                    background-color: {Theme.BDR_HOVER.name()};
                }}
            """)

    def _on_speed_change(self, val: float):
        self.speed_dps = val
        self._update_speed_btns_style()
        self.setSpeed.emit(val)

    def _on_step_change(self, val: float):
        self.step_deg = val
        self._update_step_btns_style()
        self.setStep.emit(val)

    def set_enabled_state(self, enabled: bool):
        """Update UI state based on motor enable/disable"""
        if enabled:
            self.btn_enable.setText("ENABLED (Enter/E)")
            self.btn_enable.setEnabled(False)
        else:
            self.btn_enable.setText("ENABLE (Enter/E)")
            # Only enable the button if NOT in monitor mode
            self.btn_enable.setEnabled(not self.is_monitor_mode)

    def set_monitor_mode_state(self, is_monitor: bool):
        """Update UI state based on monitor mode"""
        self.is_monitor_mode = is_monitor
        self._update_monitor_btn_style()
        if is_monitor:
            self.btn_enable.setEnabled(False)
            self.btn_enable.setText("LOCKED (Monitor Mode)")
        else:
            self.btn_enable.setEnabled(True)
            self.btn_enable.setText("ENABLE (Enter/E)")


    def set_selected_joint(self, joint_id: int):
        self.selected_joint = joint_id
