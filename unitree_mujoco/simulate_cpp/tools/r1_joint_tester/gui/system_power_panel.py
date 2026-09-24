"""System & Power Panel Widget"""
from .qt_compat import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QFrame, QFont, Qt, QSizePolicy
)
from .theme import Theme

class ValueBox(QFrame):
    def __init__(self, title, unit=""):
        super().__init__()
        self.unit = unit
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet(f"background-color: {Theme.BG_CARD.name()}; border: 1px solid {Theme.BDR_DEFAULT.name()}; border-radius: 4px;")
        layout = QVBoxLayout(self)
        
        lbl = QLabel(title)
        lbl.setFont(QFont(Theme.FONT_FAMILY, 9))
        lbl.setStyleSheet(f"color: {Theme.TXT_LABEL.name()}; border: none;")
        
        self.val = QLabel(f"-- {unit}")
        self.val.setFont(QFont(Theme.MONO_FONT, 14, QFont.Bold))
        self.val.setStyleSheet(f"color: {Theme.VAL_REAL.name()}; border: none;")
        self.val.setAlignment(Qt.AlignCenter)
        
        layout.addWidget(lbl)
        layout.addWidget(self.val)

    def set_value(self, val_str, color=None):
        self.val.setText(f"{val_str} {self.unit}")
        if color:
            self.val.setStyleSheet(f"color: {color}; border: none;")


class SystemPowerPanel(QWidget):
    def __init__(self):
        super().__init__()
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        # BMS Section
        lbl_bms = QLabel("Battery Management System (BMS)")
        lbl_bms.setFont(QFont(Theme.FONT_FAMILY, 14, QFont.Bold))
        layout.addWidget(lbl_bms)
        
        grid_bms = QGridLayout()
        self.soc_box = ValueBox("State of Charge", "%")
        self.vol_box = ValueBox("Voltage", "V")
        self.cur_box = ValueBox("Current", "A")
        self.temp_box = ValueBox("Temperature", "°C")
        
        grid_bms.addWidget(self.soc_box, 0, 0)
        grid_bms.addWidget(self.vol_box, 0, 1)
        grid_bms.addWidget(self.cur_box, 0, 2)
        grid_bms.addWidget(self.temp_box, 0, 3)
        layout.addLayout(grid_bms)
        
        # IMU Section
        lbl_imu = QLabel("Inertial Measurement Unit (IMU)")
        lbl_imu.setFont(QFont(Theme.FONT_FAMILY, 14, QFont.Bold))
        layout.addWidget(lbl_imu)
        
        grid_imu = QGridLayout()
        self.rpy_box = ValueBox("Roll / Pitch / Yaw", "deg")
        self.acc_box = ValueBox("Accel (x,y,z)", "g")
        self.quat_box = ValueBox("Quaternion", "")
        self.itemp_box = ValueBox("Board Temp", "°C")
        
        grid_imu.addWidget(self.rpy_box, 0, 0, 1, 2)
        grid_imu.addWidget(self.acc_box, 0, 2, 1, 2)
        grid_imu.addWidget(self.quat_box, 1, 0, 1, 3)
        grid_imu.addWidget(self.itemp_box, 1, 3, 1, 1)
        
        layout.addLayout(grid_imu)
        layout.addStretch()

    def update_state(self, state):
        # BMS
        c = Theme.VAL_REAL.name()
        if state.soc < 20: c = Theme.ALERT.name()
        self.soc_box.set_value(f"{state.soc}", c)
        self.vol_box.set_value(f"{state.bms_voltage:.1f}")
        self.cur_box.set_value(f"{state.bms_current:.1f}")
        self.temp_box.set_value(f"{state.bms_temp:.0f}")
        
        # IMU
        self.rpy_box.set_value(f"{state.imu_roll_deg:+.1f} / {state.imu_pitch_deg:+.1f} / {state.imu_yaw_deg:+.1f}")
        self.acc_box.set_value(f"{state.imu_accel[0]:+.1f}, {state.imu_accel[1]:+.1f}, {state.imu_accel[2]:+.1f}")
        q = getattr(state, 'imu_quat', [1,0,0,0])
        self.quat_box.set_value(f"[{q[0]:.2f}, {q[1]:.2f}, {q[2]:.2f}, {q[3]:.2f}]")
        self.itemp_box.set_value(f"{getattr(state, 'imu_temp', 0):.0f}")
