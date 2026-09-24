"""IMU Panel Widget - Displays IMU telemetry data"""
import math
from .qt_compat import Qt, QWidget, QVBoxLayout, QGridLayout, QLabel, QFrame, QFont

from .theme import Theme


class IMUPanel(QWidget):
    """IMU Telemetry Display Panel"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # Title
        title = QLabel("IMU TELEMETRY")
        title.setFont(QFont(Theme.FONT_FAMILY, 12, QFont.Bold))
        title.setStyleSheet(f"color: {Theme.VAL_TGT.name()}; background: transparent;")
        layout.addWidget(title)

        # IMU Frame
        imu_frame = QFrame()
        imu_frame.setObjectName("card")
        imu_frame.setStyleSheet(f"""
            QFrame#card {{
                background-color: {Theme.BG_CARD.name()};
                border: 1px solid {Theme.BDR_DEFAULT.name()};
                border-radius: 6px;
                padding: 10px;
            }}
        """)

        imu_layout = QGridLayout(imu_frame)
        imu_layout.setSpacing(8)

        # Row 0: RPY in degrees
        self.roll_val = QLabel("0.0°")
        self.pitch_val = QLabel("0.0°")
        self.yaw_val = QLabel("0.0°")

        imu_layout.addWidget(QLabel("Roll:"), 0, 0)
        self.roll_val.setFont(QFont(Theme.MONO_FONT, 11, QFont.Bold))
        self.roll_val.setStyleSheet(f"color: {Theme.VAL_REAL.name()};")
        imu_layout.addWidget(self.roll_val, 0, 1)

        imu_layout.addWidget(QLabel("Pitch:"), 0, 2)
        self.pitch_val.setFont(QFont(Theme.MONO_FONT, 11, QFont.Bold))
        self.pitch_val.setStyleSheet(f"color: {Theme.VAL_REAL.name()};")
        imu_layout.addWidget(self.pitch_val, 0, 3)

        imu_layout.addWidget(QLabel("Yaw:"), 0, 4)
        self.yaw_val.setFont(QFont(Theme.MONO_FONT, 11, QFont.Bold))
        self.yaw_val.setStyleSheet(f"color: {Theme.VAL_REAL.name()};")
        imu_layout.addWidget(self.yaw_val, 0, 5)

        # Row 1: RPY in radians
        self.roll_rad = QLabel("(0.00 rad)")
        self.pitch_rad = QLabel("(0.00 rad)")
        self.yaw_rad = QLabel("(0.00 rad)")

        for i, label in enumerate([self.roll_rad, self.pitch_rad, self.yaw_rad]):
            label.setFont(QFont(Theme.MONO_FONT, 9))
            label.setStyleSheet(f"color: {Theme.TXT_LABEL.name()};")

        imu_layout.addWidget(self.roll_rad, 1, 1)
        imu_layout.addWidget(self.pitch_rad, 1, 3)
        imu_layout.addWidget(self.yaw_rad, 1, 5)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"background-color: {Theme.BDR_DEFAULT.name()};")
        imu_layout.addWidget(sep, 2, 0, 1, 6)

        # Gyroscope section
        gyro_title = QLabel("GYROSCOPE (rad/s)")
        gyro_title.setFont(QFont(Theme.FONT_FAMILY, 9, QFont.Bold))
        gyro_title.setStyleSheet(f"color: {Theme.TXT_LABEL.name()};")
        imu_layout.addWidget(gyro_title, 3, 0, 1, 6)

        self.gyro_x = QLabel("X: 0.00")
        self.gyro_y = QLabel("Y: 0.00")
        self.gyro_z = QLabel("Z: 0.00")

        for i, label in enumerate([self.gyro_x, self.gyro_y, self.gyro_z]):
            label.setFont(QFont(Theme.MONO_FONT, 10))
            label.setStyleSheet(f"color: {Theme.VAL_TGT.name()};")

        imu_layout.addWidget(self.gyro_x, 4, 0)
        imu_layout.addWidget(self.gyro_y, 4, 2)
        imu_layout.addWidget(self.gyro_z, 4, 4)

        layout.addWidget(imu_frame)
        layout.addStretch()

    def update_imu(self, roll: float, pitch: float, yaw: float, gyro: list):
        """Update IMU values"""
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        self.roll_val.setText(f"{roll_deg:6.1f}°")
        self.pitch_val.setText(f"{pitch_deg:6.1f}°")
        self.yaw_val.setText(f"{yaw_deg:6.1f}°")

        self.roll_rad.setText(f"({roll:5.2f} rad)")
        self.pitch_rad.setText(f"({pitch:5.2f} rad)")
        self.yaw_rad.setText(f"({yaw:5.2f} rad)")

        if len(gyro) >= 3:
            self.gyro_x.setText(f"X: {gyro[0]:5.2f}")
            self.gyro_y.setText(f"Y: {gyro[1]:5.2f}")
            self.gyro_z.setText(f"Z: {gyro[2]:5.2f}")
