"""3D Robot View Widget - Visual representation of R1 robot"""
import math

from .qt_compat import Qt, QTimer, QWidget, QVBoxLayout, QLabel, QPainter, QPen, QFont

from .theme import Theme
from utils.joint_names import JOINT_DISPLAY_NAMES


class RobotCanvas(QWidget):
    """Custom widget for robot visualization"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.joint_angles = [0.0] * 26
        self.target_angles = [0.0] * 26
        self.selected_joint = 24
        self.setMinimumSize(280, 320)
        self.setStyleSheet(f"""
            background-color: {Theme.BG_CARD.name()};
            border: 1px solid {Theme.BDR_DEFAULT.name()};
            border-radius: 6px;
        """)

    def set_joint_angles(self, angles: list):
        if len(angles) >= 26:
            self.joint_angles = angles[:26]
            self.update()

    def set_target_angles(self, angles: list):
        if len(angles) >= 26:
            self.target_angles = angles[:26]
            self.update()

    def set_selected_joint(self, joint_id: int):
        self.selected_joint = joint_id
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        if not painter.isActive():
            return

        painter.setRenderHint(QPainter.Antialiasing)
        cx = self.width() / 2
        cy = self.height() / 2 + 10
        scale = 1.4

        pen = QPen(Theme.TXT_TITLE, 2)
        painter.setPen(pen)

        angle_scale = 30 * scale

        # Torso
        torso_top = cy - 80 * scale
        torso_bottom = cy - 20 * scale
        torso_width = 40 * scale

        def get_angle(i):
            return self.joint_angles[i] if i < len(self.joint_angles) else 0.0

        painter.drawRect(int(cx - torso_width/2), int(torso_top),
                        int(torso_width), int(torso_bottom - torso_top))
        painter.setFont(QFont(Theme.FONT_FAMILY, 8))
        painter.drawText(int(cx - 15), int(torso_top - 5), "TORSO")

        # Waist
        painter.drawLine(int(cx), int(torso_bottom), int(cx), int(cy + 20))

        # Head
        head_pitch = get_angle(24)
        head_yaw = get_angle(25)
        head_cx = cx + int(head_yaw * angle_scale * 0.3)
        head_cy = torso_top - 25 * scale - int(head_pitch * angle_scale * 0.3)
        head_radius = 15 * scale
        painter.drawEllipse(int(head_cx - head_radius), int(head_cy - head_radius),
                           int(head_radius * 2), int(head_radius * 2))
        painter.drawText(int(head_cx - 15), int(head_cy - head_radius - 5), "HEAD")

        # Left Arm
        l_shoulder_pitch = get_angle(14)
        l_shoulder_roll = get_angle(15)
        l_elbow = get_angle(17)
        l_wrist = get_angle(18)
        l_shoulder_x = cx - torso_width/2 - 5
        l_shoulder_y = torso_top + 10
        l_elbow_x = l_shoulder_x + int(l_shoulder_roll * angle_scale * 0.5)
        l_elbow_y = l_shoulder_y + 40 * scale + int(l_shoulder_pitch * angle_scale * 0.3)
        painter.drawLine(int(l_shoulder_x), int(l_shoulder_y), int(l_elbow_x), int(l_elbow_y))
        l_wrist_x = l_elbow_x + int(l_shoulder_roll * angle_scale * 0.3)
        l_wrist_y = l_elbow_y + 35 * scale + int(l_elbow * angle_scale * 0.3)
        painter.drawLine(int(l_elbow_x), int(l_elbow_y), int(l_wrist_x), int(l_wrist_y))

        # Left Leg
        l_hip_pitch = get_angle(0)
        l_hip_roll = get_angle(1)
        l_knee = get_angle(3)
        l_hip_x = cx - 15
        l_hip_y = torso_bottom
        l_knee_x = l_hip_x + int(l_hip_roll * angle_scale * 0.4)
        l_knee_y = l_hip_y + 45 * scale + int(l_hip_pitch * angle_scale * 0.4)
        painter.drawLine(int(l_hip_x), int(l_hip_y), int(l_knee_x), int(l_knee_y))
        l_ankle_x = l_knee_x + int(l_hip_roll * angle_scale * 0.2)
        l_ankle_y = l_knee_y + 40 * scale + int(l_knee * angle_scale * 0.4)
        painter.drawLine(int(l_knee_x), int(l_knee_y), int(l_ankle_x), int(l_ankle_y))

        # Right Arm
        r_shoulder_pitch = get_angle(19)
        r_shoulder_roll = get_angle(20)
        r_shoulder_x = cx + torso_width/2 + 5
        r_shoulder_y = torso_top + 10
        r_elbow_x = r_shoulder_x + int(r_shoulder_roll * angle_scale * 0.5)
        r_elbow_y = r_shoulder_y + 40 * scale + int(r_shoulder_pitch * angle_scale * 0.3)
        painter.drawLine(int(r_shoulder_x), int(r_shoulder_y), int(r_elbow_x), int(r_elbow_y))
        r_wrist_x = r_elbow_x + int(r_shoulder_roll * angle_scale * 0.3)
        r_wrist_y = r_elbow_y + 35 * scale + int(l_elbow * angle_scale * 0.3)
        painter.drawLine(int(r_elbow_x), int(r_elbow_y), int(r_wrist_x), int(r_wrist_y))

        # Right Leg
        r_hip_pitch = get_angle(6)
        r_hip_roll = get_angle(7)
        r_knee = get_angle(9)
        r_hip_x = cx + 15
        r_hip_y = torso_bottom
        r_knee_x = r_hip_x + int(r_hip_roll * angle_scale * 0.4)
        r_knee_y = r_hip_y + 45 * scale + int(r_hip_pitch * angle_scale * 0.4)
        painter.drawLine(int(r_hip_x), int(r_hip_y), int(r_knee_x), int(r_knee_y))
        r_ankle_x = r_knee_x + int(r_hip_roll * angle_scale * 0.2)
        r_ankle_y = r_knee_y + 40 * scale + int(r_knee * angle_scale * 0.4)
        painter.drawLine(int(r_knee_x), int(r_knee_y), int(r_ankle_x), int(r_ankle_y))

        # Selected joint marker
        joint_positions = {
            0: (l_hip_x, l_hip_y), 1: (l_hip_x, l_hip_y), 2: (l_hip_x, l_hip_y),
            3: (l_knee_x, l_knee_y), 4: (l_ankle_x, l_ankle_y), 5: (l_ankle_x, l_ankle_y),
            6: (r_hip_x, r_hip_y), 7: (r_hip_x, r_hip_y), 8: (r_hip_x, r_hip_y),
            9: (r_knee_x, r_knee_y), 10: (r_ankle_x, r_ankle_y), 11: (r_ankle_x, r_ankle_y),
            12: (cx, torso_bottom), 13: (cx, torso_bottom),
            14: (l_shoulder_x, l_shoulder_y), 15: (l_shoulder_x, l_shoulder_y),
            16: (l_shoulder_x, l_shoulder_y), 17: (l_elbow_x, l_elbow_y),
            18: (l_wrist_x, l_wrist_y),
            19: (r_shoulder_x, r_shoulder_y), 20: (r_shoulder_x, r_shoulder_y),
            21: (r_shoulder_x, r_shoulder_y), 22: (r_elbow_x, r_elbow_y),
            23: (r_wrist_x, r_wrist_y),
            24: (head_cx, head_cy), 25: (head_cx, head_cy),
        }

        if self.selected_joint in joint_positions:
            painter.setPen(QPen(Theme.BDR_SELECTED, 3))
            px, py = joint_positions[self.selected_joint]
            painter.drawEllipse(int(px - 6), int(py - 6), 12, 12)


class Robot3DView(QWidget):
    """Simple 2D robot visualization using QPainter"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.joint_angles = [0.0] * 26
        self.target_angles = [0.0] * 26
        self._selected_joint = 24
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        title = QLabel("ROBOT VIEW")
        title.setFont(QFont(Theme.FONT_FAMILY, 12, QFont.Bold))
        title.setStyleSheet(f"color: {Theme.VAL_TGT.name()}; background: transparent;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        self.canvas = RobotCanvas()
        layout.addWidget(self.canvas, 1)

        self.joint_info = QLabel("No joint selected")
        self.joint_info.setFont(QFont(Theme.MONO_FONT, 9))
        self.joint_info.setStyleSheet(f"color: {Theme.TXT_LABEL.name()};")
        self.joint_info.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.joint_info)

    def set_joint_angles(self, angles: list):
        if len(angles) >= 26:
            self.joint_angles = angles[:26]
        self.canvas.set_joint_angles(self.joint_angles)

    def set_target_angles(self, angles: list):
        if len(angles) >= 26:
            self.target_angles = angles[:26]
        self.canvas.set_target_angles(self.target_angles)

    def set_selected_joint(self, joint_id: int):
        self._selected_joint = joint_id
        self.canvas.set_selected_joint(joint_id)
        name = JOINT_DISPLAY_NAMES.get(joint_id, f"Joint {joint_id}")
        real = math.degrees(self.joint_angles[joint_id]) if joint_id < len(self.joint_angles) else 0
        tgt = math.degrees(self.target_angles[joint_id]) if joint_id < len(self.target_angles) else 0
        self.joint_info.setText(f"{name}: Real={real:+.1f}° | Tgt={tgt:+.1f}°")
