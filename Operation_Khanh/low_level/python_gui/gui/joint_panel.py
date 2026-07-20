"""Joint Panel Widget - Displays 26 joint cards"""
import math
from typing import Optional, Dict, List

from .qt_compat import (
    Qt, Signal, QSize, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QFrame, QScrollArea, QSizePolicy, QFont, QPainter, QColor, QPen
)

from utils.joint_names import JOINT_NAMES, JOINT_IDX, JOINT_GROUPS, JOINT_DISPLAY_NAMES
from utils.safe_limits import SAFE_LIMITS_DEG, get_temp_status
from .theme import Theme


class JointCard(QFrame):
    """Single joint card widget"""
    jointSelected = Signal(int)

    def __init__(self, joint_id: int, parent=None):
        super().__init__(parent)
        self.joint_id = joint_id
        self.is_selected = False
        self.temp_status = "normal"
        self.at_limit = False
        self._setup_ui()
        self.setMinimumSize(110, 80)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def _setup_ui(self):
        self.setObjectName("card")
        self._update_card_style()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(2)

        # Header: Joint name
        self.name_label = QLabel(f"{JOINT_DISPLAY_NAMES.get(self.joint_id, JOINT_NAMES[self.joint_id])}")
        self.name_label.setFont(QFont(Theme.FONT_FAMILY, 9, QFont.Bold))
        self.name_label.setStyleSheet(f"color: {Theme.TXT_TITLE.name()}; background: transparent;")
        layout.addWidget(self.name_label)

        # Actual Angle Row
        curr_layout = QHBoxLayout()
        self.real_lbl = QLabel("Actual Angle:")
        self.real_lbl.setFont(QFont(Theme.FONT_FAMILY, 8))
        self.real_lbl.setStyleSheet(f"color: {Theme.TXT_LABEL.name()}; background: transparent;")
        self.real_val = QLabel("0.0°")
        self.real_val.setFont(QFont(Theme.MONO_FONT, 9, QFont.Bold))
        self.real_val.setStyleSheet(f"color: {Theme.VAL_REAL.name()}; background: transparent;")
        curr_layout.addWidget(self.real_lbl)
        curr_layout.addStretch()
        curr_layout.addWidget(self.real_val)
        layout.addLayout(curr_layout)

        # Target Angle Row
        tgt_layout = QHBoxLayout()
        self.tgt_lbl = QLabel("Target Angle:")
        self.tgt_lbl.setFont(QFont(Theme.FONT_FAMILY, 8))
        self.tgt_lbl.setStyleSheet(f"color: {Theme.TXT_LABEL.name()}; background: transparent;")
        self.tgt_val = QLabel("0.0°")
        self.tgt_val.setFont(QFont(Theme.MONO_FONT, 9, QFont.Bold))
        self.tgt_val.setStyleSheet(f"color: {Theme.VAL_TGT.name()}; background: transparent;")
        tgt_layout.addWidget(self.tgt_lbl)
        tgt_layout.addStretch()
        tgt_layout.addWidget(self.tgt_val)
        layout.addLayout(tgt_layout)

        # Velocity Row
        vel_layout = QHBoxLayout()
        self.vel_lbl = QLabel("Velocity:")
        self.vel_lbl.setFont(QFont(Theme.FONT_FAMILY, 8))
        self.vel_lbl.setStyleSheet(f"color: {Theme.TXT_LABEL.name()}; background: transparent;")
        self.vel_val = QLabel("0.00 rad/s")
        self.vel_val.setFont(QFont(Theme.MONO_FONT, 8))
        self.vel_val.setStyleSheet(f"color: {Theme.TXT_TITLE.name()}; background: transparent;")
        vel_layout.addWidget(self.vel_lbl)
        vel_layout.addStretch()
        vel_layout.addWidget(self.vel_val)
        layout.addLayout(vel_layout)

        # Torque Row
        trq_layout = QHBoxLayout()
        self.trq_lbl = QLabel("Torque:")
        self.trq_lbl.setFont(QFont(Theme.FONT_FAMILY, 8))
        self.trq_lbl.setStyleSheet(f"color: {Theme.TXT_LABEL.name()}; background: transparent;")
        self.trq_val = QLabel("0.00 Nm")
        self.trq_val.setFont(QFont(Theme.MONO_FONT, 8))
        self.trq_val.setStyleSheet(f"color: {Theme.TXT_TITLE.name()}; background: transparent;")
        trq_layout.addWidget(self.trq_lbl)
        trq_layout.addStretch()
        trq_layout.addWidget(self.trq_val)
        layout.addLayout(trq_layout)

        # Temperature Row
        temp_layout = QHBoxLayout()
        self.temp_lbl = QLabel("Temperature:")
        self.temp_lbl.setFont(QFont(Theme.FONT_FAMILY, 8))
        self.temp_lbl.setStyleSheet(f"color: {Theme.TXT_LABEL.name()}; background: transparent;")
        self.temp_val = QLabel("0°C")
        self.temp_val.setFont(QFont(Theme.MONO_FONT, 8))
        self.temp_val.setStyleSheet(f"color: {Theme.TXT_TITLE.name()}; background: transparent;")
        temp_layout.addWidget(self.temp_lbl)
        temp_layout.addStretch()
        temp_layout.addWidget(self.temp_val)
        layout.addLayout(temp_layout)

        # Mouse events
        self.setMouseTracking(True)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        h = self.height()
        
        # Calculate dynamic proportional font sizes based on card height
        title_size = max(8, min(14, int(h * 0.08)))
        label_size = max(7, min(12, int(h * 0.07)))
        value_size = max(7, min(12, int(h * 0.075)))

        self.name_label.setFont(QFont(Theme.FONT_FAMILY, title_size, QFont.Bold))
        for lbl_name in ['real_lbl', 'tgt_lbl', 'vel_lbl', 'trq_lbl', 'temp_lbl']:
            if hasattr(self, lbl_name):
                getattr(self, lbl_name).setFont(QFont(Theme.FONT_FAMILY, label_size))
        
        self.real_val.setFont(QFont(Theme.MONO_FONT, value_size, QFont.Bold))
        self.tgt_val.setFont(QFont(Theme.MONO_FONT, value_size, QFont.Bold))
        self.vel_val.setFont(QFont(Theme.MONO_FONT, value_size))
        self.trq_val.setFont(QFont(Theme.MONO_FONT, value_size))
        self.temp_val.setFont(QFont(Theme.MONO_FONT, value_size))

    def _update_card_style(self):
        import time
        color = Theme.BDR_SELECTED if self.is_selected else Theme.BDR_DEFAULT
        width = 2 if self.is_selected else 1

        if getattr(self, 'at_limit', False):
            color = Theme.ALERT
            width = 3 if self.is_selected else 2
        elif self.temp_status == "critical":
            # Hiệu ứng nháy viền đỏ cảnh báo quá nhiệt (nháy mỗi 0.5s)
            if int(time.time() * 2) % 2 == 0:
                color = Theme.ALERT
                width = 3
            else:
                color = Theme.BDR_DEFAULT
                width = 2
        elif self.temp_status == "warning":
            color = Theme.VAL_WARNING
            width = 2

        self.setStyleSheet(f"""
            QFrame#card {{
                background-color: {Theme.BG_CARD.name()};
                border: {width}px solid {color.name()};
                border-radius: 6px;
            }}
        """)

    def set_selected(self, selected: bool):
        self.is_selected = selected
        self._update_card_style()

    def update_values(self, real_deg: float, tgt_deg: float, vel: float, torque: float, temp: float):
        """Update displayed values"""
        self.real_val.setText(f"{real_deg:+.1f}°")
        self.tgt_val.setText(f"{tgt_deg:+.1f}°")
        self.vel_val.setText(f"{vel:+.2f} rad/s")
        self.trq_val.setText(f"{torque:+.2f} Nm")
        self.temp_val.setText(f"{temp:.0f}°C")

        # Check if joint is at or beyond its safe limit
        lim_min, lim_max = SAFE_LIMITS_DEG[self.joint_id]
        at_limit = (real_deg <= lim_min + 0.15) or (real_deg >= lim_max - 0.15)
        if at_limit != getattr(self, 'at_limit', False):
            self.at_limit = at_limit
            self._update_card_style()

        # Temperature status
        status = get_temp_status(temp)
        if status != self.temp_status or status == "critical":
            self.temp_status = status
            self._update_card_style()

        if status == "critical":
            temp_color = Theme.ALERT.name()
        elif status == "warning":
            temp_color = Theme.VAL_WARNING.name()
        else:
            temp_color = Theme.TXT_TITLE.name()

        self.temp_val.setStyleSheet(f"color: {temp_color}; background: transparent;")

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.jointSelected.emit(self.joint_id)
        super().mousePressEvent(event)


class JointPanel(QWidget):
    """Panel containing all 26 joint cards organized by groups in a grid layout"""
    jointSelected = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.joint_cards: Dict[int, JointCard] = {}
        self._selecting = False
        self._setup_ui()

    def _setup_ui(self):
        main_grid = QGridLayout(self)
        main_grid.setContentsMargins(5, 5, 5, 5)
        main_grid.setSpacing(8)

        # Columns definition matching humanoid topology:
        # Left Arm -> Left Leg -> Head & Waist -> Right Leg -> Right Arm
        columns_def = [
            ("LEFT ARM", [14, 15, 16, 17, 18]),
            ("LEFT LEG", [0, 1, 2, 3, 4, 5]),
            ("HEAD & WAIST", [24, 25, 12, 13]),
            ("RIGHT LEG", [6, 7, 8, 9, 10, 11]),
            ("RIGHT ARM", [19, 20, 21, 22, 23])
        ]

        for col_idx, (group_name, joint_ids) in enumerate(columns_def):
            # Center group title
            title = QLabel(group_name)
            title.setFont(QFont(Theme.FONT_FAMILY, 10, QFont.Bold))
            title.setStyleSheet(f"color: {Theme.TXT_LABEL.name()}; background: transparent;")
            title.setFixedHeight(20)
            main_grid.addWidget(title, 0, col_idx, Qt.AlignHCenter)

            for row_idx, joint_id in enumerate(joint_ids):
                card = JointCard(joint_id)
                card.jointSelected.connect(self._on_joint_selected)
                self.joint_cards[joint_id] = card
                # Place card starting from row 1
                main_grid.addWidget(card, row_idx + 1, col_idx)

        # Set column and row stretches to make them scale/distribute evenly
        for col_idx in range(5):
            main_grid.setColumnStretch(col_idx, 1)

        # Stretch rows 1 to 6 (the ones containing the cards) equally
        for row_idx in range(1, 7):
            main_grid.setRowStretch(row_idx, 1)

    def _on_joint_selected(self, joint_id: int):
        """Handle joint selection"""
        if not hasattr(self, '_selecting') or not self._selecting:
            self._selecting = True
            try:
                # Deselect all
                for card in self.joint_cards.values():
                    card.set_selected(False)
                # Select new
                if joint_id in self.joint_cards:
                    self.joint_cards[joint_id].set_selected(True)
                self.jointSelected.emit(joint_id)
            finally:
                self._selecting = False

    def select_joint(self, joint_id: int):
        """Programmatically select a joint"""
        self._on_joint_selected(joint_id)

    def update_joint(self, joint_id: int, real_deg: float, tgt_deg: float, vel: float, torque: float, temp: float):
        """Update a single joint's values"""
        if joint_id in self.joint_cards:
            self.joint_cards[joint_id].update_values(real_deg, tgt_deg, vel, torque, temp)

    def update_all_joints(self, motor_states, target_q: Optional[list], joint_indices: list = None):
        """Update all joints from robot state"""
        if joint_indices is None:
            joint_indices = list(range(26))

        for i in joint_indices:
            sdk_idx = JOINT_IDX[i]
            real_deg = math.degrees(motor_states[sdk_idx].q)
            tgt_deg = math.degrees(target_q[i]) if target_q is not None else 0.0
            vel = motor_states[sdk_idx].dq
            torque = motor_states[sdk_idx].tau
            temp = motor_states[sdk_idx].temp
            self.update_joint(i, real_deg, tgt_deg, vel, torque, temp)
