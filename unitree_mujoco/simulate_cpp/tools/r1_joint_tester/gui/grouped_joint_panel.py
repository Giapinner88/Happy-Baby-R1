"""Grouped Joint Panel Widget - Displays joint cards split into body parts"""
import math
from typing import Optional, Dict, List

from .qt_compat import (
    Qt, Signal, QSize, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QFrame, QScrollArea, QSizePolicy, QFont, QPainter, QColor, QPen
)

from utils.joint_names import JOINT_NAMES, JOINT_IDX, JOINT_DISPLAY_NAMES
from utils.safe_limits import SAFE_LIMITS_DEG, get_temp_status
from .theme import Theme


class DiagnosticJointCard(QFrame):
    """Detailed joint card widget with full telemetry"""
    jointSelected = Signal(int)

    def __init__(self, joint_id: int, parent=None):
        super().__init__(parent)
        self.joint_id = joint_id
        self.is_selected = False
        self.temp_status = "normal"
        self.at_limit = False
        self._setup_ui()
        self.setMinimumSize(180, 160)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def _setup_ui(self):
        self.setObjectName("card")
        self._update_card_style()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(2)

        # Header: Joint name & Mode
        hdr_layout = QHBoxLayout()
        self.name_label = QLabel(f"{JOINT_DISPLAY_NAMES.get(self.joint_id, JOINT_NAMES[self.joint_id])}")
        self.name_label.setFont(QFont(Theme.FONT_FAMILY, 10, QFont.Bold))
        self.name_label.setStyleSheet(f"color: {Theme.TXT_TITLE.name()}; background: transparent;")
        
        self.mode_val = QLabel("M:0")
        self.mode_val.setFont(QFont(Theme.MONO_FONT, 8, QFont.Bold))
        self.mode_val.setStyleSheet(f"color: {Theme.TXT_LABEL.name()}; background: transparent;")
        
        hdr_layout.addWidget(self.name_label)
        hdr_layout.addStretch()
        hdr_layout.addWidget(self.mode_val)
        layout.addLayout(hdr_layout)
        
        # Separator
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet(f"background-color: {Theme.BDR_DEFAULT.name()};")
        layout.addWidget(line)

        # Data Rows
        self.val_labels = {}
        
        def add_row(key, lbl_text, default_val, val_color=Theme.TXT_TITLE.name()):
            row = QHBoxLayout()
            lbl = QLabel(lbl_text)
            lbl.setFont(QFont(Theme.FONT_FAMILY, 8))
            lbl.setStyleSheet(f"color: {Theme.TXT_LABEL.name()}; background: transparent;")
            val = QLabel(default_val)
            val.setFont(QFont(Theme.MONO_FONT, 9, QFont.Bold))
            val.setStyleSheet(f"color: {val_color}; background: transparent;")
            row.addWidget(lbl)
            row.addStretch()
            row.addWidget(val)
            layout.addLayout(row)
            self.val_labels[key] = val

        add_row('q_real', 'Actual Angle:', '0.0°', Theme.VAL_REAL.name())
        add_row('q_tgt', 'Target Angle:', '0.0°', Theme.VAL_TGT.name())
        add_row('dq', 'Velocity:', '0.0 rad/s')
        add_row('ddq', 'Accel:', '0.0 r/s²')
        add_row('tau', 'Torque:', '0.0 Nm', Theme.VAL_WARNING.name())
        add_row('vol', 'Voltage:', '0.0 V')
        add_row('temp', 'Temp (C/I):', '0° / 0°')

        self.setMouseTracking(True)

    def _update_card_style(self):
        color = Theme.BDR_SELECTED if self.is_selected else Theme.BDR_DEFAULT
        width = 2 if self.is_selected else 1

        if getattr(self, 'at_limit', False):
            color = Theme.ALERT
            width = 3 if self.is_selected else 2
        elif self.temp_status == "critical":
            color = Theme.ALERT
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

    def update_values(self, motor_state, tgt_deg: float):
        """Update displayed values from full MotorState"""
        real_deg = math.degrees(motor_state.q)
        
        self.val_labels['q_real'].setText(f"{real_deg:+.1f}°")
        self.val_labels['q_tgt'].setText(f"{tgt_deg:+.1f}°")
        self.val_labels['dq'].setText(f"{motor_state.dq:+.1f}")
        
        # Safely get new fields, fallback to 0 if not present
        ddq = getattr(motor_state, 'ddq', 0.0)
        vol = getattr(motor_state, 'vol', 0.0)
        t_inv = getattr(motor_state, 'temp_inv', 0.0)
        mode = getattr(motor_state, 'mode', 0)
        
        self.val_labels['ddq'].setText(f"{ddq:+.0f}")
        self.val_labels['tau'].setText(f"{motor_state.tau:+.1f} Nm")
        self.val_labels['vol'].setText(f"{vol:+.1f} V")
        self.val_labels['temp'].setText(f"{motor_state.temp:.0f}° / {t_inv:.0f}°")
        self.mode_val.setText(f"M:{mode}")

        # Check limits
        lim_min, lim_max = SAFE_LIMITS_DEG[self.joint_id]
        at_limit = (real_deg <= lim_min + 0.15) or (real_deg >= lim_max - 0.15)
        if at_limit != getattr(self, 'at_limit', False):
            self.at_limit = at_limit
            self._update_card_style()

        # Temp status (use coil temp)
        status = get_temp_status(motor_state.temp)
        if status != self.temp_status:
            self.temp_status = status
            self._update_card_style()

        if status == "critical":
            temp_color = Theme.ALERT.name()
        elif status == "warning":
            temp_color = Theme.VAL_WARNING.name()
        else:
            temp_color = Theme.TXT_TITLE.name()

        self.val_labels['temp'].setStyleSheet(f"color: {temp_color}; background: transparent;")

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.jointSelected.emit(self.joint_id)
        super().mousePressEvent(event)


class GroupedJointPanel(QWidget):
    """Panel containing a specific subset of joint cards"""
    jointSelected = Signal(int)

    def __init__(self, title: str, joint_columns: List[tuple], parent=None):
        super().__init__(parent)
        self.joint_cards: Dict[int, DiagnosticJointCard] = {}
        self._selecting = False
        self.title = title
        self.joint_columns = joint_columns
        self._setup_ui()

    def _setup_ui(self):
        main_grid = QGridLayout(self)
        main_grid.setContentsMargins(10, 10, 10, 10)
        main_grid.setSpacing(10)

        for col_idx, (group_name, joint_ids) in enumerate(self.joint_columns):
            title = QLabel(group_name)
            title.setFont(QFont(Theme.FONT_FAMILY, 12, QFont.Bold))
            title.setStyleSheet(f"color: {Theme.TXT_LABEL.name()}; background: transparent;")
            title.setFixedHeight(30)
            main_grid.addWidget(title, 0, col_idx, Qt.AlignHCenter)

            for row_idx, joint_id in enumerate(joint_ids):
                card = DiagnosticJointCard(joint_id)
                card.jointSelected.connect(self._on_joint_selected)
                self.joint_cards[joint_id] = card
                main_grid.addWidget(card, row_idx + 1, col_idx)
                
            main_grid.setColumnStretch(col_idx, 1)

    def _on_joint_selected(self, joint_id: int):
        if not hasattr(self, '_selecting') or not self._selecting:
            self._selecting = True
            try:
                for card in self.joint_cards.values():
                    card.set_selected(False)
                if joint_id in self.joint_cards:
                    self.joint_cards[joint_id].set_selected(True)
                self.jointSelected.emit(joint_id)
            finally:
                self._selecting = False

    def select_joint(self, joint_id: int):
        self._on_joint_selected(joint_id)

    def deselect_all(self):
        for card in self.joint_cards.values():
            card.set_selected(False)

    def update_all_joints(self, motor_states, target_q: Optional[list]):
        for joint_id, card in self.joint_cards.items():
            sdk_idx = JOINT_IDX[joint_id]
            ms = motor_states[sdk_idx]
            tgt_deg = math.degrees(target_q[joint_id]) if target_q is not None else 0.0
            card.update_values(ms, tgt_deg)
