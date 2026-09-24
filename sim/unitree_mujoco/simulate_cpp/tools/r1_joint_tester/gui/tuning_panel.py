"""Tuning Panel Widget - Adjusts Kp and Kd"""
import math
from typing import Optional, Dict

from .qt_compat import (
    Qt, Signal, QSize, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QFrame, QSlider, QSizePolicy, QFont
)

from utils.joint_names import JOINT_NAMES, JOINT_IDX, JOINT_DISPLAY_NAMES
from .theme import Theme


class TuningCard(QFrame):
    """Card for tuning Kp and Kd of a single joint"""
    jointSelected = Signal(int)
    kpChanged = Signal(int, float)
    kdChanged = Signal(int, float)

    def __init__(self, joint_id: int, init_kp: float, init_kd: float, parent=None):
        super().__init__(parent)
        self.joint_id = joint_id
        self.is_selected = False
        self._setup_ui(init_kp, init_kd)
        self.setMinimumSize(200, 100)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def _setup_ui(self, init_kp, init_kd):
        self.setObjectName("card")
        self._update_card_style()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(4)

        # Header: Joint name
        self.name_label = QLabel(f"{JOINT_DISPLAY_NAMES.get(self.joint_id, JOINT_NAMES[self.joint_id])}")
        self.name_label.setFont(QFont(Theme.FONT_FAMILY, 9, QFont.Bold))
        self.name_label.setStyleSheet(f"color: {Theme.TXT_TITLE.name()}; background: transparent;")
        layout.addWidget(self.name_label)
        
        # Kp Row
        kp_layout = QHBoxLayout()
        kp_lbl = QLabel("Kp:")
        kp_lbl.setFont(QFont(Theme.FONT_FAMILY, 8, QFont.Bold))
        kp_lbl.setStyleSheet(f"color: {Theme.VAL_TGT.name()}; background: transparent;")
        
        self.kp_val = QLabel(f"{init_kp:.1f}")
        self.kp_val.setFixedWidth(35)
        self.kp_val.setFont(QFont(Theme.MONO_FONT, 8))
        self.kp_val.setStyleSheet(f"color: {Theme.TXT_TITLE.name()};")
        
        self.kp_slider = QSlider(Qt.Horizontal)
        self.kp_slider.setRange(0, 3000) # 0 to 300.0 (x10)
        self.kp_slider.setValue(int(init_kp * 10))
        self.kp_slider.valueChanged.connect(self._on_kp_slider_changed)
        
        kp_layout.addWidget(kp_lbl)
        kp_layout.addWidget(self.kp_slider)
        kp_layout.addWidget(self.kp_val)
        layout.addLayout(kp_layout)

        # Kd Row
        kd_layout = QHBoxLayout()
        kd_lbl = QLabel("Kd:")
        kd_lbl.setFont(QFont(Theme.FONT_FAMILY, 8, QFont.Bold))
        kd_lbl.setStyleSheet(f"color: {Theme.VAL_WARNING.name()}; background: transparent;")
        
        self.kd_val = QLabel(f"{init_kd:.2f}")
        self.kd_val.setFixedWidth(35)
        self.kd_val.setFont(QFont(Theme.MONO_FONT, 8))
        self.kd_val.setStyleSheet(f"color: {Theme.TXT_TITLE.name()};")
        
        self.kd_slider = QSlider(Qt.Horizontal)
        self.kd_slider.setRange(0, 200) # 0 to 20.0 (x10)
        self.kd_slider.setValue(int(init_kd * 10))
        self.kd_slider.valueChanged.connect(self._on_kd_slider_changed)
        
        kd_layout.addWidget(kd_lbl)
        kd_layout.addWidget(self.kd_slider)
        kd_layout.addWidget(self.kd_val)
        layout.addLayout(kd_layout)

        self.setMouseTracking(True)
        
    def _on_kp_slider_changed(self, val):
        real_val = val / 10.0
        self.kp_val.setText(f"{real_val:.1f}")
        self.kpChanged.emit(self.joint_id, real_val)
        
    def _on_kd_slider_changed(self, val):
        real_val = val / 10.0
        self.kd_val.setText(f"{real_val:.1f}")
        self.kdChanged.emit(self.joint_id, real_val)

    def _update_card_style(self):
        color = Theme.BDR_SELECTED if self.is_selected else Theme.BDR_DEFAULT
        width = 2 if self.is_selected else 1
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

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.jointSelected.emit(self.joint_id)
        super().mousePressEvent(event)


class TuningPanel(QWidget):
    """Panel containing Kp/Kd cards for all joints"""
    jointSelected = Signal(int)
    kpChanged = Signal(int, float)
    kdChanged = Signal(int, float)

    def __init__(self, init_kps: list, init_kds: list, parent=None):
        super().__init__(parent)
        self.joint_cards: Dict[int, TuningCard] = {}
        self.init_kps = init_kps
        self.init_kds = init_kds
        self._setup_ui()

    def _setup_ui(self):
        main_grid = QGridLayout(self)
        main_grid.setContentsMargins(5, 5, 5, 5)
        main_grid.setSpacing(8)

        columns_def = [
            ("LEFT ARM", [14, 15, 16, 17, 18]),
            ("LEFT LEG", [0, 1, 2, 3, 4, 5]),
            ("HEAD & WAIST", [24, 25, 12, 13]),
            ("RIGHT LEG", [6, 7, 8, 9, 10, 11]),
            ("RIGHT ARM", [19, 20, 21, 22, 23])
        ]

        for col_idx, (group_name, joint_ids) in enumerate(columns_def):
            title = QLabel(group_name)
            title.setFont(QFont(Theme.FONT_FAMILY, 10, QFont.Bold))
            title.setStyleSheet(f"color: {Theme.TXT_LABEL.name()}; background: transparent;")
            title.setFixedHeight(20)
            main_grid.addWidget(title, 0, col_idx, Qt.AlignHCenter)

            for row_idx, joint_id in enumerate(joint_ids):
                card = TuningCard(joint_id, self.init_kps[joint_id], self.init_kds[joint_id])
                card.jointSelected.connect(self.jointSelected.emit)
                card.kpChanged.connect(self.kpChanged.emit)
                card.kdChanged.connect(self.kdChanged.emit)
                self.joint_cards[joint_id] = card
                main_grid.addWidget(card, row_idx + 1, col_idx)

        for col_idx in range(5):
            main_grid.setColumnStretch(col_idx, 1)

    def select_joint(self, joint_id: int):
        for card in self.joint_cards.values():
            card.set_selected(False)
        if joint_id in self.joint_cards:
            self.joint_cards[joint_id].set_selected(True)
