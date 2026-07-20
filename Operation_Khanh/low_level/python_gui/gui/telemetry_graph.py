"""Telemetry Graph Widget - Live graphs for telemetry data"""
from collections import deque
from typing import Deque

from .qt_compat import Qt, QTimer, QWidget, QVBoxLayout, QLabel, QGridLayout, QFrame, QFont, QPainter, QColor, QPen
from .theme import Theme
from utils.joint_names import JOINT_IDX

class GraphWidget(QFrame):
    """Simple real-time graph widget using QPainter"""

    def __init__(self, title: str, max_points: int = 300, y_min: float = -2, y_max: float = 2, parent=None):
        super().__init__(parent)
        self.title = title
        self.max_points = max_points
        self.y_min = y_min
        self.y_max = y_max
        self.data: Deque[float] = deque(maxlen=max_points)
        self.data2: Deque[float] = deque(maxlen=max_points)  # For secondary line
        self.show_secondary = False
        self.yellow_threshold = None
        self.red_threshold = None

        self._setup_ui()

    def _setup_ui(self):
        self.setObjectName("card")
        self.setStyleSheet(f"""
            QFrame#card {{
                background-color: {Theme.BG_CARD.name()};
                border: 1px solid {Theme.BDR_DEFAULT.name()};
                border-radius: 6px;
            }}
        """)
        self.setMinimumHeight(100)

    def set_secondary(self, show: bool):
        """Enable secondary line"""
        self.show_secondary = show

    def set_thresholds(self, yellow: float = None, red: float = None):
        """Set warning thresholds"""
        self.yellow_threshold = yellow
        self.red_threshold = red

    def clear(self):
        """Clear all data"""
        self.data.clear()
        self.data2.clear()

    def paintEvent(self, event):
        """Custom paint event for graph"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Background
        painter.fillRect(self.rect(), Theme.BG_CARD)

        # Margins
        margin_left = 10
        margin_right = 10
        margin_top = 25
        margin_bottom = 20

        graph_rect = self.rect().adjusted(margin_left, margin_top, -margin_right, -margin_bottom)

        # Draw grid
        painter.setPen(QPen(Theme.BDR_DEFAULT, 0.5))
        for i in range(5):
            y = graph_rect.top() + i * graph_rect.height() / 4
            painter.drawLine(graph_rect.left(), y, graph_rect.right(), y)

        # Draw threshold lines
        if self.yellow_threshold is not None:
            y_yellow = graph_rect.top() + (1 - (self.yellow_threshold - self.y_min) / (self.y_max - self.y_min)) * graph_rect.height()
            painter.setPen(QPen(Theme.VAL_WARNING, 1, Qt.DashLine))
            painter.drawLine(graph_rect.left(), y_yellow, graph_rect.right(), y_yellow)

        if self.red_threshold is not None:
            y_red = graph_rect.top() + (1 - (self.red_threshold - self.y_min) / (self.y_max - self.y_min)) * graph_rect.height()
            painter.setPen(QPen(Theme.ALERT, 1, Qt.DashLine))
            painter.drawLine(graph_rect.left(), y_red, graph_rect.right(), y_red)

        # Helper to draw line
        def draw_deque(data_deque, color, width=2):
            if len(data_deque) < 2: return
            painter.setPen(QPen(color, width))
            points = []
            for i, val in enumerate(data_deque):
                x = graph_rect.left() + i * graph_rect.width() / self.max_points
                y = graph_rect.top() + (1 - (val - self.y_min) / (self.y_max - self.y_min)) * graph_rect.height()
                y = max(graph_rect.top(), min(graph_rect.bottom(), y))
                points.append((x, y))
            for i in range(len(points) - 1):
                painter.drawLine(*points[i], *points[i + 1])

        # Draw data
        draw_deque(self.data, Theme.VAL_REAL, 2)
        if self.show_secondary:
            draw_deque(self.data2, Theme.VAL_TGT, 2)

        # Draw title
        painter.setPen(QPen(Theme.TXT_LABEL, 1))
        painter.setFont(QFont(Theme.FONT_FAMILY, 9, QFont.Bold))
        painter.drawText(graph_rect.left() + 4, graph_rect.top() - 5, self.title)

        # Draw Y axis labels
        painter.setFont(QFont(Theme.MONO_FONT, 8))
        painter.setPen(QPen(Theme.TXT_DIM, 1))
        painter.drawText(graph_rect.left() - 5, graph_rect.top() + 4, f"{self.y_max:.1f}")
        painter.drawText(graph_rect.left() - 5, graph_rect.bottom() + 4, f"{self.y_min:.1f}")


class TelemetryGraph(QWidget):
    """Container for multiple telemetry graphs"""

    def __init__(self, parent=None, is_dashboard=False):
        super().__init__(parent)
        self.is_dashboard = is_dashboard
        self.graphs = {}
        self.max_history = 300  # 30 seconds at 10Hz
        
        # State tracking
        self.current_joint_id = -1
        self.last_torque = 0.0

        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(10)

        # ---------------------------------------------------------
        # CHẾ ĐỘ DASHBOARD: BỘ CHUẨN ĐOÁN LỖI CHUYÊN SÂU
        # ---------------------------------------------------------
        if self.is_dashboard:
            title = QLabel("DASHBOARD: JERK & CONFLICT DIAGNOSTICS")
            title.setFont(QFont(Theme.FONT_FAMILY, 11, QFont.Bold))
            title.setStyleSheet(f"color: {Theme.VAL_WARNING.name()}; background: transparent;")
            layout.addWidget(title)

            # 1. Jitter Graph
            self.graphs['jitter'] = GraphWidget("1. NETWORK/OS JITTER (DDS dt_ms)", y_min=0, y_max=50)
            self.graphs['jitter'].set_thresholds(yellow=25, red=40)
            layout.addWidget(self.graphs['jitter'], 1)

            # 2. Torque Spike
            self.graphs['dtau'] = GraphWidget("2. TORQUE SPIKE d(tau)/dt (Nm/s)", y_min=-100, y_max=100)
            self.graphs['dtau'].set_thresholds(yellow=50, red=80)
            layout.addWidget(self.graphs['dtau'], 1)

            # 3. Symmetry View (Hip Pitch, Knee, Ankle)
            sym_title = QLabel("3. SYMMETRY VIEW (Torque: Left vs Right)")
            sym_title.setFont(QFont(Theme.FONT_FAMILY, 10, QFont.Bold))
            sym_title.setStyleSheet(f"color: {Theme.TXT_LABEL.name()}; background: transparent; padding-top: 10px;")
            layout.addWidget(sym_title)

            self.graphs['sym_hip'] = GraphWidget("Hip Pitch Torque (L=Green, R=Blue)", y_min=-25, y_max=25)
            self.graphs['sym_hip'].set_secondary(True)
            layout.addWidget(self.graphs['sym_hip'], 1)

            self.graphs['sym_knee'] = GraphWidget("Knee Torque (L=Green, R=Blue)", y_min=-25, y_max=25)
            self.graphs['sym_knee'].set_secondary(True)
            layout.addWidget(self.graphs['sym_knee'], 1)
            
            self.graphs['sym_ankle'] = GraphWidget("Ankle Pitch Torque (L=Green, R=Blue)", y_min=-15, y_max=15)
            self.graphs['sym_ankle'].set_secondary(True)
            layout.addWidget(self.graphs['sym_ankle'], 1)

        # ---------------------------------------------------------
        # CHẾ ĐỘ THƯỜNG (START_LOW_LEVEL): CHỈ XEM KHỚP ĐANG CHỌN
        # ---------------------------------------------------------
        else:
            title = QLabel("LIVE TELEMETRY (SELECTED JOINT)")
            title.setFont(QFont(Theme.FONT_FAMILY, 12, QFont.Bold))
            title.setStyleSheet(f"color: {Theme.VAL_TGT.name()}; background: transparent;")
            layout.addWidget(title)

            self.graphs['torque'] = GraphWidget("TORQUE (Nm)", y_min=-20, y_max=20)
            self.graphs['torque'].set_thresholds(yellow=10, red=15)
            layout.addWidget(self.graphs['torque'], 1)
            layout.addStretch()


    def update_telemetry(self, joint_id: int, state):
        """Update graphs with new telemetry data from RobotState"""
        
        # ---------------------------------------------------------
        # CHẾ ĐỘ DASHBOARD
        # ---------------------------------------------------------
        if self.is_dashboard:
            # Jitter
            dt_ms = state.lowstate_dt_ms
            self.graphs['jitter'].data.append(dt_ms)
            self.graphs['jitter'].update()

            # Torque Spike (của khớp đang chọn)
            sdk_idx = JOINT_IDX[joint_id]
            torque = state.motor_states[sdk_idx].tau
            dtau = (torque - self.last_torque) * (1000.0 / max(dt_ms, 1.0)) # Đạo hàm (Nm/s)
            self.last_torque = torque
            
            self.graphs['dtau'].data.append(dtau)
            self.graphs['dtau'].update()

            # Symmetry: Hip Pitch (0 vs 6)
            self.graphs['sym_hip'].data.append(state.motor_states[JOINT_IDX[0]].tau)
            self.graphs['sym_hip'].data2.append(state.motor_states[JOINT_IDX[6]].tau)
            self.graphs['sym_hip'].update()

            # Symmetry: Knee (3 vs 9)
            self.graphs['sym_knee'].data.append(state.motor_states[JOINT_IDX[3]].tau)
            self.graphs['sym_knee'].data2.append(state.motor_states[JOINT_IDX[9]].tau)
            self.graphs['sym_knee'].update()

            # Symmetry: Ankle Pitch (4 vs 10)
            self.graphs['sym_ankle'].data.append(state.motor_states[JOINT_IDX[4]].tau)
            self.graphs['sym_ankle'].data2.append(state.motor_states[JOINT_IDX[10]].tau)
            self.graphs['sym_ankle'].update()

        # ---------------------------------------------------------
        # CHẾ ĐỘ THƯỜNG
        # ---------------------------------------------------------
        else:
            if joint_id != self.current_joint_id:
                for graph in self.graphs.values():
                    graph.clear()
                self.current_joint_id = joint_id

            sdk_idx = JOINT_IDX[joint_id]
            torque = state.motor_states[sdk_idx].tau
            
            self.graphs['torque'].data.append(torque)
            self.graphs['torque'].update()
