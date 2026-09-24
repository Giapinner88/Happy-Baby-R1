"""Telemetry Graph Widget - Live graphs for telemetry data"""
from collections import deque
from typing import Deque

from .qt_compat import Qt, QTimer, QWidget, QVBoxLayout, QLabel, QGridLayout, QFrame, QFont, QPainter, QColor, QPen

from .theme import Theme


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
        self.secondary_label = ""
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

    def set_secondary(self, label: str, data: Deque[float] = None):
        """Enable secondary line"""
        self.show_secondary = True
        self.secondary_label = label
        if data:
            self.data2 = data

    def set_thresholds(self, yellow: float = None, red: float = None):
        """Set warning thresholds"""
        self.yellow_threshold = yellow
        self.red_threshold = red

    def add_point(self, value: float, value2: float = None):
        """Add a data point"""
        self.data.append(value)
        if self.show_secondary and value2 is not None:
            self.data2.append(value2)

    def clear(self):
        """Clear all data"""
        self.data.clear()
        if self.show_secondary:
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

        # Draw data
        if len(self.data) > 1:
            painter.setPen(QPen(Theme.VAL_REAL, 2))

            points = []
            for i, val in enumerate(self.data):
                x = graph_rect.left() + i * graph_rect.width() / self.max_points
                y = graph_rect.top() + (1 - (val - self.y_min) / (self.y_max - self.y_min)) * graph_rect.height()
                y = max(graph_rect.top(), min(graph_rect.bottom(), y))
                points.append((x, y))

            for i in range(len(points) - 1):
                painter.drawLine(*points[i], *points[i + 1])

            # Draw secondary line
            if self.show_secondary and len(self.data2) > 1:
                painter.setPen(QPen(Theme.VAL_TGT, 2))
                points2 = []
                for i, val in enumerate(self.data2):
                    x = graph_rect.left() + i * graph_rect.width() / self.max_points
                    y = graph_rect.top() + (1 - (val - self.y_min) / (self.y_max - self.y_min)) * graph_rect.height()
                    y = max(graph_rect.top(), min(graph_rect.bottom(), y))
                    points2.append((x, y))

                for i in range(len(points2) - 1):
                    painter.drawLine(*points2[i], *points2[i + 1])

        # Draw title
        painter.setPen(QPen(Theme.TXT_LABEL, 1))
        painter.setFont(QFont(Theme.FONT_FAMILY, 9))
        painter.drawText(graph_rect.left() + 4, graph_rect.top() - 5, self.title)

        # Draw Y axis labels
        painter.setFont(QFont(Theme.MONO_FONT, 8))
        painter.setPen(QPen(Theme.TXT_DIM, 1))
        painter.drawText(graph_rect.left() - 5, graph_rect.top() + 4, f"{self.y_max:.1f}")
        painter.drawText(graph_rect.left() - 5, graph_rect.bottom() + 4, f"{self.y_min:.1f}")


class TelemetryGraph(QWidget):
    """Container for multiple telemetry graphs"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.graphs = {}
        self.history: dict = {}
        self.max_history = 300  # 30 seconds at 10Hz

        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(10)

        # Title
        title = QLabel("LIVE TELEMETRY")
        title.setFont(QFont(Theme.FONT_FAMILY, 12, QFont.Bold))
        title.setStyleSheet(f"color: {Theme.VAL_TGT.name()}; background: transparent;")
        layout.addWidget(title)

        # Create graphs - only torque
        self.graphs['torque'] = GraphWidget("TORQUE (Nm)", y_min=-20, y_max=20)
        self.graphs['torque'].set_thresholds(yellow=10, red=15)

        layout.addWidget(self.graphs['torque'], 1)
        layout.addStretch()

    def update_telemetry(self, joint_id: int, real_vel: float, tgt_vel: float, torque: float, temp: float):
        """Update graphs with new telemetry data"""
        # Add to history (use selected joint)
        if 'torque' not in self.history:
            self.history['torque'] = deque(maxlen=self.max_history)

        self.history['torque'].append(torque)

        # Update graphs
        self.graphs['torque'].data = self.history['torque']

        # Trigger repaint
        self.graphs['torque'].update()

    def clear(self):
        """Clear all graph data"""
        for key in self.history:
            self.history[key].clear()
        for graph in self.graphs.values():
            graph.clear()
            graph.update()
