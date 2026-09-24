"""Theme and styling for R1 Professional GUI"""
from .qt_compat import Qt, QColor, QPalette


class Theme:
    """Color theme for dark professional UI"""

    # Background colors
    BG_MAIN = QColor(11, 14, 20)        # #0B0E14
    BG_PANEL = QColor(19, 26, 36)       # #131A24
    BG_CARD = QColor(25, 33, 48)        # #191F30
    BG_BUTTON = QColor(35, 45, 62)      # #232D3E

    # Border colors
    BDR_DEFAULT = QColor(45, 56, 76)    # #2D384C
    BDR_SELECTED = QColor(242, 169, 59) # #F2A93B - Gold
    BDR_HOVER = QColor(65, 80, 108)     # #41506C

    # Text colors
    TXT_TITLE = QColor(230, 234, 242)   # #E6EAF2
    TXT_LABEL = QColor(107, 118, 134)   # #6B7686
    TXT_DIM = QColor(70, 80, 95)        # #46505F

    # Value colors
    VAL_REAL = QColor(95, 227, 139)     # #5FE38B - Green
    VAL_TGT = QColor(79, 195, 242)      # #4FC3F2 - Cyan
    VAL_WARNING = QColor(242, 169, 59)   # #F2A93B - Gold/Orange

    # Alert colors
    ALERT = QColor(242, 85, 90)        # #F2555A - Red
    ALERT_DARK = QColor(193, 58, 62)    # #C13A3E
    SUCCESS = QColor(95, 227, 139)      # #5FE38B

    # Accent
    ACCENT = QColor(242, 169, 59)       # #F2A93B
    ACCENT_DARK = QColor(196, 133, 15)   # #C4850F

    # Fonts
    FONT_FAMILY = "Segoe UI, Arial, sans-serif"
    MONO_FONT = "Consolas, Courier New, monospace"

    @staticmethod
    def get_palette() -> QPalette:
        """Get Qt palette for dark theme"""
        palette = QPalette()
        palette.setColor(QPalette.Window, Theme.BG_MAIN)
        palette.setColor(QPalette.WindowText, Theme.TXT_TITLE)
        palette.setColor(QPalette.Base, Theme.BG_PANEL)
        palette.setColor(QPalette.AlternateBase, Theme.BG_CARD)
        palette.setColor(QPalette.ToolTipBase, Theme.BG_PANEL)
        palette.setColor(QPalette.ToolTipText, Theme.TXT_TITLE)
        palette.setColor(QPalette.Text, Theme.TXT_TITLE)
        palette.setColor(QPalette.Button, Theme.BG_BUTTON)
        palette.setColor(QPalette.ButtonText, Theme.TXT_TITLE)
        palette.setColor(QPalette.BrightText, Qt.white)
        palette.setColor(QPalette.Highlight, Theme.ACCENT)
        palette.setColor(QPalette.HighlightedText, Theme.BG_MAIN)
        return palette

    @staticmethod
    def get_stylesheet() -> str:
        """Get stylesheet for widgets"""
        return """
        QMainWindow {
            background-color: #0B0E14;
        }
        QWidget {
            background-color: #0B0E14;
            color: #E6EAF2;
            font-family: "Segoe UI", Arial, sans-serif;
        }
        QLabel {
            background-color: transparent;
            color: #E6EAF2;
        }
        QPushButton {
            background-color: #232D3E;
            border: 1px solid #2D384C;
            border-radius: 4px;
            padding: 6px 12px;
            color: #E6EAF2;
        }
        QPushButton:hover {
            background-color: #2D384C;
            border-color: #41506C;
        }
        QPushButton:pressed {
            background-color: #1F2737;
        }
        QPushButton:disabled {
            background-color: #151B26;
            color: #46505F;
        }
        QToolBar {
            background-color: #131A24;
            border-bottom: 1px solid #2D384C;
            spacing: 6px;
            padding: 4px;
        }
        QStatusBar {
            background-color: #131A24;
            border-top: 1px solid #2D384C;
            color: #6B7686;
        }
        QMenuBar {
            background-color: #131A24;
            border-bottom: 1px solid #2D384C;
        }
        QMenuBar::item {
            background-color: transparent;
            padding: 4px 12px;
        }
        QMenuBar::item:selected {
            background-color: #232D3E;
        }
        QMenu {
            background-color: #131A24;
            border: 1px solid #2D384C;
        }
        QMenu::item {
            padding: 6px 24px;
        }
        QMenu::item:selected {
            background-color: #232D3E;
        }
        QGroupBox {
            border: 1px solid #2D384C;
            border-radius: 6px;
            margin-top: 12px;
            padding-top: 12px;
        }
        QGroupBox::title {
            color: #6B7686;
            subcontrol-origin: margin;
            left: 12px;
            padding: 0 4px;
        }
        QScrollArea {
            border: none;
            background-color: transparent;
        }
        QFrame#card {
            background-color: #191F30;
            border: 1px solid #2D384C;
            border-radius: 6px;
        }
        """
