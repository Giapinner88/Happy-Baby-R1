"""Qt compatibility shim — supports both PySide6 (PC1) and PyQt5 (PC2/Jetson)"""
try:
    from PySide6.QtWidgets import *
    from PySide6.QtCore import Qt, Signal, Slot, QTimer, QEvent, QCoreApplication, QSize
    from PySide6.QtGui import QAction, QColor, QFont, QIcon, QKeySequence, QPainter, QPen, QPalette
    _QT = "PySide6"
except ImportError:
    from PyQt5.QtWidgets import *
    from PyQt5.QtWidgets import QAction
    from PyQt5.QtCore import Qt, pyqtSignal as Signal, pyqtSlot as Slot
    from PyQt5.QtCore import QTimer, QEvent, QCoreApplication, QSize
    from PyQt5.QtGui import QColor, QFont, QIcon, QKeySequence, QPainter, QPen, QPalette
    # PyQt5 không có QFont.Weight enum — patch lại để tương thích
    if not hasattr(QFont, 'Bold'):
        QFont.Bold = 75
    _QT = "PyQt5"
