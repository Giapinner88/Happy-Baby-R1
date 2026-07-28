#!/usr/bin/env python3
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gui.qt_compat import QApplication, QMessageBox, Qt
from gui.main_window import MainWindow
from gui.theme import Theme
from bridge.udp_client import UDPClient


def main():
    interface = "auto"
    if len(sys.argv) > 1:
        interface = sys.argv[1]

    print("Starting R1 Sensor Dashboard (READ-ONLY MODE)...")
    print(f"Interface: {interface}")
    print("Note: Make sure the C++ DDS bridge is running with --monitor flag!")

    app = QApplication(sys.argv)
    app.setApplicationName("R1 Sensor Dashboard")
    app.setOrganizationName("Unitree")

    app.setStyle("Fusion")
    app.setPalette(Theme.get_palette())

    udp_client = UDPClient()

    try:
        udp_client.start()
        udp_client.set_monitor_mode(True)

        window = MainWindow(udp_client, interface=interface, is_dashboard=True)
        window.control_panel.hide()
        
        window.setWindowTitle("R1 Sensor Dashboard - MONITOR ONLY MODE")
        window.monitor_toolbar_label.setText("MONITOR ONLY MODE - Control Disabled")
        window.statusbar.showMessage("Ready - Reading Sensors Only")
        
        window.show()

        return_code = app.exec_()
        udp_client.stop()
        return return_code

    except KeyboardInterrupt:
        print("\nShutting down...")
        udp_client.stop()
        return 0

    except Exception as e:
        QMessageBox.critical(
            None,
            "Error",
            f"Failed to start GUI:\n{str(e)}"
        )
        return 1


if __name__ == "__main__":
    sys.exit(main())
