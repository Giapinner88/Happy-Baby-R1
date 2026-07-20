#!/usr/bin/env python3
"""
R1 Sensor Dashboard (Read-Only Monitor)
A professional PySide6-based GUI purely for reading sensors without interfering with controllers.

Usage:
    python r1_sensor_dashboard.py [interface]

Arguments:
    interface: Network interface (e.g., eno1, eth0, auto). Default: auto
"""

import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gui.qt_compat import QApplication, QMessageBox, Qt
from gui.main_window import MainWindow
from gui.theme import Theme
from bridge.udp_client import UDPClient


def main():
    """Main entry point"""
    # Parse arguments
    interface = "auto"
    if len(sys.argv) > 1:
        interface = sys.argv[1]

    print(f"Starting R1 Sensor Dashboard (READ-ONLY MODE)...")
    print(f"Interface: {interface}")
    print(f"Note: Make sure the C++ DDS bridge is running with --monitor flag!")

    # Create application
    app = QApplication(sys.argv)
    app.setApplicationName("R1 Sensor Dashboard")
    app.setOrganizationName("Unitree")

    # Set application-wide style
    app.setStyle("Fusion")
    app.setPalette(Theme.get_palette())

    # Create UDP client
    udp_client = UDPClient()

    try:
        # Start UDP client
        udp_client.start()
        
        # Enable read-only mode
        udp_client.set_monitor_mode(True)

        # Create and show main window (dashboard mode)
        window = MainWindow(udp_client, interface=interface, is_dashboard=True)
        
        # Hide control panel
        window.control_panel.hide()
        
        # Set window title for safe mode
        window.setWindowTitle("R1 Sensor Dashboard - MONITOR ONLY MODE")
        window.monitor_toolbar_label.setText("MONITOR ONLY MODE - Control Disabled")
        window.statusbar.showMessage("Ready - Reading Sensors Only")
        
        window.show()

        # Run application
        return_code = app.exec_()

        # Cleanup
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
