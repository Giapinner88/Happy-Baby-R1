#!/usr/bin/env python3
"""
R1 Professional Joint Tuner GUI
A professional PySide6-based GUI for tuning R1 robot joints.

Usage:
    python r1_professional_gui.py [interface]

Arguments:
    interface: Network interface (e.g., eno1, eth0, auto). Default: auto

Requirements:
    pip install PySide6 numpy

Note:
    This GUI requires the C++ DDS bridge to be running.
    Start the bridge first: ./r1_dds_bridge <interface>
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

    print(f"Starting R1 Professional Joint Tuner GUI...")
    print(f"Interface: {interface}")
    print(f"Note: Make sure the C++ DDS bridge is running first!")

    # Create application
    app = QApplication(sys.argv)
    app.setApplicationName("R1 Joint Tuner")
    app.setOrganizationName("Unitree")

    # Set application-wide style
    app.setStyle("Fusion")
    app.setPalette(Theme.get_palette())

    # Create UDP client
    udp_client = UDPClient()

    try:
        # Start UDP client
        udp_client.start()

        # Create and show main window
        window = MainWindow(udp_client, interface=interface)
        window.show()

        # Run application
        # exec_() tương thích PyQt5; PySide6 cũng chấp nhận exec_()
        return_code = app.exec_()

        # Cleanup
        udp_client.disable_motors()
        udp_client.stop()

        return return_code

    except KeyboardInterrupt:
        print("\nShutting down...")
        udp_client.disable_motors()
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
