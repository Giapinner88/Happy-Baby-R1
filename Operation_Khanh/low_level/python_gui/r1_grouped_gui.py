#!/usr/bin/env python3
import sys
import argparse
from PySide6.QtWidgets import QApplication

from bridge.udp_client import UDPClient
from gui.grouped_main_window import GroupedMainWindow


def main():
    parser = argparse.ArgumentParser(description="R1 Diagnostic Dashboard - Grouped GUI")
    parser.add_argument("interface", nargs="?", default="auto", help="Network interface (auto, lo, eth0, wlan0)")
    args = parser.parse_args()

    print("🚀 Starting R1 Diagnostic Dashboard (Grouped View)...")

    # Start UDP Client
    udp_client = UDPClient()
    try:
        udp_client.start()
    except RuntimeError as e:
        print(f"\n❌ Error: {e}")
        sys.exit(1)

    # Start GUI
    app = QApplication(sys.argv)
    window = GroupedMainWindow(udp_client=udp_client, interface=args.interface)
    window.show()

    try:
        ret = app.exec()
    finally:
        print("\n🛑 Cleaning up...")
        udp_client.stop()
        
    sys.exit(ret)


if __name__ == "__main__":
    main()
