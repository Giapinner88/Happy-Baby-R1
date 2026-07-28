#!/usr/bin/env python3
import sys
import argparse
from PySide6.QtWidgets import QApplication

from bridge.udp_client import UDPClient
from gui.tuning_main_window import TuningMainWindow


def main():
    parser = argparse.ArgumentParser(description="R1 Performance & Tuning GUI")
    parser.add_argument("interface", nargs="?", default="auto", help="Network interface")
    args = parser.parse_args()

    print("🚀 Khởi động R1 PD Tuning GUI...")

    udp_client = UDPClient()
    try:
        udp_client.start()
    except RuntimeError as e:
        print(f"\n❌ Lỗi: {e}")
        sys.exit(1)

    app = QApplication(sys.argv)
    window = TuningMainWindow(udp_client=udp_client, interface=args.interface)
    window.show()

    try:
        ret = app.exec()
    finally:
        print("\n🛑 Đang dọn dẹp...")
        udp_client.stop()
        
    sys.exit(ret)


if __name__ == "__main__":
    main()
