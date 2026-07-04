"""
keyboard_publisher.py  –  Bàn Phím gửi câu hỏi cho Não Bộ

Chạy trên cùng máy với chatbot_subscriber.py (loopback "lo").
[ROBOT THẬT] Truyền tên card mạng qua argument: python3 keyboard_publisher.py eth0
"""

import sys
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from chat_data import ChatData

# [ROBOT THẬT] Đổi "lo" -> tên card mạng LAN nối robot (vd: "eth0")
NETWORK_INTERFACE = "lo"


def main():
    network_interface = NETWORK_INTERFACE
    if len(sys.argv) > 1:
        network_interface = sys.argv[1]

    print(f"[BÀN PHÍM] Khởi tạo mạng DDS trên '{network_interface}'...")
    ChannelFactoryInitialize(0, network_interface)

    pub = ChannelPublisher("g1_chat_topic", ChatData)
    pub.Init()
    print("[BÀN PHÍM] ✔ Sẵn sàng! Nhập câu hỏi và Enter để gửi. (Ctrl+C để thoát)\n")

    try:
        while True:
            try:
                text = input("[BÀN PHÍM] Câu hỏi: ").strip()
            except EOFError:
                break

            if not text:
                continue

            msg = ChatData(text_data=text)
            ok = pub.Write(msg, 1.0)   # chờ tối đa 1s để ghi
            if ok:
                print("[BÀN PHÍM] ✔ Đã gửi!\n")
            else:
                print("[BÀN PHÍM] ✗ Gửi thất bại – chatbot_subscriber.py đã chạy chưa?\n")

    except KeyboardInterrupt:
        print("\n[BÀN PHÍM] Đã tắt.")
    finally:
        pub.Close()


if __name__ == "__main__":
    main()
