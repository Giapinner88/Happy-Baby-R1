#!/usr/bin/env python3
"""
teleop_send_test.py — Bộ phát teleop UDP mẫu để test TeleopReceiver (tay + đầu).

Gói khớp struct TeleopReceiver::Packet (little-endian, đóng gói chặt, 60 byte):
    uint32 magic('UTL1'), uint32 seq,
    uint8 enable, uint8 arm_valid, uint8 head_valid, uint8 pad,
    float arm_q[10]  (policy idx 14..23, rad),
    float head_yaw, float head_pitch (rad).

Tay = 5 trái (idx 0..4) + 5 phải (idx 5..9). default R1:
    [0.35, 0.18, 0, 0.87, 0,  0.35, -0.18, 0, 0.87, 0]

Dùng:
    python3 teleop_send_test.py                      # localhost:5560, vẫy tay phải + lắc đầu
    python3 teleop_send_test.py --hz 50 --secs 20
    python3 teleop_send_test.py --release            # gửi enable=0 (nhả quyền) rồi thoát

Receiver trên robot chỉ bind 127.0.0.1. Muốn test robot, chạy file này NGAY TRÊN
robot; không gửi UDP thẳng từ laptop tới IP LAN của robot.

Soi trục đầu (chốt quy ước idl 29=yaw / 30=pitch của RobotSpec.hpp) — một trục một lần:
    python3 teleop_send_test.py --hold-arms --head-yaw 0.3    # robot phải QUAY đầu
    python3 teleop_send_test.py --hold-arms --head-pitch 0.3  # robot phải GẬT đầu

Chạy song song với sim (`run_policy`, đang Flat) hoặc run_r1 (Mode L LOCOMOTION;
Mode Z ZERO TORQUE chỉ khi robot treo/được đỡ và có người giữ E-stop).
"""
import argparse
import math
import socket
import struct
import time

MAGIC = 0x314C5455  # "UTL1"
DEFAULT_ARM = [0.35, 0.18, 0.0, 0.87, 0.0, 0.35, -0.18, 0.0, 0.87, 0.0]
FMT = "<II4B10f2f"  # magic, seq, 4x uint8, 10x arm float, 2x head float  = 60 bytes


def pack(seq, arm, head_yaw, head_pitch, enable=1, arm_valid=1, head_valid=1):
    return struct.pack(FMT, MAGIC, seq, enable, arm_valid, head_valid, 0,
                       *arm, head_yaw, head_pitch)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5560)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--secs", type=float, default=15.0)
    ap.add_argument("--release", action="store_true",
                    help="gửi vài gói enable=0 (nhả quyền teleop) rồi thoát")
    ap.add_argument("--head-yaw", type=float, default=None,
                    help="giữ CỐ ĐỊNH head_yaw (rad) thay vì lắc. Dùng để soi trục đầu.")
    ap.add_argument("--head-pitch", type=float, default=None,
                    help="giữ CỐ ĐỊNH head_pitch (rad) thay vì gật.")
    ap.add_argument("--hold-arms", action="store_true",
                    help="tay giữ nguyên default (không vẫy) — cô lập phép thử đầu")
    args = ap.parse_args()
    # Chỉ định MỘT trục đầu là đủ để chốt quy ước 29=yaw/30=pitch: trục kia về 0.
    fixed_head = args.head_yaw is not None or args.head_pitch is not None

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = (args.host, args.port)
    dt = 1.0 / args.hz

    if args.release:
        for seq in range(10):
            s.sendto(pack(seq, DEFAULT_ARM, 0.0, 0.0, enable=0), dst)
            time.sleep(dt)
        print("Đã gửi enable=0 (nhả quyền teleop).")
        return

    print(f"Gửi teleop -> {dst} @ {args.hz}Hz trong {args.secs}s. Ctrl-C để dừng.")
    seq = 0
    t0 = time.time()
    try:
        while time.time() - t0 < args.secs:
            t = time.time() - t0
            arm = list(DEFAULT_ARM)
            if not args.hold_arms:
                # Tay phải (idx 5..9): nâng vai + vẫy khuỷu.
                arm[5] = DEFAULT_ARM[5] - 0.9 * (0.5 + 0.5 * math.sin(t * 2.0))  # vai pitch
                arm[8] = DEFAULT_ARM[8] + 0.5 * math.sin(t * 6.0)                # khuỷu vẫy
            if fixed_head:
                # Một trục cố định, trục kia 0 -> nhìn robot là biết trục nào là trục nào.
                head_yaw = args.head_yaw or 0.0
                head_pitch = args.head_pitch or 0.0
            else:
                # Đầu: lắc yaw + gật pitch nhẹ.
                head_yaw = 0.6 * math.sin(t * 1.5)
                head_pitch = 0.3 * math.sin(t * 1.0)
            s.sendto(pack(seq, arm, head_yaw, head_pitch), dst)
            seq += 1
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    # Nhả quyền êm khi kết thúc.
    for _ in range(5):
        s.sendto(pack(seq, DEFAULT_ARM, 0.0, 0.0, enable=0), dst)
        seq += 1
        time.sleep(dt)
    print(f"Xong. Đã gửi {seq} gói, kết thúc bằng enable=0.")


if __name__ == "__main__":
    main()
