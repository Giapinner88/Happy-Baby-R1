"""
launch.py — Menu chọn policy để chạy G1 robot.
Chạy: python launch.py
"""
import subprocess
import sys
import os

MODES = [
    {
        "name": "🚶 Locomotion (run98_3.py)",
        "desc": "Điều khiển đi bộ bằng bàn phím / gamepad",
        "cmd":  [sys.executable, "run98_3.py"],
    },
    {
        "name": "💃 Mimic — motion_data (run_mimic.py)",
        "desc": "Bắt chước chuyển động từ motion_data.npz",
        "cmd":  [sys.executable, "run_mimic.py",
                 "--policy", "policy_motion_data.onnx",
                 "--npz",    "motions/motion_data.npz"],
    },
    {
        "name": "💃 Mimic — chọn file tuỳ chỉnh",
        "desc": "Nhập đường dẫn policy và NPZ thủ công",
        "cmd":  None,  # handled below
    },
]

def pick_mode():
    print("\n" + "=" * 55)
    print("  G1 Robot Launcher")
    print("=" * 55)
    for i, m in enumerate(MODES):
        print(f"  [{i+1}] {m['name']}")
        print(f"       {m['desc']}")
    print("  [0] Thoát")
    print("=" * 55)
    while True:
        try:
            choice = int(input("Chọn mode: "))
            if 0 <= choice <= len(MODES):
                return choice
        except ValueError:
            pass
        print("  Nhập số hợp lệ!")

def run(cmd):
    print(f"\n>>> Chạy: {' '.join(cmd)}\n")
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\n>>> Đã dừng.")

def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    while True:
        choice = pick_mode()
        if choice == 0:
            print("Thoát.")
            break

        mode = MODES[choice - 1]

        if mode["cmd"] is not None:
            run(mode["cmd"])
        else:
            # Chế độ tuỳ chỉnh
            print()
            policy = input("  Đường dẫn policy ONNX (vd: policy_motion_data.onnx): ").strip()
            npz    = input("  Đường dẫn NPZ        (vd: motions/motion_data.npz): ").strip()
            if not policy or not npz:
                print("  Huỷ — thiếu thông tin.")
                continue
            run([sys.executable, "run_mimic.py", "--policy", policy, "--npz", npz])

        print("\n>>> Policy đã kết thúc. Quay về menu...\n")

if __name__ == "__main__":
    main()
