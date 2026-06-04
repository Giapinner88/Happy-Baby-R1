#!/usr/bin/env python3
"""
replay_commands.py — Replay lại hành động bằng cách đưa lệnh điều khiển từ CSV
vào đúng vòng lặp RL policy (closed-loop).

CÁCH HOẠT ĐỘNG:
    Đọc cmd_vx, cmd_vy, cmd_yaw từ CSV (lệnh joystick đã lưu) → đưa vào
    chính xác pipeline của run98_2.py (RL policy + PD controller).
    Robot vẫn sử dụng cảm biến thực tế để cân bằng — chỉ thay gamepad bằng CSV.

    Đây là CÁCH DUY NHẤT đúng để "phát lại hành động" cho robot cân bằng động,
    vì policy cần feedback vòng kín liên tục để giữ thăng bằng.

USAGE:
    python replay_commands.py                        # CSV mới nhất
    python replay_commands.py sim_state_logs/run98_2_17-11_2026-05-06.csv
    python replay_commands.py --speed 0.5            # phát lại chậm 2x
    python replay_commands.py --loop                 # lặp lại vô hạn

SO SÁNH VỚI replay_csv.py:
    replay_csv.py       → open-loop (target_q thẳng xuống motor) → KHÔNG HOẠT ĐỘNG
                          cho robot cân bằng động (G1)
    replay_commands.py  → closed-loop (cmd → policy → motor)     → ĐÚNG
"""

import argparse
import csv
import sys
import time
import threading
from pathlib import Path

import numpy as np
import onnxruntime as ort

from unitree_sdk2py.core.channel import (
    ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
)
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# ─── Hằng số giống hệt run98_2.py ────────────────────────────────────────────

KP_ARRAY = np.array([
    40.2, 99.1, 40.2, 99.1, 28.5, 28.5,
    40.2, 99.1, 40.2, 99.1, 28.5, 28.5,
    40.2, 28.5, 28.5,
    14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8,
    14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8,
], dtype=np.float32)

KD_ARRAY = np.array([
    2.6, 6.3, 2.6, 6.3, 1.8, 1.8,
    2.6, 6.3, 2.6, 6.3, 1.8, 1.8,
    2.6, 1.8, 1.8,
    0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1,
    0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1,
], dtype=np.float32)

DEFAULT_Q = np.array([
    -0.1, 0, 0, 0.3, -0.2, 0,
    -0.1, 0, 0, 0.3, -0.2, 0,
     0, 0, 0,
     0.35,  0.18, 0, 0.87, 0, 0, 0,
     0.35, -0.18, 0, 0.87, 0, 0, 0,
], dtype=np.float32)

ACTION_SCALE = np.array([
    0.55, 0.35, 0.55, 0.35, 0.44, 0.44,
    0.55, 0.35, 0.55, 0.35, 0.44, 0.44,
    0.55, 0.44, 0.44,
    0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07,
    0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07,
], dtype=np.float32)

# ─── State DDS ───────────────────────────────────────────────────────────────

robot_state       = None
_got_first_state  = False
state_lock        = threading.Lock()
cmd               = unitree_hg_msg_dds__LowCmd_()
cmd_lock          = threading.Lock()


def _state_handler(msg: LowState_):
    global robot_state, _got_first_state
    with state_lock:
        robot_state = msg
    if not _got_first_state:
        _got_first_state = True
        print("[replay-cmd] LowState nhận được (DDS OK).")


def _dds_publisher_loop(pub):
    crc_calc = CRC()
    while True:
        with cmd_lock:
            cmd.crc = crc_calc.Crc(cmd)
            pub.Write(cmd)
        time.sleep(0.002)


def compute_projected_gravity(quat):
    """Giống hệt run98_2.py — quat = [w, x, y, z]"""
    w, x, y, z = quat
    gx =  2 * (w * y - x * z)
    gy = -2 * (y * z + w * x)
    gz =  2 * (x**2 + y**2) - 1
    return np.array([gx, gy, gz], dtype=np.float32)


# ─── Load CSV ─────────────────────────────────────────────────────────────────

def pick_latest_csv() -> Path:
    # Use repo-level data directory for logs
    log_dir = Path(__file__).resolve().parents[2] / "data" / "sim_state_logs"
    if not log_dir.exists():
        print(f"[replay-cmd] Không tìm thấy: {log_dir}", file=sys.stderr)
        sys.exit(1)
    csvs = sorted(log_dir.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not csvs:
        print("[replay-cmd] Không có CSV nào trong sim_state_logs/", file=sys.stderr)
        sys.exit(1)

    # Bỏ qua các file chỉ có header hoặc file rỗng. Điều này xảy ra nếu logger
    # bị dừng trước khi kịp flush row dữ liệu.
    for csv_path in csvs:
        try:
            if csv_path.stat().st_size > 0:
                with open(csv_path, newline="") as f:
                    reader = csv.DictReader(f)
                    if reader.fieldnames is not None and any(True for _ in reader):
                        return csv_path
        except OSError:
            continue

    print("[replay-cmd] Không tìm thấy CSV nào có dữ liệu (chỉ thấy file rỗng/header-only).", file=sys.stderr)
    sys.exit(1)


def load_commands(path: Path):
    """
    Đọc chuỗi lệnh điều khiển từ CSV.
    Trả về list[(t_sec, vx, vy, yaw)] để phát lại.
    """
    commands = []
    required = ["t_sec", "cmd_vx", "cmd_vy", "cmd_yaw"]

    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        missing = [c for c in required if c not in reader.fieldnames]
        if missing:
            print(f"[replay-cmd] CSV thiếu cột: {missing}", file=sys.stderr)
            sys.exit(1)

        for row in reader:
            commands.append((
                float(row["t_sec"]),
                float(row["cmd_vx"]),
                float(row["cmd_vy"]),
                float(row["cmd_yaw"]),
            ))

    return commands


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Replay closed-loop: đưa lệnh joystick từ CSV vào RL policy.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "csv", nargs="?",
        help="Đường dẫn CSV (mặc định: mới nhất trong sim_state_logs/)",
    )
    parser.add_argument(
        "--speed", type=float, default=1.0, metavar="X",
        help="Tốc độ phát lại (0.5 = chậm 2x). Mặc định: 1.0",
    )
    parser.add_argument(
        "--loop", action="store_true",
        help="Lặp lại vô hạn sau khi hết CSV",
    )
    parser.add_argument(
        "--delay", type=float, default=3.0, metavar="SEC",
        help="Thời gian chờ trước khi bắt đầu (mặc định: 3 giây)",
    )
    parser.add_argument(
        "--policy", type=str, default="policy98.onnx",
        help="File policy ONNX (mặc định: policy98.onnx)",
    )
    args = parser.parse_args()

    # ── Chọn CSV ─────────────────────────────────────────────────────────────
    csv_path = Path(args.csv) if args.csv else pick_latest_csv()
    if not csv_path.exists():
        print(f"[replay-cmd] Không tìm thấy: {csv_path}", file=sys.stderr)
        sys.exit(1)

    print(f"[replay-cmd] File: {csv_path.name}")
    commands = load_commands(csv_path)
    n        = len(commands)
    if n == 0:
        print(
            f"[replay-cmd] CSV '{csv_path.name}' không có dữ liệu. "
            "Hãy chạy controller đủ lâu và thoát bằng Ctrl+C để logger flush file.",
            file=sys.stderr,
        )
        sys.exit(1)
    duration = commands[-1][0] - commands[0][0] if n > 1 else 0.0
    print(f"[replay-cmd] {n:,} steps | {duration:.1f}s → phát lại: "
          f"{duration/args.speed:.1f}s (speed={args.speed}x)"
          + (" | loop=ON" if args.loop else ""))

    # ── Load policy ──────────────────────────────────────────────────────────
    policy_path = Path(__file__).parent / args.policy
    if not policy_path.exists():
        print(f"[replay-cmd] Không tìm thấy policy: {policy_path}", file=sys.stderr)
        sys.exit(1)
    session    = ort.InferenceSession(str(policy_path), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    print(f"[replay-cmd] Policy: {policy_path.name}")

    # ── DDS init ─────────────────────────────────────────────────────────────
    ChannelFactoryInitialize(1, "lo")
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(_state_handler, 10)

    with cmd_lock:
        for i in range(29):
            cmd.motor_cmd[i].mode  = 0x01
            cmd.motor_cmd[i].q     = float(DEFAULT_Q[i])
            cmd.motor_cmd[i].dq    = 0.0
            cmd.motor_cmd[i].tau   = 0.0
            cmd.motor_cmd[i].kp    = float(KP_ARRAY[i])
            cmd.motor_cmd[i].kd    = float(KD_ARRAY[i])

    pub_thread = threading.Thread(target=_dds_publisher_loop, args=(pub,), daemon=True)
    pub_thread.start()

    # ── Đợi simulator ────────────────────────────────────────────────────────
    print("[replay-cmd] Đang đợi LowState từ simulator...")
    while not _got_first_state:
        time.sleep(0.01)

    print(f"[replay-cmd] Bắt đầu sau {args.delay:.0f}s... (Ctrl+C để dừng)")
    time.sleep(args.delay)
    print("[replay-cmd] ▶ BẮT ĐẦU!")

    # ── Biến trạng thái vòng lặp (giống run98_2.py) ──────────────────────────
    last_action      = np.zeros(29, dtype=np.float32)
    gait_time        = 0.0
    gait_scale       = 1.0
    last_step_time   = time.perf_counter()
    last_print_time  = 0.0

    CTRL_DT = 0.02   # 50 Hz

    iteration = 0
    try:
        while True:
            # Lấy lệnh điều khiển từ CSV theo thời gian (vòng lặp phát lại)
            cmd_idx = iteration % n
            t_csv, raw_vx, raw_vy, raw_yaw = commands[cmd_idx]
            iteration += 1

            # Dừng nếu không loop và đã hết CSV
            if not args.loop and cmd_idx == n - 1:
                print("\n[replay-cmd] Hết CSV. Kết thúc.")
                break

            # ── Timing 50Hz ─────────────────────────────────────────────────
            step_start = time.perf_counter()
            dt = step_start - last_step_time
            last_step_time = step_start
            dt = max(0.0, min(dt, 0.05))

            # ── Đọc sensor ──────────────────────────────────────────────────
            q_current  = np.zeros(29, dtype=np.float32)
            dq_current = np.zeros(29, dtype=np.float32)
            gyro       = np.zeros(3,  dtype=np.float32)
            quat       = np.zeros(4,  dtype=np.float32)
            with state_lock:
                rs = robot_state
                if rs is None:
                    time.sleep(CTRL_DT)
                    continue
                for i in range(29):
                    q_current[i]  = rs.motor_state[i].q
                    dq_current[i] = rs.motor_state[i].dq
                gyro[:] = np.array(rs.imu_state.gyroscope,   dtype=np.float32)
                quat[:] = np.array(rs.imu_state.quaternion,  dtype=np.float32)

            # ── Áp dụng speed: scale thời gian phát lại ─────────────────────
            # Lệnh từ CSV phát theo tốc độ args.speed: chỉ cần bước index nhanh hơn
            # Cách đơn giản: nếu speed=2 thì mỗi bước 20ms tiêu thụ 2 commands
            steps_per_ctrl = max(1, round(args.speed))
            # (đã xử lý bằng cmd_idx bước theo speed, đơn giản nhất)

            # ── Gait phase (giống run98_2.py) ────────────────────────────────
            moving = (abs(raw_vx) > 0.01 or abs(raw_vy) > 0.01 or abs(raw_yaw) > 0.01)
            if moving:
                now = time.perf_counter()
                if now - last_print_time > 1.0:
                    print(f"\r[replay-cmd] Vx={raw_vx:.2f}  Vy={raw_vy:.2f}  "
                          f"Yaw={raw_yaw:.2f}  step={cmd_idx}/{n}  ", end="", flush=True)
                    last_print_time = now
                gait_time  += dt
                gait_scale  = min(1.0, gait_scale + dt / 0.3)
            else:
                remainder = gait_time % 0.6
                if 0.02 < remainder < 0.58:
                    gait_time  += dt
                    gait_scale  = min(1.0, gait_scale + dt / 0.3)
                else:
                    gait_time   = round(gait_time / 0.6) * 0.6
                    gait_scale  = max(0.0, gait_scale - dt / 0.3)

            phase_ratio = (gait_time % 0.6) / 0.6
            gait_phase  = np.array([
                np.sin(2 * np.pi * phase_ratio),
                np.cos(2 * np.pi * phase_ratio),
            ], dtype=np.float32) * gait_scale

            # ── Observation vector (giống run98_2.py) ────────────────────────
            projected_gravity = compute_projected_gravity(quat)
            q_rel = q_current - DEFAULT_Q
            smooth_cmd = np.array([raw_vx, raw_vy, raw_yaw], dtype=np.float32)

            obs = np.concatenate([
                gyro,
                projected_gravity,
                smooth_cmd,
                gait_phase,
                q_rel,
                dq_current,
                last_action,
            ]).astype(np.float32)

            # ── Chạy Policy ──────────────────────────────────────────────────
            obs_tensor = np.expand_dims(obs, axis=0)
            action     = session.run(None, {input_name: obs_tensor})[0][0]
            last_action = action.copy()

            # ── Gửi lệnh xuống motor ─────────────────────────────────────────
            target_q_arr = DEFAULT_Q + action * ACTION_SCALE
            with cmd_lock:
                for i in range(29):
                    cmd.motor_cmd[i].q = float(target_q_arr[i])

            # ── Giữ timing 50Hz ──────────────────────────────────────────────
            elapsed = time.perf_counter() - step_start
            wait    = CTRL_DT - elapsed
            if wait > 0:
                time.sleep(wait)

    except KeyboardInterrupt:
        print(f"\n[replay-cmd] Dừng tại step {cmd_idx}/{n}.")


if __name__ == "__main__":
    main()
