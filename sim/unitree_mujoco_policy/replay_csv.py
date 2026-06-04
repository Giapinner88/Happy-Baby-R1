#!/usr/bin/env python3
"""
replay_csv.py — Open-loop replay of a recorded sim_state_logs CSV.

USAGE:
    python replay_csv.py                         # tự động chọn CSV mới nhất
    python replay_csv.py sim_state_logs/run98_2_16-59_2026-05-06.csv
    python replay_csv.py --speed 0.5             # phát lại chậm 2x
    python replay_csv.py --skip-unstable         # bỏ qua frame có giá trị bất thường
    python replay_csv.py --clamp 15              # kẹp target_q tối đa ±15 rad
    python replay_csv.py --start-row 50          # bắt đầu từ row 50 (sync với --init-row)

SYNC VỚI unitree_mujoco2.py:
    # Terminal 1 — sim bắt đầu từ trạng thái row 50 của CSV
    python unitree_mujoco2.py --init-csv sim_state_logs/run98_2_*.csv --init-row 50

    # Terminal 2 — replay bắt đầu gửi lệnh từ row 50 để khớp
    python replay_csv.py --start-row 50

CÁCH HOẠT ĐỘNG:
    Đọc cột target_q_{joint} từ CSV và gửi xuống motor qua DDS theo đúng
    timing gốc. Policy RL KHÔNG được chạy — replay open-loop thuần túy.

CHÚ Ý VỀ FRAME KHÔNG ỔN ĐỊNH:
    Vài frame đầu thường có giá trị target_q rất lớn (simulation chưa ổn định).
    Dùng --skip-unstable hoặc --clamp để lọc chúng ra.
"""

import argparse
import csv
import sys
import time
import threading
from pathlib import Path

import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
)
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# ─── Gain giống hệt run98_2.py ───────────────────────────────────────────────

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

JOINT_NAMES = [
    "left_hip_pitch",    "left_hip_roll",    "left_hip_yaw",
    "left_knee",         "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch",   "right_hip_roll",   "right_hip_yaw",
    "right_knee",        "right_ankle_pitch","right_ankle_roll",
    "waist_yaw",         "waist_roll",       "waist_pitch",
    "left_shoulder_pitch","left_shoulder_roll","left_shoulder_yaw",
    "left_elbow",        "left_wrist_roll",  "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch","right_shoulder_roll","right_shoulder_yaw",
    "right_elbow",       "right_wrist_roll", "right_wrist_pitch","right_wrist_yaw",
]  # 29 khớp — phải khớp với thứ tự motor_state trong SDK

TARGET_Q_COLS = [f"target_q_{n}" for n in JOINT_NAMES]

# ─── State DDS ───────────────────────────────────────────────────────────────

_got_first_state = False
state_lock        = threading.Lock()
robot_state       = None

cmd      = unitree_hg_msg_dds__LowCmd_()
cmd_lock = threading.Lock()


def _state_handler(msg: LowState_):
    global robot_state, _got_first_state
    with state_lock:
        robot_state = msg
    if not _got_first_state:
        _got_first_state = True
        print("[replay] LowState nhận được (DDS OK).")


def _dds_publisher_loop(pub):
    crc_calc = CRC()
    while True:
        with cmd_lock:
            cmd.crc = crc_calc.Crc(cmd)
            pub.Write(cmd)
        time.sleep(0.002)


# ─── Helpers ─────────────────────────────────────────────────────────────────

def pick_latest_csv() -> Path:
    """Chọn CSV mới nhất (theo thời gian sửa đổi) trong repo `data/sim_state_logs/`."""
    # prefer repo-level data directory so third_party remains read-only
    log_dir = Path(__file__).resolve().parents[2] / "data" / "sim_state_logs"
    if not log_dir.exists():
        print(f"[replay] Không tìm thấy thư mục: {log_dir}", file=sys.stderr)
        sys.exit(1)
    csvs = sorted(log_dir.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not csvs:
        print("[replay] Không có file CSV nào trong sim_state_logs/", file=sys.stderr)
        sys.exit(1)
    return csvs[0]


def load_csv(path: Path, clamp: float | None, skip_unstable: bool,
             start_row: int = 0):
    """
    Đọc CSV và trả về danh sách (t_sec, target_q[29]).

    Args:
        clamp:           Nếu không None, kẹp |target_q| <= clamp (rad).
        skip_unstable:   Nếu True, bỏ qua dòng nào có |target_q| > 30 rad.
        start_row:       Bắt đầu đọc từ row thứ N (0-indexed, không tính header).
                         Dùng để sync với --init-row của unitree_mujoco2.py.

    Returns:
        rows_t: list[float]       — timestamp giây từ đầu
        rows_q: list[np.ndarray]  — target_q[29] cho từng bước
    """
    rows_t, rows_q = [], []
    n_skipped = 0
    n_skipped_start = 0
    UNSTABLE_THRESHOLD = 30.0  # rad — ngưỡng phát hiện frame bất thường

    with open(path, newline="") as f:
        reader = csv.DictReader(f)

        # Kiểm tra header hợp lệ
        missing = [c for c in ["t_sec"] + TARGET_Q_COLS if c not in reader.fieldnames]
        if missing:
            print(f"[replay] Lỗi: CSV thiếu cột: {missing}", file=sys.stderr)
            sys.exit(1)

        for raw_idx, row in enumerate(reader):
            # Bỏ qua các row trước start_row
            if raw_idx < start_row:
                n_skipped_start += 1
                continue

            t = float(row["t_sec"])
            q = np.array([float(row[c]) for c in TARGET_Q_COLS], dtype=np.float32)

            if skip_unstable and np.any(np.abs(q) > UNSTABLE_THRESHOLD):
                n_skipped += 1
                continue

            if clamp is not None:
                q = np.clip(q, -clamp, clamp)

            rows_t.append(t)
            rows_q.append(q)

    if start_row > 0:
        print(f"[replay] Bắt đầu từ row {start_row} (bỏ qua {n_skipped_start} rows đầu).")
    if n_skipped:
        print(f"[replay] Đã bỏ qua {n_skipped} frame bất thường (|q| > {UNSTABLE_THRESHOLD} rad).")

    if not rows_t:
        print("[replay] Không còn frame nào sau khi lọc!", file=sys.stderr)
        sys.exit(1)

    return rows_t, rows_q


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Replay open-loop từ file CSV log của sim G1.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "csv", nargs="?",
        help="Đường dẫn file CSV (mặc định: CSV mới nhất trong sim_state_logs/)",
    )
    parser.add_argument(
        "--speed", type=float, default=1.0, metavar="X",
        help="Tốc độ phát lại (0.5 = chậm 2x, 2.0 = nhanh 2x). Mặc định: 1.0",
    )
    parser.add_argument(
        "--skip-unstable", action="store_true",
        help="Bỏ qua frame có |target_q| > 30 rad (frame lúc sim chưa ổn định)",
    )
    parser.add_argument(
        "--clamp", type=float, default=None, metavar="RAD",
        help="Kẹp target_q trong khoảng [-RAD, RAD]. Ví dụ: --clamp 15",
    )
    parser.add_argument(
        "--start-row", type=int, default=0, metavar="N",
        help=("Bắt đầu replay từ row thứ N trong CSV (0-indexed). "
              "Dùng để sync với: python unitree_mujoco2.py --init-row N"),
    )
    parser.add_argument(
        "--delay", type=float, default=2.0, metavar="SEC",
        help="Thời gian chờ trước khi bắt đầu replay (mặc định: 2.0 giây)",
    )
    args = parser.parse_args()

    # ── Chọn file CSV ─────────────────────────────────────────────────────────
    csv_path = Path(args.csv) if args.csv else pick_latest_csv()
    if not csv_path.exists():
        print(f"[replay] Không tìm thấy file: {csv_path}", file=sys.stderr)
        sys.exit(1)

    print(f"[replay] File: {csv_path.name}")
    rows_t, rows_q = load_csv(csv_path, args.clamp, args.skip_unstable,
                               start_row=args.start_row)

    n_rows   = len(rows_t)
    duration = rows_t[-1] - rows_t[0] if n_rows > 1 else 0.0
    print(f"[replay] {n_rows:,} steps | thời lượng gốc: {duration:.1f}s "
          f"| phát lại: {duration/args.speed:.1f}s (speed={args.speed}x)")
    if args.clamp:
        print(f"[replay] Clamp: target_q kẹp trong ±{args.clamp} rad")

    # ── Khởi tạo DDS ─────────────────────────────────────────────────────────
    ChannelFactoryInitialize(1, "lo")
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(_state_handler, 10)

    # Giữ nguyên vị trí frame đầu tiên trong khi chờ
    with cmd_lock:
        for i in range(29):
            cmd.motor_cmd[i].mode  = 0x01
            cmd.motor_cmd[i].q     = float(rows_q[0][i])
            cmd.motor_cmd[i].dq    = 0.0
            cmd.motor_cmd[i].tau   = 0.0
            cmd.motor_cmd[i].kp    = float(KP_ARRAY[i])
            cmd.motor_cmd[i].kd    = float(KD_ARRAY[i])

    pub_thread = threading.Thread(target=_dds_publisher_loop, args=(pub,), daemon=True)
    pub_thread.start()

    # ── Đợi simulator ─────────────────────────────────────────────────────────
    print("[replay] Đang đợi LowState từ simulator...")
    while not _got_first_state:
        time.sleep(0.01)

    print(f"[replay] Bắt đầu sau {args.delay:.0f} giây... (Ctrl+C để dừng)")
    time.sleep(args.delay)
    print("[replay] ▶ BẮT ĐẦU REPLAY!")

    # ── Vòng lặp replay ───────────────────────────────────────────────────────
    t_wall_start = time.perf_counter()
    t_csv_start  = rows_t[0]
    n_sent       = 0

    try:
        for i, (t_csv, q_target) in enumerate(zip(rows_t, rows_q)):
            # Tính thời điểm wall-clock cần gửi frame này
            t_wall_target = (t_csv - t_csv_start) / args.speed
            wait = t_wall_target - (time.perf_counter() - t_wall_start)
            if wait > 0:
                time.sleep(wait)

            # Gửi lệnh xuống motor
            with cmd_lock:
                for j in range(29):
                    cmd.motor_cmd[j].q = float(q_target[j])
            n_sent += 1

            # Hiển thị tiến độ mỗi ~1 giây (50 steps)
            if i % 50 == 0:
                pct     = (i + 1) / n_rows * 100
                elapsed = time.perf_counter() - t_wall_start
                print(f"\r[replay] {pct:5.1f}%  step {i+1:>5}/{n_rows}  "
                      f"t_csv={t_csv:.2f}s  elapsed={elapsed:.1f}s   ",
                      end="", flush=True)

    except KeyboardInterrupt:
        print(f"\n[replay] Dừng sớm tại step {n_sent}/{n_rows}.")
    else:
        print(f"\n[replay] ✅ Hoàn thành. Đã phát {n_sent:,} steps.")


if __name__ == "__main__":
    main()
