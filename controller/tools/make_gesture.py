#!/usr/bin/env python3
"""
make_gesture.py — Biến file capture (từ tools/record_motion) thành gesture npz chuẩn
cho overlay tay khi Locomotion (policies/gestures/<ten>.npz | <ten>.loop.npz).

Quy trình "dễ lưu, dễ dùng":
  1) Xem có gì:      python3 tools/make_gesture.py list motions/captures_20260725
  2) Soi 1 file:     python3 tools/make_gesture.py show motions/captures_20260725/capture_005_L2_X.npz
     (in biên độ tay theo thời gian dạng ASCII -> chọn khoảng frame đẹp)
  3) Cắt & lưu:      python3 tools/make_gesture.py cut motions/.../capture_005_L2_X.npz \
                        --start 120 --end 400 --name vaytay --loop \
                        --smooth-sigma 2 --resample-factor 2
     -> policies/gestures/vaytay.loop.npz (tự kiểm dải khớp, tốc độ, độ khép vòng)
  4) Gán slot: thêm vào config/gestures.yaml:   gesture_slot_3: vaytay
     (bấm double-click nút tương ứng slot 3 = Trái). Khởi động lại run_r1 để nạp.

Kiểu gesture:
  --loop      : động tác LẶP (vẫy tay...) — nên cắt đúng 1 chu kỳ; --loop-blend N
                crossfade N frame cuối vào N frame đầu cho vòng lặp mượt.
  (mặc định)  : động tác GIỮ — chạy tới frame cuối rồi giữ nguyên tư thế đó
                (bắt tay, trái tim...). Frame cuối chính là tư thế giữ.

File ra giữ nguyên layout 24 khớp + fps (GesturePlayer chỉ đọc 10 cột tay 14..23).
"""
import argparse
import os
import sys

import numpy as np

# Khớp tay trong layout policy 24 khớp
ARM_COLS = list(range(14, 24))
ARM_NAMES = ["L_sh_pitch", "L_sh_roll", "L_sh_yaw", "L_elbow", "L_wr_roll",
             "R_sh_pitch", "R_sh_roll", "R_sh_yaw", "R_elbow", "R_wr_roll"]

# Dải khớp tay thật (r1.xml) + margin 0.05 rad — khớp bảng trong PLAN_teleop_webxr_bridge.md
M = 0.05
ARM_LIMITS = [(-3.1416 + M, 2.0944 - M), (-0.2269 + M, 2.4784 - M), (-1.9199 + M, 1.9199 - M),
              (-0.9756 + M, 2.1852 - M), (-1.9199 + M, 1.9199 - M),
              (-3.1416 + M, 2.0944 - M), (-2.4785 + M, 0.2268 - M), (-1.9199 + M, 1.9199 - M),
              (-0.9756 + M, 2.1852 - M), (-1.9199 + M, 1.9199 - M)]

DEFAULT_ARM = np.array([0.35, 0.18, 0.0, 0.87, 0.0, 0.35, -0.18, 0.0, 0.87, 0.0])

MAX_ARM_SPEED = 6.0   # rad/s: cảnh báo nếu gesture nhanh hơn (dễ gây lắc khi đang đi)


def gaussian_smooth(values, sigma, periodic):
    """Lọc Gaussian zero-phase; loop dùng biên tuần hoàn để không tạo mối nối."""
    if sigma <= 0.0 or len(values) < 3:
        return values.copy()
    radius = max(1, int(np.ceil(3.0 * sigma)))
    offsets = np.arange(-radius, radius + 1)
    weights = np.exp(-0.5 * (offsets / sigma) ** 2)
    weights /= weights.sum()

    if periodic:
        out = np.zeros_like(values)
        for offset, weight in zip(offsets, weights):
            out += weight * np.roll(values, offset, axis=0)
        return out

    padded = np.pad(values, ((radius, radius), (0, 0)), mode="edge")
    out = np.zeros_like(values)
    for k, weight in enumerate(weights):
        out += weight * padded[k:k + len(values)]
    return out


def cubic_resample(values, factor, periodic):
    """Nội suy Catmull-Rom; loop dùng chỉ số tuần hoàn để liên tục qua mép."""
    if factor <= 1 or len(values) < 4:
        return values.copy()

    n = len(values)
    out_n = n * factor if periodic else (n - 1) * factor + 1
    x = np.arange(out_n, dtype=np.float64) / factor
    i1 = np.floor(x).astype(np.int64)
    t = (x - i1)[:, None]

    if periodic:
        i1 %= n
        i0 = (i1 - 1) % n
        i2 = (i1 + 1) % n
        i3 = (i1 + 2) % n
    else:
        i1 = np.clip(i1, 0, n - 1)
        i0 = np.clip(i1 - 1, 0, n - 1)
        i2 = np.clip(i1 + 1, 0, n - 1)
        i3 = np.clip(i1 + 2, 0, n - 1)

    p0, p1, p2, p3 = values[i0], values[i1], values[i2], values[i3]
    return 0.5 * (
        2.0 * p1
        + (-p0 + p2) * t
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t ** 2
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t ** 3
    )


def prepend_default_arm(seg, fps, seconds):
    """Thêm đoạn ease-in quintic từ tay mặc định tới frame đầu của capture."""
    frames = int(round(seconds * fps))
    if frames < 2:
        return seg
    ramp = np.repeat(seg[:1], frames, axis=0)
    u = np.linspace(0.0, 1.0, frames)[:, None]
    blend = u ** 3 * (10.0 - 15.0 * u + 6.0 * u ** 2)
    target = seg[0, ARM_COLS][None, :]
    ramp[:, ARM_COLS] = DEFAULT_ARM[None, :] + blend * (target - DEFAULT_ARM[None, :])
    return np.concatenate([ramp[:-1], seg], axis=0)


def append_default_arm(seg, fps, seconds):
    """Thêm đoạn ease-out quintic từ frame cuối trở về tay mặc định."""
    frames = int(round(seconds * fps))
    if frames < 2:
        return seg
    ramp = np.repeat(seg[-1:], frames, axis=0)
    u = np.linspace(0.0, 1.0, frames)[:, None]
    blend = u ** 3 * (10.0 - 15.0 * u + 6.0 * u ** 2)
    source = seg[-1, ARM_COLS][None, :]
    ramp[:, ARM_COLS] = source + blend * (DEFAULT_ARM[None, :] - source)
    return np.concatenate([seg, ramp[1:]], axis=0)


def append_loop_bridge(seg, fps, seconds):
    """Thêm cầu quintic từ frame cuối về frame đầu để loop khép kín, zero-velocity."""
    frames = int(round(seconds * fps))
    if frames < 3:
        return seg
    bridge = np.repeat(seg[-1:], frames, axis=0)
    u = np.linspace(0.0, 1.0, frames)[:, None]
    blend = u ** 3 * (10.0 - 15.0 * u + 6.0 * u ** 2)
    source = seg[-1][None, :]
    target = seg[0][None, :]
    bridge = source + blend * (target - source)
    # Không lặp lại hai endpoint: seg đã chứa source; player tự nối frame cuối
    # (gần target) về frame đầu (target).
    return np.concatenate([seg, bridge[1:-1]], axis=0)


def load(path):
    d = np.load(path)
    if "joint_pos" not in d:
        sys.exit(f"LOI: {path} khong co key 'joint_pos'")
    jp = d["joint_pos"].astype(np.float64)
    if jp.ndim != 2 or jp.shape[1] != 24:
        sys.exit(f"LOI: joint_pos shape {jp.shape}, can (frames, 24)")
    fps = float(d["fps"][0]) if "fps" in d else 50.0
    return jp, fps


def cmd_list(args):
    files = sorted(f for f in os.listdir(args.folder) if f.endswith(".npz"))
    if not files:
        print(f"(khong co .npz trong {args.folder})")
        return
    print(f"{'FILE':38s} {'FRAMES':>7s} {'FPS':>4s} {'GIAY':>6s}  {'TAY CHUYEN DONG (rad)':s}")
    for f in files:
        jp, fps = load(os.path.join(args.folder, f))
        arm = jp[:, ARM_COLS]
        motion = float((arm.max(0) - arm.min(0)).max())  # biên độ lớn nhất trong 10 khớp tay
        mark = "  <-- co dong tac tay" if motion > 0.25 else ""
        print(f"{f:38s} {jp.shape[0]:7d} {fps:4.0f} {jp.shape[0]/fps:6.1f}  {motion:5.2f}{mark}")


def cmd_show(args):
    jp, fps = load(args.file)
    arm = jp[:, ARM_COLS]
    n = jp.shape[0]
    # Biên độ tổng của tay mỗi frame so với default -> đồ thị ASCII để chọn khoảng cắt
    dev = np.abs(arm - DEFAULT_ARM).max(axis=1)
    width = 60
    bins = min(n, 100)
    idxs = np.linspace(0, n - 1, bins).astype(int)
    print(f"{args.file}: {n} frames @ {fps:.0f}fps ({n/fps:.1f}s)")
    print(f"Lech tay so voi default (max/10 khop) — moi dong ~{n/bins:.0f} frame:")
    for i in idxs:
        bar = "#" * int(dev[i] / max(1e-6, dev.max()) * width)
        print(f"  f{i:5d} t={i/fps:6.2f}s |{bar:<{width}s}| {dev[i]:.2f}")
    print("\nGoi y: chon --start/--end quanh vung co '#' cao (dong tac), tranh mep vao/ra.")


def cmd_cut(args):
    parts = []
    fps = None
    for path in args.file:
        part, part_fps = load(path)
        if fps is None:
            fps = part_fps
        elif abs(part_fps - fps) > 1e-6:
            sys.exit(f"LOI: fps khong khop ({fps} va {part_fps})")
        parts.append(part)
    jp = np.concatenate(parts, axis=0)
    if args.fps_override > 0.0:
        fps = args.fps_override
    n = jp.shape[0]
    s, e = args.start, args.end if args.end > 0 else n - 1
    if not (0 <= s < e < n):
        sys.exit(f"LOI: khoang cat [{s},{e}] ngoai [0,{n-1}]")
    seg = jp[s:e + 1].copy()

    if args.prepend_default_s > 0.0:
        seg = prepend_default_arm(seg, fps, args.prepend_default_s)
    if args.append_default_s > 0.0:
        seg = append_default_arm(seg, fps, args.append_default_s)

    # Loop: crossfade N frame cuoi vao N frame dau cho vong lap muot
    if args.loop and args.loop_bridge_s > 0.0:
        seg = append_loop_bridge(seg, fps, args.loop_bridge_s)
    elif args.loop and args.loop_blend > 0:
        b = min(args.loop_blend, len(seg) // 3)
        for k in range(b):
            w = (k + 1) / (b + 1)
            seg[-b + k] = (1 - w) * seg[-b + k] + w * seg[k]

    # Lọc zero-phase trước rồi nội suy cubic. Với loop, cả hai phép dùng biên
    # tuần hoàn nên không tạo cú giật khi con trỏ quay từ cuối về đầu.
    if args.smooth_sigma > 0.0:
        seg = gaussian_smooth(seg, args.smooth_sigma, args.loop)
    if args.resample_factor > 1:
        seg = cubic_resample(seg, args.resample_factor, args.loop)
        fps *= args.resample_factor

    arm = seg[:, ARM_COLS]

    # === KIEM TRA CHAT LUONG ===
    ok = True
    # 1. Dai khop
    for j, (lo, hi) in enumerate(ARM_LIMITS):
        mn, mx = arm[:, j].min(), arm[:, j].max()
        if mn < lo or mx > hi:
            print(f"  ⚠ {ARM_NAMES[j]}: [{mn:.2f},{mx:.2f}] VUOT dai [{lo:.2f},{hi:.2f}] -> se bi clamp")
            ok = False
    # 2. Toc do khop
    vel = np.abs(np.diff(arm, axis=0)) * fps
    vmax = vel.max()
    j_fast = ARM_NAMES[int(np.unravel_index(vel.argmax(), vel.shape)[1])]
    if vmax > MAX_ARM_SPEED:
        print(f"  ⚠ Toc do dinh {vmax:.1f} rad/s ({j_fast}) > {MAX_ARM_SPEED} -> de gay lac khi dang di")
        ok = False
    else:
        print(f"  ✓ Toc do dinh {vmax:.1f} rad/s ({j_fast})")
    # 3. Loop: do khep vong | Giu: frame cuoi la tu the giu
    if args.loop:
        gap = float(np.abs(arm[0] - arm[-1]).max())
        if gap > 0.15:
            print(f"  ⚠ Vong lap ho {gap:.2f} rad (dau vs cuoi) -> tang --loop-blend hoac chon lai khoang")
            ok = False
        else:
            print(f"  ✓ Vong lap khep, ho {gap:.2f} rad")
    else:
        far = float(np.abs(arm[-1] - DEFAULT_ARM).max())
        print(f"  • Tu the GIU (frame cuoi) lech default {far:.2f} rad"
              + (" — xa, robot se giu tay o do toi khi bam lai" if far > 0.8 else ""))
    # 4. Buoc nhay dau vao (blend-in cua player se xu ly, chi thong bao)
    start_gap = float(np.abs(arm[0] - DEFAULT_ARM).max())
    print(f"  • Frame dau lech default {start_gap:.2f} rad (blend-in {0.4}s cua player se lam muot)")

    out_dir = args.out_dir
    os.makedirs(out_dir, exist_ok=True)
    suffix = ".loop.npz" if args.loop else ".npz"
    out = os.path.join(out_dir, args.name + suffix)
    if os.path.exists(out) and not args.force:
        sys.exit(f"LOI: {out} da ton tai (dung --force de ghi de)")
    np.savez(out, joint_pos=seg.astype(np.float32), fps=np.array([fps], dtype=np.float64))
    print(f"\n{'✓ DAT' if ok else '⚠ LUU VOI CANH BAO'}: {out} ({len(seg)} frames @ {fps:.0f}fps, "
          f"{len(seg)/fps:.1f}s, {'LAP' if args.loop else 'GIU frame cuoi'})")
    print(f"Gan slot: them vao config/gestures.yaml ->  gesture_slot_<N>: {args.name}")
    print("Roi khoi dong lai run_r1 de nap. (Slot: 1=Len 2=Xuong 3=Trai 4=Phai 5=A 6=B 7=X 8=Y)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("list", help="liet ke cac capture trong folder")
    p.add_argument("folder")
    p.set_defaults(fn=cmd_list)

    p = sub.add_parser("show", help="do thi ASCII bien do tay -> chon khoang cat")
    p.add_argument("file")
    p.set_defaults(fn=cmd_show)

    p = sub.add_parser("cut", help="cat khoang frame -> gesture npz chuan (tu kiem tra)")
    p.add_argument("file", nargs="+", help="mot hoac nhieu capture, ghep theo thu tu")
    p.add_argument("--start", type=int, required=True, help="frame bat dau")
    p.add_argument("--end", type=int, default=-1, help="frame ket thuc (mac dinh: het file)")
    p.add_argument("--name", required=True, help="ten gesture (vd: vaytay)")
    p.add_argument("--loop", action="store_true", help="kieu LAP (mac dinh: GIU frame cuoi)")
    p.add_argument("--loop-blend", type=int, default=0,
                   help="crossfade N frame kiểu cũ (mặc định 0; nên dùng --smooth-sigma)")
    p.add_argument("--loop-bridge-s", type=float, default=0.0,
                   help="them cau quintic N giay tu frame cuoi ve frame dau cho loop")
    p.add_argument("--fps-override", type=float, default=0.0,
                   help="ghi de fps metadata cua capture khi recorder gan sai tan so")
    p.add_argument("--smooth-sigma", type=float, default=0.0,
                   help="loc Gaussian zero-phase, sigma theo frame goc (vd: 2.0)")
    p.add_argument("--resample-factor", type=int, default=1,
                   help="noi suy cubic tang tan so mau N lan (vd: 2 = 50->100fps)")
    p.add_argument("--prepend-default-s", type=float, default=0.0,
                   help="them ease-in quintic tu tay mac dinh trong N giay")
    p.add_argument("--append-default-s", type=float, default=0.0,
                   help="them ease-out quintic ve tay mac dinh trong N giay")
    p.add_argument("--out-dir", default=os.path.join(os.path.dirname(__file__), "..", "policies", "gestures"))
    p.add_argument("--force", action="store_true")
    p.set_defaults(fn=cmd_cut)

    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
