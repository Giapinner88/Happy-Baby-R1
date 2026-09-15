#!/usr/bin/env python3
"""Đọc một episode theo định dạng Unitree EpisodeWriter và báo cáo nội dung.

Dùng cho PLAN.md Giai đoạn 1: sau khi thu được vài episode bằng stack Unitree,
chạy script này để trả lời ba câu hỏi của cổng ra — states khác actions ở đâu,
ảnh được tham chiếu thế nào, và episode dài bao nhiêu bước.

    python3 experiments/r1_dataset/quest3_sim_v1/tools/inspect_episode.py \
        <thư-mục-chứa-data.json>

Chỉ dùng thư viện chuẩn, nên chạy được trong bất kỳ môi trường Python nào.
Script chỉ đọc; nó không sửa gì trong episode.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


GROUPS = ("left_arm", "right_arm", "left_ee", "right_ee", "body")


def _qpos(block: object, group: str) -> list[float]:
    """Lấy qpos của một nhóm khớp; trả về [] nếu thiếu hoặc sai kiểu."""

    if not isinstance(block, dict):
        return []
    entry = block.get(group)
    if not isinstance(entry, dict):
        return []
    values = entry.get("qpos")
    if not isinstance(values, list):
        return []
    return [float(v) for v in values if isinstance(v, (int, float))]


def _rms(a: list[float], b: list[float]) -> float | None:
    """RMS của hiệu hai vector cùng độ dài; None nếu không so sánh được."""

    if not a or len(a) != len(b):
        return None
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)) / len(a))


def inspect(episode_dir: Path) -> int:
    data_path = episode_dir / "data.json"
    if not data_path.is_file():
        print(f"Không tìm thấy {data_path}", file=sys.stderr)
        return 2
    try:
        payload = json.loads(data_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        # EpisodeWriter ghi dần và chỉ đóng ngoặc khi save_episode chạy xong.
        # Một episode bị cắt giữa chừng sẽ hỏng đúng theo kiểu này.
        print(f"data.json không phải JSON hợp lệ: {exc}", file=sys.stderr)
        print("Nếu episode bị dừng đột ngột, mảng \"data\" có thể chưa được đóng.", file=sys.stderr)
        return 2

    info = payload.get("info", {})
    text = payload.get("text", {})
    items = payload.get("data", [])

    print(f"Episode: {episode_dir}")
    print(f"  bước ghi được : {len(items)}")
    print(f"  fps khai báo  : {info.get('image', {}).get('fps')}")
    print(f"  kích thước ảnh: {info.get('image', {}).get('width')}x{info.get('image', {}).get('height')}")
    print(f"  thời lượng ước tính: {len(items) / float(info.get('image', {}).get('fps') or 30):.1f} s")
    print()
    print("Nhãn task:")
    for key in ("goal", "desc", "steps"):
        if text.get(key):
            print(f"  {key:5s}: {text[key]}")
    print()

    joint_names = info.get("joint_names", {})
    print("Bộ khớp khai báo:")
    for group in GROUPS:
        names = joint_names.get(group) or []
        print(f"  {group:9s}: {len(names)} khớp {names if names else '(rỗng)'}")
    print()

    if not items:
        print("Episode không có bước nào. Không kiểm tra tiếp được.")
        return 1

    # Ảnh: có tham chiếu nào trỏ tới file không tồn tại không.
    referenced = 0
    missing: list[str] = []
    camera_keys: set[str] = set()
    for item in items:
        colors = item.get("colors") or {}
        for key, rel in colors.items():
            camera_keys.add(key)
            referenced += 1
            if not (episode_dir / rel).is_file():
                missing.append(rel)
    on_disk = len(list((episode_dir / "colors").glob("*"))) if (episode_dir / "colors").is_dir() else 0

    print("Ảnh:")
    print(f"  camera        : {sorted(camera_keys) or '(không có)'}")
    print(f"  tham chiếu    : {referenced}")
    print(f"  file trên đĩa : {on_disk}")
    if missing:
        print(f"  THIẾU FILE    : {len(missing)}, ví dụ {missing[:3]}")
    elif referenced and referenced != on_disk:
        print("  CẢNH BÁO      : số tham chiếu khác số file trên đĩa")
    print()

    # states vs actions — điểm dễ ghi sai nhất.
    print("states so với actions (RMS hiệu, radian):")
    identical_everywhere = True
    for group in GROUPS:
        residuals = []
        for item in items:
            r = _rms(_qpos(item.get("states"), group), _qpos(item.get("actions"), group))
            if r is not None:
                residuals.append(r)
        if not residuals:
            print(f"  {group:9s}: không có dữ liệu so sánh được")
            continue
        mean = sum(residuals) / len(residuals)
        peak = max(residuals)
        if peak > 1e-9:
            identical_everywhere = False
        print(f"  {group:9s}: trung bình {mean:.6f}  lớn nhất {peak:.6f}  ({len(residuals)} bước)")
    print()

    if identical_everywhere:
        print("CẢNH BÁO: states và actions giống hệt nhau ở mọi bước.")
        print("  Nghĩa là cả hai đang được ghi từ cùng một nguồn. Dataset kiểu này")
        print("  không dạy được gì: chính sách học xong sẽ chỉ đoán lại trạng thái")
        print("  hiện tại thay vì đoán lệnh điều khiển.")
    else:
        print("states khác actions — đúng như mong đợi.")
        print("  actions là lệnh gửi xuống, states là vị trí đo được sau khi khớp")
        print("  đuổi theo lệnh đó. Hiệu giữa chúng chính là sai số bám.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("episode_dir", type=Path, help="Thư mục episode_XXXX chứa data.json")
    args = parser.parse_args()
    return inspect(args.episode_dir.expanduser().resolve())


if __name__ == "__main__":
    raise SystemExit(main())
