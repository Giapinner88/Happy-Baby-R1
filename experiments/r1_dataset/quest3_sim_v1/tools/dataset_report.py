#!/usr/bin/env python3
"""Gộp mọi run của một protocol và chấm cổng chất lượng trên dữ liệu THẬT.

D002 đòi ba thứ trước khi được phép huấn luyện: đủ episode, cân bằng chữ số, và
không rò rỉ vị trí. Cho tới giờ ba con số đó mới chỉ được kiểm trên dữ liệu mô
phỏng từ `MarkerRandomizer`. Công cụ này đọc episode đã ghi ra đĩa và chấm lại.

    python3 experiments/r1_dataset/quest3_sim_v1/tools/dataset_report.py --protocol d002

Thoát 0 khi mọi cổng đạt, 1 khi có cổng trượt, 2 khi không đọc được dữ liệu.
Chỉ dùng thư viện chuẩn nên chạy được ở mọi môi trường.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
EXPERIMENT = ROOT / "experiments" / "r1_dataset" / "quest3_sim_v1"
CHI_SQUARE_CRITICAL_P05_DF3 = 7.815


def load_accepted_episodes(protocol: str) -> tuple[list[dict], list[str]]:
    """Đọc mọi episode ĐƯỢC GIỮ. Episode bị loại nằm ở nhánh khác nên bỏ qua."""

    episodes: list[dict] = []
    problems: list[str] = []
    root = EXPERIMENT / protocol.upper() / "runs"
    for data_path in sorted(root.glob("*/episodes/*/data.json")):
        try:
            payload = json.loads(data_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            problems.append(f"không đọc được {data_path}: {exc}")
            continue
        info = payload.get("info") or {}
        items = payload.get("data") or []
        # Ảnh phải có thật, không chỉ được tham chiếu. Một dataset thiếu ảnh
        # hỏng lúc huấn luyện chứ không hỏng lúc kiểm, nên phải bắt ở đây.
        missing = sum(
            1
            for item in items
            for relative in (item.get("colors") or {}).values()
            if not (data_path.parent / relative).is_file()
        )
        if missing:
            problems.append(f"{data_path.parent.name}: thiếu {missing} file ảnh")
        episodes.append(
            {
                "run": data_path.parents[2].name,
                "episode": data_path.parent.name,
                "path": data_path,
                "item_count": len(items),
                "measured_fps": (info.get("image") or {}).get("fps"),
                "task": info.get("task"),
                "missing_images": missing,
            }
        )
    return episodes, problems


def chi_square_uniform(counts: dict[int, int], categories: int) -> float:
    total = sum(counts.values())
    if total == 0:
        return 0.0
    expected = total / categories
    return sum(
        (counts.get(category, 0) - expected) ** 2 / expected for category in range(categories)
    )


def build_report(protocol: str, episodes: list[dict]) -> dict:
    with_task = [e for e in episodes if e["task"]]
    digit_counts = Counter(e["task"]["target_digit"] for e in with_task)
    slot_counts = Counter(e["task"]["target_slot"] for e in with_task)
    slot_total = len(
        (with_task[0]["task"]["slot_positions_m"]) if with_task else []
    ) or 4
    # Hoán vị = chữ số nào ở ô nào. Tập test phải giữ lại theo hoán vị chưa từng
    # thấy, nên phải biết có bao nhiêu hoán vị phân biệt.
    permutations = Counter(
        tuple(sorted(e["task"]["digit_to_slot"].items())) for e in with_task
    )
    per_digit_slots: dict[int, Counter] = {}
    for e in with_task:
        per_digit_slots.setdefault(e["task"]["target_digit"], Counter())[
            e["task"]["target_slot"]
        ] += 1
    fps = [e["measured_fps"] for e in episodes if e["measured_fps"]]
    return {
        "protocol": protocol,
        "episode_count": len(episodes),
        "episode_with_task_count": len(with_task),
        "item_count": sum(e["item_count"] for e in episodes),
        "item_count_per_episode": {
            "min": min((e["item_count"] for e in episodes), default=0),
            "max": max((e["item_count"] for e in episodes), default=0),
        },
        "measured_fps": {
            "min": round(min(fps), 2) if fps else None,
            "max": round(max(fps), 2) if fps else None,
        },
        "target_digit_counts": dict(sorted(digit_counts.items())),
        "target_digit_missing": sorted(set(range(1, 10)) - set(digit_counts)),
        "target_slot_counts": dict(sorted(slot_counts.items())),
        "slot_uniformity_chi_square": round(chi_square_uniform(slot_counts, slot_total), 3),
        "slot_uniformity_critical_value": CHI_SQUARE_CRITICAL_P05_DF3,
        "distinct_permutations": len(permutations),
        "permutation_reuse_max": max(permutations.values()) if permutations else 0,
        "per_digit_slot_counts": {
            digit: dict(sorted(row.items())) for digit, row in sorted(per_digit_slots.items())
        },
        "missing_image_total": sum(e["missing_images"] for e in episodes),
    }


def check_gates(report: dict, min_episodes: int, min_per_digit: int) -> list[tuple[str, bool, str]]:
    digit_counts = report["target_digit_counts"]
    weakest = min(digit_counts.values()) if digit_counts else 0
    chi = report["slot_uniformity_chi_square"]
    # Tập test giữ theo hoán vị chưa từng thấy: cần ít nhất vài hoán vị phân biệt
    # thì mới chia được, nếu không "khái quát hoá" chỉ là chia ngẫu nhiên trá hình.
    return [
        (
            f"đủ {min_episodes} episode",
            report["episode_count"] >= min_episodes,
            f"{report['episode_count']}/{min_episodes}",
        ),
        (
            f"mỗi chữ số >= {min_per_digit}",
            weakest >= min_per_digit and not report["target_digit_missing"],
            f"yếu nhất {weakest}, thiếu hẳn {report['target_digit_missing'] or 'không'}",
        ),
        (
            "không rò rỉ vị trí",
            chi < CHI_SQUARE_CRITICAL_P05_DF3,
            f"chi-bình-phương {chi} < {CHI_SQUARE_CRITICAL_P05_DF3}",
        ),
        (
            "mọi episode có khối task",
            report["episode_with_task_count"] == report["episode_count"],
            f"{report['episode_with_task_count']}/{report['episode_count']}",
        ),
        (
            "không thiếu ảnh",
            report["missing_image_total"] == 0,
            f"thiếu {report['missing_image_total']}",
        ),
        (
            "đủ hoán vị để chia tập test",
            report["distinct_permutations"] >= max(3, report["episode_count"] // 5),
            f"{report['distinct_permutations']} hoán vị phân biệt",
        ),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--protocol", default="d002")
    parser.add_argument("--min-episodes", type=int, default=100)
    parser.add_argument("--min-per-digit", type=int, default=8)
    parser.add_argument("--json-out", type=Path, help="Ghi báo cáo đầy đủ ra file.")
    parser.add_argument(
        "--suggest-priority",
        action="store_true",
        help="In danh sách chữ số cần ưu tiên cho phiên thu kế tiếp, thiếu nhiều trước.",
    )
    args = parser.parse_args()

    episodes, problems = load_accepted_episodes(args.protocol)
    if not episodes:
        print(f"Không tìm thấy episode nào cho protocol {args.protocol!r}.", file=sys.stderr)
        return 2

    report = build_report(args.protocol, episodes)
    print(f"=== {args.protocol.upper()} — {report['episode_count']} episode, {report['item_count']} item")
    print(f"item mỗi episode : {report['item_count_per_episode']['min']}–{report['item_count_per_episode']['max']}")
    print(f"nhịp đo được     : {report['measured_fps']['min']}–{report['measured_fps']['max']} Hz")
    print(f"chữ số mục tiêu  : {report['target_digit_counts']}")
    print(f"ô mục tiêu       : {report['target_slot_counts']}")
    print(f"hoán vị phân biệt: {report['distinct_permutations']} (dùng lại nhiều nhất {report['permutation_reuse_max']} lần)")
    if problems:
        print("\nVấn đề dữ liệu:")
        for line in problems:
            print(f"  {line}")

    print("\n=== Cổng chất lượng")
    gates = check_gates(report, args.min_episodes, args.min_per_digit)
    for name, passed, detail in gates:
        print(f"  [{'ĐẠT ' if passed else 'TRƯỢT'}] {name:<32} {detail}")
    report["gates"] = [{"name": n, "passed": p, "detail": d} for n, p, d in gates]

    if args.suggest_priority:
        counts = report["target_digit_counts"]
        target = max(counts.values(), default=0)
        deficit = {d: target - counts.get(d, 0) for d in range(1, 10)}
        order = [d for d, gap in sorted(deficit.items(), key=lambda kv: (-kv[1], kv[0])) for _ in range(gap)]
        print("\n=== Ưu tiên cho phiên kế tiếp")
        print(f"  thiếu so với chữ số nhiều nhất ({target}): "
              f"{ {d: g for d, g in sorted(deficit.items()) if g} }")
        print(f"  --target-digit-priority {','.join(str(d) for d in order)}" if order
              else "  đã cân bằng, không cần ưu tiên")

    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"\nBáo cáo đầy đủ: {args.json_out}")

    return 0 if all(p for _n, p, _d in gates) else 1


if __name__ == "__main__":
    raise SystemExit(main())
