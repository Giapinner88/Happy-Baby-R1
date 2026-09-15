"""Ngẫu nhiên hoá bố cục tấm số theo từng episode.

Rủi ro trung tâm của task VLA "chỉ tới con số được yêu cầu": nếu số 2 luôn nằm
ở ô thứ hai, mô hình học ánh xạ `"2" -> ô thứ hai` mà không bao giờ nhìn chữ số.
Nó đạt điểm cao rồi hỏng ngay khi đổi chỗ các tấm, và ta không biết cho tới lúc
đó. Module này làm lối tắt ấy không dùng được.

Phần chọn bố cục ở đây là **thuần tuý**: không import Isaac, không đụng USD, chỉ
sinh ra một mô tả. Nhờ vậy các bảo đảm chống lối tắt kiểm tra được bằng test
thường. Phần đặt tấm vào cảnh nằm ở `apply_layout`, tách riêng.

Cân bằng chữ số mục tiêu dùng hàng đợi xáo trộn chứ không phải bốc ngẫu nhiên
độc lập: bốc độc lập cho phân bố đều về kỳ vọng nhưng lệch đáng kể trên một
phiên trăm episode, mà lệch lớp là thứ ta phải loại trừ chứ không phải hy vọng.
"""

from __future__ import annotations

import random
import re
from dataclasses import dataclass, field
from typing import Sequence


DEFAULT_PROMPTS = (
    "Reach the right hand to number {n}.",
    "Touch the plate showing {n}.",
    "Move the right hand onto marker {n}.",
    "Put your right hand on number {n}.",
    "Go to the {n} plate.",
)


@dataclass(frozen=True)
class MarkerLayout:
    """Một bố cục cụ thể cho một episode."""

    target_digit: int
    digit_to_slot: dict[int, int]
    slot_positions_m: tuple[tuple[float, float, float], ...]
    prompt_template: str
    prompt: str

    @property
    def target_slot(self) -> int:
        return self.digit_to_slot[self.target_digit]

    @property
    def target_position_m(self) -> tuple[float, float, float]:
        return self.slot_positions_m[self.target_slot]

    def as_task_record(self) -> dict[str, object]:
        """Khối ghi vào `info.task` của episode, đủ để chấm điểm tự động."""

        return {
            "target_digit": self.target_digit,
            "digit_to_slot": {str(k): v for k, v in sorted(self.digit_to_slot.items())},
            "slot_positions_m": [list(p) for p in self.slot_positions_m],
            "target_slot": self.target_slot,
            "target_position_m": list(self.target_position_m),
            "prompt_template": self.prompt_template,
            "prompt": self.prompt,
        }


@dataclass
class MarkerRandomizer:
    """Sinh bố cục cho từng episode, cân bằng chữ số mục tiêu trên cả phiên."""

    digits: Sequence[int]
    slot_x_m: float
    slot_y_m: Sequence[float]
    slot_z_m: float
    slot_y_jitter_m: float = 0.0
    prompt_templates: Sequence[str] = DEFAULT_PROMPTS
    seed: int | None = None
    priority_digits: Sequence[int] = ()
    """Chữ số phải được dùng làm mục tiêu trước, theo đúng thứ tự khai.

    Hàng đợi xáo trộn chỉ cân bằng TRONG một phiên: mỗi tiến trình tạo một
    randomizer mới, và phiên thực tế chỉ dài 4-7 episode chứ không phải 9. Trên
    bốn phiên đầu của D002, chữ số 3 vì thế chưa từng được chọn. Danh sách này
    cho phép phiên kế tiếp bù đúng những chữ số còn thiếu, tính từ dữ liệu đã
    thu chứ không phải từ hy vọng.
    """
    fixed_layout_digits: Sequence[int] = ()
    """Bố cục cố định theo thứ tự ô 0..N-1; rỗng thì tiếp tục random."""
    prompt_override: str | None = None
    """Prompt thủ công; prompt literal có đúng một chữ số sẽ tự đặt target."""
    _rng: random.Random = field(init=False)
    _target_queue: list[int] = field(default_factory=list, init=False)
    _prompt_target_digit: int | None = field(default=None, init=False)
    history: list[MarkerLayout] = field(default_factory=list, init=False)

    def __post_init__(self) -> None:
        if len(self.digits) < len(self.slot_y_m):
            raise ValueError(
                f"Cần ít nhất {len(self.slot_y_m)} chữ số cho {len(self.slot_y_m)} ô, "
                f"chỉ có {len(self.digits)}."
            )
        if len(set(self.digits)) != len(self.digits):
            raise ValueError("Danh sách chữ số có phần tử trùng.")
        if self.slot_y_jitter_m < 0.0:
            raise ValueError("slot_y_jitter_m không được âm.")
        if not self.prompt_templates:
            raise ValueError("Phải có ít nhất một mẫu câu lệnh.")
        unknown = [d for d in self.priority_digits if d not in set(self.digits)]
        if unknown:
            raise ValueError(f"priority_digits có chữ số ngoài kho: {unknown}")
        if self.fixed_layout_digits:
            fixed = list(self.fixed_layout_digits)
            if len(fixed) != self.slot_count:
                raise ValueError(
                    f"fixed_layout_digits phải có đúng {self.slot_count} chữ số, "
                    f"nhận {len(fixed)}."
                )
            if len(set(fixed)) != len(fixed):
                raise ValueError("fixed_layout_digits có chữ số trùng.")
            fixed_unknown = [d for d in fixed if d not in set(self.digits)]
            if fixed_unknown:
                raise ValueError(f"fixed_layout_digits có chữ số ngoài kho: {fixed_unknown}")
            absent_targets = [d for d in self.priority_digits if d not in set(fixed)]
            if absent_targets:
                raise ValueError(
                    "priority_digits phải nằm trong fixed_layout_digits; "
                    f"không có trên bàn: {absent_targets}"
                )
        if self.prompt_override is not None:
            if not self.prompt_override.strip():
                raise ValueError("prompt_override không được rỗng.")
            if "{n}" not in self.prompt_override:
                prompt_digits = {
                    int(value)
                    for value in re.findall(r"(?<!\d)([1-9])(?!\d)", self.prompt_override)
                }
                explicit_targets = set(self.priority_digits)
                if len(explicit_targets) > 1:
                    raise ValueError(
                        "prompt ngữ nghĩa chỉ nhận đúng một target tường minh; "
                        f"nhận {sorted(explicit_targets)}."
                    )
                if explicit_targets:
                    self._prompt_target_digit = next(iter(explicit_targets))
                    if len(prompt_digits) == 1 and next(iter(prompt_digits)) != self._prompt_target_digit:
                        raise ValueError(
                            f"prompt ghi số {next(iter(prompt_digits))} nhưng target tường minh "
                            f"là {self._prompt_target_digit}."
                        )
                elif len(prompt_digits) == 1:
                    self._prompt_target_digit = next(iter(prompt_digits))
                else:
                    raise ValueError(
                        "không thể suy một target duy nhất từ prompt; hãy truyền một "
                        "TARGET_DIGIT tường minh."
                    )
                if self._prompt_target_digit not in set(self.digits):
                    raise ValueError(
                        f"chữ số trong prompt không nằm trong kho: {self._prompt_target_digit}"
                    )
                if self.fixed_layout_digits and self._prompt_target_digit not in set(
                    self.fixed_layout_digits
                ):
                    raise ValueError(
                        f"số {self._prompt_target_digit} trong prompt không có trên bàn."
                    )
        self._rng = random.Random(self.seed)
        self._priority = list(self.priority_digits)

    @property
    def slot_count(self) -> int:
        return len(self.slot_y_m)

    def _next_target(self) -> int:
        """Chữ số mục tiêu kế tiếp, lấy từ hàng đợi xáo trộn.

        Chọn mục tiêu TRƯỚC rồi mới bốc các tấm còn lại, chứ không bốc tấm trước
        rồi tìm mục tiêu trong số đó. Cách sau nghe hợp lý nhưng phá vỡ vòng
        tròn: chỉ 4 trong 9 chữ số có mặt mỗi lượt, nên đầu hàng đợi thường
        không có mặt và phải nhảy cóc. Đo trên 180 episode, cách đó cho chênh
        lệch 7 giữa chữ số nhiều nhất và ít nhất; cách này cho chênh lệch 1.
        """

        if self._prompt_target_digit is not None:
            return self._prompt_target_digit
        if self._priority:
            return self._priority.pop(0)
        if not self._target_queue:
            self._target_queue = list(self.fixed_layout_digits or self.digits)
            self._rng.shuffle(self._target_queue)
        return self._target_queue.pop()

    def next_layout(self) -> MarkerLayout:
        target = self._next_target()
        # Mục tiêu luôn phải có mặt trên bàn; các tấm còn lại bốc từ phần kho
        # còn lại.
        if self.fixed_layout_digits:
            present = list(self.fixed_layout_digits)
        else:
            others = [d for d in self.digits if d != target]
            present = [target] + self._rng.sample(others, self.slot_count - 1)
            self._rng.shuffle(present)
        digit_to_slot = {digit: slot for slot, digit in enumerate(present)}

        positions = tuple(
            (
                self.slot_x_m,
                y + self._rng.uniform(-self.slot_y_jitter_m, self.slot_y_jitter_m),
                self.slot_z_m,
            )
            for y in self.slot_y_m
        )
        template = self.prompt_override or self._rng.choice(list(self.prompt_templates))
        layout = MarkerLayout(
            target_digit=target,
            digit_to_slot=digit_to_slot,
            slot_positions_m=positions,
            prompt_template=template,
            prompt=template.replace("{n}", str(target)),
        )
        self.history.append(layout)
        return layout

    def slot_share_by_digit(self) -> dict[int, dict[int, float]]:
        """Bảng chéo chữ_số_mục_tiêu × ô, chuẩn hoá theo hàng. Để soi bằng mắt."""

        counts: dict[int, dict[int, int]] = {}
        for layout in self.history:
            row = counts.setdefault(layout.target_digit, {})
            row[layout.target_slot] = row.get(layout.target_slot, 0) + 1
        shares: dict[int, dict[int, float]] = {}
        for digit, row in counts.items():
            total = sum(row.values())
            shares[digit] = {slot: n / total for slot, n in sorted(row.items())}
        return shares

    def target_slot_counts(self) -> dict[int, int]:
        """Mục tiêu rơi vào ô nào, gộp mọi chữ số."""

        counts = {slot: 0 for slot in range(self.slot_count)}
        for layout in self.history:
            counts[layout.target_slot] += 1
        return counts

    def slot_uniformity_chi_square(self) -> float:
        """Thống kê chi-bình-phương của phân bố ô mục tiêu so với phân bố đều.

        Đây mới là phép kiểm đúng cho rò rỉ vị trí, thay cho ngưỡng "không ô nào
        quá 35%" mà tôi đặt lúc đầu. Ngưỡng tỉ lệ cứng không dùng được ở cỡ mẫu
        nhỏ: mỗi chữ số chỉ làm mục tiêu khoảng 11 lần trong 100 episode, và với
        4 ô thì một ô chiếm 45% xảy ra thường xuyên chỉ do ngẫu nhiên. Ngưỡng đó
        sẽ loại oan những dataset hoàn toàn sạch.

        Với 3 bậc tự do, giá trị tới hạn ở mức 5% là 7.815. Lớn hơn thì phân bố
        ô lệch hơn mức ngẫu nhiên giải thích được và cần xem lại.
        """

        counts = self.target_slot_counts()
        total = sum(counts.values())
        if total == 0:
            return 0.0
        expected = total / self.slot_count
        return sum((n - expected) ** 2 / expected for n in counts.values())

    def target_digit_counts(self) -> dict[int, int]:
        counts: dict[int, int] = {}
        for layout in self.history:
            counts[layout.target_digit] = counts.get(layout.target_digit, 0) + 1
        return counts

    def balance_report(self) -> dict[str, object]:
        """Báo cáo gọn để in ra cuối phiên và ghi vào bằng chứng."""

        digits = self.target_digit_counts()
        return {
            "episode_count": len(self.history),
            "target_digit_counts": dict(sorted(digits.items())),
            "target_digit_spread": (max(digits.values()) - min(digits.values())) if digits else 0,
            "target_slot_counts": self.target_slot_counts(),
            "slot_uniformity_chi_square": round(self.slot_uniformity_chi_square(), 3),
            "slot_uniformity_critical_value_p05_df3": 7.815,
            "slot_share_by_digit": {
                d: {s: round(v, 3) for s, v in row.items()}
                for d, row in sorted(self.slot_share_by_digit().items())
            },
        }


def apply_layout(layout: MarkerLayout, prim_paths: dict[int, str], parked_position_m) -> None:
    """Đặt các tấm được chọn vào ô của chúng, đẩy các tấm còn lại ra khỏi cảnh.

    Tấm không dùng bị vừa dời đi vừa ẩn: chỉ dời thì vẫn có thể lọt vào khung nếu
    camera nhìn xuống dưới sàn, chỉ ẩn thì vẫn còn hộp bao trong cảnh.
    """

    import omni.usd
    from pxr import Gf, UsdGeom

    stage = omni.usd.get_context().get_stage()
    for digit, path in prim_paths.items():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        slot = layout.digit_to_slot.get(digit)
        used = slot is not None
        position = layout.slot_positions_m[slot] if used else tuple(parked_position_m)

        xformable = UsdGeom.Xformable(prim)
        translate_op = None
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(*(float(v) for v in position)))

        imageable = UsdGeom.Imageable(prim)
        if used:
            imageable.MakeVisible()
        else:
            imageable.MakeInvisible()


__all__ = ["DEFAULT_PROMPTS", "MarkerLayout", "MarkerRandomizer", "apply_layout"]
