"""Kiểm tra các bảo đảm chống lối tắt của bộ ngẫu nhiên hoá tấm số.

Phần chọn bố cục là thuần tuý nên test được không cần Isaac. Đó là chủ đích:
những bảo đảm quan trọng nhất của D002 phải kiểm được bằng test thường, không
phải bằng cách nhìn ảnh render.
"""

import unittest
from collections import Counter

from teleop.r1.dataset_randomizer import MarkerRandomizer


SLOT_Y = [0.21, 0.07, -0.07, -0.21]


def _randomizer(seed=0, digits=range(1, 10), jitter=0.0) -> MarkerRandomizer:
    return MarkerRandomizer(
        digits=list(digits), slot_x_m=0.37, slot_y_m=SLOT_Y, slot_z_m=0.885,
        slot_y_jitter_m=jitter, seed=seed,
    )


class LayoutShapeTests(unittest.TestCase):
    def test_each_episode_places_exactly_one_digit_per_slot(self) -> None:
        rng = _randomizer()
        for _ in range(50):
            layout = rng.next_layout()
            self.assertEqual(len(layout.digit_to_slot), len(SLOT_Y))
            self.assertEqual(sorted(layout.digit_to_slot.values()), list(range(len(SLOT_Y))))

    def test_the_target_is_always_one_of_the_placed_digits(self) -> None:
        rng = _randomizer()
        for _ in range(100):
            layout = rng.next_layout()
            self.assertIn(layout.target_digit, layout.digit_to_slot)

    def test_target_position_is_the_slot_the_target_digit_occupies(self) -> None:
        rng = _randomizer()
        for _ in range(20):
            layout = rng.next_layout()
            self.assertEqual(
                layout.target_position_m, layout.slot_positions_m[layout.digit_to_slot[layout.target_digit]]
            )

    def test_prompt_names_the_target_digit(self) -> None:
        rng = _randomizer()
        for _ in range(20):
            layout = rng.next_layout()
            self.assertIn(str(layout.target_digit), layout.prompt)


class ShortcutResistanceTests(unittest.TestCase):
    """Nếu các test này hỏng thì dataset dạy mô hình đoán vị trí thay vì đọc số."""

    def test_target_slots_are_uniform_within_chance(self) -> None:
        # Ngưỡng tỉ lệ cứng không dùng được ở cỡ mẫu nhỏ: với 4 ô và ~11 mẫu mỗi
        # chữ số, một ô chiếm 45% xảy ra thường xuyên chỉ do ngẫu nhiên. Phép
        # kiểm đúng là chi-bình-phương trên phân bố gộp.
        for seed in range(5):
            rng = _randomizer(seed=seed)
            for _ in range(400):
                rng.next_layout()
            self.assertLess(
                rng.slot_uniformity_chi_square(), 7.815,
                f"seed {seed}: {rng.target_slot_counts()}",
            )

    def test_the_balance_report_carries_what_the_quality_gate_needs(self) -> None:
        rng = _randomizer()
        for _ in range(90):
            rng.next_layout()
        report = rng.balance_report()
        for key in ("episode_count", "target_digit_counts", "target_digit_spread",
                    "target_slot_counts", "slot_uniformity_chi_square"):
            self.assertIn(key, report)
        self.assertEqual(report["episode_count"], 90)

    def test_every_digit_reaches_every_slot(self) -> None:
        rng = _randomizer()
        seen: dict[int, set[int]] = {}
        for _ in range(600):
            layout = rng.next_layout()
            for digit, slot in layout.digit_to_slot.items():
                seen.setdefault(digit, set()).add(slot)
        for digit, slots in seen.items():
            self.assertEqual(slots, set(range(len(SLOT_Y))), f"chữ số {digit} không tới đủ mọi ô")

    def test_target_digits_stay_balanced_across_a_run(self) -> None:
        # Hàng đợi xáo trộn, không phải bốc độc lập: 9 chữ số qua 180 episode
        # phải cho khoảng 20 lần mỗi cái, lệch rất ít.
        rng = _randomizer()
        counts = Counter(rng.next_layout().target_digit for _ in range(180))
        self.assertEqual(set(counts), set(range(1, 10)))
        # Chọn mục tiêu trước rồi mới bốc tấm quanh nó cho vòng tròn chính xác.
        self.assertLessEqual(max(counts.values()) - min(counts.values()), 1, counts)

    def test_a_short_run_still_covers_most_digits(self) -> None:
        rng = _randomizer()
        counts = Counter(rng.next_layout().target_digit for _ in range(9))
        self.assertGreaterEqual(len(counts), 7, counts)


class PriorityDigitTests(unittest.TestCase):
    """Bù chữ số còn thiếu giữa các phiên.

    Hàng đợi xáo trộn chỉ cân bằng TRONG một phiên. Bốn phiên đầu của D002 dài
    4-7 episode nên chữ số 3 chưa từng được chọn — cân bằng giữa các phiên không
    tự đến, phải chỉ định.
    """

    def test_priority_digits_are_used_first_in_the_declared_order(self) -> None:
        rng = MarkerRandomizer(
            digits=list(range(1, 10)), slot_x_m=0.37, slot_y_m=SLOT_Y, slot_z_m=0.885,
            seed=0, priority_digits=[3, 3, 8],
        )
        self.assertEqual([rng.next_layout().target_digit for _ in range(3)], [3, 3, 8])

    def test_the_shuffled_queue_resumes_once_the_priority_list_is_drained(self) -> None:
        rng = MarkerRandomizer(
            digits=list(range(1, 10)), slot_x_m=0.37, slot_y_m=SLOT_Y, slot_z_m=0.885,
            seed=0, priority_digits=[3],
        )
        targets = [rng.next_layout().target_digit for _ in range(19)]
        self.assertEqual(targets[0], 3)
        # 18 lượt sau đó là hai vòng đầy đủ của hàng đợi.
        self.assertEqual(sorted(targets[1:]), sorted(list(range(1, 10)) * 2))

    def test_a_priority_digit_outside_the_pool_is_refused(self) -> None:
        with self.assertRaises(ValueError):
            MarkerRandomizer(
                digits=[1, 2, 3, 4], slot_x_m=0.37, slot_y_m=SLOT_Y, slot_z_m=0.885,
                priority_digits=[7],
            )

    def test_a_prioritised_digit_is_still_placed_on_the_table(self) -> None:
        rng = MarkerRandomizer(
            digits=list(range(1, 10)), slot_x_m=0.37, slot_y_m=SLOT_Y, slot_z_m=0.885,
            seed=0, priority_digits=[3, 3, 3],
        )
        for _ in range(3):
            layout = rng.next_layout()
            self.assertIn(3, layout.digit_to_slot)


class ManualLayoutTests(unittest.TestCase):
    def test_fixed_layout_preserves_declared_slot_order(self) -> None:
        rng = MarkerRandomizer(
            digits=list(range(1, 10)), slot_x_m=0.37, slot_y_m=SLOT_Y,
            slot_z_m=0.885, priority_digits=[4], fixed_layout_digits=[5, 4, 8, 2],
        )
        layout = rng.next_layout()
        self.assertEqual(layout.target_digit, 4)
        self.assertEqual(layout.digit_to_slot, {5: 0, 4: 1, 8: 2, 2: 3})

    def test_manual_prompt_is_exact_and_can_expand_target(self) -> None:
        literal = MarkerRandomizer(
            digits=list(range(1, 10)), slot_x_m=0.37, slot_y_m=SLOT_Y,
            slot_z_m=0.885, priority_digits=[4], fixed_layout_digits=[5, 4, 8, 2],
            prompt_override="Please touch number 4.",
        ).next_layout()
        templated = MarkerRandomizer(
            digits=list(range(1, 10)), slot_x_m=0.37, slot_y_m=SLOT_Y,
            slot_z_m=0.885, priority_digits=[4], fixed_layout_digits=[5, 4, 8, 2],
            prompt_override="Please touch number {n}.",
        ).next_layout()
        self.assertEqual(literal.prompt, "Please touch number 4.")
        self.assertEqual(templated.prompt, "Please touch number 4.")

    def test_literal_prompt_can_be_the_only_target_source(self) -> None:
        layout = MarkerRandomizer(
            digits=list(range(1, 10)), slot_x_m=0.37, slot_y_m=SLOT_Y,
            slot_z_m=0.885, fixed_layout_digits=[5, 4, 8, 2],
            prompt_override="Please touch number 8.",
        ).next_layout()
        self.assertEqual(layout.target_digit, 8)
        self.assertEqual(layout.prompt, "Please touch number 8.")

    def test_literal_prompt_and_explicit_target_must_agree(self) -> None:
        with self.assertRaises(ValueError):
            MarkerRandomizer(
                digits=list(range(1, 10)), slot_x_m=0.37, slot_y_m=SLOT_Y,
                slot_z_m=0.885, priority_digits=[6], fixed_layout_digits=[5, 4, 6, 8],
                prompt_override="Please touch number 8.",
            )

    def test_semantic_prompt_uses_explicit_ground_truth_target(self) -> None:
        layout = MarkerRandomizer(
            digits=list(range(1, 10)), slot_x_m=0.37, slot_y_m=SLOT_Y,
            slot_z_m=0.885, priority_digits=[8], fixed_layout_digits=[5, 4, 6, 8],
            prompt_override="Put the right hand on the greatest number.",
        ).next_layout()
        self.assertEqual(layout.target_digit, 8)
        self.assertEqual(layout.prompt, "Put the right hand on the greatest number.")

    def test_semantic_prompt_without_ground_truth_target_is_refused(self) -> None:
        with self.assertRaises(ValueError):
            MarkerRandomizer(
                digits=list(range(1, 10)), slot_x_m=0.37, slot_y_m=SLOT_Y,
                slot_z_m=0.885, fixed_layout_digits=[5, 4, 6, 8],
                prompt_override="Put the right hand on the greatest number.",
            )

    def test_fixed_layout_rejects_duplicate_or_absent_target(self) -> None:
        with self.assertRaises(ValueError):
            MarkerRandomizer(
                digits=list(range(1, 10)), slot_x_m=0.37, slot_y_m=SLOT_Y,
                slot_z_m=0.885, fixed_layout_digits=[5, 4, 4, 2],
            )
        with self.assertRaises(ValueError):
            MarkerRandomizer(
                digits=list(range(1, 10)), slot_x_m=0.37, slot_y_m=SLOT_Y,
                slot_z_m=0.885, priority_digits=[6], fixed_layout_digits=[5, 4, 8, 2],
            )


class ConfigurationTests(unittest.TestCase):
    def test_a_pool_smaller_than_the_slot_count_is_refused(self) -> None:
        with self.assertRaises(ValueError):
            MarkerRandomizer(digits=[1, 2], slot_x_m=0.37, slot_y_m=SLOT_Y, slot_z_m=0.885)

    def test_duplicate_digits_are_refused(self) -> None:
        with self.assertRaises(ValueError):
            MarkerRandomizer(digits=[1, 1, 2, 3], slot_x_m=0.37, slot_y_m=SLOT_Y, slot_z_m=0.885)

    def test_the_same_seed_reproduces_the_same_run(self) -> None:
        a = [l.as_task_record() for l in (_randomizer(seed=7).next_layout() for _ in range(20))]
        b = [l.as_task_record() for l in (_randomizer(seed=7).next_layout() for _ in range(20))]
        self.assertEqual(a, b)

    def test_jitter_moves_slots_but_stays_within_the_declared_bound(self) -> None:
        rng = _randomizer(jitter=0.015)
        for _ in range(50):
            layout = rng.next_layout()
            for base, position in zip(SLOT_Y, layout.slot_positions_m):
                self.assertLessEqual(abs(position[1] - base), 0.015 + 1e-9)
                self.assertAlmostEqual(position[0], 0.37)

    def test_task_record_carries_everything_scoring_needs(self) -> None:
        record = _randomizer().next_layout().as_task_record()
        for key in ("target_digit", "digit_to_slot", "slot_positions_m", "target_slot",
                    "target_position_m", "prompt_template", "prompt"):
            self.assertIn(key, record)


if __name__ == "__main__":
    unittest.main()
