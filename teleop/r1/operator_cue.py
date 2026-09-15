"""Hiển thị mục tiêu cho người thu mà không làm rò nhãn vào observation.

HUD desktop thuộc lớp giao diện ``omni.ui``, không phải USD prim. Ảnh phát tới
Quest được trang trí trên một bản sao sau khi sensor camera đã render. Bộ ghi
episode luôn đọc trực tiếp tensor RGB gốc, nên cả hai đường hiển thị đều nằm
ngoài ``colors/color_0`` của dataset.
"""

from __future__ import annotations

from typing import Any


class OperatorCue:
    """Giữ lời nhắc hiện tại cho console, Isaac UI và head-view tùy chọn."""

    def __init__(self, text_template: str, desktop_hud: bool = True) -> None:
        if "{n}" not in text_template:
            raise ValueError("operator cue template phải chứa {n}.")
        self.text_template = text_template
        self.text = ""
        self._target_label = None
        self._layout_label = None
        self._window = None
        if desktop_hud:
            self._build_desktop_hud()

    def _build_desktop_hud(self) -> None:
        """Tạo cửa sổ Kit độc lập; thất bại UI không được làm hỏng run."""

        try:
            import omni.ui as ui

            self._window = ui.Window("VLA TARGET — OPERATOR ONLY", width=520, height=150)
            with self._window.frame:
                with ui.VStack(spacing=8):
                    self._target_label = ui.Label(
                        "Đang chuẩn bị mục tiêu…",
                        height=70,
                        style={"font_size": 36},
                    )
                    self._layout_label = ui.Label("Lời nhắc này không được ghi vào dataset.")
        except Exception as exc:  # noqa: BLE001 - console/head-view vẫn dùng được
            self._window = None
            print(f"[operator-cue] không tạo được HUD Isaac: {exc}", flush=True)

    @staticmethod
    def _digits_by_slot(layout: Any) -> list[int]:
        return [
            digit
            for digit, _slot in sorted(layout.digit_to_slot.items(), key=lambda item: item[1])
        ]

    def show(self, layout: Any) -> str:
        """Đổi lời nhắc trước khi người vận hành bắt đầu episode."""

        self.text = self.text_template.format(n=layout.target_digit)
        digits = self._digits_by_slot(layout)
        layout_text = "BỐN SỐ TRÊN BÀN: " + "   ".join(str(value) for value in digits)
        print(f"\n[operator-cue] {self.text} | {layout_text}\n", flush=True)
        if self._target_label is not None:
            self._target_label.text = self.text
        if self._layout_label is not None:
            self._layout_label.text = layout_text + "  •  KHÔNG GHI VÀO ẢNH DATASET"
        return self.text

    def decorate_head_view(self, rgb):
        """Chèn lời nhắc vào bản sao dành riêng cho kính, không sửa ảnh nguồn."""

        if not self.text:
            return rgb.copy()
        import cv2

        output = rgb.copy()
        height, width = output.shape[:2]
        # Dải chữ nằm sát mép trên và mỏng: người vận hành cần liếc thấy con số,
        # không cần nó chiếm chỗ. Bản trước cao tới 1/5 khung hình và che mất
        # phần trên tầm nhìn của robot.
        bar_height = max(26, min(40, height // 12))
        overlay = output.copy()
        cv2.rectangle(overlay, (0, 0), (width, bar_height), (0, 0, 0), thickness=-1)
        # Pha mờ thay vì đen đặc: thấy được chữ mà vẫn thấy cảnh phía sau.
        cv2.addWeighted(overlay, 0.55, output, 0.45, 0.0, dst=output)
        scale = max(0.45, min(0.85, width / 900.0))
        cv2.putText(
            output,
            self.text,
            (12, int(bar_height * 0.74)),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            (255, 255, 255),
            thickness=2,
            lineType=cv2.LINE_AA,
        )
        return output

    def close(self) -> None:
        if self._window is not None:
            self._window.visible = False


__all__ = ["OperatorCue"]
