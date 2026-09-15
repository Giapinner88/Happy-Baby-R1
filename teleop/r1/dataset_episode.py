"""Ghi episode theo định dạng `EpisodeWriter` của Unitree.

Định dạng được tái sử dụng, còn lớp thì không. Package `teleop` của repo này che
mất package `teleop` của vendor, nên `from teleop.utils.episode_writer import
EpisodeWriter` không import được — đã thử và hỏng ở `ModuleNotFoundError`. Thay
vì bẻ `sys.path` và làm hỏng chính các import của repo, module này sinh ra đúng
lược đồ JSON đó. `tests/teleop/test_r1_dataset_episode.py` đọc mã nguồn của
vendor và kiểm tra rằng hai bên không trôi khỏi nhau.

Bố cục sinh ra:

    episodes/episode_0000/
    ├── colors/000000_color_0.jpg
    └── data.json      {"info": ..., "text": ..., "data": [...]}

Ranh giới episode theo đoạn lệnh được chấp nhận: bóp cò phải bắt đầu ghi và nhả
cò phải kết thúc episode. Cò trái vẫn giữ nghĩa reset session của controller.
"""

from __future__ import annotations

import datetime
import json
import queue
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence


@dataclass(frozen=True)
class RejectionPolicy:
    """Điều kiện để một episode bị loại.

    Hai điều kiện còn lại mà profile khai — bridge ngắt kết nối, và deadman nhả
    giữa chừng — được thoả **bằng cấu trúc** chứ không phải bằng kiểm tra: bộ ghi
    cắt episode ở mọi bước không được chấp nhận, nên không episode nào chứa được
    một khoảng ngắt. Kiểm tra lại ở đây sẽ không bao giờ đúng.
    """

    max_projected_fraction: float = 0.20
    min_items: int = 30
    min_measured_fps: float | None = None
    max_measured_fps: float | None = None
    require_unique_camera_frames: bool = False
    fps_gate_warmup_frames: int = 60

    def validate(self) -> None:
        if not 0.0 <= self.max_projected_fraction <= 1.0:
            raise ValueError("max_projected_fraction phải trong [0, 1].")
        if self.min_items < 1:
            raise ValueError("min_items phải ít nhất là 1.")
        if self.min_measured_fps is not None and self.min_measured_fps <= 0.0:
            raise ValueError("acceptance.min_measured_dataset_fps phải dương.")
        if self.max_measured_fps is not None and self.max_measured_fps <= 0.0:
            raise ValueError("acceptance.max_measured_dataset_fps phải dương.")
        if (
            self.min_measured_fps is not None
            and self.max_measured_fps is not None
            and self.min_measured_fps > self.max_measured_fps
        ):
            raise ValueError("Khoảng acceptance measured dataset FPS không hợp lệ.")
        if self.fps_gate_warmup_frames < 2:
            raise ValueError("acceptance.fps_gate_warmup_frames phải ít nhất là 2.")

    def evaluate(
        self,
        item_count: int,
        projected_count: int,
        *,
        measured_fps: float | None = None,
        duplicate_camera_frame_count: int = 0,
        missing_camera_frame_id_count: int = 0,
    ) -> list[str]:
        """Trả về danh sách lý do loại; rỗng nghĩa là giữ."""

        reasons: list[str] = []
        if item_count < self.min_items:
            reasons.append(f"too_short: {item_count} item < {self.min_items}")
        if item_count:
            fraction = projected_count / item_count
            if fraction > self.max_projected_fraction:
                reasons.append(
                    f"mostly_projected: {fraction:.2f} > {self.max_projected_fraction:.2f}"
                )
        if measured_fps is not None:
            if self.min_measured_fps is not None and measured_fps < self.min_measured_fps:
                reasons.append(
                    f"dataset_fps_below_gate: {measured_fps:.3f} < {self.min_measured_fps:.3f}"
                )
            if self.max_measured_fps is not None and measured_fps > self.max_measured_fps:
                reasons.append(
                    f"dataset_fps_above_gate: {measured_fps:.3f} > {self.max_measured_fps:.3f}"
                )
        if self.require_unique_camera_frames:
            if duplicate_camera_frame_count:
                reasons.append(
                    "duplicate_camera_frames: "
                    f"{duplicate_camera_frame_count} source frame id không tăng"
                )
            if missing_camera_frame_id_count:
                reasons.append(
                    "missing_camera_frame_ids: "
                    f"{missing_camera_frame_id_count} item không có source frame id"
                )
        return reasons


@dataclass(frozen=True)
class EpisodeConfig:
    """Phần bản ghi của một profile dataset."""

    fps: float
    image_width: int
    image_height: int
    joint_names: dict[str, list[str]]
    task_text: dict[str, str]
    author: str = "happy-baby-r1"
    version: str = "1.0.0"
    rejection: RejectionPolicy = field(default_factory=RejectionPolicy)
    writer_queue_size: int = 64
    jpeg_quality: int = 90

    def validate(self) -> None:
        if self.fps <= 0.0:
            raise ValueError("record.fps phải dương.")
        if self.writer_queue_size < 1:
            raise ValueError("record.writer.queue_size phải ít nhất là 1.")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("record.writer.jpeg_quality phải trong [1, 100].")
        for group in ("left_arm", "right_arm", "left_ee", "right_ee", "body"):
            if group not in self.joint_names:
                raise ValueError(f"record.joint_names thiếu nhóm {group!r}.")
        self.rejection.validate()


def load_episode_config(payload: dict[str, Any]) -> EpisodeConfig | None:
    """Lấy phần bản ghi ra khỏi một profile dataset đã đọc."""

    record = payload.get("record")
    if record is None:
        return None
    size = list(record.get("image_size_wh") or (640, 480))
    rejection_payload = payload.get("episode_rejection") or {}
    acceptance_payload = payload.get("acceptance") or {}
    rejection = RejectionPolicy(
        max_projected_fraction=float(rejection_payload.get("max_projected_fraction", 0.20)),
        min_items=int(rejection_payload.get("min_items", 30)),
        min_measured_fps=(
            float(acceptance_payload["min_measured_dataset_fps"])
            if acceptance_payload.get("min_measured_dataset_fps") is not None
            else None
        ),
        max_measured_fps=(
            float(acceptance_payload["max_measured_dataset_fps"])
            if acceptance_payload.get("max_measured_dataset_fps") is not None
            else None
        ),
        require_unique_camera_frames=bool(
            acceptance_payload.get("require_unique_camera_frames", False)
        ),
        fps_gate_warmup_frames=int(acceptance_payload.get("fps_gate_warmup_frames", 60)),
    )
    writer = record.get("writer") or {}
    config = EpisodeConfig(
        fps=float(record.get("fps", 30)),
        image_width=int(size[0]),
        image_height=int(size[1]),
        joint_names={k: list(v) for k, v in (record.get("joint_names") or {}).items()},
        task_text={k: str(v) for k, v in (payload.get("task_text") or {}).items()},
        rejection=rejection,
        writer_queue_size=int(writer.get("queue_size", 64)),
        jpeg_quality=int(writer.get("jpeg_quality", 90)),
    )
    config.validate()
    return config


def _joint_block(names: Sequence[str], values: Sequence[float] | None) -> dict[str, list[float]]:
    """Một mục qpos/qvel/torque. Trường bỏ trống là rỗng, không phải số không.

    Phân biệt này quan trọng: `[]` nghĩa là 'không đo', còn `[0.0, ...]` nghĩa là
    'đo được và bằng không'. R1 không có end effector nên hai nhóm ee luôn rỗng.
    """

    if not names or values is None:
        return {"qpos": [], "qvel": [], "torque": []}
    return {"qpos": [float(v) for v in values[: len(names)]], "qvel": [], "torque": []}


@dataclass(frozen=True)
class _ImageWriteJob:
    destination: Path
    rgb: Any


@dataclass(frozen=True)
class _EpisodeFinalizeJob:
    episode_dir: Path
    payload: dict[str, Any]
    rejected_destination: Path | None


_STOP_WRITER = object()


@dataclass
class EpisodeRecorder:
    """Cắt luồng điều khiển thành episode và ghi ra đĩa theo lược đồ Unitree.

    Isaac chỉ chuyển tensor camera về một mảng CPU độc lập rồi đặt nó vào hàng
    đợi. Nén JPEG, ghi ảnh, ghi JSON và chuyển episode bị loại đều chạy trên một
    worker riêng. Đây là cùng ranh giới producer/consumer mà xr_teleoperate dùng,
    nhưng hàng đợi được giới hạn để một ổ đĩa chậm không thể ăn hết RAM.
    """

    staging_dir: Path
    camera: Any
    physics_dt_s: float
    config: EpisodeConfig
    controlled_joint_names: list[str]
    episode_index: int = -1
    _items: list[dict[str, Any]] = field(default_factory=list, init=False)
    _episode_dir: Path | None = field(default=None, init=False)
    _projected_count: int = field(default=0, init=False)
    _episode_dropped_count: int = field(default=0, init=False)
    _task_record: dict[str, Any] | None = field(default=None, init=False)
    _last_source_camera_frame: int | None = field(default=None, init=False)
    _duplicate_camera_frame_count: int = field(default=0, init=False)
    _missing_camera_frame_id_count: int = field(default=0, init=False)
    _index_rows: list[dict[str, Any]] = field(default_factory=list, init=False)
    episode_summaries: list[dict[str, Any]] = field(default_factory=list, init=False)
    _write_queue: queue.Queue[Any] = field(init=False, repr=False)
    _frame_slots: threading.BoundedSemaphore = field(init=False, repr=False)
    _writer_thread: threading.Thread = field(init=False, repr=False)
    _writer_error: BaseException | None = field(default=None, init=False, repr=False)
    _writer_closed: bool = field(default=False, init=False, repr=False)
    _writer_lock: threading.Lock = field(init=False, repr=False)
    _capture_attempt_count: int = field(default=0, init=False)
    _queued_frame_count: int = field(default=0, init=False)
    _written_frame_count: int = field(default=0, init=False)
    _dropped_frame_count: int = field(default=0, init=False)
    _queue_high_watermark: int = field(default=0, init=False)
    _frames_in_flight: int = field(default=0, init=False, repr=False)
    _write_time_s: float = field(default=0.0, init=False)

    def __post_init__(self) -> None:
        self.config.validate()
        # Metadata/finalize jobs không mang ảnh và phải luôn enqueue tức thì.
        # Semaphore mới là giới hạn RAM: nó đếm cả ảnh đang chờ lẫn ảnh worker
        # đang encode, đồng thời chừa queue điều khiển khỏi backpressure của ảnh.
        self._write_queue = queue.Queue()
        self._frame_slots = threading.BoundedSemaphore(self.config.writer_queue_size)
        self._writer_lock = threading.Lock()
        self._writer_thread = threading.Thread(
            target=self._writer_loop,
            name="r1-dataset-writer",
            daemon=True,
        )
        self._writer_thread.start()

    def _set_writer_error(self, exc: BaseException) -> None:
        with self._writer_lock:
            if self._writer_error is None:
                self._writer_error = exc

    def _raise_writer_error(self) -> None:
        with self._writer_lock:
            error = self._writer_error
        if error is not None:
            raise RuntimeError(f"Dataset writer thất bại: {error}") from error

    def _writer_loop(self) -> None:
        """Consumer duy nhất sở hữu phần nén và mọi ghi dữ liệu episode."""

        while True:
            job = self._write_queue.get()
            try:
                if job is _STOP_WRITER:
                    return
                # Sau lỗi đầu tiên, vẫn rút cạn queue để producer/finalize không
                # deadlock; run sẽ bị báo lỗi thay vì tạo một dataset nửa vời.
                with self._writer_lock:
                    failed = self._writer_error is not None
                if failed:
                    continue
                if isinstance(job, _ImageWriteJob):
                    import cv2

                    started = time.monotonic()
                    bgr = cv2.cvtColor(job.rgb, cv2.COLOR_RGB2BGR)
                    written = cv2.imwrite(
                        str(job.destination),
                        bgr,
                        [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality],
                    )
                    if not written:
                        raise OSError(f"Không ghi được ảnh dataset: {job.destination}")
                    elapsed = time.monotonic() - started
                    with self._writer_lock:
                        self._written_frame_count += 1
                        self._write_time_s += elapsed
                elif isinstance(job, _EpisodeFinalizeJob):
                    (job.episode_dir / "data.json").write_text(
                        json.dumps(job.payload, ensure_ascii=False, indent=4) + "\n",
                        encoding="utf-8",
                    )
                    if job.rejected_destination is not None:
                        job.rejected_destination.parent.mkdir(parents=True, exist_ok=True)
                        job.episode_dir.rename(job.rejected_destination)
                else:  # pragma: no cover - lỗi lập trình, không phải input
                    raise TypeError(f"Dataset writer nhận job không hỗ trợ: {type(job)!r}")
            except BaseException as exc:  # noqa: BLE001 - phải chuyển lỗi về main thread
                self._set_writer_error(exc)
            finally:
                if isinstance(job, _ImageWriteJob):
                    with self._writer_lock:
                        self._frames_in_flight -= 1
                    self._frame_slots.release()
                self._write_queue.task_done()

    def _enqueue_required(self, job: Any) -> None:
        """Đưa metadata/sentinel vào queue; các job này không được phép bị bỏ."""

        self._raise_writer_error()
        self._write_queue.put_nowait(job)

    def _wait_for_writer(self) -> None:
        self._write_queue.join()
        self._raise_writer_error()

    @property
    def can_accept_frame(self) -> bool:
        """Gợi ý không chặn để caller có thể bỏ qua GPU→CPU copy khi queue đầy."""

        self._raise_writer_error()
        with self._writer_lock:
            return self._frames_in_flight < self.config.writer_queue_size

    @property
    def is_between_episodes(self) -> bool:
        """True khi chưa có episode nào đang mở, tức bước tới sẽ mở một cái mới."""

        return self._episode_dir is None

    @property
    def has_items(self) -> bool:
        """True khi episode đang mở đã chứa ít nhất một mẫu học."""

        return bool(self._items)

    def _group_slice(self, group: str) -> slice | None:
        """Vị trí của một nhóm khớp trong vector khớp được điều khiển."""

        names = self.config.joint_names.get(group) or []
        if not names:
            return None
        try:
            start = self.controlled_joint_names.index(names[0])
        except ValueError:
            return None
        return slice(start, start + len(names))

    def begin_episode(self, task_record: dict[str, Any] | None = None) -> None:
        """Chuẩn bị episode mới. Item đầu tiên được ghi khi deadman được giữ."""

        if self._episode_dir is not None:
            self.end_episode()
        self.episode_index += 1
        self._episode_dir = self.staging_dir / f"episode_{self.episode_index:04d}"
        (self._episode_dir / "colors").mkdir(parents=True, exist_ok=True)
        self._items = []
        self._projected_count = 0
        self._episode_dropped_count = 0
        self._task_record = dict(task_record) if task_record else None
        self._last_source_camera_frame = None
        self._duplicate_camera_frame_count = 0
        self._missing_camera_frame_id_count = 0

    def add_item(
        self,
        control_step: int,
        elapsed_s: float,
        state_qpos: Sequence[float] | None,
        action_qpos: Sequence[float] | None,
        solver_solution_kind: str | None = None,
        rgb: Any | None = None,
        source_camera_frame: int | None = None,
    ) -> bool:
        """Ghép một khung với state/action rồi xếp việc ghi vào worker.

        Trả ``False`` nếu bounded queue đã đầy. Mẫu đó không được thêm vào JSON,
        nên số ảnh và số item luôn bằng nhau; episode được đánh dấu loại lúc đóng.
        """

        if self._episode_dir is None:
            self.begin_episode()
        assert self._episode_dir is not None

        self._raise_writer_error()
        self._capture_attempt_count += 1
        if not self._frame_slots.acquire(blocking=False):
            self._dropped_frame_count += 1
            self._episode_dropped_count += 1
            return False

        try:
            if rgb is None:
                self.camera.update(self.physics_dt_s)
                rgb = self.camera.data.output["rgb"][0, ..., :3].detach().cpu().numpy()

            import numpy as np

            # Tensor/annotator buffers có thể được Isaac tái sử dụng ở lần render sau.
            # Worker phải nhận một bản sở hữu riêng, liên tục trong RAM.
            frame = np.ascontiguousarray(rgb, dtype=np.uint8).copy()
        except BaseException:
            self._frame_slots.release()
            raise

        idx = len(self._items)
        relative = f"colors/{idx:06d}_color_0.jpg"
        with self._writer_lock:
            self._frames_in_flight += 1
            self._queue_high_watermark = max(
                self._queue_high_watermark, self._frames_in_flight
            )
        self._write_queue.put_nowait(
            _ImageWriteJob(destination=self._episode_dir / relative, rgb=frame)
        )
        self._queued_frame_count += 1

        if solver_solution_kind == "projected":
            self._projected_count += 1
        if source_camera_frame is None:
            self._missing_camera_frame_id_count += 1
        else:
            source_camera_frame = int(source_camera_frame)
            if (
                self._last_source_camera_frame is not None
                and source_camera_frame <= self._last_source_camera_frame
            ):
                self._duplicate_camera_frame_count += 1
            self._last_source_camera_frame = source_camera_frame

        states: dict[str, Any] = {}
        actions: dict[str, Any] = {}
        for group in ("left_arm", "right_arm", "left_ee", "right_ee"):
            span = self._group_slice(group)
            names = self.config.joint_names.get(group) or []
            states[group] = _joint_block(names, state_qpos[span] if span and state_qpos else None)
            actions[group] = _joint_block(names, action_qpos[span] if span and action_qpos else None)
        body_span = self._group_slice("body")
        body_names = self.config.joint_names.get("body") or []
        states["body"] = {"qpos": _joint_block(body_names, state_qpos[body_span] if body_span and state_qpos else None)["qpos"]}
        actions["body"] = {"qpos": _joint_block(body_names, action_qpos[body_span] if body_span and action_qpos else None)["qpos"]}

        self._items.append(
            {
                "idx": idx,
                "colors": {"color_0": relative},
                "depths": {},
                "states": states,
                "actions": actions,
                "tactiles": {},
                "audios": {},
                "sim_state": {
                    "control_step": control_step,
                    "elapsed_s": elapsed_s,
                    "solver_solution_kind": solver_solution_kind,
                    "source_camera_frame": source_camera_frame,
                },
            }
        )
        self._index_rows.append(
            {
                "episode": self.episode_index,
                "idx": idx,
                "control_step": control_step,
                "elapsed_s": elapsed_s,
                "color_0": f"episode_{self.episode_index:04d}/{relative}",
            }
        )
        return True

    def _measured_fps(self) -> float:
        """Nhịp thật của episode, suy từ dấu thời gian của chính các item."""

        if len(self._items) < 2:
            return 0.0
        span = self._items[-1]["sim_state"]["elapsed_s"] - self._items[0]["sim_state"]["elapsed_s"]
        if span <= 0.0:
            return 0.0
        return round((len(self._items) - 1) / span, 3)

    @property
    def current_item_count(self) -> int:
        """Số mẫu của episode đang mở, dùng cho fail-fast gate ở runner."""

        return len(self._items)

    @property
    def current_measured_fps(self) -> float:
        """Nhịp wall-clock hiện tại của episode; không suy từ FPS cấu hình."""

        return self._measured_fps()

    @property
    def current_duplicate_camera_frame_count(self) -> int:
        """Số source frame id không tăng trong episode đang mở."""

        return self._duplicate_camera_frame_count

    def _episode_text(self) -> dict[str, str]:
        """Resolve nhãn ngôn ngữ theo đúng mục tiêu của episode hiện tại.

        `info.task.prompt` là nguồn nhãn động của D002.  Ghi cùng câu đó vào
        `text.goal` để consumer theo lược đồ Unitree/VLA không phải biết thêm
        trường riêng của dự án. Các trường khác vẫn hỗ trợ `{n}` trong profile.
        """

        task = self._task_record or {}
        digit = task.get("target_digit")
        values: dict[str, str] = {}
        for key, raw in self.config.task_text.items():
            values[key] = raw.replace("{n}", str(digit)) if digit is not None else raw
        prompt = task.get("prompt")
        if prompt:
            values["goal"] = str(prompt)
        return values

    def end_episode(self, *, wait: bool = True) -> None:
        """Đóng episode hiện tại và áp chính sách loại.

        Episode bị loại **không bị xoá**. Nó được chuyển sang `episodes_rejected/`
        cùng với lý do, vì một dataset lặng lẽ vứt bớt dữ liệu thì không ai kiểm
        lại được là đã vứt cái gì và vì sao.
        """

        if self._episode_dir is None:
            return
        if not self._items and not self._episode_dropped_count:
            (self._episode_dir / "colors").rmdir()
            self._episode_dir.rmdir()
            self.episode_index -= 1
            self._episode_dir = None
            return

        measured_fps = self._measured_fps()
        reasons = self.config.rejection.evaluate(
            len(self._items),
            self._projected_count,
            measured_fps=measured_fps,
            duplicate_camera_frame_count=self._duplicate_camera_frame_count,
            missing_camera_frame_id_count=self._missing_camera_frame_id_count,
        )
        if self._episode_dropped_count:
            reasons.append(f"writer_overflow: {self._episode_dropped_count} capture bị bỏ")
        payload = {
            "info": {
                "version": self.config.version,
                "date": datetime.date.today().strftime("%Y-%m-%d"),
                "author": self.config.author,
                "image": {
                    "width": self.config.image_width,
                    "height": self.config.image_height,
                    # Nhịp ĐO ĐƯỢC của chính episode này, không phải nhịp khai
                    # báo trong profile. Bộ giải không giữ nổi nhịp yêu cầu, nên
                    # ghi con số mong muốn vào đây là nói dối về thời gian.
                    "fps": measured_fps,
                    "fps_requested": self.config.fps,
                },
                "depth": {"width": 0, "height": 0, "fps": 0},
                "audio": {"sample_rate": 0, "channels": 0, "format": "PCM", "bits": 16},
                "joint_names": {k: list(v) for k, v in self.config.joint_names.items()},
                "tactile_names": {"left_ee": [], "right_ee": []},
                "sim_state": "isaaclab_r1_simulation_only",
                # Bố cục của chính episode này: chữ số nào ở ô nào, mục tiêu là
                # cái nào, câu lệnh ra sao. Thiếu khối này thì không chấm điểm
                # tự động được và không kiểm được rò rỉ vị trí.
                "task": self._task_record,
                "accepted": not reasons,
                "rejection_reasons": reasons,
                "projected_item_count": self._projected_count,
                "writer_dropped_capture_count": self._episode_dropped_count,
                "duplicate_camera_frame_count": self._duplicate_camera_frame_count,
                "missing_camera_frame_id_count": self._missing_camera_frame_id_count,
            },
            "text": self._episode_text(),
            "data": self._items,
        }
        rejected_destination = None
        if reasons:
            rejected_root = self.staging_dir.parent / (self.staging_dir.name + ".rejected")
            rejected_destination = rejected_root / self._episode_dir.name
        self._enqueue_required(
            _EpisodeFinalizeJob(
                episode_dir=self._episode_dir,
                payload=payload,
                rejected_destination=rejected_destination,
            )
        )

        summary = {
            "episode": self.episode_index,
            "item_count": len(self._items),
            "projected_item_count": self._projected_count,
            "writer_dropped_capture_count": self._episode_dropped_count,
            "measured_fps": measured_fps,
            "duplicate_camera_frame_count": self._duplicate_camera_frame_count,
            "missing_camera_frame_id_count": self._missing_camera_frame_id_count,
            "accepted": not reasons,
            "rejection_reasons": reasons,
            "target_digit": (self._task_record or {}).get("target_digit"),
            "target_slot": (self._task_record or {}).get("target_slot"),
        }
        self.episode_summaries.append(summary)
        if reasons:
            # Chỉ mục cấp run chỉ trỏ tới ảnh của episode được giữ; ảnh của
            # episode bị loại vẫn còn trên đĩa nhưng ở nhánh khác.
            self._index_rows = [r for r in self._index_rows if r["episode"] != self.episode_index]
        self._items = []
        self._projected_count = 0
        self._episode_dropped_count = 0
        self._task_record = None
        self._last_source_camera_frame = None
        self._duplicate_camera_frame_count = 0
        self._missing_camera_frame_id_count = 0
        self._episode_dir = None
        if wait:
            self._wait_for_writer()

    def writer_stats(self) -> dict[str, Any]:
        """Thống kê backpressure/IO để một run chậm có nguyên nhân đo được."""

        with self._writer_lock:
            written = self._written_frame_count
            total_write_s = self._write_time_s
            failed = self._writer_error is not None
        return {
            "dataset_writer_mode": "bounded_async_thread",
            "dataset_writer_queue_capacity": self.config.writer_queue_size,
            "dataset_writer_queue_high_watermark": self._queue_high_watermark,
            "dataset_writer_capture_attempt_count": self._capture_attempt_count,
            "dataset_writer_queued_frame_count": self._queued_frame_count,
            "dataset_writer_written_frame_count": written,
            "dataset_writer_dropped_frame_count": self._dropped_frame_count,
            "dataset_writer_mean_image_write_s": (
                round(total_write_s / written, 6) if written else None
            ),
            "dataset_writer_failed": failed,
            "dataset_writer_jpeg_quality": self.config.jpeg_quality,
        }

    def close(self) -> None:
        """Flush và dừng worker. Gọi nhiều lần là an toàn."""

        if self._writer_closed:
            self._raise_writer_error()
            return
        # Kể cả khi worker đã ghi nhận lỗi, vẫn phải gửi sentinel để thread
        # thoát. `_enqueue_required` cố ý raise sớm nên không dùng cho teardown.
        self._write_queue.put_nowait(_STOP_WRITER)
        self._write_queue.join()
        self._writer_thread.join(timeout=5.0)
        self._writer_closed = True
        if self._writer_thread.is_alive():
            raise RuntimeError("Dataset writer không dừng sau 5 giây.")
        self._raise_writer_error()

    def finalize(self, output_dir: Path) -> dict[str, Any]:
        """Đóng episode dang dở và ghi chỉ mục cấp run kèm bản kiểm đếm."""

        self.end_episode(wait=False)
        self.close()
        accepted = [e for e in self.episode_summaries if e["accepted"]]
        rejected = [e for e in self.episode_summaries if not e["accepted"]]
        reason_counts: dict[str, int] = {}
        for episode in rejected:
            for reason in episode["rejection_reasons"]:
                key = reason.split(":")[0]
                reason_counts[key] = reason_counts.get(key, 0) + 1
        summary = {
            "episode_count_total": len(self.episode_summaries),
            "episode_count_accepted": len(accepted),
            "episode_count_rejected": len(rejected),
            "rejection_reason_counts": reason_counts,
            "rejection_policy": {
                "max_projected_fraction": self.config.rejection.max_projected_fraction,
                "min_items": self.config.rejection.min_items,
                "min_measured_fps": self.config.rejection.min_measured_fps,
                "max_measured_fps": self.config.rejection.max_measured_fps,
                "require_unique_camera_frames": (
                    self.config.rejection.require_unique_camera_frames
                ),
            },
            "episodes": self.episode_summaries,
            "frames": self._index_rows,
        }
        (output_dir / "episode_index.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )
        return {
            "episode_count_accepted": len(accepted),
            "episode_count_rejected": len(rejected),
            "episode_item_counts": [e["item_count"] for e in accepted],
            "episode_rejection_reason_counts": reason_counts,
            **self.writer_stats(),
        }


__all__ = ["EpisodeConfig", "EpisodeRecorder", "RejectionPolicy", "load_episode_config"]
