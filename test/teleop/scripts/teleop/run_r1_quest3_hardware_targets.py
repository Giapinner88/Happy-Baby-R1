#!/usr/bin/env python3
"""Map Quest JSONL to relative-session R1-A5 arms/head joint targets.

This workstation process performs no DDS writes.  It reuses the coupled IK
from the simulation pilot in ``arms_head`` mode and emits a 12-joint JSONL
stream for the fail-closed robot receiver.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import queue
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from teleop.r1 import (  # noqa: E402
    R1A5WholeUpperBodyOwnership,
    R1TeleopCommand,
    R1TeleopMapper,
    TeleopCalibration,
    TeleopLimits,
    UpperBodyIKConfig,
    Vector3,
)
from teleop.r1.kinematics import load_arm_chain  # noqa: E402
from teleop.r1.mapping import R1TeleopTargets  # noqa: E402
from teleop.r1.upper_body_kinematics import retarget_nominal  # noqa: E402
from teleop.r1.whole_upper_body import (  # noqa: E402
    WholeUpperBodyIsaacLabSink,
    WholeUpperBodyLiveConfig,
)

# Thu tu cua MO HINH IK (upper_body_kinematics): chuoi dong hoc dau la
# waist_yaw_link -> head_pitch -> head_yaw, nen mo hinh xuat [.., pitch, yaw].
# KHONG doi thu tu nay: doi la sai FK.
MODEL_JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "head_pitch_joint", "head_yaw_joint",
)

# Thu tu CUA STREAM gui xuong robot: chuan la controller/src/config/RobotSpec.hpp
# (idl 29 = head_yaw, 30 = head_pitch).  high_level_sidecar.parse_target so tuple ten
# CHINH XAC, nen lech thu tu = bi tu choi im lang, teleop khong bao gio bat.
JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "head_yaw_joint", "head_pitch_joint",
)
# model -> stream: chi hoan vi 2 phan tu cuoi, tinh mot lan.
STREAM_FROM_MODEL = tuple(MODEL_JOINT_NAMES.index(name) for name in JOINT_NAMES)


class TargetHandle:
    def __init__(self, initial: tuple[float, ...]) -> None:
        self.positions = np.asarray(initial, dtype=float)

    def joint_positions(self, joint_names: object) -> tuple[float, ...]:
        return tuple(float(value) for value in self.positions)

    def write_joint_targets(self, joint_names: object, positions_rad: object) -> None:
        self.positions = np.asarray(positions_rad, dtype=float)


def _reader(lines: "queue.Queue[str | None]") -> None:
    for line in sys.stdin:
        if line.strip():
            lines.put(line)
    lines.put(None)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--control-hz", type=float, default=10.0)
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--diagnostics-log", type=Path)
    parser.add_argument(
        "--mapping", choices=("relative", "absolute"), default="relative",
        help="relative: bam DO LECH co tay so voi luc bop co (triet tieu khac biet hinh the). "
             "absolute: dua thang pose co tay nguoi vao khung waist robot — vai nguoi ~0.20m "
             "so voi vai R1 ~0.086m nen muc tieu se nam ngoai tam voi ~0.11m moi ben.")
    parser.add_argument(
        "--hold-max-s", type=float, default=2.0,
        help="Giu (phat lai target cuoi) toi da bao lau khi IK tu choi hoac frame tre. "
             "Qua nguong nay thi ngung phat -> watchdog cua sidecar nha an toan.")
    parser.add_argument(
        "--init-pose", choices=("arms_down", "profile"), default="arms_down",
        help="arms_down: tu the goc = HAI TAY THONG THANG XUONG (shoulder_pitch=0.98, khuyu=0 "
             "-> co tay o [0.00, +-0.18, -0.19], thang duoi vai). profile: dung nguyen "
             "nominal trong file profile (toan 0 = tay chia RA TRUOC 33cm).")
    parser.add_argument(
        "--nominal-shoulder-pitch", type=float, default=0.98,
        help="Goc shoulder_pitch cho init-pose=arms_down. 0.98 rad la gia tri lam co tay nam "
             "thang duoi vai (do bang FK tu R1.urdf); doi neu muon tay hoi cheo ra truoc/sau.")
    parser.add_argument(
        "--head-source", choices=("direct", "ik"), default="direct",
        help="direct: lay yaw/pitch dau THANG tu huong kinh, bo task dau khoi IK. R1 chi co "
             "yaw+pitch nen khong bao gio khop duoc huong dau 3-DoF cua nguoi -> task dau "
             "khong hoi tu, dot het 40 vong moi frame va keo ca tay theo. ik: giu task dau.")
    parser.add_argument(
        "--ik-max-iterations", type=int, default=6,
        help="Tran vong lap IK moi frame (profile de 40). Solver AM SEED tu nghiem truoc nen "
             "cat bot vong khong lam mat bam: frame sau giai tiep tu cho frame truoc dung. "
             "Gioi han 6 vong de giai IK khong chiem gan het watchdog 0.75s.")
    parser.add_argument(
        "--ik-position-tolerance-m", type=float, default=0.01,
        help="Dung sai vi tri (profile de 0.002 = 2mm). Trong bao an toan +-0.15 rad thi 1cm "
             "la du, ma 2mm bat solver chay them hang chuc vong moi frame.")
    parser.add_argument(
        "--keepalive-s", type=float, default=0.3,
        help="Phat lai target cuoi neu vong lap chinh im lau hon nguong nay. Mot lan giai IK "
             "cham hon watchdog 0.75s cua sidecar se lam dut phien neu khong co cai nay.")
    parser.add_argument(
        "--on-release", choices=("park", "stop"), default="park",
        help="park: nha co -> GIU tay o vi tri cuoi, bop lai la dieu khien tiep (chot lai "
             "goc phien nen tay khong giat). stop: nha co -> ket thuc phien.")
    parser.add_argument(
        "--release-debounce-s", type=float, default=5.0,
        help="Co phai duoc coi la NHA khi deadman=false lien tuc qua lau nay. "
             "Trong cua so nay H4 gui STOP va cho phep bop lai; qua nguong moi dong phien.")
    parser.add_argument("--stream-contract", choices=("legacy", "h4"), default="legacy")
    parser.add_argument(
        "--profile",
        type=Path,
        default=ROOT / "experiments/r1_teleop/quest3_sim_v1/T007/config/r1_t007_whole_upper_body_live.json",
    )
    parser.add_argument(
        "--mapping-config",
        type=Path,
        default=ROOT / "experiments/r1_teleop/quest3_sim_v1/T001/config/r1_quest3_sim_v1.json",
    )
    args = parser.parse_args()
    if args.control_hz <= 0.0 or args.duration_s <= 0.0:
        raise SystemExit("control rate and duration must be positive")

    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    declared = dict(profile["whole_upper_body"])
    declared_nominal = [float(value) for value in declared["nominal_joint_position_rad"]]
    if args.init_pose == "arms_down":
        # Thu tu 13 khop: [waist_yaw, L(pitch,roll,yaw,elbow,wrist), R(...), head_pitch, head_yaw]
        # Profile de toan 0, nhung o tu the 0 canh tay CHIA RA TRUOC 33 cm chu khong thong
        # xuong.  Robot luc limp thi tay buong thang, nen goc phai khop voi thuc te do:
        # neu khong, moc retarget lech san mot khoang bang chinh do chenh nay.
        declared_nominal[1] = args.nominal_shoulder_pitch   # trai
        declared_nominal[6] = args.nominal_shoulder_pitch   # phai
    nominal, fixed_waist_yaw = retarget_nominal(
        tuple(declared_nominal),
        str(declared.get("body_mode", "waist_yaw")),
        "arms_head",
    )
    ik_declared = dict(declared["ik"])
    # Do tu phien that: 31-35 vong/frame -> 0.54-1.17 s, vuot watchdog 0.75 s cua sidecar
    # va lam dut phien.  Sai so vi tri luc do da la 0.002-0.013 m, tuc solver dang dot
    # hang chuc vong chi de tu 1cm xuong 2mm.
    ik_declared["max_iterations"] = int(args.ik_max_iterations)
    ik_declared["position_tolerance_m"] = float(args.ik_position_tolerance_m)
    if args.head_source == "direct":
        # Trong so 0 + dung sai pi: task dau khong con gate hoi tu, solver chi lo hai tay.
        # validate() doi weight > 0 nen dung 1e-6: task dau con ton tai ve mat hinh thuc
        # nhung khong con anh huong den nghiem, va dung sai pi khien no khong gate hoi tu.
        ik_declared["head_orientation_weight"] = 1e-6
        ik_declared["head_orientation_tolerance_rad"] = float(np.pi)
    config = WholeUpperBodyLiveConfig(
        urdf_path=(ROOT / str(declared["urdf_path"])).resolve(),
        nominal_joint_position_rad=nominal,
        max_joint_velocity_rad_s=min(0.5, float(declared["max_joint_velocity_rad_s"])),
        max_joint_acceleration_rad_s2=min(1.0, float(declared["max_joint_acceleration_rad_s2"])),
        control_dt_s=1.0 / args.control_hz,
        ik=UpperBodyIKConfig(**ik_declared),
        source_target_frame=str(declared["source_target_frame"]),
        allow_nonconverged_solution=False,
        body_mode="arms_head",
        fixed_waist_yaw_rad=fixed_waist_yaw,
        seed_restart_residual_m=(
            float(declared["seed_restart_residual_m"])
            if declared.get("seed_restart_residual_m") is not None else None
        ),
        allow_projected_position_solution=bool(declared.get("allow_projected_position_solution", False)),
    )
    handle = TargetHandle(nominal)
    sink = WholeUpperBodyIsaacLabSink(handle, config)
    if tuple(sink.model.joint_names) != MODEL_JOINT_NAMES:
        raise SystemExit(f"unexpected arms_head joint order: {sink.model.joint_names}")

    mapping = json.loads(args.mapping_config.read_text(encoding="utf-8"))
    translation = mapping.get("calibration", {}).get("translation_m", [0.0, 0.0, 0.0])
    mapper = R1TeleopMapper(
        TeleopCalibration(
            translation_m=Vector3(*(float(value) for value in translation)),
            yaw_rad=float(mapping.get("calibration", {}).get("yaw_rad", 0.0)),
            source_frame=str(mapping.get("source_frame", "quest_headset")),
            robot_frame=str(mapping.get("robot_frame", "r1_base")),
        ),
        TeleopLimits(command_timeout_s=float(mapping.get("command_timeout_s", 0.5)), allow_velocity=False),
    )
    ownership = R1A5WholeUpperBodyOwnership(body_mode="arms_head")

    # --- Retarget theo phien -------------------------------------------------
    # Vai nguoi rong ~+-0.20 m, vai R1 chi +-0.086 m.  Dua pose co tay TUYET DOI vao
    # khung waist robot -> muc tieu lech ra ngoai ~0.11 m moi ben, dung bang sai so IK
    # do duoc trong phien that (0.105-0.158 m).  Muc tieu khong toi duoc thi solver
    # khong bao gio hoi tu: can 40 vong moi frame (0.63 s, ~1.6 nghiem/giay) thay vi
    # 2-11 vong (17-160 ms).  Bam DO LECH thay vi vi tri tuyet doi se triet tieu ca
    # rong vai, chieu cao lan cho dung, va tay robot xuat phat ngay tai cho no dang o.
    left_chain = load_arm_chain("left")
    right_chain = load_arm_chain("right")
    robot_zero = {
        "left": left_chain.endpoint_position(np.asarray(nominal[0:5], dtype=float)),
        "right": right_chain.endpoint_position(np.asarray(nominal[5:10], dtype=float)),
    }
    human_zero: dict[str, np.ndarray] = {}
    head_zero: list[float] = []          # [yaw, pitch] cua NGUOI luc bop co
    parked = False

    def retarget(target: R1TeleopTargets) -> R1TeleopTargets:
        if args.mapping == "absolute":
            return target
        pair = (("left", target.left_wrist_target), ("right", target.right_wrist_target))
        if any(pose is None for _, pose in pair):
            return target
        if not human_zero:
            for side, pose in pair:
                human_zero[side] = np.array([pose.position.x, pose.position.y, pose.position.z])
            # Dau cung theo phien: sidecar da bat dau bang dau robot O GIUA, nen lenh
            # dung = do lech huong dau nguoi so voi luc bop co.  Khong tru goc nay thi
            # dau robot giat mot phat bang do lech giua he kinh va zero cua robot.
            head_zero[:] = [float(target.head_yaw_rad), float(target.head_pitch_rad)]
            print("[MAP] chot goc phien: nguoi L={} R={} | robot L={} R={}".format(
                np.round(human_zero["left"], 3), np.round(human_zero["right"], 3),
                np.round(robot_zero["left"], 3), np.round(robot_zero["right"], 3)),
                file=sys.stderr, flush=True)
        shifted = {}
        for side, pose in pair:
            now = np.array([pose.position.x, pose.position.y, pose.position.z])
            p = robot_zero[side] + (now - human_zero[side])
            shifted[side] = dataclasses.replace(
                pose, position=Vector3(float(p[0]), float(p[1]), float(p[2])))
        return dataclasses.replace(
            target, left_wrist_target=shifted["left"], right_wrist_target=shifted["right"])
    lines: "queue.Queue[str | None]" = queue.Queue()
    threading.Thread(target=_reader, args=(lines,), daemon=True).start()
    stop_keepalive = threading.Event()
    deadline = time.monotonic() + args.duration_s
    previous_sequence = -1
    stream_started = False
    period = 1.0 / args.control_hz
    # Giu phien song qua cac gian doan ngan.  Mot frame bi IK tu choi, hoac mot
    # frame den tre (mapper coi la stale khi > command_timeout_s), TRUOC DAY lam
    # ca tien trinh return 0 -> stdout dong -> sidecar input_watchdog -> ket thuc
    # phien.  Gio: phat lai target cuoi (robot giu nguyen tu the) trong toi da
    # hold_max_s, roi moi buong cho watchdog nha.  Chi CO NHA THAT (deadman=false
    # lien tuc qua release_debounce_s) moi ket thuc phien co chu y.
    # Hai bo dem TACH BIET: previous_sequence loc frame Quest VAO (theo seq cua
    # bridge), out_sequence danh so frame RA (sidecar chi doi tang dan).  Dung
    # chung mot bien se lam frame hold bom previous_sequence len va vut mat cac
    # frame Quest that co seq nho hon.
    out_sequence = 0
    last_positions: list[float] | None = None
    hold_since: float | None = None
    release_since: float | None = None
    last_source_at = 0.0
    last_accepted_at = 0.0
    last_hold_note = 0.0

    emit_lock = threading.Lock()
    last_emit_at = [time.monotonic()]     # list de closure ghi duoc

    def emit(positions: list[float] | None, upstream_sequence_id: int,
             kind: str, enabled: bool = True) -> bool:
        nonlocal out_sequence
        with emit_lock:
            out_sequence += 1
            seq = out_sequence
            payload = {
                "schema_version": 1,
                "sequence_id": seq,
                "upstream_sequence_id": upstream_sequence_id,
                # H4 requires the deadman to be explicit on every target.
                # The legacy sidecar ignores this additional field.
                "enabled": enabled,
                "sent_monotonic_s": time.monotonic(),
            }
            if enabled:
                payload.update(
                    joint_names=JOINT_NAMES,
                    positions_rad=positions,
                    solution_kind=kind,
                )
            try:
                print(json.dumps(payload, separators=(",", ":")), flush=True)
            except BrokenPipeError:
                return False
            # Thu tu sequence va thu tu ghi ra pipe phai trung nhau. Truoc day
            # keepalive co the chen giua hai buoc, lam sidecar bo frame moi.
            last_emit_at[0] = time.monotonic()
        return True

    def keepalive() -> None:
        """Giu nhip phat khi vong lap chinh dang KET trong mot lan giai IK dai.

        Sidecar cat phien khi khong nhan input qua 0.75 s.  Mot lan solve 1.3 s se
        vuot nguong do ma vong lap chinh khong the tu cuu vi no dang bi chan trong
        numpy.  Luong nay phat lai target cuoi de phien song, khong tao target moi.
        """
        while not stop_keepalive.is_set():
            time.sleep(args.keepalive_s / 3.0)
            now = time.monotonic()
            if (not stream_started or last_positions is None or release_since is not None
                    or parked or now - last_source_at > args.hold_max_s
                    or now - last_accepted_at > args.hold_max_s):
                continue
            if now - last_emit_at[0] >= args.keepalive_s:
                emit(last_positions, previous_sequence, "keepalive")

    def note_hold(reason: str, now: float) -> None:
        nonlocal last_hold_note
        if now - last_hold_note >= 1.0:
            last_hold_note = now
            held_s = 0.0 if hold_since is None else now - hold_since
            print(f"[HOLD] {reason}; giu target cuoi {held_s:.1f}s/{args.hold_max_s:.1f}s",
                  file=sys.stderr, flush=True)
    # Khoi dong SAU khi moi bien/ham closure da ton tai.
    threading.Thread(target=keepalive, daemon=True).start()

    while time.monotonic() < deadline:
        loop_start = time.monotonic()
        newest: R1TeleopCommand | None = None
        closed = False
        while True:
            try:
                line = lines.get_nowait()
            except queue.Empty:
                break
            if line is None:
                closed = True
                break
            try:
                candidate = R1TeleopCommand.from_dict(json.loads(line))
            except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
                print(f"[WARN] invalid Quest command: {exc}", file=sys.stderr, flush=True)
                continue
            if candidate.sequence_id <= previous_sequence:
                continue
            previous_sequence = candidate.sequence_id
            newest = candidate
        now = time.monotonic()
        accepted_this_tick = False
        h4_stop_sent = False
        hold_reason: str | None = None

        if newest is not None:
            if newest.reset_requested:
                if stream_started:
                    print("[STOP] left-trigger reset requested; restart to establish a new neutral", file=sys.stderr)
                    return 0
                sink.reset_session()
                human_zero.clear()   # chot lai goc khi nguoi yeu cau reset
                stream_started = False

            # NHA THAT chi tinh theo co trong chinh lenh, KHONG tinh theo
            # target.enabled (cai do con false khi frame den tre).
            if newest.deadman_enabled:
                release_since = None
                if parked:
                    # Bop lai sau khi park: tay NGUOI da di chuyen trong luc nghi.  Chot
                    # lai ca hai goc (nguoi = hien tai, robot = cho tay dang dung) neu
                    # khong tay robot se nhay mot phat bang do lech tich luy.
                    parked = False
                    human_zero.clear()
                    head_zero.clear()
                    if last_positions is not None:
                        robot_zero["left"] = left_chain.endpoint_position(
                            np.asarray(last_positions[0:5], dtype=float))
                        robot_zero["right"] = right_chain.endpoint_position(
                            np.asarray(last_positions[5:10], dtype=float))
                    print("[PARK] bop co lai — chot lai goc phien tai tu the hien tai",
                          file=sys.stderr, flush=True)
            elif release_since is None:
                release_since = now

            target = mapper.map(newest, now)
            if not target.enabled:
                # deadman=false -> cho debounce xu ly ben duoi; con lai la tre.
                hold_reason = ("deadman released" if not newest.deadman_enabled
                               else "frame tre (> command_timeout_s)")
                if args.stream_contract == "h4" and stream_started:
                    # Explicit STOP heartbeats keep H4 re-armable while the
                    # release grace window is open.
                    if not emit(None, previous_sequence, "deadman_released", enabled=False):
                        return 0
                    h4_stop_sent = True
            else:
                last_source_at = now
                target = retarget(target)
                if (last_positions is not None and stream_started and
                        time.monotonic() - last_emit_at[0] >= args.keepalive_s):
                    emit(last_positions, previous_sequence, "keepalive")
                solve_started = time.monotonic()
                sink.apply_upper_body(target, ownership.upper_body)
                solve_elapsed = time.monotonic() - solve_started
                application = sink.last_application or {}
                diagnostic = {
                    "sequence_id": newest.sequence_id,
                    "solve_elapsed_s": solve_elapsed,
                    "accepted": bool(application.get("accepted")),
                    "reason": application.get("reason"),
                    "solution_kind": application.get("solver_solution_kind"),
                    "ik": application.get("ik"),
                }
                if args.diagnostics_log is not None:
                    args.diagnostics_log.parent.mkdir(parents=True, exist_ok=True)
                    with args.diagnostics_log.open("a", encoding="utf-8") as stream:
                        stream.write(json.dumps(diagnostic, separators=(",", ":")) + "\n")
                if solve_elapsed > 0.5 or not application.get("accepted"):
                    status = "rejected" if not application.get("accepted") else "slow"
                    reason = application.get("reason") or "solve_time"
                    print(f"[IK] {status}: {reason} ({solve_elapsed:.2f}s)",
                          file=sys.stderr, flush=True)
                if application.get("accepted"):
                    positions = [float(handle.positions[index]) for index in STREAM_FROM_MODEL]
                    if args.head_source == "direct":
                        # stream order: [...10 khop tay, head_yaw, head_pitch]
                        yaw0, pitch0 = (head_zero if head_zero else (0.0, 0.0))
                        positions[10] = float(target.head_yaw_rad) - yaw0
                        positions[11] = float(target.head_pitch_rad) - pitch0
                    if not emit(positions, newest.sequence_id,
                                str(application.get("solver_solution_kind"))):
                        return 0
                    last_positions = positions
                    last_accepted_at = time.monotonic()
                    stream_started = True
                    accepted_this_tick = True
                    hold_since = None
                else:
                    hold_reason = f"IK tu choi ({application.get('reason')})"
        elif stream_started:
            hold_reason = "khong co frame moi trong tick nay"
            if args.stream_contract == "h4":
                if not emit(None, previous_sequence, "source_gap", enabled=False):
                    return 0
                h4_stop_sent = True

        # Nha co that (deadman=false lien tuc qua debounce).
        if (release_since is not None and stream_started and
                now - release_since >= args.release_debounce_s):
            if args.on_release == "stop":
                print(f"[STOP] deadman released ({now - release_since:.2f}s) — ket thuc phien",
                      file=sys.stderr, flush=True)
                return 0
            if not parked:
                parked = True
                print(f"[PARK] nha co ({now - release_since:.2f}s) — GIU tay o vi tri cuoi. "
                      "Bop co lai de dieu khien tiep; Ctrl-C de ket thuc.",
                      file=sys.stderr, flush=True)

        # Chua accept duoc frame nao trong tick: GIU target cuoi de sidecar khong
        # bi input_watchdog, nhung chi trong hold_max_s roi buong cho watchdog nha.
        if parked and last_positions is not None:
            # Giu vo thoi han: nguoi van hanh CHU DONG nha co.  Mat ket noi thi tien
            # trinh nay chet -> sidecar het input -> watchdog nha tay: fail-safe con nguyen.
            if not emit(last_positions, previous_sequence, "park"):
                return 0
        elif (stream_started and not accepted_this_tick and last_positions is not None
              and not h4_stop_sent):
            if hold_since is None:
                hold_since = now
            if now - hold_since <= args.hold_max_s:
                note_hold(hold_reason or "khong ro", now)
                if not emit(last_positions, previous_sequence, "hold"):
                    return 0
            else:
                print(f"[STOP] giu qua {args.hold_max_s:.1f}s ({hold_reason}) — ngung phat, "
                      "de watchdog cua sidecar nha tay ve policy", file=sys.stderr, flush=True)
                return 0

        if closed:
            return 0
        remaining = period - (time.monotonic() - loop_start)
        if remaining > 0.0:
            time.sleep(remaining)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
