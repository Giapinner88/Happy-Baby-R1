"""Reuse P4/P6 and native gesture protocols. Never publish DDS motor commands."""
import importlib
import fcntl
import json
from pathlib import Path
import socket
import sys
import time
import uuid



class RobotBackends:
    def __init__(self, config):
        self.config = config
        self.approach = None
        self.search = None
        profile = json.loads(Path(config["approach_config"]).read_text())
        # Manager is reused directly, so retain the same process lock as its CLI.
        self.instance = open(profile["instance_lock"], "a")
        fcntl.flock(self.instance, fcntl.LOCK_EX | fcntl.LOCK_NB)
        app = Path(config["app_root"])
        for path in (app, app / "experiments/r1_object_grounding/p6_robot_local"):
            sys.path.insert(0, str(path))

    def owner(self, command, request_id=None, gesture_id=None):
        request_id = request_id or uuid.uuid4().hex
        payload = {"v": 1, "request_id": request_id, "command": command,
                   "gesture_id": gesture_id, "source": "voice"}
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(1.5)
            connection.connect(self.config["gesture_socket"])
            connection.sendall(json.dumps(payload).encode() + b"\n")
            with connection.makefile("rb") as reader:
                raw = reader.readline(65537)
        if len(raw) > 65536 or not raw.endswith(b"\n"):
            raise ValueError("invalid gesture reply size")
        result = json.loads(raw)
        if (not isinstance(result, dict) or type(result.get("ok")) is not bool
                or not isinstance(result.get("status"), str)
                or result.get("request_id", request_id) != request_id):
            raise ValueError("invalid gesture owner reply")
        return result

    def start_gesture(self, task):
        if self.config["mode"] == "shadow":
            return {"ok": True, "status": "shadow", "executed": False}
        return self.owner("play", task["request_id"], task["gesture_id"])

    def run(self, task, cancel, progress):
        action = task["action"]
        if self.config["mode"] == "shadow" and action != "approach":
            return {"status": "shadow", "executed": False,
                    "answer": "Chế độ quan sát: robot chưa thực hiện chuyển động."}
        if action == "gesture":
            while True:
                state = self.owner("status")
                progress(state)
                if state.get("ok") is not True:
                    raise RuntimeError("gesture status rejected")
                if state["status"] == "idle":
                    return {"status": "cancelled" if cancel.is_set() else "completed",
                            "owner": state, "answer": "Cử chỉ đã kết thúc."}
                cancel.wait(self.config["poll_s"]) if not cancel.is_set() else time.sleep(self.config["poll_s"])
        if action == "find_and_point":
            result = self._search(task, cancel, face_only=False)
            if result.get("status") == "found":
                result["answer"] = "Nó ở đằng kia nè."
                result["pointing_accuracy_verified"] = False
            return result
        if action == "approach":
            return self._approach(task, cancel, progress)
        raise ValueError("unknown action")

    # Reasons a look-once round can end with; see VisualSegments / runtime.
    CAMERA_GOAL = ("estimated_visual_goal", "goal_reached_kinematic")
    LOST_TARGET = ("look_once alignment not converged", "initial visual measurement timeout", "target_lost_latched",
                   "visual recheck timeout")

    def _face(self, task, cancel, progress):
        """P4 search in face-only mode, then release the search owner so approach can take the APV."""
        faced = self._search(task, cancel, face_only=True)
        progress({"phase": "search", "search": faced})
        if faced.get("status") == "found":
            try:
                from experiments.r1_object_grounding.p6_onboard.local_bridge import call_agent
                call_agent({"method": "release"}, timeout_s=2)
            except Exception as exc:
                faced["release_error"] = str(exc)
        return faced

    def _approach_round(self, task, cancel, progress, config):
        module = importlib.import_module("runtime")
        module.validate_config(config)
        self.approach = module.Manager(config)
        if cancel.is_set():
            return {"status": "cancelled", "active": False}, 0.
        code, admitted = self.approach.submit({"query": task["query"]})
        if code != 202:
            return dict(admitted, active=False), 0.
        used = 0.
        while True:
            if cancel.is_set():
                self.approach.cancel.set()
            state = self.approach.snapshot()
            if isinstance(state.get("total_commanded_m"), (int, float)):
                used = max(used, float(state["total_commanded_m"]))
            progress(state)
            if not state["active"]:
                return state, used
            time.sleep(self.config["poll_s"])

    def _approach(self, task, cancel, progress):
        """Find + face, then walk; if the object is gone or moved when a round ends, look for it again and re-approach.

        A round that ends by camera (estimated_visual_goal) is final. A round that ends by dead reckoning (tracker lost) is
        followed by another round, whose fresh grounding either confirms the goal without moving or walks to the object's
        new place; a round that cannot see the object at all triggers a new find + face (head scan / 180 deg turn).
        Bounded by relocate_rounds, the shared commanded-distance budget and relocate_time_s.
        """
        settings = self.config.get("search", {})
        execute = self.config["mode"] == "execute"
        config = json.loads(Path(self.config["approach_config"]).read_text())
        if config["execute"] != execute:
            raise ValueError("assistant/approach mode mismatch")
        search = execute and settings.get("approach_search", False)
        if search:
            faced = self._face(task, cancel, progress)
            if faced.get("status") != "found":
                return faced
            if cancel.is_set():
                return {"status": "cancelled", "answer": "Mình dừng rồi."}
        relocations = settings.get("relocate_rounds", 0) if search else 0
        budget = config.get("visual_segments", {}).get("max_total_commanded_m", 3.)
        began = time.monotonic();used_total = 0.;moved = False;rounds = []
        while True:
            round_config = json.loads(json.dumps(config))
            if "visual_segments" in round_config:  # one distance budget shared by all rounds
                round_config["visual_segments"]["max_total_commanded_m"] = budget - used_total
            state, used = self._approach_round(task, cancel, progress, round_config)
            used_total += used
            reason = state.get("reason");kind = state.get("state")
            rounds.append({"reason": reason, "state": kind, "commanded_m": round(used, 3)})
            state["approach_rounds"] = rounds
            if cancel.is_set():
                return dict(state, status="cancelled", answer="Mình dừng rồi.")
            if not config["execute"]:
                return dict(state, status="shadow", answer="Đã kết thúc quan sát; robot chưa đi tới vật.")
            if kind == "NEEDS_CLARIFICATION":
                return dict(state, status="needs_clarification")
            if reason in self.CAMERA_GOAL:
                answer = "Mình tìm lại được rồi, tới nơi rồi nè." if moved else "Tới nơi rồi nè."
                return dict(state, status="estimated_goal", camera_confirmed=True, object_moved=moved,
                            physical_arrival_verified=False, answer=answer)
            lost = kind == "NOT_FOUND" or reason in self.LOST_TARGET
            done_by_estimate = reason == "estimated_goal_dead_reckoned"
            exhausted = (len(rounds) > relocations or budget - used_total < .3
                         or time.monotonic() - began > settings.get("relocate_time_s", 150.))
            if "owner ACK rejected" in str(reason):
                # The leg controller revoked the approach session (manual stick input, mode/eligibility change,
                # emergency stop). Never reopen it automatically: that would override the operator.
                return dict(state, status="failed", owner_revoked=True,
                            answer="Mình dừng lại vì tay cầm vừa lấy lại quyền điều khiển. Bạn nói lại khi sẵn sàng nhé.")
            if exhausted or not (lost or done_by_estimate):
                if done_by_estimate:
                    return dict(state, status="estimated_goal", camera_confirmed=False, object_moved=moved,
                                physical_arrival_verified=False,
                                answer="Mình tới gần rồi.")
                if kind == "NOT_FOUND" or lost:
                    return dict(state, status="not_found", object_moved=moved,
                                answer="Mình không thấy nó nữa, bạn chỉ lại giúp mình nhé.")
                return dict(state, status="failed", answer="Mình chưa đi tới được, bạn thử lại giúp mình nhé.")
            if lost:
                # The object is not where the robot is looking: search again (head scan, 180 deg turn) and face it.
                moved = True
                faced = self._face(task, cancel, progress)
                if faced.get("status") != "found":
                    return dict(faced, object_moved=True, approach_rounds=rounds,
                                answer="Mình không tìm lại được nó, bạn chỉ giúp mình nhé.")
                if cancel.is_set():
                    return {"status": "cancelled", "answer": "Mình dừng rồi."}
            # After a dead-reckoned stop the next round's fresh grounding confirms the goal or follows the object.


    def _search(self, task, cancel, face_only):
        # This stdlib orchestrator launches image processing in its own env.
        self.search = importlib.import_module("experiments.r1_object_grounding.p5_voice_search.server")
        if cancel.is_set():
            return {"status": "cancelled"}
        settings = self.config["search"]
        with open(self.config["camera_lock"], "a") as lease:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return self.search.run_search(
                task["query"], adapter_argv=Path(settings["adapter_argv"]),
                runtime_root=Path(self.config["hb_root"]),
                bbox_config=Path(settings["bbox_config"]),
                vision_config=Path(settings["vision_config"]),
                output_root=Path(self.config["runs_dir"]) / "search",
                head_only=settings["head_only"], body_turns=settings["body_turns"],
                body_step_deg=settings["body_step_deg"], batch_head_capture=True,
                python_executable=self.config["search_python"], cancel_event=cancel,
                face_only=face_only)

    def cancel(self, task):
        if self.config["mode"] == "shadow":
            if self.approach and self.approach.active:
                self.approach.cancel.set()
            return {"status": "cancel_requested"}
        if task["action"] == "gesture":
            return self.owner("cancel")
        if task["action"] == "approach":
            if self.approach and self.approach.cancel:
                self.approach.cancel.set()
            active = self.search.ACTIVE if self.search else None
            if active:  # still in the find-and-face phase
                active["cancel"].touch()
                from experiments.r1_object_grounding.p6_onboard.local_bridge import call_agent
                return call_agent({"method": "release"}, timeout_s=2)
            return {"status": "cancel_requested"}
        if task["action"] == "find_and_point":
            active = self.search.ACTIVE if self.search else None
            if active:
                active["cancel"].touch()
            # Release sender promptly even while vision/cloud is finishing.
            from experiments.r1_object_grounding.p6_onboard.local_bridge import call_agent
            return call_agent({"method": "release"}, timeout_s=2)
        return {"status": "cancel_requested"}

    def recover(self, task):
        """A failed worker must not free motion ownership on an uncertain reply."""
        if task["action"] == "gesture":
            result = self.owner("status")
            return result.get("ok") is True and result.get("status") == "idle"
        if task["action"] == "approach":
            if self.approach is None:
                return False
            state = self.approach.snapshot()
            stop = state.get("stop") or {}
            return (not state["active"] and ((not stop and not state.get("motion_requests_sent", 0))
                    or (stop.get("stop_acknowledged") is True and stop.get("release_acknowledged") is not False)))
        return False  # Search worker must actually exit before admission resumes.
