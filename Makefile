# Happy Baby R1 — operator entry points.
#
# This file is a thin dispatcher: every target shells out to a documented
# entry point. It only validates required operator input and assembles the
# corresponding command-line arguments.
#
#   make help              list every target
#   make teleop HOST_IP=192.168.1.106
#                          run the T007 coupled whole-upper-body Quest pilot
#   make teleop-dry-run    show the allocated paths and commands, run nothing
#
# Override any variable on the command line, e.g.
#   make teleop HOST_IP=192.168.1.106 DURATION_S=300

SHELL := /bin/bash
PYTHON ?= python3

# --- Teleop pilot -----------------------------------------------------------
HOST_IP     ?=
HOST_IP_TAG  = $(subst .,_,$(strip $(HOST_IP)))
DURATION_S  ?= 180
PHYSICS_HZ  ?= 200
CONTROL_HZ  ?= 30
VIDEO_FPS   ?= 10
DEVICE      ?= cuda:1
CERT_FILE   ?= $(HOME)/.config/xr_teleoperate/happybaby_$(HOST_IP_TAG)/cert.pem
KEY_FILE    ?= $(HOME)/.config/xr_teleoperate/happybaby_$(HOST_IP_TAG)/key.pem
# arms_head | waist_yaw | full_upper_body. Empty keeps the profile's own value.
BODY_MODE   ?=
# Temporary connectivity baseline for the current Wi-Fi. Override with
# `TELEOP_ARGS=--single-view` after the live bridge has been verified.
TELEOP_ARGS ?= --headless --no-video
WHOLE_UPPER_BODY_CONFIG ?= experiments/r1_teleop/quest3_sim_v1/T007/config/r1_t007_whole_upper_body_live.json

TELEOP_CMD = $(PYTHON) scripts/teleop/run_t007_upper_body_pilot.py \
	--host-ip $(HOST_IP) \
	--duration-s $(DURATION_S) \
	--physics-hz $(PHYSICS_HZ) \
	--control-hz $(CONTROL_HZ) \
	--video-fps $(VIDEO_FPS) \
	--device $(DEVICE) \
	--cert-file $(CERT_FILE) \
	--key-file $(KEY_FILE) \
	--whole-upper-body-config $(WHOLE_UPPER_BODY_CONFIG) \
	$(if $(BODY_MODE),--body-mode $(BODY_MODE)) \
	$(TELEOP_ARGS)

TELEOP_UPSTREAM_CMD = $(PYTHON) scripts/teleop/run_t007_upper_body_pilot.py \
	--host-ip $(HOST_IP) \
	--duration-s $(DURATION_S) \
	--physics-hz $(PHYSICS_HZ) \
	--control-hz $(CONTROL_HZ) \
	--video-fps $(VIDEO_FPS) \
	--device $(DEVICE) \
	--cert-file $(CERT_FILE) \
	--key-file $(KEY_FILE) \
	--upstream-solver \
	$(TELEOP_ARGS)

.DEFAULT_GOAL := help
.PHONY: help check-teleop-network teleop teleop-arms teleop-dry-run teleop-head-only test-teleop \
	dataset-d001 dataset-d001-live dataset-d001-dry-run dataset-d002-live dataset-d002-dry-run \
	check-d002-lerobot-input dataset-d002-lerobot-30fps dataset-d002-upload dataset-d002-publish-30fps \
	teleop-hardware-prepare teleop-hardware teleop-upstream-solve teleop-upstream-stream \
	teleop-arms-differential teleop-arms-dry-run

help:
	@echo "Happy Baby R1 targets:"
	@echo "  make teleop           T007 coupled whole-upper-body Quest pilot (simulation-only)"
	@echo "  make teleop-arms      arms + head via the UNMODIFIED vendor xr_teleoperate IK"
	@echo "  make teleop-arms-dry-run      print the three-process upstream pipeline, run nothing"
	@echo "  make teleop-arms-differential the old arms+head path, solved in this repo"
	@echo "  make teleop-dry-run   print allocated run paths and both commands, run nothing"
	@echo "  make teleop-head-only T001-B head-only connectivity pilot"
	@echo "  make test-teleop      run the teleop test suite"
	@echo "  make dataset-d001 REPLAY_RUN=<t007-run-id>  D001 dataset from a recorded session (no headset)"
	@echo "  make dataset-d001-live HOST_IP=...          D001 dataset from a live Quest session"
	@echo "  make dataset-d001-dry-run REPLAY_RUN=...    print the allocated run path, run nothing"
	@echo "  make dataset-d002-live HOST_IP=...          D002 VLA numbered-table dataset with operator HUD"
	@echo "  make dataset-d002-dry-run HOST_IP=...       print the D002 live commands, run nothing"
	@echo "  make dataset-d002-lerobot-30fps D002_RUN_ID=...  convert arms+head and validate a true-30-FPS D002 run"
	@echo "  make dataset-d002-publish-30fps D002_RUN_ID=...  convert 12-DoF, validate, then upload private to HF"
	@echo "  make dataset-d002-upload D002_LEROBOT_ROOT=...   upload a validated 12-DoF/30-FPS dataset"
	@echo "  make teleop-upstream-solve   solve a recorded trace with the vendor xr_teleoperate IK"
	@echo "  make teleop-upstream-stream  stream joint targets from the vendor IK (online path)"
	@echo "  make teleop-hardware-prepare  preflight + copy only; never starts or arms robot"
	@echo "  make teleop-hardware  foreground R1 arms/head; prompts for fixture/E-stop confirmation"
	@echo ""
	@echo "Dataset vars: REPLAY_RUN DATASET_DEVICE DATASET_ARGS D002_RUN_ID D002_LEROBOT_ROOT D002_HF_REPO"
	@echo "Variables: SOURCE_RUN UPSTREAM_PYTHON HOST_IP DURATION_S PHYSICS_HZ CONTROL_HZ VIDEO_FPS DEVICE CERT_FILE KEY_FILE BODY_MODE TELEOP_ARGS WHOLE_UPPER_BODY_CONFIG"
	@echo "BODY_MODE: arms_head (torso frozen) | waist_yaw (default) | full_upper_body (+waist roll)"
	@echo "Example:   make teleop HOST_IP=192.168.1.106 BODY_MODE=arms_head"

check-teleop-network:
	@test -n "$(strip $(HOST_IP))" || { \
		echo "[FAIL] HOST_IP is required. Example: make teleop HOST_IP=192.168.1.106"; \
		exit 2; \
	}

## Run the coupled whole-upper-body teleop pilot end to end.
## Allocates the run id, starts the Quest bridge piped into Isaac Sim, and
## writes evidence under experiments/r1_teleop/quest3_sim_v1/T007/runs/.
teleop: check-teleop-network
	$(TELEOP_CMD)

## Arms + head, solved by the unmodified vendor xr_teleoperate IK.
## Nothing in this repository solves on this path: the bridge feeds the vendor
## solver, and the simulator only applies the joints it returns. The torso is
## frozen because the vendor model locks waist yaw and both head joints.
teleop-arms: check-teleop-network
	$(TELEOP_UPSTREAM_CMD)

## The previous arms+head path, solved by this repository's differential
## controller. Kept so the two can still be compared on one trace.
teleop-arms-differential:
	$(MAKE) teleop BODY_MODE=arms_head \
		WHOLE_UPPER_BODY_CONFIG=experiments/r1_teleop/quest3_sim_v1/T007/config/r1_t007_differential_live.json

teleop-dry-run: check-teleop-network
	$(TELEOP_CMD) --dry-run

teleop-arms-dry-run: check-teleop-network
	$(TELEOP_UPSTREAM_CMD) --dry-run

teleop-head-only: check-teleop-network
	$(PYTHON) scripts/teleop/run_t001_b_pilot.py \
		--host-ip $(HOST_IP) \
		--cert-file $(CERT_FILE) \
		--key-file $(KEY_FILE) \
		$(TELEOP_ARGS)

test-teleop:
	$(PYTHON) -m pytest tests/teleop -q

# --- D001 dataset -----------------------------------------------------------
# Thu dataset "chỉ tay vào vật". Cả hai đường đều cấp một run id dưới
# experiments/r1_dataset/quest3_sim_v1/D001/runs/ và áp chính sách loại episode
# khai trong profile; episode bị loại được giữ lại trong episodes_rejected/.
REPLAY_RUN      ?= t007_whole_upper_body_20260820T140045Z
DATASET_DEVICE  ?= cuda:0
DATASET_ARGS    ?=

D001_CMD = $(PYTHON) scripts/teleop/run_d001_dataset.py --device $(DATASET_DEVICE) $(DATASET_ARGS)

## Phát lại một phiên T007 đã ghi thành episode. Không cần đeo kính.
dataset-d001:
	$(D001_CMD) --replay-run $(REPLAY_RUN)

dataset-d001-dry-run:
	$(D001_CMD) --replay-run $(REPLAY_RUN) --dry-run

## Phiên Quest trực tiếp. Cần kính và chứng chỉ khớp HOST_IP.
dataset-d001-live: check-teleop-network
	$(D001_CMD) --host-ip $(HOST_IP) --cert-file $(CERT_FILE) --key-file $(KEY_FILE)

# D002 dùng solver xr_teleoperate ở process tv riêng; Isaac chỉ nhận joint stream.
# GUI head view là mặc định ở đây vì HUD mục tiêu phải nhìn được trước khi bóp cò.
D002_CONFIG = experiments/r1_dataset/quest3_sim_v1/D002/config/r1_d002_number_pointing.json
D002_CMD = $(PYTHON) scripts/teleop/run_d001_dataset.py --device $(DATASET_DEVICE) \
	--dataset-config $(D002_CONFIG) --gui --viewport-camera head --solver upstream $(DATASET_ARGS)

dataset-d002-live: check-teleop-network
	$(D002_CMD) --host-ip $(HOST_IP) --cert-file $(CERT_FILE) --key-file $(KEY_FILE)

dataset-d002-dry-run: check-teleop-network
	$(D002_CMD) --host-ip $(HOST_IP) --cert-file $(CERT_FILE) --key-file $(KEY_FILE) --dry-run

# Chỉ dùng các target 30 FPS này với run có info.image.fps_requested=30 và
# info.image.fps đo được gần 30 ở mọi episode. --fps 30 chuẩn hoá timestamp
# LeRobot sau gate đó; nó không thể nội suy thêm frame cho một run 8 FPS.
D002_RUN_ID        ?=
D002_RAW_DIR        = experiments/r1_dataset/quest3_sim_v1/D002/runs/$(D002_RUN_ID)/episodes
D002_LEROBOT_ROOT  ?= data/lerobot/$(D002_RUN_ID)_30fps
D002_HF_REPO       ?= vasco281204/r1-d002-number-pointing-30fps
LEROBOT_ENV        ?= lerobot

check-d002-lerobot-input:
	@test -n "$(strip $(D002_RUN_ID))" || { \
		echo "[FAIL] D002_RUN_ID is required (and must be a run recorded at true 30 FPS)."; \
		exit 2; \
	}
	@test -d "$(D002_RAW_DIR)" || { echo "[FAIL] Missing raw episode directory: $(D002_RAW_DIR)"; exit 2; }
	@test -n "$(strip $(D002_LEROBOT_ROOT))" || { echo "[FAIL] D002_LEROBOT_ROOT is required."; exit 2; }
	@test -n "$(strip $(D002_HF_REPO))" || { echo "[FAIL] D002_HF_REPO is required."; exit 2; }

dataset-d002-lerobot-30fps: check-d002-lerobot-input
	$(PYTHON) scripts/teleop/check_r1_dataset_fps.py \
		--raw-dir "$(D002_RAW_DIR)" --expected-fps 30 \
		--min-measured-fps 27 --max-measured-fps 33
	conda run --no-capture-output -n $(LEROBOT_ENV) \
		python scripts/teleop/convert_r1_xr_to_lerobot.py \
		--raw-dir "$(D002_RAW_DIR)" --profile a5 --fps 30 \
		--camera-map color_0=head_camera --repo-id "$(D002_HF_REPO)" \
		--output-root "$(D002_LEROBOT_ROOT)"
	$(PYTHON) scripts/teleop/sanitize_lerobot_manifest.py \
		--dataset-root "$(D002_LEROBOT_ROOT)"
	conda run --no-capture-output -n $(LEROBOT_ENV) \
		python scripts/teleop/validate_r1_lerobot_dataset.py \
		--repo-id "$(D002_HF_REPO)" --root "$(D002_LEROBOT_ROOT)" --profile a5 \
		--report "$(D002_LEROBOT_ROOT)/meta/training_validation.json"

dataset-d002-upload:
	@test -f "$(D002_LEROBOT_ROOT)/meta/training_validation.json" || { \
		echo "[FAIL] Missing validation report under $(D002_LEROBOT_ROOT)."; exit 2; \
	}
	@$(PYTHON) -c 'import json, pathlib, sys; r=json.loads(pathlib.Path(sys.argv[1]).read_text()); expected={"schema":"happy_baby_r1.training_dataset_validation","schema_version":2,"status":"passed","joint_contract":"arms_head","state_dim":12,"action_dim":12,"fps":30}; bad={k:(r.get(k),v) for k,v in expected.items() if r.get(k)!=v}; assert not bad, f"training validation is not the required 12-DoF/30-FPS contract: {bad}"' \
		"$(D002_LEROBOT_ROOT)/meta/training_validation.json"
	conda run --no-capture-output -n $(LEROBOT_ENV) \
		hf upload "$(D002_HF_REPO)" "$(D002_LEROBOT_ROOT)" . \
		--repo-type dataset --private --commit-message "Upload D002 true-30-FPS dataset"

dataset-d002-publish-30fps: dataset-d002-lerobot-30fps
	$(MAKE) dataset-d002-upload D002_LEROBOT_ROOT="$(D002_LEROBOT_ROOT)" D002_HF_REPO="$(D002_HF_REPO)"

# Safe one-command staging path. This deliberately stops at the hardware
# entrypoint/gate checks and never installs, starts, enables, or arms a service.
teleop-hardware-prepare:
	./hardware/teleop/scripts/sync_from_workspace.sh
	./hardware/teleop/scripts/check_vuer.sh
	ROBOT="$(ROBOT)" ./hardware/teleop/scripts/deploy_teleop.sh deploy

teleop-hardware: check-teleop-network
	ROBOT="$(ROBOT)" HOST_IP="$(HOST_IP)" DURATION_S="$(DURATION_S)" \
		CERT_FILE="$(CERT_FILE)" KEY_FILE="$(KEY_FILE)" \
		CONFIRM_SUSPENDED_WITH_ESTOP="$(CONFIRM_SUSPENDED_WITH_ESTOP)" \
		./scripts/teleop/run_r1_quest3_hardware.sh

# --- Vendor xr_teleoperate solver -------------------------------------------
# Runs `R1_A5_ArmIK` from third_party unmodified, in the `tv` environment where
# CasADi and the Pinocchio 3 CasADi bindings live. The Isaac environment has
# neither, which is why the solver is a separate process rather than an import.
SOURCE_RUN       ?= experiments/r1_teleop/quest3_sim_v1/T007/runs/t007_whole_upper_body_20260823T122433Z
UPSTREAM_OUT     ?= experiments/r1_teleop/quest3_sim_v1/T007/runs/t007_upstream_ik_$(shell date -u +%Y%m%dT%H%M%SZ)
UPSTREAM_PYTHON  ?= conda run --no-capture-output -n tv python

## Solve a recorded trace offline with the vendor solver and write T007 evidence.
## Produces the same offline_joint_trajectory.npz the Isaac replay path consumes.
teleop-upstream-solve:
	$(PYTHON) scripts/teleop/solve_r1_t007_upstream_ik.py \
		--source-run $(SOURCE_RUN) \
		--output-dir $(UPSTREAM_OUT)

## Stream joint targets from the vendor solver: stdin commands, stdout targets.
## This is the online path, and the one the hardware sidecar consumes.
teleop-upstream-stream:
	$(UPSTREAM_PYTHON) scripts/teleop/run_r1_upstream_ik_stream.py $(TELEOP_ARGS)
