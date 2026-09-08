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
# Isaac Sim 5.1's USDRT evidence-camera path currently supports cuda:0 only.
# Headless/no-video baselines may still override this with DEVICE=cuda:1.
DEVICE      ?= cuda:0
CERT_FILE   ?= $(HOME)/.config/xr_teleoperate/happybaby_$(HOST_IP_TAG)/cert.pem
KEY_FILE    ?= $(HOME)/.config/xr_teleoperate/happybaby_$(HOST_IP_TAG)/key.pem
# arms_head | waist_yaw | full_upper_body. Empty keeps the profile's own value.
BODY_MODE   ?=
# Open Isaac Sim and record one evidence-camera view by default. For a lighter
# connectivity-only run, override with `TELEOP_ARGS="--headless --no-video"`.
TELEOP_ARGS ?= --single-view
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
	@echo "  make teleop-upstream-solve   solve a recorded trace with the vendor xr_teleoperate IK"
	@echo "  make teleop-upstream-stream  stream joint targets from the vendor IK (online path)"
	@echo "  make teleop-hardware-prepare  preflight + copy only; never starts or arms robot"
	@echo "  make teleop-hardware  foreground R1 arms/head; prompts for fixture/E-stop confirmation"
	@echo ""
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
