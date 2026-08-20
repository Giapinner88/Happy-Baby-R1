# Happy Baby R1 — operator entry points.
#
# This file is a thin dispatcher: every target shells out to a script that is
# already the documented entry point. It adds no logic of its own, so a target
# and its underlying command can never disagree about what a run does.
#
#   make help              list every target
#   make teleop            run the T007 coupled whole-upper-body Quest pilot
#   make teleop-dry-run    show the allocated paths and commands, run nothing
#
# Override any variable on the command line, e.g.
#   make teleop HOST_IP=10.42.0.5 DURATION_S=300

SHELL := /bin/bash
PYTHON ?= python3

# --- Teleop pilot -----------------------------------------------------------
HOST_IP     ?= 192.168.1.106
DURATION_S  ?= 180
PHYSICS_HZ  ?= 100
CONTROL_HZ  ?= 20
CERT_FILE   ?= $(HOME)/.config/xr_teleoperate/happybaby_192_168_1_106/cert.pem
KEY_FILE    ?= $(HOME)/.config/xr_teleoperate/happybaby_192_168_1_106/key.pem
# arms_head | waist_yaw | full_upper_body. Empty keeps the profile's own value.
BODY_MODE   ?=
TELEOP_ARGS ?=

TELEOP_CMD = $(PYTHON) scripts/teleop/run_t007_upper_body_pilot.py \
	--host-ip $(HOST_IP) \
	--duration-s $(DURATION_S) \
	--physics-hz $(PHYSICS_HZ) \
	--control-hz $(CONTROL_HZ) \
	--cert-file $(CERT_FILE) \
	--key-file $(KEY_FILE) \
	$(if $(BODY_MODE),--body-mode $(BODY_MODE)) \
	$(TELEOP_ARGS)

.DEFAULT_GOAL := help
.PHONY: help teleop teleop-arms teleop-dry-run teleop-head-only test-teleop teleop-hardware-prepare teleop-hardware

help:
	@echo "Happy Baby R1 targets:"
	@echo "  make teleop           T007 coupled whole-upper-body Quest pilot (simulation-only)"
	@echo "  make teleop-arms      same pilot with the torso frozen (arms + head only)"
	@echo "  make teleop-dry-run   print allocated run paths and both commands, run nothing"
	@echo "  make teleop-head-only T001-B head-only connectivity pilot"
	@echo "  make test-teleop      run the teleop test suite"
	@echo "  make teleop-hardware-prepare  preflight + copy only; never starts or arms robot"
	@echo "  make teleop-hardware  foreground R1 arms/head; prompts for fixture/E-stop confirmation"
	@echo ""
	@echo "Variables: HOST_IP DURATION_S PHYSICS_HZ CONTROL_HZ CERT_FILE KEY_FILE BODY_MODE TELEOP_ARGS"
	@echo "BODY_MODE: arms_head (torso frozen) | waist_yaw (default) | full_upper_body (+waist roll)"
	@echo "Example:   make teleop HOST_IP=10.42.0.5 BODY_MODE=arms_head"

## Run the coupled whole-upper-body teleop pilot end to end.
## Allocates the run id, starts the Quest bridge piped into Isaac Sim, and
## writes evidence under experiments/r1_teleop/quest3_sim_v1/T007/runs/.
teleop:
	$(TELEOP_CMD)

## Same pilot with the torso frozen: only the two arms and the head move.
teleop-arms:
	$(MAKE) teleop BODY_MODE=arms_head

teleop-dry-run:
	$(TELEOP_CMD) --dry-run

teleop-head-only:
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

teleop-hardware:
	ROBOT="$(ROBOT)" HOST_IP="$(HOST_IP)" DURATION_S="$(DURATION_S)" \
		CERT_FILE="$(CERT_FILE)" KEY_FILE="$(KEY_FILE)" \
		CONFIRM_SUSPENDED_WITH_ESTOP="$(CONFIRM_SUSPENDED_WITH_ESTOP)" \
		./scripts/teleop/run_r1_quest3_hardware.sh
