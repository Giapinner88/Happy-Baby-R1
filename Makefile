# Happy Baby R1 — canonical Quest 3 teleop entry points.

SHELL := /bin/bash
PYTHON ?= python3

HOST_IP      ?=
HOST_IP_TAG   = $(subst .,_,$(strip $(HOST_IP)))
DURATION_S   ?= 180
PHYSICS_HZ   ?= 200
CONTROL_HZ   ?= 30
VIDEO_FPS    ?= 10
DEVICE       ?= cuda:0
CERT_FILE    ?= $(HOME)/.config/xr_teleoperate/happybaby_$(HOST_IP_TAG)/cert.pem
KEY_FILE     ?= $(HOME)/.config/xr_teleoperate/happybaby_$(HOST_IP_TAG)/key.pem
TELEOP_ARGS  ?= --single-view
ROBOT_CAMERA_WEBRTC_URL ?=
ROBOT_CAMERA_ZMQ_ENDPOINT ?=
HB_ROBOT_CAMERA ?= 1
ROBOT_CAMERA_LOCAL_PORT ?= 8765
ROBOT_CAMERA_REMOTE_PORT ?= 8765
HB_TELEOP_MIRROR_SIM ?= 1
ISAAC_SIM_PYTHON ?= $(HOME)/isaacsim_5.1/python.sh
ISAACLAB_ROOT ?= $(abspath ../Happy-Baby-R1-vla/third_party/IsaacLab)
ROBOT_CAMERA_ARGS = $(if $(strip $(ROBOT_CAMERA_WEBRTC_URL)),--robot-camera-webrtc-url "$(ROBOT_CAMERA_WEBRTC_URL)",)

.DEFAULT_GOAL := help
.PHONY: help check-network teleop teleop-dry-run teleop-hardware-prepare teleop-hardware test

help:
	@echo "Happy Baby R1 teleop:"
	@echo "  make teleop HOST_IP=<workstation-ip>          Quest -> vendor IK -> Isaac"
	@echo "    optional: ROBOT_CAMERA_WEBRTC_URL=https://<camera-host>:<port>/offer"
	@echo "              ROBOT_CAMERA_ZMQ_ENDPOINT=tcp://<camera-host>:<port> records MP4"
	@echo "  make teleop-dry-run HOST_IP=<workstation-ip>  validate and print the sim pipeline"
	@echo "  make teleop-hardware HOST_IP=<ip> ROBOT=<user@ip>  Quest -> vendor IK -> R1"
	@echo "    built-in R1 camera is default: starts in Quest passthrough; right A toggles full-field robot view"
	@echo "    stale robot video falls back to passthrough; originals: results/smoke/<run>/robot_camera_original"
	@echo "    camera follows the ROBOT SSH target; no separate camera IP/port is required"
	@echo "    opt out: HB_ROBOT_CAMERA=0 make teleop-hardware HOST_IP=<ip> ROBOT=<user@ip>"
	@echo "  make teleop-hardware-prepare ROBOT=<user@ip>  preflight and deploy; never arms motors"
	@echo "  make test                                     canonical teleop tests"

check-network:
	@test -n "$(strip $(HOST_IP))" || { \
		echo "[FAIL] HOST_IP is required, for example HOST_IP=192.168.1.19"; \
		exit 2; \
	}

teleop: check-network
	$(PYTHON) scripts/teleop/run_r1_baseline.py \
		--host-ip $(HOST_IP) --duration-s $(DURATION_S) \
		--physics-hz $(PHYSICS_HZ) --control-hz $(CONTROL_HZ) \
		--video-fps $(VIDEO_FPS) --device $(DEVICE) \
		--cert-file $(CERT_FILE) --key-file $(KEY_FILE) \
		$(ROBOT_CAMERA_ARGS) $(TELEOP_ARGS)

teleop-dry-run: check-network
	$(MAKE) teleop HOST_IP="$(HOST_IP)" DURATION_S="$(DURATION_S)" \
		PHYSICS_HZ="$(PHYSICS_HZ)" CONTROL_HZ="$(CONTROL_HZ)" \
		VIDEO_FPS="$(VIDEO_FPS)" DEVICE="$(DEVICE)" \
		CERT_FILE="$(CERT_FILE)" KEY_FILE="$(KEY_FILE)" \
		ROBOT_CAMERA_WEBRTC_URL="$(ROBOT_CAMERA_WEBRTC_URL)" \
		ROBOT_CAMERA_ZMQ_ENDPOINT="$(ROBOT_CAMERA_ZMQ_ENDPOINT)" \
		HB_ROBOT_CAMERA="$(HB_ROBOT_CAMERA)" \
		ROBOT_CAMERA_LOCAL_PORT="$(ROBOT_CAMERA_LOCAL_PORT)" \
		ROBOT_CAMERA_REMOTE_PORT="$(ROBOT_CAMERA_REMOTE_PORT)" \
		HB_TELEOP_MIRROR_SIM="$(HB_TELEOP_MIRROR_SIM)" \
		TELEOP_ARGS="$(TELEOP_ARGS) --dry-run"

teleop-hardware-prepare:
	ROBOT="$(ROBOT)" ./hardware/teleop/scripts/deploy_teleop.sh deploy

teleop-hardware: check-network
	ROBOT="$(ROBOT)" HOST_IP="$(HOST_IP)" DURATION_S="$(DURATION_S)" \
		CERT_FILE="$(CERT_FILE)" KEY_FILE="$(KEY_FILE)" \
		ROBOT_CAMERA_WEBRTC_URL="$(ROBOT_CAMERA_WEBRTC_URL)" \
		ROBOT_CAMERA_ZMQ_ENDPOINT="$(ROBOT_CAMERA_ZMQ_ENDPOINT)" \
		HB_ROBOT_CAMERA="$(HB_ROBOT_CAMERA)" \
		ROBOT_CAMERA_LOCAL_PORT="$(ROBOT_CAMERA_LOCAL_PORT)" \
		ROBOT_CAMERA_REMOTE_PORT="$(ROBOT_CAMERA_REMOTE_PORT)" \
		HB_TELEOP_MIRROR_SIM="$(HB_TELEOP_MIRROR_SIM)" \
		ISAAC_SIM_PYTHON="$(ISAAC_SIM_PYTHON)" \
		ISAACLAB_ROOT="$(ISAACLAB_ROOT)" \
		CONFIRM_SUSPENDED_WITH_ESTOP="$(CONFIRM_SUSPENDED_WITH_ESTOP)" \
		./scripts/teleop/run_r1_quest3_hardware.sh

test:
	$(PYTHON) -m pytest tests/teleop hardware/teleop/tests -q
