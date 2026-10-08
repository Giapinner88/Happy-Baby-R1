# Happy Baby R1 — build and operate the robot stack. The repo root is HB_ROOT.
#
# On the robot (Ubuntu 20.04, aarch64):
#   git clone <repo> ~/HB && cd ~/HB
#   make build && make preflight && make install && make status
# From the dev machine (robot auto-discovered, or ROBOT=unitree@<ip>):
#   make deploy | make deploy-policy
#
# Nothing here arms motors. Arming is the R3 handover in docs/controls.md.

SHELL := /bin/bash
HB_ROOT := $(CURDIR)
OPS := $(HB_ROOT)/integration/scripts
UV ?= $(HOME)/.local/bin/uv
ARGS ?=
export HB_ROOT

STACK_CTL := status stop-all stop-high stop-integration stop-voice stop-presets restart-all \
             start-high start-integration start-voice start-presets start-all

.DEFAULT_GOAL := help
.PHONY: help build build-controller build-voice build-integration preflight install \
        teleop-runtime teleop-check test test-controller test-python deploy deploy-policy rollback \
        $(STACK_CTL)

help:
	@echo "On the robot:"
	@echo "  make build              controller + voice bridge + integration + voice env"
	@echo "                          (parts: build-controller | build-voice | build-integration)"
	@echo "  make preflight          model/config/asset checks, no motor output"
	@echo "  make install            install/refresh services (sudo); controller restarts only if DISARMED"
	@echo "  make teleop-runtime     install the robot-local Quest/IK runtime (sudo)"
	@echo "  make teleop-check       read-only rt/lowstate check, creates no publisher"
	@echo "  make status | stop-all | stop-high | restart-all | start-all | start-high | ..."
	@echo "  make test               all unit/contract tests"
	@echo "From the dev machine:"
	@echo "  make deploy             backup + sync + ARM64 build + preflight + safe restart"
	@echo "  make deploy-policy      policy/locomotion config only (ARGS=--with-dance|--dry-run)"
	@echo "  make rollback           restore the previous deploy backup on the robot"
	@echo "Operator reference: docs/controls.md"

build:
	bash $(OPS)/build_on_robot.sh all
build-controller:
	bash $(OPS)/build_on_robot.sh high
build-voice:
	bash $(OPS)/build_on_robot.sh voice
build-integration:
	bash $(OPS)/build_on_robot.sh integration

preflight:
	bash $(OPS)/preflight.sh all

install:
	sudo HB_ROOT=$(HB_ROOT) bash $(OPS)/activate_services.sh

teleop-runtime:
	bash teleop/scripts/bootstrap_robot_runtime.sh
	sudo HB_ROOT=$(HB_ROOT) bash teleop/scripts/install_robot_runtime.sh --start

teleop-check:
	PYTHONPATH=teleop/src python3 -m teleop.hardware.run_teleop \
		--interface $${UNITREE_NETWORK_INTERFACE:-eth10} --timeout-s 5

$(STACK_CTL):
	bash $(OPS)/stack_ctl.sh $@

test: test-controller test-python

test-controller:
	cmake -S controller -B controller/build -DCMAKE_BUILD_TYPE=Release
	cmake --build controller/build --parallel $$(nproc)
	ctest --test-dir controller/build --output-on-failure

test-python:
	cd teleop && PYTHONPATH=src python3 -m pytest tests -q
	python3 -m pytest voice_presets/tests -q
	PYTHONPATH=voice $(UV) run --frozen --project voice --with pytest \
		python -m pytest voice/tests integration/tests -q

deploy:
	bash $(OPS)/deploy.sh
deploy-policy:
	bash $(OPS)/deploy_policy.sh $(ARGS)
rollback:
	bash $(OPS)/deploy_stack.sh rollback
