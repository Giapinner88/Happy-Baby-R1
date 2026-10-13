# Happy Baby R1 — bí danh `make` cho ./hb (lệnh chính; xem `./hb help`).
# Giữ để `make build`, `make deploy`… cũ vẫn chạy; mọi logic nằm trong ./hb.
#
# Robot:   make build && make preflight && make install
# Máy dev: make deploy | make deploy-policy | make deploy-dance

HB := ./hb
ARGS ?=

.DEFAULT_GOAL := help
.PHONY: help build preflight install teleop-runtime teleop-check test test-controller test-python \
        deploy deploy-policy deploy-dance diff rollback status \
        restart-all restart-high stop-all stop-high start-all start-high \
        dance-list dance-prune dance-restore

help:
	@$(HB) help

build:            ; $(HB) build $(ARGS)
preflight:        ; $(HB) preflight
install:          ; $(HB) install all
teleop-runtime:   ; $(HB) install teleop
teleop-check:     ; $(HB) teleop-check
test:             ; $(HB) test all
test-controller:  ; $(HB) test cpp
test-python:      ; $(HB) test py

deploy:           ; $(HB) deploy all $(ARGS)
deploy-policy:    ; $(HB) deploy policy $(ARGS)
deploy-dance:     ; $(HB) deploy dance $(ARGS)
diff:             ; $(HB) diff
rollback:         ; $(HB) rollback
status:           ; $(HB) status

restart-all:      ; $(HB) restart all
restart-high:     ; $(HB) restart high
stop-all:         ; $(HB) stop all
stop-high:        ; $(HB) stop high
start-all:        ; $(HB) start all
start-high:       ; $(HB) start high

dance-list:       ; $(HB) dance list
dance-prune:      ; $(HB) dance prune $(ARGS)
dance-restore:    ; $(HB) dance restore $(ARGS)
