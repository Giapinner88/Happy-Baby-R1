#!/bin/bash
set -euo pipefail

MODULE_DIR=/unitree/module/dex3_service_pc4
OTA_CLI=/unitree/sbin/ota_pipe_cli
STATE_FILE="$MODULE_DIR/.dex3_usb_compat_backup"

if [[ $(id -u) -ne 0 ]]; then
  echo "ERROR: run this uninstaller with sudo." >&2
  exit 1
fi
if [[ ! -f "$STATE_FILE" ]]; then
  echo "No active Dex3 USB compatibility installation found."
  exit 0
fi

BACKUP_DIR=$(cat "$STATE_FILE")
if [[ ! -f "$BACKUP_DIR/dex3_start_l.sh" || \
      ! -f "$BACKUP_DIR/dex3_start_r.sh" ]]; then
  echo "ERROR: backup is incomplete: $BACKUP_DIR" >&2
  exit 1
fi

"$OTA_CLI" -a default -c stopservice -n dex3_service_l -T 5000000 || true
"$OTA_CLI" -a default -c stopservice -n dex3_service_r -T 5000000 || true
sleep 2

cp -a "$BACKUP_DIR/dex3_start_l.sh" "$MODULE_DIR/dex3_start_l.sh"
cp -a "$BACKUP_DIR/dex3_start_r.sh" "$MODULE_DIR/dex3_start_r.sh"
rm -f "$MODULE_DIR/libdex3_claim_interface1.so" "$STATE_FILE"

"$OTA_CLI" -a default -c startservice -n dex3_service_l -T 5000000
"$OTA_CLI" -a default -c startservice -n dex3_service_r -T 5000000

echo "Dex3 USB compatibility layer removed; original scripts restored."
echo "Backup retained at $BACKUP_DIR"
