#!/bin/bash
set -euo pipefail

MODULE_DIR=/unitree/module/dex3_service_pc4
SERVICE_BIN="$MODULE_DIR/g1_dex_protocol_v2.0"
SOURCE_DIR=/home/unitree/dex3_usb_compat
SOURCE_LIB="$SOURCE_DIR/libdex3_claim_interface1.so"
INSTALLED_LIB="$MODULE_DIR/libdex3_claim_interface1.so"
START_LEFT="$MODULE_DIR/dex3_start_l.sh"
START_RIGHT="$MODULE_DIR/dex3_start_r.sh"
OTA_CLI=/unitree/sbin/ota_pipe_cli
PROBE=/home/unitree/Foundation_Unitree/build/r1_gateway/r1_cpp_dex3_state_probe
EXPECTED_SERVICE_SHA=b5b62f1aa45b76ea53b2b8dc8c86afc93cf06f41e93934600ae4f0dfb7adf50f
STAMP=$(date +%Y%m%d_%H%M%S)
BACKUP_DIR="$MODULE_DIR/dex3_usb_compat_backup_$STAMP"
RESULT_DIR="$SOURCE_DIR/results"
RESULT_LOG="$RESULT_DIR/install_test_$STAMP.log"

if [[ $(id -u) -ne 0 ]]; then
  echo "ERROR: run this installer with sudo." >&2
  exit 1
fi

mkdir -p "$RESULT_DIR"
exec > >(tee -a "$RESULT_LOG") 2>&1

echo "Dex3 USB compatibility install/test: $STAMP"

if [[ ! -x "$SERVICE_BIN" || ! -f "$START_LEFT" || ! -f "$START_RIGHT" ]]; then
  echo "ERROR: expected Dex3 service installation is incomplete."
  exit 1
fi
if [[ ! -f "$SOURCE_LIB" ]]; then
  echo "ERROR: missing $SOURCE_LIB; run build_on_robot.sh first."
  exit 1
fi

CURRENT_SERVICE_SHA=$(sha256sum "$SERVICE_BIN" | awk '{print $1}')
if [[ "$CURRENT_SERVICE_SHA" != "$EXPECTED_SERVICE_SHA" ]]; then
  echo "ERROR: service SHA changed: $CURRENT_SERVICE_SHA"
  echo "Expected: $EXPECTED_SERVICE_SHA"
  exit 1
fi
if ! file "$SOURCE_LIB" | grep -q 'ARM aarch64'; then
  echo "ERROR: compatibility library is not an AArch64 ELF shared object."
  exit 1
fi

mkdir -p "$BACKUP_DIR"
cp -a "$START_LEFT" "$START_RIGHT" "$BACKUP_DIR/"
sha256sum "$SERVICE_BIN" "$START_LEFT" "$START_RIGHT" "$SOURCE_LIB" \
  >"$BACKUP_DIR/SHA256SUMS.before"

install -o root -g root -m 0755 "$SOURCE_LIB" "$INSTALLED_LIB"
for script in "$START_LEFT" "$START_RIGHT"; do
  if ! grep -qF "$INSTALLED_LIB" "$script"; then
    sed -i "/^exec \/unitree\/module\/dex3_service_pc4\/g1_dex_protocol_v2\.0/i export LD_PRELOAD=$INSTALLED_LIB\${LD_PRELOAD:+:\$LD_PRELOAD}" "$script"
  fi
done

echo "$BACKUP_DIR" >"$MODULE_DIR/.dex3_usb_compat_backup"
chmod 0600 "$MODULE_DIR/.dex3_usb_compat_backup"

restart_services() {
  "$OTA_CLI" -a default -c stopservice -n dex3_service_l -T 5000000 || true
  "$OTA_CLI" -a default -c stopservice -n dex3_service_r -T 5000000 || true
  sleep 2
  "$OTA_CLI" -a default -c startservice -n dex3_service_l -T 5000000
  "$OTA_CLI" -a default -c startservice -n dex3_service_r -T 5000000
  sleep 18
}

rollback() {
  echo "Test failed; restoring original startup scripts."
  "$OTA_CLI" -a default -c stopservice -n dex3_service_l -T 5000000 || true
  "$OTA_CLI" -a default -c stopservice -n dex3_service_r -T 5000000 || true
  cp -a "$BACKUP_DIR/dex3_start_l.sh" "$START_LEFT"
  cp -a "$BACKUP_DIR/dex3_start_r.sh" "$START_RIGHT"
  rm -f "$INSTALLED_LIB" "$MODULE_DIR/.dex3_usb_compat_backup"
  "$OTA_CLI" -a default -c startservice -n dex3_service_l -T 5000000 || true
  "$OTA_CLI" -a default -c startservice -n dex3_service_r -T 5000000 || true
  echo "Rollback complete. Evidence remains in $RESULT_LOG"
}

restart_services

echo "Running read-only HandState probes."
set +e
"$PROBE" --interface eth10 --side left \
  --topic rt/lf/dex3/left/state --duration 5
LEFT_RESULT=$?
"$PROBE" --interface eth10 --side right \
  --topic rt/lf/dex3/right/state --duration 5
RIGHT_RESULT=$?
set -e

echo "Probe results: left=$LEFT_RESULT right=$RIGHT_RESULT"
pgrep -af 'g1_dex_protocol_v2.0' || true
journalctl -k --since "5 minutes ago" --no-pager | \
  grep -E 'Dex3 Thread|usb 1-1\.(1|2)' | tail -n 80 || true

if [[ $LEFT_RESULT -ne 0 || $RIGHT_RESULT -ne 0 ]]; then
  rollback
  exit 2
fi

echo "PASS: both hands publish valid HandState. Compatibility layer retained."
echo "Backup: $BACKUP_DIR"
echo "Evidence: $RESULT_LOG"
