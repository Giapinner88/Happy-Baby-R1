#!/bin/bash
set -euo pipefail

USB_DEVICE=/sys/bus/usb/devices/1-1.9
EXPECTED_USB_ID=1a86:80b5
OTA_CLI=/unitree/sbin/ota_pipe_cli
PROBE=/home/unitree/Foundation_Unitree/build/r1_gateway/r1_cpp_dex3_state_probe
RESULT_DIR=/home/unitree/dex3_usb_compat/results
STAMP=$(date +%Y%m%d_%H%M%S)
RESULT_LOG="$RESULT_DIR/disable_port9_test_$STAMP.log"
KEEP_MARKER=/home/unitree/dex3_usb_compat/port9_disabled_for_dex3
PORT_DISABLED=0
KEEP_DISABLED=0

if [[ $(id -u) -ne 0 ]]; then
  echo "ERROR: run with sudo." >&2
  exit 1
fi

mkdir -p "$RESULT_DIR"
exec > >(tee -a "$RESULT_LOG") 2>&1

restart_dex3() {
  "$OTA_CLI" -a default -c stopservice -n dex3_service_l -T 5000000 || true
  "$OTA_CLI" -a default -c stopservice -n dex3_service_r -T 5000000 || true
  sleep 2
  "$OTA_CLI" -a default -c startservice -n dex3_service_l -T 5000000
  "$OTA_CLI" -a default -c startservice -n dex3_service_r -T 5000000
  sleep 18
}

restore_on_exit() {
  if [[ $PORT_DISABLED -eq 1 && $KEEP_DISABLED -eq 0 && \
        -e "$USB_DEVICE/authorized" ]]; then
    echo "Restoring USB port 1-1.9."
    printf '1' >"$USB_DEVICE/authorized" || true
    restart_dex3 || true
  fi
}
trap restore_on_exit EXIT

if [[ ! -e "$USB_DEVICE/authorized" ]]; then
  echo "ERROR: $USB_DEVICE is not present."
  exit 1
fi

CURRENT_USB_ID="$(tr -d '\n' <"$USB_DEVICE/idVendor"):$(tr -d '\n' <"$USB_DEVICE/idProduct")"
if [[ "$CURRENT_USB_ID" != "$EXPECTED_USB_ID" ]]; then
  echo "ERROR: refusing to disable unexpected device $CURRENT_USB_ID."
  exit 1
fi

if findmnt -rn -S /dev/sda | grep -q .; then
  echo "ERROR: /dev/sda is mounted; refusing to disable USB port."
  exit 1
fi

echo "Disabling unmounted USB card reader $CURRENT_USB_ID at 1-1.9."
printf '0' >"$USB_DEVICE/authorized"
PORT_DISABLED=1
sleep 5

restart_dex3

set +e
"$PROBE" --interface eth10 --side left \
  --topic rt/lf/dex3/left/state --duration 5
LEFT_RESULT=$?
"$PROBE" --interface eth10 --side right \
  --topic rt/lf/dex3/right/state --duration 5
RIGHT_RESULT=$?
set -e

echo "Probe results with port 1-1.9 disabled: left=$LEFT_RESULT right=$RIGHT_RESULT"
if [[ $LEFT_RESULT -ne 0 || $RIGHT_RESULT -ne 0 ]]; then
  echo "FAIL: disabling port 1-1.9 did not restore both Dex3 state streams."
  exit 2
fi

KEEP_DISABLED=1
printf '%s\n' "$STAMP" >"$KEEP_MARKER"
echo "PASS: both hands publish HandState; port 1-1.9 remains disabled."
echo "It will be re-enabled automatically on reboot, or manually with:"
echo "  echo 1 | sudo tee $USB_DEVICE/authorized"
echo "Evidence: $RESULT_LOG"
