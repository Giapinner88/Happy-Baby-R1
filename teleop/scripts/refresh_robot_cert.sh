#!/usr/bin/env bash
# Generate a robot-owned HTTPS certificate for its current Wi-Fi address.
set -euo pipefail

[[ "${EUID}" -eq 0 ]] || { echo "Run with sudo" >&2; exit 1; }
HOST_INTERFACE="${HB_TELEOP_RUNTIME_HOST_INTERFACE:-wlan0}"
CERT_FILE="${HB_TELEOP_RUNTIME_CERT_FILE:-/etc/hb/teleop/cert.pem}"
KEY_FILE="${HB_TELEOP_RUNTIME_KEY_FILE:-/etc/hb/teleop/key.pem}"
HOSTNAME_ALIAS="${HB_TELEOP_RUNTIME_HOSTNAME:-hb-r1.local}"
RUN_USER="${HB_RUN_USER:-unitree}"
CERT_DIR="$(dirname "$CERT_FILE")"
IP="$(ip -4 -o addr show dev "$HOST_INTERFACE" scope global | awk 'NR == 1 {split($4,a,"/"); print a[1]}')"
[[ -n "$IP" ]] || { echo "[CERT] no IPv4 on $HOST_INTERFACE" >&2; exit 1; }
install -d -m 0755 "$CERT_DIR"

if [[ -r "$CERT_FILE" && -r "$KEY_FILE" ]] && \
   openssl x509 -in "$CERT_FILE" -noout -ext subjectAltName 2>/dev/null \
   | grep -Eq "IP Address:$IP" && \
   openssl x509 -in "$CERT_FILE" -noout -ext subjectAltName 2>/dev/null \
   | grep -Eq "DNS:$HOSTNAME_ALIAS"; then
    chown "$RUN_USER:$RUN_USER" "$KEY_FILE" "$CERT_FILE"
    chmod 600 "$KEY_FILE"
    echo "[CERT] existing certificate covers $IP/$HOSTNAME_ALIAS"
    exit 0
fi

TMP_CONF="$(mktemp)"
trap 'rm -f "$TMP_CONF"' EXIT
cat >"$TMP_CONF" <<EOF
[req]
distinguished_name=req_dn
x509_extensions=v3_req
prompt=no
[req_dn]
CN=$HOSTNAME_ALIAS
[v3_req]
subjectAltName=DNS:$HOSTNAME_ALIAS,IP:$IP
keyUsage=digitalSignature,keyEncipherment
extendedKeyUsage=serverAuth
EOF
umask 077
openssl req -x509 -nodes -newkey rsa:2048 -sha256 -days 825 \
    -keyout "$KEY_FILE" -out "$CERT_FILE" -config "$TMP_CONF" >/dev/null 2>&1
chown "$RUN_USER:$RUN_USER" "$KEY_FILE" "$CERT_FILE"
chmod 600 "$KEY_FILE"
echo "[CERT] generated SAN DNS:$HOSTNAME_ALIAS,IP:$IP"
