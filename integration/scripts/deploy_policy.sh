#!/usr/bin/env bash
# Fast policy-only deploy for controller.
#
# This deliberately does not build or mirror the whole HB stack. It reads the
# effective flat_model/flat_policy_contract, runs the already-built local
# run_r1 preflight, syncs the selected ONNX + locomotion config + manifest, and
# restarts hb_high_level only after DISARMED has been confirmed on the robot.
# Policy selection remains explicit in config/tuning.yaml; this script does not
# infer a compatible config from a newly copied policy.
set -euo pipefail

HB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HIGH_DIR="$HB_ROOT/controller"
DEST="${DEST:-/home/unitree/HB}"
HIGH_BIN="${HIGH_BIN:-$HIGH_DIR/build/run_r1}"
DRY_RUN=0
WITH_DANCE=0
KEEP_PREFLIGHT_LOG=0
source "$HB_ROOT/integration/scripts/status_log.sh"
source "$HB_ROOT/integration/scripts/dance_preflight_summary.sh"

usage() {
    cat <<'USAGE'
Usage:
  make deploy-policy                                    locomotion policy + restart high-level
  make deploy-policy ARGS=--with-dance                  locomotion + dance assets + one restart
  make deploy-policy ARGS=--dry-run                     preview locomotion sync, no writes/restart
  make deploy-policy ARGS="--with-dance --dry-run"      preview locomotion + dance sync

Optional environment:
  ROBOT=unitree@<ip-or-hostname>  override robot discovery
  DEST=/home/unitree/HB           remote HB directory
  HIGH_BIN=/path/to/run_r1        existing local preflight binary
USAGE
}

for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=1 ;;
        --with-dance) WITH_DANCE=1 ;;
        --help|-h) usage; exit 0 ;;
        *) usage >&2; exit 2 ;;
    esac
done

fail() {
    hb_fail "deploy_policy: $*"
    exit 1
}

[[ -x "$HIGH_BIN" ]] || fail "missing local preflight binary: $HIGH_BIN (use make deploy for a full deploy/build)"
[[ -r "$HIGH_DIR/config/tuning.yaml" ]] || fail "missing config: $HIGH_DIR/config/tuning.yaml"

# shellcheck disable=SC1091
source "$HB_ROOT/integration/scripts/config_reader.sh"
SELECTED="$(hb_config_get "$HIGH_DIR/config/tuning.yaml" flat_model)"
SELECTED_CONTRACT="$(hb_config_get "$HIGH_DIR/config/tuning.yaml" flat_policy_contract)"
[[ -n "$SELECTED" && "$SELECTED" != */* && "$SELECTED" != "." && "$SELECTED" != ".." ]] || \
    fail "flat_model must name a file: '$SELECTED'"
[[ -n "$SELECTED_CONTRACT" ]] || fail "flat_policy_contract is empty"

case "$SELECTED_CONTRACT" in
    legacy_83)
        EXPECTED_PREFIX="policies/flat/"
        ;;
    flat_plus_h4_v1|flat_plus_h5_v1|flat_plus_gait_h4_v1)
        EXPECTED_PREFIX="policies/locomotion/flat_plus/"
        ;;
    *)
        fail "unsupported flat_policy_contract '$SELECTED_CONTRACT'"
        ;;
esac

ORT_LIB="$HIGH_DIR/thirdparty/onnxruntime/lib"
if [[ "$(uname -m)" == "aarch64" || "$(uname -m)" == "arm64" ]]; then
    ORT_LIB="$HIGH_DIR/thirdparty/onnxruntime_aarch64/lib"
fi
export LD_LIBRARY_PATH="$ORT_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

PREFLIGHT_LOG="$(mktemp /tmp/hb-policy-preflight.XXXXXX.log)"
MANIFEST_TMP=""
cleanup() {
    (( KEEP_PREFLIGHT_LOG )) || rm -f -- "$PREFLIGHT_LOG"
    [[ -z "$MANIFEST_TMP" ]] || rm -f -- "$MANIFEST_TMP"
}
trap cleanup EXIT

if ! HB_PROJECT_DIR="$HIGH_DIR" "$HIGH_BIN" --preflight >"$PREFLIGHT_LOG" 2>&1; then
    KEEP_PREFLIGHT_LOG=1
    fail "local preflight failed; details: $PREFLIGHT_LOG (nothing deployed)"
fi

MODEL_LINE="$(grep -m1 '^\[Preflight\] MODEL ' "$PREFLIGHT_LOG" || true)"
[[ -n "$MODEL_LINE" ]] || fail "preflight did not print [Preflight] MODEL"
MODEL_REL="$(sed -n 's/.* rel=\([^ ]*\).*/\1/p' <<<"$MODEL_LINE")"
RUNTIME_CONTRACT="$(sed -n 's/.* contract=\([^ ]*\).*/\1/p' <<<"$MODEL_LINE")"
MODEL_INPUT="$(sed -n 's/.* input=\([^ ]*\).*/\1/p' <<<"$MODEL_LINE")"
MODEL_OUTPUT="$(sed -n 's/.* output=\([^ ]*\).*/\1/p' <<<"$MODEL_LINE")"

[[ "$RUNTIME_CONTRACT" == "$SELECTED_CONTRACT" ]] || \
    fail "runtime contract=$RUNTIME_CONTRACT but config selects $SELECTED_CONTRACT"
[[ "$MODEL_REL" == "$EXPECTED_PREFIX$SELECTED" ]] || \
    fail "runtime loads '$MODEL_REL', expected '${EXPECTED_PREFIX}${SELECTED}'"

MODEL="$HIGH_DIR/$MODEL_REL"
[[ -s "$MODEL" ]] || fail "selected model is missing or empty: $MODEL"
MODEL_SHA256="$(sha256sum "$MODEL" | awk '{print $1}')"

DANCE_CONFIG="$HIGH_DIR/config/dance.yaml"
DANCE_DIR="$HIGH_DIR/policies/dance"
DANCE_CONFIG_SHA256=""
if (( WITH_DANCE )); then
    [[ -s "$DANCE_CONFIG" ]] || fail "missing dance config: $DANCE_CONFIG"
    [[ -d "$DANCE_DIR" ]] || fail "missing dance policy directory: $DANCE_DIR"
    DANCE_CONFIG_SHA256="$(sha256sum "$DANCE_CONFIG" | awk '{print $1}')"
fi

MANIFEST="$HB_ROOT/integration/config/model_manifest.conf"
MANIFEST_TMP="$(mktemp /tmp/hb-policy-manifest.XXXXXX)"
cat >"$MANIFEST_TMP" <<EOF
# Walking policy accepted by the current controller runner.
MODEL_REL=$MODEL_REL
MODEL_SHA256=$MODEL_SHA256
MODEL_CONTRACT=$SELECTED_CONTRACT
MODEL_INPUT=$MODEL_INPUT
MODEL_OUTPUT=$MODEL_OUTPUT
EOF

hb_ok "locomotion policy: $SELECTED ($SELECTED_CONTRACT)"
if (( WITH_DANCE )); then
    hb_summarize_dance_preflight "$DANCE_CONFIG" "$PREFLIGHT_LOG"
fi

# Dò robot sau preflight để không mất thời gian nếu artifact local đã hỏng.
# shellcheck disable=SC1091
source "$HIGH_DIR/scripts/_find_robot.sh"
find_robot
hb_info "Deploying to $ROBOT"

if [[ -n "${HB_SSH_CONTROL_PATH:-}" ]]; then
    SSH_CONTROL_PATH="$HB_SSH_CONTROL_PATH"
    SSH=(ssh -o ControlMaster=auto -o ControlPersist=300 -o "ControlPath=$SSH_CONTROL_PATH")
    RSYNC_SSH="ssh -o ControlMaster=auto -o ControlPersist=300 -o ControlPath=$SSH_CONTROL_PATH"
else
    # Avoid stale ControlMaster sockets causing "mux_client... Broken pipe".
    # The policy-only transfer is still fast without multiplexing.
    SSH=(ssh -o ControlMaster=no -o ControlPath=none)
    RSYNC_SSH="ssh -o ControlMaster=no -o ControlPath=none"
fi
RSYNC_BASE=(-az --no-perms --omit-dir-times -e "$RSYNC_SSH")
REMOTE_MODEL="$DEST/controller/$MODEL_REL"
REMOTE_CONFIG="$DEST/controller/config/locomotion.yaml"
REMOTE_MANIFEST="$DEST/integration/config/model_manifest.conf"
REMOTE_DANCE_CONFIG="$DEST/controller/config/dance.yaml"
REMOTE_DANCE_DIR="$DEST/controller/policies/dance/"

DANCE_RSYNC_EXCLUDES=(
    --exclude 'backup/'
    --exclude 'artifacts/'
    --exclude '__pycache__/'
    --exclude '*.pyc'
    --exclude '*.pt'
    --exclude '*.pth'
    --exclude '*.onnx.*'
    --exclude '.cache/'
    --exclude 'logs/'
)

if (( DRY_RUN )); then
    rsync "${RSYNC_BASE[@]}" --dry-run "$MODEL" "$ROBOT:$REMOTE_MODEL"
    rsync "${RSYNC_BASE[@]}" --dry-run "$HIGH_DIR/config/locomotion.yaml" "$ROBOT:$REMOTE_CONFIG"
    rsync "${RSYNC_BASE[@]}" --dry-run "$MANIFEST_TMP" "$ROBOT:$REMOTE_MANIFEST"
    if (( WITH_DANCE )); then
        rsync "${RSYNC_BASE[@]}" --dry-run "$DANCE_CONFIG" "$ROBOT:$REMOTE_DANCE_CONFIG"
        rsync "${RSYNC_BASE[@]}" --dry-run --delete "${DANCE_RSYNC_EXCLUDES[@]}" \
            "$DANCE_DIR/" "$ROBOT:$REMOTE_DANCE_DIR"
    fi
    hb_ok "DRY-RUN complete; remote files and service unchanged"
    exit 0
fi

"${SSH[@]}" "$ROBOT" \
    "mkdir -p '$DEST/controller/$(dirname "$MODEL_REL")' '$DEST/controller/config' '$DEST/integration/config' '$DEST/controller/policies/dance'"

rsync "${RSYNC_BASE[@]}" "$MODEL" "$ROBOT:$REMOTE_MODEL"
rsync "${RSYNC_BASE[@]}" "$HIGH_DIR/config/locomotion.yaml" "$ROBOT:$REMOTE_CONFIG"
rsync "${RSYNC_BASE[@]}" "$MANIFEST_TMP" "$ROBOT:$REMOTE_MANIFEST"
if (( WITH_DANCE )); then
    rsync "${RSYNC_BASE[@]}" "$DANCE_CONFIG" "$ROBOT:$REMOTE_DANCE_CONFIG"
    rsync "${RSYNC_BASE[@]}" --delete "${DANCE_RSYNC_EXCLUDES[@]}" \
        "$DANCE_DIR/" "$ROBOT:$REMOTE_DANCE_DIR"
fi
hb_ok "files synchronized"

# Never restart high-level while armed. The guard is evaluated on the robot,
# immediately before restart; if it fails, files remain staged but motors are
# not interrupted.
"${SSH[@]}" -t "$ROBOT" '
set -e
STATUS=/run/hb/status.env
grep -qx "high_state=DISARMED" "$STATUS"
grep -qx "high_armed=0" "$STATUS"
sudo systemctl restart hb_high_level.service
sleep 2
systemctl is-active --quiet hb_high_level.service
'

REMOTE_SHA="$("${SSH[@]}" "$ROBOT" "sha256sum '$REMOTE_MODEL' | awk '{print \$1}'")"
[[ "$REMOTE_SHA" == "$MODEL_SHA256" ]] || \
    fail "remote SHA mismatch: local=$MODEL_SHA256 remote=$REMOTE_SHA"
if (( WITH_DANCE )); then
    REMOTE_DANCE_CONFIG_SHA256="$("${SSH[@]}" "$ROBOT" "sha256sum '$REMOTE_DANCE_CONFIG' | awk '{print \$1}'")"
    [[ "$REMOTE_DANCE_CONFIG_SHA256" == "$DANCE_CONFIG_SHA256" ]] || \
        fail "remote dance.yaml SHA mismatch: local=$DANCE_CONFIG_SHA256 remote=$REMOTE_DANCE_CONFIG_SHA256"
fi

if ! cmp -s "$MANIFEST_TMP" "$MANIFEST"; then
    mv -- "$MANIFEST_TMP" "$MANIFEST"
else
    rm -f -- "$MANIFEST_TMP"
fi

"${SSH[@]}" "$ROBOT" "systemctl is-active --quiet hb_high_level.service"
hb_ok "high-level service active; policy checksum verified"
if (( WITH_DANCE )); then
    hb_ok "dance config checksum verified; assets load on next high-level start"
fi
hb_ok "DEPLOY_OK"
