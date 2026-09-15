#!/usr/bin/env bash
# Export a source-only R1-A5 Quest teleop handoff. This never deploys or runs it.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

HIGH_LEVEL_REV="${HB_HIGH_LEVEL_REV:-bb70a20}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_ROOT="${1:-$ROOT/results/smoke}"
BUNDLE_NAME="${STAMP}_r1_quest3_hardware_bundle"
BUNDLE_DIR="$OUTPUT_ROOT/$BUNDLE_NAME"
ARCHIVE="$OUTPUT_ROOT/$BUNDLE_NAME.tar.gz"

if [[ -e "$BUNDLE_DIR" || -e "$ARCHIVE" ]]; then
    echo "[FAIL] Output already exists: $BUNDLE_NAME" >&2
    exit 2
fi
git cat-file -e "$HIGH_LEVEL_REV^{commit}"

mkdir -p "$BUNDLE_DIR"

# Current workstation-owned path, including the contextual documentation and
# relevant uncommitted research work needed by a new laptop/operator.
rsync -aR --exclude '__pycache__/' --exclude '*.pyc' \
    AGENTS.md \
    README.md \
    Makefile \
    assets/R1.urdf \
    assets/R1/R1.usd \
    config/README.md \
    config/cyclonedds_config.xml \
    config/netplan_static_ethernet.yaml \
    evidence/ \
    teleop/ \
    tests/__init__.py \
    tests/teleop/ \
    scripts/teleop/ \
    hardware/teleop/ \
    docs/README.md \
    docs/architecture/ \
    docs/hardware/ \
    docs/operations/ \
    docs/teleop/ \
    docs/safety/ \
    docs/templates/test_log_template.md \
    decisions/r1_teleop/ \
    "$BUNDLE_DIR/"

# Preserve every experimental definition and the compact T001--T006 evidence
# required by the regression suite. The 281 MB T007 bulk outputs remain in the
# full workspace; one small contract-complete T007 run keeps registry discovery
# verifiable on the receiving laptop.
rsync -aR --exclude 'runs/' --exclude 'figures/' \
    experiments/r1_teleop/quest3_sim_v1/ "$BUNDLE_DIR/"
rsync -aR \
    experiments/registry.json \
    experiments/r1_teleop/quest3_sim_v1/T001/runs/ \
    experiments/r1_teleop/quest3_sim_v1/T001/figures/ \
    experiments/r1_teleop/quest3_sim_v1/T002/runs/ \
    experiments/r1_teleop/quest3_sim_v1/T002/figures/ \
    experiments/r1_teleop/quest3_sim_v1/T003/runs/ \
    experiments/r1_teleop/quest3_sim_v1/T003/figures/ \
    experiments/r1_teleop/quest3_sim_v1/T004/runs/ \
    experiments/r1_teleop/quest3_sim_v1/T004/figures/ \
    experiments/r1_teleop/quest3_sim_v1/T005/runs/ \
    experiments/r1_teleop/quest3_sim_v1/T005/figures/ \
    experiments/r1_teleop/quest3_sim_v1/T006/runs/ \
    experiments/r1_teleop/quest3_sim_v1/T006/figures/ \
    experiments/r1_teleop/quest3_sim_v1/T007/runs/t007_whole_upper_body_20260820T130250Z/ \
    experiments/r1_teleop/quest3_sim_v1/T008/runs/ \
    "$BUNDLE_DIR/"

# Vendor transport source is copied read-only into the handoff; it is never
# changed in the workspace.
rsync -aR --exclude '__pycache__/' --exclude '*.pyc' \
    third_party/xr_teleoperate/teleop/televuer/src/ "$BUNDLE_DIR/"

# Small upstream R1 reference set used to audit model/joint conventions.
rsync -aR \
    third_party/xr_teleoperate_v1_6/README.md \
    third_party/xr_teleoperate_v1_6/CHANGELOG.md \
    third_party/xr_teleoperate_v1_6/assets/r1/r1_a5.urdf \
    third_party/xr_teleoperate_v1_6/teleop/robot_control/robot_arm.py \
    "$BUNDLE_DIR/"

# The sole-owner high-level implementation lives on develop, not in the current
# teleop branch. Export only build/runtime source and libraries needed by the
# minimal profile; omit dance/gesture assets and unrelated policies.
git archive --format=tar "$HIGH_LEVEL_REV" \
    hardware/high_level/CMakeLists.txt \
    hardware/high_level/README.md \
    hardware/high_level/policies/flat/policy_11_07.onnx \
    hardware/high_level/src \
    hardware/high_level/scripts \
    hardware/high_level/thirdparty/cnpy \
    hardware/high_level/thirdparty/onnxruntime \
    hardware/high_level/thirdparty/onnxruntime_aarch64 \
    | tar -xf - -C "$BUNDLE_DIR"

mkdir -p "$BUNDLE_DIR/hardware/high_level/config"
cp hardware/teleop/config/high_level_teleop_suspended.yaml \
   "$BUNDLE_DIR/hardware/high_level/config/tuning.yaml"

# Ensure the deploy copy of teleop/r1 is synchronized to the current workspace
# without changing hardware-only modules inside the bundle.
rsync -a --delete --exclude hardware/ --exclude '__pycache__/' --exclude '*.pyc' \
    "$BUNDLE_DIR/teleop/" "$BUNDLE_DIR/hardware/teleop/src/teleop/"

CURRENT_COMMIT="$(git rev-parse HEAD)"
WORKTREE_STATE="clean"
git diff --quiet -- . || WORKTREE_STATE="dirty"
UNTRACKED_COUNT="$(git ls-files --others --exclude-standard | wc -l | tr -d ' ')"
if [[ "$UNTRACKED_COUNT" != "0" ]]; then
    WORKTREE_STATE="dirty"
fi
printf '%s\n' \
    "workspace: $ROOT" \
    "commit:    $CURRENT_COMMIT" \
    "worktree:  $WORKTREE_STATE" \
    "synced_at: $STAMP" \
    "bundle:    $BUNDLE_NAME" \
    > "$BUNDLE_DIR/hardware/teleop/src/SOURCE.txt"
printf '%s\n' \
    "bundle: $BUNDLE_NAME" \
    "created_utc: $STAMP" \
    "workstation_commit: $CURRENT_COMMIT" \
    "workstation_worktree: $WORKTREE_STATE" \
    "workstation_untracked_files: $UNTRACKED_COUNT" \
    "high_level_revision: $(git rev-parse "$HIGH_LEVEL_REV^{commit}")" \
    "hardware_scope: suspended_fixed_arms_head_only" \
    "motor_authority: hardware/high_level sole rt/lowcmd publisher" \
    > "$BUNDLE_DIR/PROVENANCE.txt"

git diff --binary -- . > "$BUNDLE_DIR/WORKTREE.patch"
cp "$BUNDLE_DIR/hardware/teleop/LAPTOP_HANDOFF.md" \
   "$BUNDLE_DIR/START_HERE.md"

(
    cd "$BUNDLE_DIR"
    find . -type f ! -name SHA256SUMS -print0 \
        | sort -z \
        | xargs -0 sha256sum > SHA256SUMS
)

tar -czf "$ARCHIVE" -C "$OUTPUT_ROOT" "$BUNDLE_NAME"
echo "[OK] Bundle directory: $BUNDLE_DIR"
echo "[OK] Archive: $ARCHIVE"
echo "[SAFE] No robot connection, service action or motor command was performed."
