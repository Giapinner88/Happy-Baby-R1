#!/usr/bin/env python3
"""Static contracts for the canonical local build and deploy entry points."""

from pathlib import Path
import unittest


HB_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = HB_ROOT / "integration" / "scripts"


class BuildDeployPathTests(unittest.TestCase):
    def test_build_script_builds_canonical_tree_with_configurable_parallelism(self):
        hb = (HB_ROOT / "hb").read_text()
        build = hb.split("build_dev_controller() {", 1)[1].split("\n}\n", 1)[0]
        self.assertIn('cmake -S "$HIGH_DIR" -B "$HIGH_DIR/build" -DCMAKE_BUILD_TYPE=Release', build)
        self.assertIn('--parallel "${HB_BUILD_JOBS:-$(nproc)}"', build)
        self.assertNotIn("--target run_r1", build)

    def test_hb_routes_through_the_guarded_scripts(self):
        hb = (HB_ROOT / "hb").read_text()
        # Service control always goes through stack_ctl.sh (DISARMED guard),
        # locally on the robot or over SSH via deploy_stack.sh.
        self.assertIn('exec bash "$OPS/stack_ctl.sh" "$1"', hb)
        self.assertIn('exec "$OPS/deploy_stack.sh" "$1"', hb)
        self.assertIn('exec "$OPS/deploy_stack.sh" deploy "${DEPLOY_ARGS[@]}"', hb)
        self.assertNotIn("deploy_policy.sh", hb)
        self.assertNotIn("systemctl", hb)

    def test_old_deploy_targets_are_aliases_of_the_new_components(self):
        hb = (HB_ROOT / "hb").read_text()
        aliases = hb.split("deploy_args() {", 1)[1].split("\n}\n", 1)[0]
        self.assertIn("policy|dance|high)", aliases)
        self.assertIn("target=controller", aliases)
        self.assertIn("presets|integration)", aliases)
        self.assertIn("target=voice", aliases)

    def test_local_preflight_uses_canonical_incremental_build(self):
        deploy = (SCRIPTS / "deploy_stack.sh").read_text()
        check = deploy.split("local_controller_check() {", 1)[1].split("\n}\n", 1)[0]
        self.assertIn('cmake --build "$HIGH_DIR/build" --target run_r1', check)
        self.assertIn('"$HIGH_DIR/build/run_r1" --preflight', check)
        self.assertNotIn("mktemp -d", check)
        manifest = (SCRIPTS / "update_model_manifest.sh").read_text()
        self.assertNotIn("hb-model-check", manifest)

    def test_dance_sync_excludes_non_runtime_payload(self):
        deploy = (SCRIPTS / "deploy_stack.sh").read_text()
        sync = deploy.split("sync_dance() {", 1)[1].split("\n}\n", 1)[0]
        self.assertIn("--delete", sync)
        for excluded in ("backup/", "artifacts/", "__pycache__/", "*.pyc", "*.pt", "*.pth", "*.onnx.*", "logs/"):
            self.assertIn(excluded, sync + deploy.split("COMMON_EXCLUDES=", 1)[1].split("\n", 1)[0]
                          + deploy.split("TRAINING_EXCLUDES=", 1)[1].split("\n", 1)[0])

    def test_backup_never_contains_dance_build_or_secrets_and_rotates(self):
        steps = (SCRIPTS / "robot_steps.sh").read_text()
        backup = steps.split("step_backup() {", 1)[1].split("\n}\n", 1)[0]
        for excluded in ("controller/policies/dance", "controller/build", "*/.env", "*.env"):
            self.assertIn(f"--exclude='{excluded}'", backup)
        self.assertIn('tail -n +"$((keep + 1))"', backup)
        self.assertIn("HB_*.tar.gz", backup)

    def test_rollback_extract_refreshes_mtime_so_old_src_is_rebuilt(self):
        steps = (SCRIPTS / "robot_steps.sh").read_text()
        extract = steps.split("step_rollback_extract() {", 1)[1].split("\n}\n", 1)[0]
        self.assertIn("tar -xzmf", extract)


if __name__ == "__main__":
    unittest.main()
