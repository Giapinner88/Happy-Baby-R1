#!/usr/bin/env python3
"""Static contracts for the canonical local build and deploy entry points."""

from pathlib import Path
import unittest


HB_ROOT = Path(__file__).resolve().parents[2]


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
        self.assertIn('exec "$OPS/deploy_stack.sh" deploy --accept-policy', hb)
        self.assertIn('exec "$OPS/deploy_policy.sh" --with-dance "$@"', hb)
        self.assertNotIn("systemctl", hb)

    def test_deploy_script_uses_canonical_binary_without_discovery(self):
        deploy = (HB_ROOT / "integration" / "scripts" / "deploy_policy.sh").read_text()
        self.assertIn('HIGH_BIN="${HIGH_BIN:-$HIGH_DIR/build/run_r1}"', deploy)
        self.assertNotIn('find "$HIGH_DIR"', deploy)

    def test_dance_sync_excludes_non_runtime_payload(self):
        deploy = (HB_ROOT / "integration" / "scripts" / "deploy_policy.sh").read_text()
        for excluded in (
            "backup/",
            "artifacts/",
            "__pycache__/",
            "*.pyc",
            "*.pt",
            "*.pth",
            "*.onnx.*",
            "logs/",
        ):
            self.assertIn(f"--exclude '{excluded}'", deploy)


if __name__ == "__main__":
    unittest.main()
