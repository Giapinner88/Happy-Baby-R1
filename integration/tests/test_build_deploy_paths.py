#!/usr/bin/env python3
"""Static contracts for the canonical local build and deploy entry points."""

from pathlib import Path
import unittest


HB_ROOT = Path(__file__).resolve().parents[2]


class BuildDeployPathTests(unittest.TestCase):
    def test_build_script_builds_canonical_tree_with_configurable_parallelism(self):
        build = (HB_ROOT / "controller" / "scripts" / "build.sh").read_text()
        self.assertIn('cmake --build build --parallel "$JOBS"', build)
        self.assertIn('JOBS="${HB_BUILD_JOBS:-$(nproc)}"', build)
        self.assertNotIn('cmake --build build --target run_r1 --parallel', build)

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
