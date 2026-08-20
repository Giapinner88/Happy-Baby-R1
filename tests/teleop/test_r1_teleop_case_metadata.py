"""Metadata and atomic-case checks for the T001–T003 evidence layout."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

from evidence.catalog import find_experiment, load_registry
from scripts.teleop.run_r1_t003_trajectory import load_config, select_case


ROOT = Path(__file__).resolve().parents[2]
EXPERIMENT_ROOT = ROOT / "experiments" / "r1_teleop" / "quest3_sim_v1"
T001_ROOT = EXPERIMENT_ROOT / "T001"
T002_ROOT = EXPERIMENT_ROOT / "T002"
T003_ROOT = EXPERIMENT_ROOT / "T003"
CATALOGS = [
    T001_ROOT / "metadata" / "evidence_catalog.json",
    T002_ROOT / "metadata" / "evidence_catalog.json",
    T003_ROOT / "metadata" / "evidence_catalog.json",
]
CASE_CONFIG = T003_ROOT / "config" / "r1_t003_case_matrix.json"


class T003CaseMatrixTests(unittest.TestCase):
    def test_each_case_selects_one_protocol_and_only_its_declared_injection(self) -> None:
        base = load_config(CASE_CONFIG)
        selected = {case["case_id"]: select_case(base, case["case_id"]) for case in base["cases"]}
        self.assertEqual(selected["nominal"]["protocol_id"], "t003_a")
        self.assertEqual(selected["nominal"]["safety_injections"], [])
        for name, reason in (
            ("deadman", "deadman_released"),
            ("timeout", "command_timeout"),
            ("sequence", "sequence_id_not_increasing"),
        ):
            self.assertEqual(selected[name]["protocol_id"], "t003_b")
            self.assertEqual(len(selected[name]["safety_injections"]), 1)
            self.assertEqual(selected[name]["validity"]["required_safety_events"], [reason])

    def test_unknown_case_is_refused(self) -> None:
        with self.assertRaises(SystemExit):
            select_case(load_config(CASE_CONFIG), "all_safety_events_in_one_run")


class EvidenceCatalogTests(unittest.TestCase):
    def test_catalog_references_retained_runs_and_records_exclusions(self) -> None:
        selected = False
        excluded = False
        for catalog in CATALOGS:
            payload = json.loads(catalog.read_text(encoding="utf-8"))
            self.assertEqual(payload["record_type"], "editable_protocol_case_catalog")
            owner_root = catalog.parents[1]
            for protocol in payload["protocols"]:
                for case in protocol["cases"]:
                    selected |= bool(case["selected_for_follow_up"])
                    excluded |= not bool(case["selected_for_follow_up"])
                    self.assertIn("selection_reason", case)
                    for source in case["source_runs"]:
                        self.assertTrue((owner_root / "runs" / source["run_id"]).is_dir(), source)
        self.assertTrue(selected)
        self.assertTrue(excluded)

    def test_t001_b_bridge_logs_belong_to_their_own_runs(self) -> None:
        payload = json.loads((T001_ROOT / "metadata" / "evidence_catalog.json").read_text(encoding="utf-8"))
        run_root = T001_ROOT / "runs"
        self.assertEqual(list(run_root.glob("*.bridge.jsonl")), [])
        relocations = payload["artifact_relocations"]
        self.assertEqual(len(relocations), 1)
        for artifact in relocations[0]["files"]:
            self.assertTrue(artifact["from"].startswith("legacy central runs/"))
            self.assertTrue((EXPERIMENT_ROOT / artifact["to"]).is_file())

    def test_registry_attributes_new_t003_prefixes_to_the_split_protocols(self) -> None:
        experiment = find_experiment(load_registry(ROOT), "r1_teleop")
        self.assertEqual(experiment.protocol_of(type("Run", (), {"run_id": "t003_a_nominal_example"})()), "t003_a")
        self.assertEqual(experiment.protocol_of(type("Run", (), {"run_id": "t003_b_timeout_example"})()), "t003_b")

    def test_registry_discovers_protocol_owned_run_roots(self) -> None:
        experiment = find_experiment(load_registry(ROOT), "r1_teleop")
        run_paths = {run.path.parent.relative_to(EXPERIMENT_ROOT).as_posix() for run in experiment.runs()}
        self.assertEqual(run_paths, {"T001/runs", "T002/runs", "T003/runs", "T004/runs", "T005/runs", "T006/runs", "T007/runs"})


if __name__ == "__main__":
    unittest.main()
