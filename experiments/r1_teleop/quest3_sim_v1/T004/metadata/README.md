# T004 study metadata

The case manifest in `../config/r1_t004_calibration_study.json` is authoritative
for the pre-execution matrix. This directory records human-editable selection
and aggregation decisions after runs exist; it must not rewrite run provenance,
raw trace CSV, metrics or status.

[`initial_sweep_selection.json`](initial_sweep_selection.json) maps the 22
predeclared cases to the retained first-sweep run IDs and aggregate output.
