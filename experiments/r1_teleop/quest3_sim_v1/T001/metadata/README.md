# Teleop protocol/case metadata

[`evidence_catalog.json`](evidence_catalog.json) is the canonical **editable**
index of T001 evaluation parts and cases. It separates a physical execution
unit (`runs/<run-id>/`) from the scientific case that consumes it.

Do not edit a historical run's `metadata.json`, `status.json`, raw trace, or
measurement to reclassify it. Edit this catalog and the owning protocol record
instead; the catalog must retain every excluded or superseded case with a reason.
