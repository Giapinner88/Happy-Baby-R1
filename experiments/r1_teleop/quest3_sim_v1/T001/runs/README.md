# Immutable evidence runs

Each direct child directory is one execution unit and contains that run's
immutable raw data, resolved configuration, provenance, status and derived
artifacts. A scientific case can select all or a declared subset of one run;
the mapping belongs to [`../metadata/evidence_catalog.json`](../metadata/evidence_catalog.json).

T001-B has two producer processes. The launcher stages its bridge log under
`.staging/` while IsaacLab creates the run directory, then moves it into the
completed run as `bridge_connection.jsonl`. `.staging/` is transient and never
evidence. Historical bridge logs were normalized into their matching run
directories; their prior paths are retained in the catalog migration record.

Do not create source code, editable inputs, or protocol documents here. Do not
rename, delete, or overwrite a completed run.
