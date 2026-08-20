"""Rendering experiments and runs as text tables.

Reporting stays strictly descriptive. It shows what each record says, including
`unassessed` and incomplete runs, and never infers a scientific outcome from the
fact that a process exited zero.
"""

from __future__ import annotations

from .catalog import Experiment
from .record import RunRecord


def format_bytes(count: int) -> str:
    value = float(count)
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if value < 1024.0 or unit == "TB":
            return f"{value:.0f} {unit}" if unit == "B" else f"{value:.1f} {unit}"
        value /= 1024.0
    return f"{value:.1f} TB"


def _plural(count: int, noun: str) -> str:
    return f"{count} {noun}" if count == 1 else f"{count} {noun}s"


def render_table(headers: list[str], rows: list[list[str]]) -> str:
    """Left-aligned fixed-width table; returns a header-only table when empty."""

    widths = [len(header) for header in headers]
    for row in rows:
        for index, cell in enumerate(row):
            widths[index] = max(widths[index], len(cell))
    lines = [
        "  ".join(header.ljust(widths[index]) for index, header in enumerate(headers)).rstrip(),
        "  ".join("-" * widths[index] for index in range(len(headers))),
    ]
    for row in rows:
        lines.append("  ".join(cell.ljust(widths[index]) for index, cell in enumerate(row)).rstrip())
    return "\n".join(lines)


def render_index(experiments: list[Experiment]) -> str:
    """One row per experiment: protocol shape, run counts, outcome spread."""

    rows: list[list[str]] = []
    for experiment in experiments:
        runs = experiment.runs()
        grouped = experiment.runs_by_protocol()
        with_runs = sum(1 for protocol in experiment.protocols if grouped.get(protocol))
        shape = f"{_plural(len(experiment.protocols), 'protocol')}, {with_runs} with runs"
        rows.append([experiment.id, shape, str(len(runs)), _outcome_summary(runs)])
    return render_table(["EXPERIMENT", "PROTOCOLS", "RUNS", "OUTCOMES"], rows)


def _outcome_summary(runs: list[RunRecord]) -> str:
    if not runs:
        return "-"
    counts: dict[str, int] = {}
    for run in runs:
        key = f"{run.execution_status}/{run.scientific_outcome}"
        counts[key] = counts.get(key, 0) + 1
    return ", ".join(f"{key} x{value}" for key, value in sorted(counts.items()))


def render_protocols(experiment: Experiment) -> str:
    """Declared protocols, their run counts and whether a record exists.

    Protocols with no runs are listed too: the pipeline's shape, including which
    parts have produced nothing, is the thing worth seeing.
    """

    grouped = experiment.runs_by_protocol()
    rows: list[list[str]] = []
    for protocol in experiment.protocols:
        runs = grouped.get(protocol, [])
        record = experiment.root / experiment.protocol_records.get(
            protocol, f"protocols/{protocol}.md"
        )
        outcomes = ", ".join(sorted({run.scientific_outcome for run in runs})) if runs else "-"
        rows.append(
            [
                protocol,
                str(len(runs)),
                outcomes,
                str(record.relative_to(experiment.root)) if record.is_file() else "not written",
            ]
        )
    unattributed = grouped.get(None, [])
    for run in unattributed:
        rows.append(["(unattributed)", run.run_id, run.scientific_outcome, "-"])
    return render_table(["PROTOCOL", "RUNS", "OUTCOMES", "RECORD"], rows)


def render_runs(experiment: Experiment) -> str:
    """One row per run, with contract validation folded in."""

    rows: list[list[str]] = []
    for run in experiment.runs():
        validation = run.validate()
        if validation.errors:
            health = f"{len(validation.errors)} error(s)"
        elif validation.warnings:
            health = f"{len(validation.warnings)} warning(s)"
        else:
            health = "ok"
        rows.append(
            [
                run.run_id,
                experiment.protocol_of(run) or "-",
                run.execution_status,
                run.scientific_outcome,
                (run.updated_at or "-")[:19],
                (run.git_commit or "-")[:8],
                str(len(run.data_files())),
                str(sum(value["file_count"] for value in run.bulk_directories().values())),
                format_bytes(run.total_byte_size()),
                health,
            ]
        )
    # DATA counts loose evidence files in the run root; BULK counts everything
    # inside logs/, outputs/, videos/ and the other generated-output trees, which
    # is where a locomotion run keeps almost all of its bytes.
    return render_table(
        ["RUN", "OWNER", "EXECUTION", "OUTCOME", "UPDATED", "COMMIT", "DATA", "BULK", "SIZE", "CONTRACT"],
        rows,
    )


def render_run_detail(experiment: Experiment, run: RunRecord) -> str:
    """Everything one run holds: records, data files, bulk output, issues."""

    lines = [
        f"Run:        {run.run_id}",
        f"Experiment: {experiment.id}",
        f"Owner:      {experiment.protocol_of(run) or '-'}",
        f"Path:       {run.path}",
        f"Execution:  {run.execution_status}",
        f"Outcome:    {run.scientific_outcome}",
    ]
    reason = run.status.get("reason")
    if reason:
        lines.append(f"Reason:     {reason}")
    lines.append(f"Updated:    {run.updated_at or '-'}")
    lines.append(f"Commit:     {run.git_commit or '-'}")
    lines.append(f"Total size: {format_bytes(run.total_byte_size())}")

    lines.append("")
    lines.append("Contract records")
    record_rows = []
    from .contract import ARTIFACTS, canonical_name

    for key in ARTIFACTS:
        resolved = run.resolve(key)
        if resolved is None:
            record_rows.append([canonical_name(key), "absent", "-"])
        else:
            note = "legacy name" if resolved.name != canonical_name(key) else ""
            record_rows.append([resolved.name, "present", note or "-"])
    lines.append(_indent(render_table(["FILE", "STATE", "NOTE"], record_rows)))

    data_files = run.data_files()
    lines.append("")
    lines.append(f"Data files ({len(data_files)})")
    if data_files:
        lines.append(
            _indent(
                render_table(
                    ["FILE", "SIZE"],
                    [[path.name, format_bytes(path.stat().st_size)] for path in data_files],
                )
            )
        )
    else:
        lines.append("  none")

    bulk = run.bulk_directories()
    if bulk:
        lines.append("")
        lines.append("Bulk output directories")
        lines.append(
            _indent(
                render_table(
                    ["DIRECTORY", "FILES", "SIZE"],
                    [
                        [name, str(value["file_count"]), format_bytes(value["byte_size"])]
                        for name, value in sorted(bulk.items())
                    ],
                )
            )
        )

    validation = run.validate()
    lines.append("")
    lines.append(f"Contract check: {'ok' if validation.ok else 'FAILED'}")
    for issue in validation.issues:
        lines.append(f"  {issue}")
    if not validation.issues:
        lines.append("  no issues")
    return "\n".join(lines)


def _indent(text: str, prefix: str = "  ") -> str:
    return "\n".join(prefix + line if line else line for line in text.splitlines())


__all__ = [
    "format_bytes",
    "render_index",
    "render_run_detail",
    "render_runs",
    "render_protocols",
    "render_table",
]
