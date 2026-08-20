# P001 checkpoint sidecar diagnostic

This is a bounded, read-only diagnostic for an archived P001 checkpoint. It
answers a practical question: under fixed,
single-robot commands, what can the selected checkpoint do now?  It is not an
acceptance evaluation or a basis for policy promotion.

The editable protocol is `sidecar_evaluation.json`.  Every invocation creates
a new directory below the source run's `sidecar_evaluations/`; it never writes
to the training log, checkpoint, resolved configuration, or status file.

It evaluates one environment on GPU 0, with fixed `stand`, `forward_005`, and
`forward_010` commands.  The policy observation corruption is disabled for
repeatable diagnosis; the manifest records this compatibility choice.  It also
reapplies P001's plane/no-push/no-curriculum environment contract. Each case
runs in a separate IsaacLab process, so a failure cannot suppress the other
cases. It produces an isolated video, raw CSV and NPZ state/action/command
trace, termination terms, and two diagnostic plots per case.

From the repository root, select the newest stable checkpoint from the active
run and launch it with:

```bash
scripts/training/run_p001_archival_evaluation.sh \
  experiments/r1_locomotion/isaaclab/runs/P001_velocity_port_pilot_20260727T033124Z
```

The wrapper sets `CUDA_VISIBLE_DEVICES=0`.  Run it only after a checkpoint has
finished writing; it hashes the selected checkpoint before evaluation and
records that hash in the manifest.  The source training process continues on
GPU 1.
