# Figure design — T004 initial calibration sensitivity

- **Figure ID:** `t004_initial_calibration_sensitivity`
- **Role:** diagnostic evidence figure.
- **Status:** generated and visually verified from the complete case table.
- **Source analysis:** [`../../analysis.md`](../../analysis.md).
- **Source:** all 22 selected cases in `case_table.csv`; none excluded.

The figure asks how the declared OAT calibration perturbations change the
mapper's target displacement and the fixed IK solver's minimum reported joint
limit margin. It must visibly retain every individual case and distinguish
right recorded source trace from left synthetic mirror.

Panel A plots `minimum_limit_margin_rad`; panel B plots
`1000 * max_target_displacement_m`. Each point is one case. The only numerical
transformation is metres to millimetres in panel B; there is no smoothing,
interpolation, normalization or aggregation. Crosses would show non-convergence
or a clamp; none occurred in this selection.

The figure supports only a diagnostic claim about this synthetic mapper+IK
screen. It does not support real calibration accuracy, collision clearance,
dynamic tracking or hardware safety.

Generate it with:

```bash
MPLCONFIGDIR=/tmp/mpl conda run --no-capture-output -n r1_env python \
  scripts/teleop/plot_r1_t004_calibration_study.py \
  --case-table experiments/r1_teleop/quest3_sim_v1/T004/figures/t004_initial_20260811T044700Z/case_table.csv \
  --output-dir experiments/r1_teleop/quest3_sim_v1/T004/figures/t004_initial_20260811T044700Z
```
