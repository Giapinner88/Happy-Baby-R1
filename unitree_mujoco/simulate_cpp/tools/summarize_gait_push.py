"""Metrics from physics-substep CSV, rejecting touchdown contact chatter.

Foot heights are relative to median site height in stance on the flat plane.
Stride is same-foot touchdown displacement (not alternating-foot step length).
Landing force is peak in first 30 ms of a touchdown after >=40 ms airborne.
"""
import csv
import json
from pathlib import Path
import numpy as np

OUT = Path(__file__).resolve().parents[1] / "results/gait_compare_20260909"


def read_csv(path):
    with path.open() as stream:
        reader = csv.DictReader(stream)
        rows = [r for r in reader if None not in r and all(v is not None for v in r.values())]
    columns = {}
    for k in rows[0] if rows else []:
        try:
            columns[k] = np.array([float(r[k]) for r in rows])
        except ValueError:
            pass
    return columns


def stats(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    return {"n": len(values), "median": float(np.median(values)) if len(values) else None,
            "p95": float(np.percentile(values, 95)) if len(values) else None}


def summarize(name):
    d = read_csv(OUT / f"{name}_physics.csv")
    ppath = OUT / f"{name}.csv"
    p = read_csv(ppath) if ppath.exists() else {}
    if not d:
        return {"error": "empty physics CSV"}
    t = d["time"]
    dt = np.median(np.diff(t))
    mask = (t >= 8) & (t <= 16)
    result = {"physics_end_s": float(t[-1]), "physics_dt_s": float(dt),
              "physics_vx_mean": float(np.mean(d["base_vx"][mask])),
              "physics_vy_mean": float(np.mean(d["base_vy"][mask])),
              "base_z_range_mm": float(np.ptp(d["base_z"][mask])*1000),
              "min_base_z_m": float(np.min(d["base_z"][t >= 5]))}
    log = (OUT / f"{name}.log").read_text()
    result["autotest_pass"] = "RESULT=PASS" in log
    result["fall_guard"] = "BLOCKED=FALLEN" in log or "FAIL fallen" in log or "[FALL" in log
    if "gz" in p:
        result["absolute_tilt_max_deg"] = float(np.degrees(np.arccos(np.clip(-p["gz"], -1, 1))).max())
        result["policy_measured_end_s"] = float(p["t"][-1])
        late = p["t"] >= p["t"][-1]-2
        result["last_2s_displacement_m"] = float(np.hypot(np.ptp(p["drift_x"][late]), np.ptp(p["drift_y"][late])))
    for side in ("left", "right"):
        c = d[f"{side}_contact"] > .5
        # Bridge <=10-ms contact gaps caused by individual foot geoms chattering.
        gaps_start = np.flatnonzero(~c[1:] & c[:-1])+1
        gaps_end = np.flatnonzero(c[1:] & ~c[:-1])+1
        for a in gaps_start:
            ends = gaps_end[gaps_end > a]
            if len(ends) and t[ends[0]]-t[a] <= .010001:
                c[a:ends[0]] = True
        z = d[f"{side}_foot_z"]
        stance_z = np.median(z[c & mask]) if np.any(c & mask) else 0
        touch = np.flatnonzero(c[1:] & ~c[:-1])+1
        liftoff = np.flatnonzero(~c[1:] & c[:-1])+1
        peaks, strides, forces, downward, stance_knees, periods = [], [], [], [], [], []
        previous_td = None
        for i in touch:
            prev = liftoff[liftoff < i]
            if not len(prev):
                continue
            j = prev[-1]
            if t[i]-t[j] < .04 or not (8 <= t[j] and t[i] <= 16):
                continue
            peaks.append((np.max(z[j:i])-stance_z)*1000)
            downward.append(max(0, -d[f"{side}_foot_vz"][i-1]))
            end = min(len(t), i+int(round(.03/dt))+1)
            forces.append(np.max(d[f"{side}_force_n"][i:end]))
            if previous_td is not None:
                strides.append(d[f"{side}_foot_x"][i]-d[f"{side}_foot_x"][previous_td])
                periods.append(t[i]-t[previous_td])
            previous_td = i
            next_lo = liftoff[liftoff > i]
            if len(next_lo):
                n = next_lo[0]
                lo, hi = i+int(.25*(n-i)), i+int(.7*(n-i))
                if hi > lo:
                    stance_knees.extend(np.degrees(d[f"{side}_knee_q"][lo:hi]))
        result[side] = {"clearance_mm": stats(peaks), "stride_m": stats(strides),
                        "stride_period_s": stats(periods), "touchdown_peak_N": stats(forces),
                        "pre_touchdown_down_mps": stats(downward), "midstance_knee_deg": stats(stance_knees)}
    if (OUT / f"{name}.push").exists():
        result["push_logged_count"] = log.count("[PUSH]")
        result["pushes"] = []
        vx = .4 if "_walk_" in name else 0
        for start, force in ((9, 40), (15, 80)):
            end = start+.2
            after = (t >= end) & (t < start+5)
            speederr = np.hypot(d["base_vx"]-vx, d["base_vy"])
            # Recovery: 0.5 s continuously within 0.1 m/s of commanded planar speed.
            n = max(1, int(round(.5/dt)))
            ok = (speederr < .1).astype(int)
            sustained = np.convolve(ok, np.ones(n, dtype=int), mode="valid") == n
            starts = t[:len(sustained)]
            good = np.flatnonzero(sustained & (starts >= end) & (starts+.5 < start+5))
            # Normal walking speed oscillates every step; also evaluate a causal
            # one-stride (.6-s) average, using the same rule for all policies.
            filt_n = max(1, int(round(.6/dt)))
            kernel = np.ones(filt_n)/filt_n
            smooth_x = np.convolve(d["base_vx"], kernel, mode="valid")
            smooth_y = np.convolve(d["base_vy"], kernel, mode="valid")
            smooth_t = t[filt_n-1:]
            smooth_err = np.hypot(smooth_x-vx, smooth_y)
            smooth_ok = np.convolve((smooth_err < .1).astype(int), np.ones(n,dtype=int), mode="valid") == n
            smooth_starts = smooth_t[:len(smooth_ok)]
            recovered = np.flatnonzero(smooth_ok & (smooth_starts >= end+.6) & (smooth_starts+.5 < start+5))
            result["pushes"].append({"force_N": force, "impulse_Ns": force*.2,
                "recovery_s": float(starts[good[0]]-end) if len(good) else None,
                "cycle_mean_recovery_s": float(smooth_starts[recovered[0]]-end) if len(recovered) else None,
                "max_speed_error_mps": float(np.max(speederr[after])) if np.any(after) else None,
                "min_z_m": float(np.min(d["base_z"][after])) if np.any(after) else None})
    return result


if __name__ == "__main__":
    summary = {}
    for path in sorted(OUT.glob("*_physics.csv")):
        name = path.name.removesuffix("_physics.csv")
        if "RESULT=" not in (OUT / f"{name}.log").read_text():
            continue
        try:
            summary[name] = summarize(name)
        except Exception as e:
            summary[name] = {"error": repr(e)}
    (OUT / "metrics.json").write_text(json.dumps(summary, indent=2))
    for name, r in summary.items():
        print(name, json.dumps(r))
