"""Create standalone gait comparison figures from measured C++ physics logs."""
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from summarize_gait_push import OUT, read_csv

summary=json.loads((OUT/"metrics.json").read_text())
models=[("h4","H4","#666666"),("old","Old gait","#d88b22"),("new","Walk-quality","#168786")]
fig, axes=plt.subplots(2, 3, figsize=(13,7), constrained_layout=True)
specs=[("clearance_mm","Swing clearance (mm)"),("stride_m","Same-foot stride (m)"),
       ("touchdown_peak_N","Touchdown peak force (N, first 30 ms)"),
       ("midstance_knee_deg","Midstance knee flexion (degrees)"),
       ("pre_touchdown_down_mps","Pre-touchdown downward speed (m/s)")]
for ax,(key,title) in zip(axes.flat,specs):
    for j,(model,label,color) in enumerate(models):
        r=summary[f"{model}_v04"]
        ax.bar(np.arange(2)+(j-1)*.24,[r[s][key]["median"] for s in ("left","right")],
               width=.23,label=label,color=color)
    ax.set_xticks([0,1],["Left","Right"]);ax.set_title(title,fontsize=10)
    ax.grid(axis="y",alpha=.2)
axes.flat[0].axhline(35,color="black",ls=":",lw=1,label="Training mid-swing lower target")
ax=axes.flat[-1]
for j,(model,label,color) in enumerate(models):
    ax.bar(j,summary[f"{model}_v04"]["physics_vx_mean"],color=color)
ax.axhline(.4,color="black",ls="--",lw=1)
ax.set_xticks(range(3),[x[1] for x in models]);ax.set_title("Mean world-forward velocity (m/s)",fontsize=10)
axes.flat[1].legend(frameon=False)
fig.suptitle("R1 / command vx=0.4 m/s / flat MuJoCo / steady interval t=8..16 s\nMedians; one rollout per policy; larger knee angle = more bent",fontsize=12)
fig.savefig(OUT/"walking_comparison.png",dpi=170)
plt.close(fig)

if all((OUT/f"{m}_walk_back_physics.csv").exists() for m,_,_ in models):
    fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True,constrained_layout=True)
    for m,label,color in models:
        d=read_csv(OUT/f"{m}_walk_back_physics.csv")
        axes[0].plot(d["time"],d["base_vx"],label=label,color=color,lw=1)
        p=read_csv(OUT/f"{m}_walk_back.csv")
        tilt=np.degrees(np.arccos(np.clip(-p["gz"],-1,1)))
        axes[1].plot(p["t"]+5,tilt,color=color,lw=1)
    for ax in axes:
        ax.axvspan(9,9.2,color="red",alpha=.15)
        ax.axvspan(15,15.2,color="red",alpha=.15)
        ax.grid(alpha=.2);ax.set_xlim(7,21)
    axes[0].axhline(.4,ls=":",color="black");axes[0].legend(frameon=False)
    axes[0].set_ylabel("World vx (m/s)");axes[1].set_ylabel("Absolute torso tilt (degrees)")
    axes[1].set_xlabel("Physics time (s); torso clock aligned approximately (+5 s)")
    fig.suptitle("Rearward pelvis push while walking: 40 N then 80 N, each 0.2 s")
    fig.savefig(OUT/"push_comparison.png",dpi=170)
