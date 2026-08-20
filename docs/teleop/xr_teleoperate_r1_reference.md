# xr_teleoperate R1 upstream reference

The upstream repository was cloned as a pinned **read-only vendor reference**:

- URL: `https://github.com/unitreerobotics/xr_teleoperate.git`
- local path: `third_party/xr_teleoperate_v1_6/`
- pinned commit: `845b25a32f7febedf220e830952a7134897adb9d`
- clone form: shallow clone with its shallow submodules, on 2026-08-11.

That revision contains the `R1_A5` and `R1_A7` arm IK/controller paths.  This
does **not** replace the project simulation method or enable hardware control:
upstream's R1 controller can reach DDS paths, while the T-series experiments
remain IsaacLab-only and must not import or execute it.  An integration needs a
separate compatibility review of its joint list, frame convention, asset,
command path and safety boundary before any code is shared.
