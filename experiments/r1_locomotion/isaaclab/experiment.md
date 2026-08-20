# R1 IsaacLab Locomotion

This experiment owns new R1 IsaacLab/Unitree RL Lab locomotion runs for the
registered `Unitree-R1-Velocity` and experiment-specific `Unitree-R1-Stand`
tasks. It does not claim compatibility with MJLab checkpoints merely because
both workflows target R1 locomotion.

`legacy_inventory.json` records historical paths purged during the data-area
reset. Those entries are not available as local evidence and must not be
silently promoted.

Run it directly in Conda environment `unitree_sim_env`; Docker is not part of
this experiment's active protocol.

The archived [P001 velocity-task port pilot](P001_velocity_port_pilot/) did
not establish stable standing. The active bounded protocol is
[I002 nominal stable standing](I002_nominal_stand/).
