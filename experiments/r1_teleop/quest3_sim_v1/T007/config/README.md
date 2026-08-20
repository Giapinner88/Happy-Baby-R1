# T007 configuration

`r1_t007_arm_head_live.json` is the editable schema-2 live profile. It owns the
vendor virtual-endpoint definition, absolute 1:1 mapping, q=0 startup/reset,
joint-limited projection policy, and simulation rate limits. Every new T007 run
must snapshot this file as `experiment_config.json`; schema-1 runs are legacy
and must not be reinterpreted with this endpoint model.

The active `1.5 rad/s`, `4.0 rad/s²` rate pair is a simulation-only revision
selected by replaying `t007_live_arm_head_20260817T114118Z`. The earlier
`1.0 rad/s`, `2.0 rad/s²` snapshot remains authoritative for that run. Dynamic
tracking metrics across the revision must name the rate pair and are not
identical-protocol replicates.
`r1_t007_whole_upper_body_live.json` is the schema-3 coupled pilot. It gives
`waist_yaw_joint`, both five-joint arms, and both head joints to one IK solve.
It is not directly comparable with schema-2 independent-arm runs and is not a
hardware configuration.
