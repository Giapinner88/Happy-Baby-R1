"""Fail-closed R1 hardware boundary.

``run_teleop`` is read-only preflight. ``high_level_sidecar`` reads lowstate and
forwards bounded targets over loopback UTL1; neither module owns ``rt/lowcmd``.
DDS is initialized only by an explicit CLI invocation.
"""
