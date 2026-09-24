#!/usr/bin/env python3
"""Generate the missing 5-degree platform scenes used by meta_policy acceptance."""

from __future__ import annotations

import argparse
import math
from pathlib import Path


ANGLES = (5, 10, 20, 25)
ROOT = Path(__file__).resolve().parents[2] / "unitree_robots" / "r1"


def scene_xml(angle_deg: int) -> str:
  angle = math.radians(angle_deg)
  half = 0.5 * angle
  cosine = math.cos(angle)
  sine = math.sin(angle)
  quat_w = math.cos(half)
  quat_y = -math.sin(half)
  uphill_x = 1.0 + 6.0 * cosine + 0.05 * sine
  uphill_z = 6.0 * sine - 0.05 * cosine
  downhill_x = -1.0 - 6.0 * cosine + 0.05 * sine
  downhill_z = -6.0 * sine - 0.05 * cosine
  if angle_deg <= 10:
    rgb1, rgb2 = "0.18 0.38 0.18", "0.08 0.20 0.08"
  elif angle_deg <= 20:
    rgb1, rgb2 = "0.48 0.38 0.12", "0.24 0.16 0.05"
  else:
    rgb1, rgb2 = "0.45 0.20 0.12", "0.22 0.08 0.05"
  return f'''<mujoco model="r1 slope {angle_deg} degrees with spawn platform">
  <compiler angle="radian" meshdir="assets"/>
  <include file="unitree_r1/xmls/r1.xml"/>

  <statistic center="0 0 0.8" extent="2"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-130" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0"
             width="512" height="3072"/>
    <texture type="2d" name="slope_checker" builtin="checker" mark="edge"
             rgb1="{rgb1}" rgb2="{rgb2}"
             markrgb="0.85 0.85 0.85" width="300" height="300"/>
    <material name="slope_material" texture="slope_checker" texuniform="true"
              texrepeat="12 8" reflectance="0.1"/>
  </asset>

  <default>
    <geom friction="0.9 0.005 0.0001" solimp="0.9 0.95 0.001" solref="0.002 1"/>
  </default>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>

    <!-- 2 m flat spawn platform, +X uphill ramp and -X downhill ramp. -->
    <geom name="spawn_platform_{angle_deg}deg" type="box" pos="0 0 -0.05"
          size="1 8 0.05" priority="1" friction="1.0 0.005 0.0001"
          material="slope_material"/>
    <geom name="slope_{angle_deg}deg_uphill" type="box"
          pos="{uphill_x:.10f} 0 {uphill_z:.10f}" size="6 8 0.05"
          quat="{quat_w:.10f} 0 {quat_y:.10f} 0"
          priority="1" friction="1.0 0.005 0.0001"
          material="slope_material"/>
    <geom name="slope_{angle_deg}deg_downhill" type="box"
          pos="{downhill_x:.10f} 0 {downhill_z:.10f}" size="6 8 0.05"
          quat="{quat_w:.10f} 0 {quat_y:.10f} 0"
          priority="1" friction="1.0 0.005 0.0001"
          material="slope_material"/>
  </worldbody>

  <!-- Training-compatible upright reset on the center platform. -->
  <keyframe>
    <key name="stand_on_platform_{angle_deg}"
         qpos="0 0 0.76 1 0 0 0
               -0.1 0 0 0.3 -0.2 0
               -0.1 0 0 0.3 -0.2 0
               0 0
               0.35 0.18 0 0.87 0
               0.35 -0.18 0 0.87 0"/>
  </keyframe>
</mujoco>
'''


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument(
    "--check", action="store_true", help="fail if generated files are stale or missing"
  )
  args = parser.parse_args()
  stale: list[Path] = []
  for angle in ANGLES:
    path = ROOT / f"scene_slope_{angle}_platform.xml"
    expected = scene_xml(angle)
    if args.check:
      if not path.is_file() or path.read_text() != expected:
        stale.append(path)
    else:
      path.write_text(expected)
      print(path)
  if stale:
    for path in stale:
      print(f"STALE: {path}")
    raise SystemExit(1)


if __name__ == "__main__":
  main()
