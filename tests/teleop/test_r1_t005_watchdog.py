from __future__ import annotations
import json, unittest
from pathlib import Path
from teleop.r1.mapping import R1TeleopMapper, TeleopCalibration, TeleopLimits
from teleop.r1.watchdog_replay import replay
ROOT=Path(__file__).resolve().parents[2]
CONFIG=ROOT/'experiments/r1_teleop/quest3_sim_v1/T005/config/r1_t005_watchdog_cases.json'
class T005WatchdogTests(unittest.TestCase):
 def setUp(self): self.config=json.loads(CONFIG.read_text())
 def test_every_declared_case_has_exact_hold_outcome_and_no_base_dispatch(self):
  for case in self.config['cases']:
   result=replay(case['events'],R1TeleopMapper(TeleopCalibration(),TeleopLimits(self.config['command_timeout_s'])))
   holds=[event[1] for event in result.sink_events if event[0]=='hold']
   self.assertEqual(holds,case['expected_holds'],case['case_id'])
   self.assertFalse(any(event[0]=='base_velocity' for event in result.sink_events),case['case_id'])
   if case['must_recover']: self.assertEqual(result.sink_events[-1][0],'upper_body',case['case_id'])
 def test_packet_drop_below_timeout_stays_enabled(self):
  case=next(x for x in self.config['cases'] if x['case_id']=='packet_drop_below_timeout')
  result=replay(case['events'],R1TeleopMapper(TeleopCalibration(),TeleopLimits(self.config['command_timeout_s'])))
  self.assertTrue(all(row['enabled'] for row in result.records))
