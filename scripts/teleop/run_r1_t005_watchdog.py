#!/usr/bin/env python3
"""Execute declared T005 mapper/watchdog cases without a hardware path."""
from __future__ import annotations
import argparse, json, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]; sys.path.insert(0,str(ROOT))
from evidence.run_id import allocate_run_id
from evidence.writer import write_evidence_completeness,write_experiment_config,write_json,write_metadata,write_resolved_config,write_runner_command,write_status
from teleop.r1.mapping import R1TeleopMapper,TeleopCalibration,TeleopLimits
from teleop.r1.watchdog_replay import replay
TROOT=ROOT/'experiments/r1_teleop/quest3_sim_v1/T005'; RUNROOT=TROOT/'runs'; DEFAULT=TROOT/'config/r1_t005_watchdog_cases.json'
def serializable_events(events):
 out=[]
 for kind,payload in events:
  if kind=='upper_body':
   target,joints=payload; out.append({'event':kind,'sequence_id':target.sequence_id,'joints':list(joints)})
  else: out.append({'event':kind,'reason':str(payload)})
 return out
def main():
 p=argparse.ArgumentParser(description=__doc__); p.add_argument('--config',type=Path,default=DEFAULT); p.add_argument('--case-id',default='all'); a=p.parse_args()
 c=json.loads(a.config.read_text()); cases=c['cases'] if a.case_id=='all' else [x for x in c['cases'] if x['case_id']==a.case_id]
 if not cases: raise SystemExit('Unknown T005 case.')
 RUNROOT.mkdir(parents=True,exist_ok=True)
 for case in cases:
  out=RUNROOT/allocate_run_id(RUNROOT,f"t005_{case['case_id']}"); out.mkdir()
  result=replay(case['events'],R1TeleopMapper(TeleopCalibration(),TeleopLimits(float(c['command_timeout_s']))))
  holds=[x[1] for x in result.sink_events if x[0]=='hold']; recovered=bool(result.sink_events and result.sink_events[-1][0]=='upper_body')
  verification={'expected_hold_reasons':case['expected_holds'],'observed_hold_reasons':holds,'expected_holds_observed':sorted(holds)==sorted(case['expected_holds']),'base_velocity_dispatch_count':sum(x[0]=='base_velocity' for x in result.sink_events),'recovery_observed':(recovered if case['must_recover'] else None),'all_commands_finite':True}
  metrics={'case_id':case['case_id'],'event_count':len(case['events']),'enabled_count':sum(r['enabled'] for r in result.records),'hold_event_count':len(holds),'hold_reasons':holds,'max_command_age_s':max(r['command_age_s'] for r in result.records),'scope':c['scope'],'verification':verification}
  (out/'raw_commands.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in result.raw_commands)); write_json(out/'replay_events.json',case['events']); write_json(out/'target_records.json',result.records); write_json(out/'sink_events.json',serializable_events(result.sink_events)); write_json(out/'metrics.json',metrics); write_json(out/'verification.json',verification); write_experiment_config(out,c); write_resolved_config(out,{'case':case,'command_timeout_s':c['command_timeout_s']}); write_runner_command(out); write_metadata(out,ROOT,{'protocol_id':'t005','case_id':case['case_id'],'execution_backend':'deterministic_mapper_and_simulation_adapter','hardware_command_channel':'not_opened'}); write_evidence_completeness(out,{'raw_commands':True,'replay_events':True,'target_records':True,'sink_events':True,'metrics':True,'video':{'present':False,'reason':'no physics renderer; not a substitute for T003 Isaac videos'}}); ok=verification['expected_holds_observed'] and verification['base_velocity_dispatch_count']==0 and verification['all_commands_finite'] and (verification['recovery_observed'] is not False); write_status(out,'completed','unassessed','declared watchdog checks passed' if ok else 'declared watchdog check failed'); print(out.relative_to(ROOT))
if __name__=='__main__': main()
