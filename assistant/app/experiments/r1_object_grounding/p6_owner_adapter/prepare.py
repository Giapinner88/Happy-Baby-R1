"""Stage exact guarded owner hook; do not edit canonical or active robot tree."""
import argparse
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent


def digest(data):return hashlib.sha256(data).hexdigest()


def replace_once(text,old,new):
    if text.count(old)!=1:raise ValueError('integration anchor mismatch')
    return text.replace(old,new)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source-root',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args()
    header='controller/src/app/Application.hpp';policy='controller/src/app/ApplicationPolicy.cpp'
    original={name:(a.source_root/name).read_bytes() for name in [header,policy]}
    text=original[header].decode()
    text=replace_once(text,'#include "../input/SearchYawReceiver.hpp"','#include "../input/SearchYawReceiver.hpp"\n#include "../input/ApproachReceiver.hpp"')
    text=replace_once(text,'    SearchYawReceiver search_yaw_;','    SearchYawReceiver search_yaw_;\n    approach_owner::Receiver approach_receiver_;')
    changed={header:text.encode()}
    text=original[policy].decode()
    anchor='''    if (state_ == AppState::kLocomotion && !holding && !teleop_locks_base &&
        !history_gesture_holds_base) {
        cmd_vx_ = requested_vx;'''
    hook='''    // APV1 is opt-in, zero-only by default. Preserve manual/START/history/
    // emergency gates; never create another LowCmd publisher.
    approach_receiver_.Apply(requested_vx, requested_vy, requested_yaw,
        manual_motion,
        state_fresh_ && armed_.load() && cmd.is_active &&
            state_ == AppState::kLocomotion && loco_mode_ == LocoMode::kWalk &&
            !holding && !teleop_locks_base && !history_gesture_holds_base,
        cmd.want_emergency_stop);
'''+anchor
    changed[policy]=replace_once(text,anchor,hook).encode()
    changed['controller/src/input/ApproachReceiver.hpp']=(HERE/'ApproachReceiver.hpp').read_bytes()
    out=a.output_dir;out.mkdir(parents=True,exist_ok=False)
    for name,data in changed.items():
        path=out/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
    (out/'manifest.json').write_text(json.dumps({'base_sha256':{n:digest(d) for n,d in original.items()},'output_sha256':{n:digest(d) for n,d in changed.items()},'mode':'staged_not_active','default':'zero_only'},indent=2)+'\n')
    print(out)


if __name__=='__main__':main()
