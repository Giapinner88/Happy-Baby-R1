import re
import json

def get_meta():
    import os
    policy_path = os.path.join(os.path.dirname(__file__), "..", "policy", "policy_r1.onnx")
    with open(policy_path, 'rb') as f:
        data = f.read()
    
    # 1. Find joint_names
    match_jn = list(re.finditer(b'joint_names', data))
    if match_jn:
        idx = match_jn[0].end()
        # The string "joint_names" is likely followed by some protobuf framing.
        # Let's just find the next '[' and ']' characters.
        start = data.find(b'[', idx)
        end = data.find(b']', start)
        print("JOINT_NAMES:")
        print(data[start:end+1].decode('utf-8'))
    
    match_acs = list(re.finditer(b'action_scale', data))
    if match_acs:
        idx = match_acs[0].end()
        start = data.find(b'[', idx)
        end = data.find(b']', start)
        print("ACTION_SCALE:")
        print(data[start:end+1].decode('utf-8'))

get_meta()
