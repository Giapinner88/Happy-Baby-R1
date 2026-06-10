import re

import os
policy_path = os.path.join(os.path.dirname(__file__), "..", "policy", "policy_r1.onnx")
with open(policy_path, 'rb') as f:
    data = f.read()

# find "joint_names" followed by something, then '[' ... ']'
m = re.search(b'joint_names.*?(\[[^\]]+\])', data, re.DOTALL)
if m:
    print("JOINT_NAMES:")
    print(m.group(1).decode('utf-8'))

m = re.search(b'action_scale.*?(\[[^\]]+\])', data, re.DOTALL)
if m:
    print("ACTION_SCALE:")
    print(m.group(1).decode('utf-8'))
