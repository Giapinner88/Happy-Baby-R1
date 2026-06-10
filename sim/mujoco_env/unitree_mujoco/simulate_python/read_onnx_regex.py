import re
import json

def main():
    with open("/home/khanh248/Documents/HB/Mujoco/unitree_mujoco/simulate_python/policy_r1.onnx", "rb") as f:
        data = f.read()

    # Search for all ascii printables of length > 20
    # or let's search for JSON-like substrings starting with { or [ and ending with } or ]
    # that contain keys like "joint_names", "action_scale" etc.
    
    keys = [b"joint_names", b"action_scale", b"stiffness", b"damping", b"DEFAULT_Q", b"default_joint_pos"]
    for key in keys:
        matches = list(re.finditer(key, data))
        print(f"\nMatches for: {key.decode()}")
        for m in matches:
            start_idx = max(0, m.start() - 100)
            end_idx = min(len(data), m.end() + 1000)
            chunk = data[start_idx:end_idx]
            
            # Find JSON arrays/dicts
            # Let's find first '[' or '{' after the key match
            rel_match_pos = m.start() - start_idx
            json_start = -1
            for i in range(rel_match_pos, len(chunk)):
                if chunk[i] in (ord('['), ord('{')):
                    json_start = i
                    break
            if json_start != -1:
                # Find matching closing bracket
                bracket_type = chunk[json_start]
                closing_char = ord(']') if bracket_type == ord('[') else ord('}')
                depth = 0
                json_end = -1
                for j in range(json_start, len(chunk)):
                    if chunk[j] == bracket_type:
                        depth += 1
                    elif chunk[j] == closing_char:
                        depth -= 1
                        if depth == 0:
                            json_end = j + 1
                            break
                if json_end != -1:
                    json_bytes = chunk[json_start:json_end]
                    try:
                        val = json.loads(json_bytes.decode('utf-8', errors='ignore'))
                        print(f"Found JSON: {json.dumps(val, indent=2)}")
                    except Exception as e:
                        # Print raw bytes
                        print(f"Raw chunk: {json_bytes.decode('utf-8', errors='ignore')}")

if __name__ == "__main__":
    main()
