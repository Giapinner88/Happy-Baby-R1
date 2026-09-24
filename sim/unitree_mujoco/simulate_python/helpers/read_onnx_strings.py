import re

def main():
    import os
    policy_path = os.path.join(os.path.dirname(__file__), "..", "policy", "policy_motion_data.onnx")
    with open(policy_path, "rb") as f:
        data = f.read()
    
    # Extract ASCII strings
    strings = re.findall(b'[a-zA-Z0-9_\\-\\s\\,\\:\\.\\{\\}\\[\\]\\\"\\\'\\_]{20,}', data)
    print("Found matching strings:")
    for s in strings:
        try:
            s_str = s.decode('ascii').strip()
            if any(k in s_str.lower() for k in ["joint", "scale", "stiffness", "damping", "default", "kp", "kd"]):
                print(s_str)
        except Exception:
            pass

if __name__ == "__main__":
    main()
