def main():
    import os
    policy_path = os.path.join(os.path.dirname(__file__), "..", "policy", "policy_motion_data.onnx")
    with open(policy_path, "rb") as f:
        data = f.read()

    keys = [b"joint_names", b"joint_stiffness", b"joint_damping", b"default_joint_pos", b"action_scale", b"observation_names"]
    for key in keys:
        idx = data.find(key)
        if idx != -1:
            print(f"\n--- {key.decode()} ---")
            # Let's read the next 500 bytes and print any printable string
            sub = data[idx + len(key): idx + len(key) + 500]
            # Print as ascii where possible
            res = []
            for b in sub:
                if 32 <= b <= 126:
                    res.append(chr(b))
                else:
                    res.append(f"\\x{b:02x}")
            print("".join(res))

if __name__ == "__main__":
    main()
