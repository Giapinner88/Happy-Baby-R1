import onnxruntime as ort
import json

def main():
    sess = ort.InferenceSession("/home/khanh248/Documents/HB/Mujoco/unitree_mujoco/simulate_python/policy_r1_270.onnx")
    meta = sess.get_modelmeta()
    
    print("--- ONNX Model Inputs ---")
    for i in sess.get_inputs():
        print(f"Name: {i.name}, Shape: {i.shape}, Type: {i.type}")

    print("\n--- ONNX Model Outputs ---")
    for o in sess.get_outputs():
        print(f"Name: {o.name}, Shape: {o.shape}, Type: {o.type}")

    print("\n--- ONNX Model Metadata Map ---")
    for k, v in meta.custom_metadata_map.items():
        print(f"Key: {k}")
        try:
            val = json.loads(v)
            print(f"Value (JSON): {json.dumps(val, indent=2)}")
        except Exception:
            print(f"Value (Raw): {v}")

if __name__ == "__main__":
    main()
