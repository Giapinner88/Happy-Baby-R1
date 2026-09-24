import onnxruntime as ort
import os
policy_path = os.path.join(os.path.dirname(__file__), "..", "policy", "policy_r1.onnx")
sess = ort.InferenceSession(policy_path, providers=['CPUExecutionProvider'])
print('Input shape:', sess.get_inputs()[0].shape)
