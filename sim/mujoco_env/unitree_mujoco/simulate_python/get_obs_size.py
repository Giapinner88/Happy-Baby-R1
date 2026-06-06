import onnxruntime as ort
sess = ort.InferenceSession('policy_r1.onnx', providers=['CPUExecutionProvider'])
print('Input shape:', sess.get_inputs()[0].shape)
