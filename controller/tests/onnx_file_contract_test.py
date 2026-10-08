"""Generate small, valid ONNX graphs with incompatible HB tensor contracts.

Optional host test; requires onnx/numpy, no DDS and no policy artifact changes.
Usage: python onnx_file_contract_test.py /path/to/onnx_file_contract_test
"""
import pathlib
import subprocess
import sys
import tempfile

import numpy as np
import onnx
from onnx import helper, numpy_helper, TensorProto

with tempfile.TemporaryDirectory(prefix="hb-onnx-contract-") as tmp:
    for name, dtype, out_shape, in_shape, accept in [
        ("valid", np.float32, [1, 24], [1, 83], True),
        ("dynamic_batch", np.float32, [1, 24], ["batch", 83], True),
        ("double_output", np.float64, [1, 24], [1, 83], False),
        ("int_output", np.int32, [1, 24], [1, 83], False),
        ("rank1_output", np.float32, [24], [1, 83], False),
        ("rank3_output", np.float32, [1, 2, 24], [1, 83], False),
        ("batch2_output", np.float32, [2, 24], [1, 83], False),
        ("rank3_input", np.float32, [1, 24], [1, 83, 1], False),
        ("batch2_input", np.float32, [1, 24], [2, 83], False),
    ]:
        values = np.zeros(out_shape, dtype=dtype)
        tensor = numpy_helper.from_array(values)
        graph = helper.make_graph(
            [helper.make_node("Constant", [], ["action"], value=tensor)], name,
            [helper.make_tensor_value_info("obs", TensorProto.FLOAT, in_shape)],
            [helper.make_tensor_value_info("action", tensor.data_type, out_shape)],
        )
        model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)], ir_version=8)
        onnx.checker.check_model(model)
        path = pathlib.Path(tmp) / f"{name}.onnx"
        onnx.save(model, path)
        subprocess.run([sys.argv[1], str(path), "accept" if accept else "reject"], check=True)
print("ONNX_FILE_CONTRACT_OK cases=9")
