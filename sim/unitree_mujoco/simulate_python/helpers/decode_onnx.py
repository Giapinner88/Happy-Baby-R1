import sys

def parse_protobuf(data, start, end, prefix=""):
    pos = start
    while pos < end:
        # Read varint tag
        if pos >= len(data):
            break
        tag_start = pos
        tag = 0
        shift = 0
        while True:
            b = data[pos]
            pos += 1
            tag |= (b & 0x7F) << shift
            shift += 7
            if not (b & 0x80):
                break
        
        field_num = tag >> 3
        wire_type = tag & 7
        
        if wire_type == 0:  # Varint
            val = 0
            shift = 0
            while True:
                b = data[pos]
                pos += 1
                val |= (b & 0x7F) << shift
                shift += 7
                if not (b & 0x80):
                    break
            # print(f"{prefix}Field {field_num}: Varint {val}")
        elif wire_type == 1:  # 64-bit
            pos += 8
        elif wire_type == 2:  # Length-delimited
            length = 0
            shift = 0
            while True:
                b = data[pos]
                pos += 1
                length |= (b & 0x7F) << shift
                shift += 7
                if not (b & 0x80):
                    break
            field_data_start = pos
            pos += length
            
            # If this is graph (field 4 in ModelProto)
            if field_num == 4 and prefix == "":
                # ModelProto.graph
                # print("Found GraphProto")
                parse_protobuf(data, field_data_start, field_data_start + length, prefix + "  ")
            # If this is input (field 11 in GraphProto)
            elif field_num == 11 and prefix == "  ":
                print("Found input ValueInfoProto:")
                parse_value_info(data, field_data_start, field_data_start + length, prefix + "    ")
            # If this is output (field 12 in GraphProto)
            elif field_num == 12 and prefix == "  ":
                print("Found output ValueInfoProto:")
                parse_value_info(data, field_data_start, field_data_start + length, prefix + "    ")
        elif wire_type == 5:  # 32-bit
            pos += 4
        else:
            raise ValueError(f"Unknown wire type {wire_type}")

def parse_value_info(data, start, end, prefix):
    pos = start
    name = ""
    type_start, type_end = 0, 0
    while pos < end:
        b = data[pos]
        pos += 1
        tag = b # assuming single byte field tag for low numbers
        field_num = tag >> 3
        wire_type = tag & 7
        if wire_type == 2:
            length = data[pos]
            pos += 1
            if field_num == 1: # name
                name = data[pos:pos+length].decode('utf-8')
                pos += length
            elif field_num == 2: # type (TypeProto)
                type_start = pos
                type_end = pos + length
                pos += length
        else:
            # skip
            if wire_type == 0:
                while data[pos] & 0x80:
                    pos += 1
                pos += 1
            elif wire_type == 1:
                pos += 8
            elif wire_type == 5:
                pos += 4
    
    print(f"{prefix}Name: {name}")
    if type_start > 0:
        parse_type_proto(data, type_start, type_end, prefix + "  ")

def parse_type_proto(data, start, end, prefix):
    pos = start
    while pos < end:
        b = data[pos]
        pos += 1
        tag = b
        field_num = tag >> 3
        wire_type = tag & 7
        if wire_type == 2:
            length = data[pos]
            pos += 1
            if field_num == 1: # tensor_type
                parse_tensor_type_proto(data, pos, pos + length, prefix + "  ")
                pos += length
            else:
                pos += length
        else:
            # skip
            if wire_type == 0:
                while data[pos] & 0x80:
                    pos += 1
                pos += 1
            elif wire_type == 1:
                pos += 8
            elif wire_type == 5:
                pos += 4

def parse_tensor_type_proto(data, start, end, prefix):
    pos = start
    while pos < end:
        b = data[pos]
        pos += 1
        tag = b
        field_num = tag >> 3
        wire_type = tag & 7
        if wire_type == 0:
            val = 0
            shift = 0
            while True:
                b = data[pos]
                pos += 1
                val |= (b & 0x7F) << shift
                shift += 7
                if not (b & 0x80):
                    break
            if field_num == 1: # elem_type
                print(f"{prefix}Elem Type: {val}")
        elif wire_type == 2:
            length = data[pos]
            pos += 1
            if field_num == 2: # shape (TensorShapeProto)
                parse_tensor_shape_proto(data, pos, pos + length, prefix + "  ")
                pos += length
            else:
                pos += length
        else:
            # skip
            if wire_type == 1:
                pos += 8
            elif wire_type == 5:
                pos += 4

def parse_tensor_shape_proto(data, start, end, prefix):
    pos = start
    dims = []
    while pos < end:
        b = data[pos]
        pos += 1
        tag = b
        field_num = tag >> 3
        wire_type = tag & 7
        if wire_type == 2:
            length = data[pos]
            pos += 1
            # dim (repeated Dimension)
            dim_val = parse_dimension(data, pos, pos + length)
            dims.append(dim_val)
            pos += length
        else:
            # skip
            if wire_type == 0:
                while data[pos] & 0x80:
                    pos += 1
                pos += 1
            elif wire_type == 1:
                pos += 8
            elif wire_type == 5:
                pos += 4
    print(f"{prefix}Shape: {dims}")

def parse_dimension(data, start, end):
    pos = start
    while pos < end:
        b = data[pos]
        pos += 1
        tag = b
        field_num = tag >> 3
        wire_type = tag & 7
        if wire_type == 0:
            val = 0
            shift = 0
            while True:
                b = data[pos]
                pos += 1
                val |= (b & 0x7F) << shift
                shift += 7
                if not (b & 0x80):
                    break
            if field_num == 1: # dim_value
                return val
        elif wire_type == 2:
            length = data[pos]
            pos += 1
            if field_num == 2: # dim_param
                return data[pos:pos+length].decode('utf-8')
            pos += length
    return None

if __name__ == '__main__':
    import os
    policy_path = os.path.join(os.path.dirname(__file__), "..", "policy", "policy_r1.onnx")
    with open(policy_path, 'rb') as f:
        data = f.read()
    parse_protobuf(data, 0, len(data))
