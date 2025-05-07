import struct
import numpy as np


def read_array_from_bin(filename):
    with open(filename, "rb") as f:
        data = f.read()
    size = struct.unpack("l", data[:8])[0]
    rows = struct.unpack("l", data[8:16])[0]
    cols = struct.unpack("l", data[16:24])[0]
    assert size == rows * cols, "size != rows * cols"
    array = np.frombuffer(data[24:], dtype=np.float32, count=rows * cols).reshape((cols, rows)).T
    return array
