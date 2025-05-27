import pathlib
import numpy as np
import struct
import matplotlib.pyplot as plt


def read_array_from_bin(filename, dtype=np.float64):
    with open(filename, "rb") as f:
        data = f.read()
    size = struct.unpack("l", data[:8])[0]
    rows = struct.unpack("l", data[8:16])[0]
    cols = struct.unpack("l", data[16:24])[0]
    assert size == rows * cols, "size != rows * cols"
    array = np.frombuffer(data[24:], dtype=dtype, count=rows * cols).reshape((cols, rows)).T
    return array


def main():
    proj_dir = pathlib.Path(__file__).parent.parent
    data_dir = proj_dir / "data" / "gazebo"
    poses = read_array_from_bin(data_dir / "poses.dat")
    print(f"poses.shape: {poses.shape}")
    ranges = read_array_from_bin(data_dir / "ranges.dat")
    print(f"ranges.shape: {ranges.shape}")
    angles = read_array_from_bin(data_dir / "thetas.dat")
    print(f"thetas.shape: {angles.shape}")

    sin_th = np.sin(angles).flatten()
    cos_th = np.cos(angles).flatten()

    points = []
    for i in range(poses.shape[1]):
        x = poses[0, i]
        y = poses[1, i]
        theta = poses[2, i]
        pts = np.array([ranges[:, i] * cos_th, ranges[:, i] * sin_th])
        rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        pts = rotation @ pts + np.array([[x], [y]])
        points.append(pts)
    points = np.concatenate(points, axis=1)
    print(f"points.shape: {points.shape}")

    plt.figure(figsize=(10, 10))
    plt.plot(points[0, :], points[1, :], "b.", markersize=0.1)
    plt.show()


if __name__ == "__main__":
    main()
