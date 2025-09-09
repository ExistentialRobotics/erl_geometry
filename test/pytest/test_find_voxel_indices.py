import time

import torch
from pyerl_geometry import find_voxel_indices, morton_encode
import os


def test_find_voxel_indices(dim: int):
    print("Testing find_voxel_indices with dim =", dim)
    script_dir = os.path.dirname(__file__)
    data_dir = os.path.join(script_dir, "../../data")

    if dim == 3:
        points_fp = os.path.join(data_dir, "semi_sparse_octree_points.bin")
        children_fp = os.path.join(data_dir, "semi_sparse_octree_children.bin")
        node_indices_fp = os.path.join(data_dir, "semi_sparse_octree_node_indices.bin")

    else:
        points_fp = os.path.join(data_dir, "semi_sparse_quadtree_points.bin")
        children_fp = os.path.join(data_dir, "semi_sparse_quadtree_children.bin")
        node_indices_fp = os.path.join(data_dir, "semi_sparse_quadtree_node_indices.bin")

    with open(points_fp, "rb") as f:
        points = f.read()
        points = torch.frombuffer(points, dtype=torch.float64).clone().reshape(-1, dim)
    with open(children_fp, "rb") as f:
        children = f.read()
        children = torch.frombuffer(children, dtype=torch.int64).clone().reshape(-1, 2**dim)
    with open(node_indices_fp, "rb") as f:
        node_indices_gt = f.read()
        node_indices_gt = torch.frombuffer(node_indices_gt, dtype=torch.int64).clone().reshape(-1)

    resolution = 0.05
    level = 7
    coord_offset = 1 << level
    coords = (points / resolution).floor().to(torch.int32) + coord_offset
    coords = coords.to(torch.uint32)
    codes = morton_encode(coords)

    # CPU
    repeats = 10
    t0 = time.time()
    for _ in range(repeats):
        node_indices = find_voxel_indices(codes, dim, level, children, True)
    t1 = time.time()
    dt_cpu = (t1 - t0) / (points.shape[0] * repeats)
    print(f"CPU time per point: {dt_cpu*1e6:.6f} us")

    assert torch.all(node_indices == node_indices_gt)

    # GPU
    codes = codes.cuda()
    children = children.cuda()
    torch.cuda.synchronize()
    t0 = time.time()
    for _ in range(repeats):
        node_indices = find_voxel_indices(codes, dim, level, children, True)
        torch.cuda.synchronize()
    t1 = time.time()
    dt_gpu = (t1 - t0) / (points.shape[0] * repeats)
    print(f"GPU time per point: {dt_gpu*1e6:.6f} us")

    assert torch.all(node_indices.cpu() == node_indices_gt)


if __name__ == "__main__":
    test_find_voxel_indices(3)
    test_find_voxel_indices(2)
