from erl_geometry import MarchingCubes

#
import open3d as o3d
import numpy as np


def test_marching_cubes():
    radius = 1.0
    grid_size = 71
    metric_min = -2
    metric_max = 2
    metric_res = (metric_max - metric_min) / grid_size

    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=100)
    o3d.io.write_triangle_mesh("sphere.ply", sphere)

    x = np.linspace(metric_min, metric_max, grid_size)
    x, y, z = np.meshgrid(x, x, x, indexing="ij")  # row-major
    points = np.stack([x.flatten(), y.flatten(), z.flatten()], axis=0)  # (3, N)
    sdf = np.linalg.norm(points, axis=0) - radius  # (N,)

    valid_cubes = MarchingCubes.collect_valid_cubes(
        [grid_size, grid_size, grid_size],
        sdf,
        iso_value=0.0,
        row_major=True,
        parallel=True,
    )

    # remove some cubes that have z > 0.8
    z_idx = int((0.8 - metric_min) / metric_res)
    valid_cubes = [[cube for cube in cubes if cube.coords[0] < z_idx] for cubes in valid_cubes]

    vertices, triangles, face_normals = MarchingCubes.process_valid_cubes(
        valid_cubes,
        [metric_min, metric_min, metric_min],
        [metric_res, metric_res, metric_res],
        [grid_size, grid_size, grid_size],
        sdf,
        iso_value=0.0,
        row_major=True,
        parallel=True,
    )

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)
    mesh.triangle_normals = o3d.utility.Vector3dVector(face_normals)

    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

    pass


if __name__ == "__main__":
    test_marching_cubes()
