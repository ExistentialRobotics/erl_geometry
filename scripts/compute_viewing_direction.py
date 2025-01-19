import numpy as np


def compute_viewing_direction(u, v, camera_fx, camera_fy, camera_cx, camera_cy, camera_rotation):
    xu = -(u - camera_cx) / camera_fx
    yu = -(v - camera_cy) / camera_fy
    d = np.array([1.0, xu, yu])  # in camera frame
    d = d / np.linalg.norm(d)
    d = np.dot(camera_rotation, d)
    return d
