import argparse
import os
from typing import List

import cv2
import numpy as np
import open3d as o3d
import pandas as pd
import rosbag
import rospy
import tf2_py as tf2
import tf_conversions
import transforms3d as t3d
from geometry_msgs.msg import TransformStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tqdm import tqdm

# example: python rosbag_extract_newer_college.py \
#   --rosbag <newer_college_dir>/2021-07-01-10-37-38-quad-easy.bag \
#   --gt-pose-file <newer_college_dir>/ground_truth/gt-nc-quad-easy.csv \
#   --output-dir <processed_newer_college_dir>

oRc = np.array(
    [
        [0, -1, 0],
        [0, 0, -1],
        [1, 0, 0],
    ],
    dtype=np.float64,
)


def transform_to_matrix(msg_tf: TransformStamped) -> np.ndarray:
    position = (
        msg_tf.transform.translation.x,
        msg_tf.transform.translation.y,
        msg_tf.transform.translation.z,
    )
    quaternion = (
        msg_tf.transform.rotation.x,
        msg_tf.transform.rotation.y,
        msg_tf.transform.rotation.z,
        msg_tf.transform.rotation.w,
    )
    mat = tf_conversions.toMatrix(tf_conversions.fromTf((position, quaternion)))
    return mat  # remove last row, which is [0, 0, 0, 1]


def get_camera_optical_poses(lidar_pose: np.ndarray) -> List[np.ndarray]:
    """
    given the pose of a 360 LiDAR, compute the poses of four cameras that cover the 360 view.

    Args:
        lidar_pose: The pose of the 360 LiDAR as a 4x4 transformation matrix.
    """
    if lidar_pose.shape[0] == 3:
        lidar_pose = np.concatenate([lidar_pose, np.array([[0, 0, 0, 1]])], axis=0)
    # the four cameras are looking at -135, -45, 45, 135 degrees
    camera_poses = []
    for i in range(4):
        angle = i * np.pi / 2 - np.pi * 3 / 4
        pose = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0, 0],
                [np.sin(angle), np.cos(angle), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        pose[:3, :3] = pose[:3, :3] @ oRc.T  # camera -> optical
        camera_pose = lidar_pose @ pose  # world -> lidar -> camera -> optical
        camera_poses.append(camera_pose)
    return camera_poses


def get_ring_indices(lidar_points: np.ndarray) -> np.ndarray:
    indices = np.zeros(lidar_points.shape[0], dtype=np.int32)
    indices.fill(-1)

    dist = np.linalg.norm(lidar_points, axis=1)
    valid_mask = (dist > 0) & np.isfinite(dist)
    lidar_points = lidar_points[valid_mask]
    dist = dist[valid_mask]
    dirs = lidar_points / dist[:, np.newaxis]
    azimuths = np.arctan2(dirs[:, 1], dirs[:, 0])  # [-pi, pi)
    elevations = np.arcsin(dirs[:, 2])  # [-pi/2, pi/2]
    azimuth_res = 2 * np.pi / 1024.0
    elevation_res = np.pi / 256.0  # 90 degrees / 128 lines
    azimuth_indices = np.floor((azimuths + np.pi) / azimuth_res).astype(np.int32) % 1024
    elevation_indices = np.floor((elevations + np.pi / 4) / elevation_res).astype(np.int32) % 128

    # print(f"azimuth_indices min: {azimuth_indices.min()}, max: {azimuth_indices.max()}")
    # print(f"elevation_indices min: {elevation_indices.min()}, max: {elevation_indices.max()}")

    ring_indices = (127 - elevation_indices) * 1024 + azimuth_indices
    indices[valid_mask] = ring_indices
    return indices


def reorder_with_ring_indices(data: np.ndarray, ring_indices: np.ndarray) -> np.ndarray:
    reordered_data = np.zeros_like(data)
    valid_mask = ring_indices >= 0
    reordered_data[ring_indices[valid_mask]] = data[valid_mask]
    return reordered_data


def get_depth_images(
    lidar_points: np.ndarray,
    image_width: int,
    image_height: int,
    camera_intrinsic: np.ndarray,
) -> List[np.ndarray]:
    """
    given a point cloud from 360-LiDAR, generate the corresponding 4 depth images that have the same observation.

    Args:
        lidar_points: The point cloud from the 360-LiDAR as a Nx3 array, in the LiDAR frame.
        image_width: The width of the output depth images.
        image_height: The height of the output depth images.
        camera_intrinsic: The intrinsic parameters of the camera as a 3x3 matrix.

    Returns:
        A list of 4 depth images corresponding to the 4 camera poses.
    """
    dist = np.linalg.norm(lidar_points, axis=1)

    valid_mask = (dist > 0) & np.isfinite(dist)
    lidar_points = lidar_points[valid_mask]
    dist = dist[valid_mask]

    # a = azimuth, e = elevation
    # direction = [cos(a) * cos(e), sin(a) * cos(e), sin(e)]
    azimuths = np.arctan2(lidar_points[:, 1], lidar_points[:, 0])  # [-pi, pi)
    # elevations = np.arcsin(lidar_points[:, 2] / dist)  # [-pi/2, pi/2]

    # each camera has a horizontal FOV of 90 degrees.
    # azimuth of cam0: [-180, -90)
    # azimuth of cam1: [-90, 0)
    # azimuth of cam2: [0, 90)
    # azimuth of cam3: [90, 180)

    cam_indices = (azimuths / (np.pi / 2) + 2).astype(np.int32)  # [0, 3]
    assert np.all(cam_indices >= 0) and np.all(cam_indices < 4)

    depth_images = []
    for i in range(4):
        cam_mask = cam_indices == i
        if np.any(cam_mask):
            lidar_points_cam = lidar_points[cam_mask]

            angle = i * np.pi / 2 - np.pi * 3 / 4
            rotation = np.array(
                [
                    [np.cos(angle), -np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0, 0, 1],
                ]
            )
            lidar_points_cam = lidar_points_cam @ rotation  # rotate to camera frame
            lidar_points_cam = lidar_points_cam @ oRc.T  # rotate to optical frame

            pixels = np.stack(
                [
                    lidar_points_cam[:, 0] / lidar_points_cam[:, 2],
                    lidar_points_cam[:, 1] / lidar_points_cam[:, 2],
                    np.ones_like(lidar_points_cam[:, 0]),  # homogeneous coordinates
                ],
                axis=0,
            )
            pixels = (camera_intrinsic @ pixels).astype(np.int32)
            mask = (pixels[0] >= 0) & (pixels[0] < image_width) & (pixels[1] >= 0) & (pixels[1] < image_height)
            pixels = pixels[:, mask]
            lidar_points_cam = lidar_points_cam[mask]
            depth_image = np.zeros((image_height, image_width), dtype=np.float32)
            depth_image[pixels[1], pixels[0]] = lidar_points_cam[:, 2]
            depth_images.append(depth_image)
        else:
            depth_images.append(np.zeros((image_height, image_width)))

    return depth_images


def depth_image_to_points(depth_image: np.ndarray, camera_intrinsic: np.ndarray) -> np.ndarray:
    """
    Convert a depth image to 3D points in camera coordinates.

    Args:
        depth_image: The depth image as a 2D array.
        camera_intrinsic: The intrinsic parameters of the camera as a 3x3 matrix.

    Returns:
        A Nx3 array of 3D points in camera coordinates.
    """
    # Get the pixel coordinates
    height, width = depth_image.shape
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    u = u.flatten()
    v = v.flatten()

    # Get the depth values
    z = depth_image[v, u] / 1000.0  # convert from mm to m

    # filter
    mask = z > 0
    u = u[mask]
    v = v[mask]
    z = z[mask]

    # Compute the 3D points in camera coordinates
    pts = np.stack([u, v, np.ones_like(u)], axis=-1)
    mat = np.linalg.inv(camera_intrinsic)
    pts = pts @ mat.T
    pts = pts * z[:, np.newaxis]  # scale by depth

    return pts


def load_transform_stamped(msg_tf_raw):
    msg_tf = TransformStamped()
    msg_tf.header = Header()
    msg_tf.header.frame_id = msg_tf_raw.header.frame_id
    msg_tf.header.seq = msg_tf_raw.header.seq
    msg_tf.header.stamp = msg_tf_raw.header.stamp

    msg_tf.child_frame_id = msg_tf_raw.child_frame_id

    msg_tf.transform.translation.x = msg_tf_raw.transform.translation.x
    msg_tf.transform.translation.y = msg_tf_raw.transform.translation.y
    msg_tf.transform.translation.z = msg_tf_raw.transform.translation.z
    msg_tf.transform.rotation.x = msg_tf_raw.transform.rotation.x
    msg_tf.transform.rotation.y = msg_tf_raw.transform.rotation.y
    msg_tf.transform.rotation.z = msg_tf_raw.transform.rotation.z
    msg_tf.transform.rotation.w = msg_tf_raw.transform.rotation.w
    return msg_tf


def main():
    parser = argparse.ArgumentParser(description="Extract data from rosbag")
    parser.add_argument("--rosbag", type=str, help="rosbag file", required=True)
    parser.add_argument(
        "--sensor-topic",
        type=str,
        help="sensor topic",
        default="/robot/dlio/odom_node/pointcloud/deskewed",
    )
    parser.add_argument("--save-clean", action="store_true", help="save cleaned point cloud")
    parser.add_argument("--global-pcd-downsample", type=int, default=10, help="downsample global point cloud")
    parser.add_argument("--global-min-distance", type=float, default=0.6, help="min distance for global point cloud")
    parser.add_argument("--global-max-distance", type=float, default=50.0, help="max distance for global point cloud")
    parser.add_argument("--output-dir", type=str, help="output directory", default=".")
    args = parser.parse_args()

    rosbag_file = args.rosbag
    sensor_topic = args.sensor_topic
    output_dir = os.path.realpath(args.output_dir)

    ply_output_dir = os.path.join(output_dir, "ply")
    if not os.path.exists(ply_output_dir):
        os.makedirs(ply_output_dir, exist_ok=True)

    depth_output_dir = os.path.join(output_dir, "depth")
    if not os.path.exists(depth_output_dir):
        for i in range(4):
            os.makedirs(os.path.join(depth_output_dir, f"cam{i}"), exist_ok=True)

    # intensity_output_dir = os.path.join(output_dir, "intensity")
    # if not os.path.exists(intensity_output_dir):
    #     os.makedirs(intensity_output_dir, exist_ok=True)

    range_output_dir = os.path.join(output_dir, "range")
    if not os.path.exists(range_output_dir):
        os.makedirs(range_output_dir, exist_ok=True)

    bag = rosbag.Bag(rosbag_file, "r")
    # list all topics
    type_and_topic_info_list = bag.get_type_and_topic_info()
    max_topic_path_len = max([len(topic_path) for topic_path in type_and_topic_info_list.topics])
    for topic in type_and_topic_info_list.topics:
        msg_type = type_and_topic_info_list.topics[topic].msg_type
        topic = " " * (max_topic_path_len - len(topic)) + topic
        print(topic, msg_type, sep="    ")

    # get sensor frame id
    msg_pc: PointCloud2 = next(bag.read_messages(topics=[sensor_topic]))[1]
    sensor_frame_id = msg_pc.header.frame_id
    print(f"sensor topic: {sensor_topic}")
    print(f"sensor frame_id: {sensor_frame_id}")
    print(f"msg_pc.height: {msg_pc.height}, msg_pc.width: {msg_pc.width}")

    # build tf buffer
    tf_buffer = tf2.BufferCore(rospy.Duration(1000000000))
    for topic, msg, time_stamp in tqdm(
        bag.read_messages(topics=["/tf", "/tf_static"]),
        ncols=80,
        desc="Building TF buffer",
        total=max(bag.get_message_count("/tf"), bag.get_message_count("/tf_static")),
    ):
        if topic == "/tf_static":
            for msg_tf_raw in msg.transforms:
                msg_tf = load_transform_stamped(msg_tf_raw)
                tf_buffer.set_transform_static(msg_tf, "default_authority")
        else:
            for msg_tf_raw in msg.transforms:
                msg_tf = load_transform_stamped(msg_tf_raw)
                tf_buffer.set_transform(msg_tf, "default_authority")

    camera_intrinsic = np.array(
        [
            [128, 0, 127.5],
            [0, 64, 63.5],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )
    np.savetxt(os.path.join(output_dir, "camera_intrinsic.csv"), camera_intrinsic, delimiter=",")

    # save pose and point cloud data
    lidar_poses = []
    lidar_poses16 = []
    camera_poses = [[] for _ in range(4)]
    camera_poses16 = [[] for _ in range(4)]
    all_pts = []
    all_pts_by_depth = []
    seq = 0
    plt_image = None
    import matplotlib.pyplot as plt
    # the point cloud is published in the world frame
    world_frame_id = "odom"
    scan_frame_id = "robot/odom"
    sensor_frame_id = "robot/dlio/os_sensor"
    for topic, msg_pc, time_stamp in tqdm(
        bag.read_messages(topics=[sensor_topic]),
        total=bag.get_message_count(sensor_topic),
        ncols=120,
    ):
        try:
            pose: TransformStamped = tf_buffer.lookup_transform_core(
                world_frame_id, sensor_frame_id, msg_pc.header.stamp
            )
            lidar_poses.append(
                [
                    pose.transform.translation.x,
                    pose.transform.translation.y,
                    pose.transform.translation.z,
                    pose.transform.rotation.x,
                    pose.transform.rotation.y,
                    pose.transform.rotation.z,
                    pose.transform.rotation.w,
                ]
            )
            lidar_pose = transform_to_matrix(pose)
            lidar_poses16.append(lidar_pose.flatten())
            pose: TransformStamped = tf_buffer.lookup_transform_core(
                scan_frame_id, sensor_frame_id, msg_pc.header.stamp
            )
            lidar_pose_in_scan = transform_to_matrix(pose)
        except Exception as e:
            tqdm.write(f"Error: {e}")
            continue

        data = list(
            point_cloud2.read_points(
                msg_pc,
                field_names=("x", "y", "z"),
                # field_names=("x", "y", "z", "intensity", "ring", "range"),
                skip_nans=False,
            )
        )
        data = np.array(data, dtype=np.float64)

        # transform points to world frame
        scan_pose = lidar_pose @ np.linalg.inv(lidar_pose_in_scan)  # world -> scan
        translation = scan_pose[:3, 3].reshape(1, 3)
        rotation = scan_pose[:3, :3]
        pts_world = data[:, :3]
        pts_world = pts_world @ rotation.T + translation  # scan -> world

        # we also need to transform the points back to the lidar frame
        translation = lidar_pose_in_scan[:3, 3].reshape(1, 3)
        rotation = lidar_pose_in_scan[:3, :3]
        pts_local = data[:, :3]
        pts_local = (pts_local - translation) @ rotation  # p' = R^T * (p - t), pts has shape (N, 3)

        distances = np.linalg.norm(pts_local, axis=1)
        mask = np.isfinite(distances)
        distances[~mask] = 0.0  # set inf to 0
        mask = mask & (distances > 0)

        # save point cloud
        pcd = o3d.geometry.PointCloud()
        if args.save_clean:
            pts_tmp = pts_local[mask][:, :3]
            if len(pts_tmp) == 0:
                tqdm.write(f"Skipping seq {seq} due to no valid points")
                continue
            pcd.points = o3d.utility.Vector3dVector(pts_tmp)
        else:
            pcd.points = o3d.utility.Vector3dVector(pts_local[:, :3])
        pcd_file = os.path.join(ply_output_dir, f"{seq:04d}.ply")
        o3d.io.write_point_cloud(pcd_file, pcd, write_ascii=False)  # non-ascii makes reading faster!

        # save depth images and camera poses
        cam_poses = get_camera_optical_poses(lidar_pose)
        depth_images = get_depth_images(pts_local[:, :3], 256, 128, camera_intrinsic)

        # visualize
        img = np.hstack(depth_images[::-1])
        if plt_image is None:
            plt_image = plt.imshow(img, cmap="jet")
        else:
            plt_image.set_data(img)
            # update color scale
            plt_image.set_clim(vmin=0, vmax=np.max(img))
        plt.pause(0.01)

        for i in range(4):
            cam_pose = cam_poses[i]
            quat = t3d.quaternions.mat2quat(cam_pose[:3, :3])
            camera_poses[i].append(
                [
                    cam_pose[0, 3],
                    cam_pose[1, 3],
                    cam_pose[2, 3],
                    quat[1],
                    quat[2],
                    quat[3],
                    quat[0],  # w
                ]
            )
            camera_poses16[i].append(cam_pose.flatten())

            depth_image = (depth_images[i] * 1000.0).astype(np.uint16)  # convert to mm
            cv2.imwrite(os.path.join(depth_output_dir, f"cam{i}", f"{seq:04d}.png"), depth_image)

            pts_cam = depth_image_to_points(depth_image, camera_intrinsic)
            pts_world_from_depth = pts_cam @ cam_pose[:3, :3].T + cam_pose[:3, [3]].T
            if args.global_pcd_downsample > 1:
                n = len(pts_world_from_depth) // args.global_pcd_downsample
                indices = np.random.permutation(len(pts_world_from_depth))[:n]
                pts_world_from_depth = pts_world_from_depth[indices]
            all_pts_by_depth.append(pts_world_from_depth)

        # add points to the global point cloud
        if args.global_min_distance > 0:
            mask = distances > args.global_min_distance
            pts_world = pts_world[mask]
            distances = distances[mask]
            if len(pts_world) == 0:
                tqdm.write(f"Skipping adding seq {seq} to the global point cloud due to min distance filter")
                continue

        if args.global_max_distance < float("inf"):
            mask = distances < args.global_max_distance
            pts_world = pts_world[mask]
            if len(pts_world) == 0:
                tqdm.write(f"Skipping adding seq {seq} to the global point cloud due to max distance filter")
                continue

        if args.global_pcd_downsample > 1:
            n = len(pts_world) // args.global_pcd_downsample
            indices = np.random.permutation(len(pts_world))[:n]
            pts_world = pts_world[indices]

        all_pts.append(pts_world)

        seq += 1

    lidar_poses = np.array(lidar_poses, dtype=np.float64)
    filepath = os.path.join(output_dir, "poses.csv")
    np.savetxt(filepath, lidar_poses, delimiter=",")
    print(f"Saved {len(lidar_poses)} LiDAR poses to {filepath}")

    for i, camera_poses_i in enumerate(camera_poses):
        camera_poses_i = np.array(camera_poses_i, dtype=np.float64)
        filepath = os.path.join(output_dir, f"camera_poses_{i}.csv")
        np.savetxt(filepath, camera_poses_i, delimiter=",")
        print(f"Saved {len(camera_poses_i)} camera poses to {filepath}")

    lidar_poses16 = np.array(lidar_poses16, dtype=np.float64)
    filepath = os.path.join(output_dir, "traj.txt")
    np.savetxt(filepath, lidar_poses16, delimiter=" ")
    print(f"Saved {len(lidar_poses16)} LiDAR poses to {filepath}")

    for i, camera_poses16_i in enumerate(camera_poses16):
        camera_poses16_i = np.array(camera_poses16_i, dtype=np.float64)
        filepath = os.path.join(output_dir, f"camera_traj_{i}.txt")
        np.savetxt(filepath, camera_poses16_i, delimiter=" ")
        print(f"Saved {len(camera_poses16_i)} camera poses to {filepath}")

    all_pts = np.concatenate(all_pts, axis=0)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_pts)
    pcd_file = os.path.join(output_dir, "all_points.ply")
    o3d.io.write_point_cloud(pcd_file, pcd, write_ascii=False)
    print(f"Saved {len(all_pts)} points to {pcd_file}")

    all_pts_by_depth = np.concatenate(all_pts_by_depth, axis=0)
    pcd_by_depth = o3d.geometry.PointCloud()
    pcd_by_depth.points = o3d.utility.Vector3dVector(all_pts_by_depth)
    pcd_by_depth_file = os.path.join(output_dir, "all_points_by_depth.ply")
    o3d.io.write_point_cloud(pcd_by_depth_file, pcd_by_depth, write_ascii=False)
    print(f"Saved {len(all_pts_by_depth)} points to {pcd_by_depth_file}")

    return


if __name__ == "__main__":
    main()
