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

# example: python rosbag_extract_newer_college.py --filename <cow_and_lady_dir>/data.bag --output-dir <cow_and_lady_dir>

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
    return mat[:3]  # remove last row, which is [0, 0, 0, 1]


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


def main():
    parser = argparse.ArgumentParser(description="Extract data from rosbag")
    parser.add_argument("--rosbag", type=str, help="rosbag file", required=True)
    parser.add_argument("--gt-pose-file", type=str, help="ground truth pose file", required=True)
    parser.add_argument("--sensor-topic", type=str, help="sensor topic", default="/os_cloud_node/points")
    parser.add_argument("--save-clean", action="store_true", help="save cleaned point cloud")
    parser.add_argument("--global-pcd-downsample", type=int, default=10, help="downsample global point cloud")
    parser.add_argument("--global-min-distance", type=float, default=0.6, help="min distance for global point cloud")
    parser.add_argument("--global-max-distance", type=float, default=50.0, help="max distance for global point cloud")
    parser.add_argument("--output-dir", type=str, help="output directory", default=".")
    args = parser.parse_args()

    rosbag_file = args.rosbag
    gt_pose_file = args.gt_pose_file
    sensor_topic = args.sensor_topic
    output_dir = os.path.realpath(args.output_dir)

    ply_output_dir = os.path.join(output_dir, "ply")
    if not os.path.exists(ply_output_dir):
        os.makedirs(ply_output_dir, exist_ok=True)

    depth_output_dir = os.path.join(output_dir, "depth")
    if not os.path.exists(depth_output_dir):
        for i in range(4):
            os.makedirs(os.path.join(depth_output_dir, f"cam{i}"), exist_ok=True)

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
    df = pd.read_csv(gt_pose_file)
    tf_buffer = tf2.BufferCore(rospy.Duration(1000000000))
    for i, row in tqdm(df.iterrows(), total=len(df), ncols=120, desc="Building tf buffer"):
        msg_tf = TransformStamped()
        msg_tf.header = Header()
        msg_tf.header.frame_id = "map"
        msg_tf.header.seq = i
        msg_tf.header.stamp = rospy.Time(int(row["#sec"]), int(row["nsec"]))
        msg_tf.child_frame_id = sensor_frame_id
        msg_tf.transform.translation.x = row["x"]
        msg_tf.transform.translation.y = row["y"]
        msg_tf.transform.translation.z = row["z"]
        msg_tf.transform.rotation.x = row["qx"]
        msg_tf.transform.rotation.y = row["qy"]
        msg_tf.transform.rotation.z = row["qz"]
        msg_tf.transform.rotation.w = row["qw"]
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
    camera_poses = [[] for _ in range(4)]
    all_pts = []
    seq = 0
    for topic, msg_pc, time_stamp in tqdm(
        bag.read_messages(topics=[sensor_topic]),
        total=bag.get_message_count(sensor_topic),
        ncols=120,
    ):
        try:
            pose: TransformStamped = tf_buffer.lookup_transform_core("map", sensor_frame_id, msg_pc.header.stamp)
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
        except Exception as e:
            tqdm.write(f"Error: {e}")
            continue

        pts = list(point_cloud2.read_points(msg_pc, field_names=("x", "y", "z"), skip_nans=False))
        pts = np.array(pts, dtype=np.float64)

        distances = np.linalg.norm(pts, axis=1)
        mask = np.isfinite(distances) & (distances > 0)
        dist = np.copy(distances).reshape(128, 1024)
        dist[~np.isfinite(dist)] = 0.0  # set NaN to 0

        # save point cloud
        pcd = o3d.geometry.PointCloud()
        if args.save_clean:
            pts_tmp = pts[mask]
            if len(pts_tmp) == 0:
                tqdm.write(f"Skipping seq {seq} due to no valid points")
                continue
            pcd.points = o3d.utility.Vector3dVector(pts_tmp)
        else:
            pcd.points = o3d.utility.Vector3dVector(pts)
        pcd_file = os.path.join(ply_output_dir, f"{seq:04d}.ply")
        o3d.io.write_point_cloud(pcd_file, pcd, write_ascii=False)  # non-ascii makes reading faster!

        # save depth images and camera poses
        lidar_pose = transform_to_matrix(pose)
        cam_poses = get_camera_optical_poses(lidar_pose)
        depth_images = get_depth_images(pts, 256, 128, camera_intrinsic)

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

            depth_image = (depth_images[i] * 1000.0).astype(np.uint16)  # convert to mm
            cv2.imwrite(os.path.join(depth_output_dir, f"cam{i}", f"{seq:04d}.png"), depth_image)

        # add points to the global point cloud
        if args.global_min_distance > 0:
            mask = distances > args.global_min_distance
            pts = pts[mask]
            distances = distances[mask]
            if len(pts) == 0:
                tqdm.write(f"Skipping adding seq {seq} to the global point cloud due to min distance filter")
                continue

        if args.global_max_distance < float("inf"):
            mask = distances < args.global_max_distance
            pts = pts[mask]
            if len(pts) == 0:
                tqdm.write(f"Skipping adding seq {seq} to the global point cloud due to max distance filter")
                continue

        pts = np.dot(lidar_pose[:3, :3], pts.T).T + lidar_pose[:3, 3]

        if args.global_pcd_downsample > 1:
            n = len(pts) // args.global_pcd_downsample
            indices = np.random.permutation(len(pts))[:n]
            pts = pts[indices]

        all_pts.append(pts)

        seq += 1

    lidar_poses = np.array(lidar_poses, dtype=np.float64)
    np.savetxt(os.path.join(output_dir, "poses.csv"), lidar_poses, delimiter=",")

    for i, camera_poses_i in enumerate(camera_poses):
        camera_poses_i = np.array(camera_poses_i, dtype=np.float64)
        np.savetxt(os.path.join(output_dir, f"camera_poses_{i}.csv"), camera_poses_i, delimiter=",")

    print(f"Saved {len(lidar_poses)} poses to {os.path.join(output_dir, 'poses.csv')}")
    print(f"Saved {len(all_pts)} point clouds to {ply_output_dir}")

    all_pts = np.concatenate(all_pts, axis=0)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_pts)
    pcd_file = os.path.join(output_dir, "all_points.ply")
    o3d.io.write_point_cloud(pcd_file, pcd, write_ascii=False)

    print(f"Saved {len(all_pts)} points to {pcd_file}")

    return


if __name__ == "__main__":
    main()
