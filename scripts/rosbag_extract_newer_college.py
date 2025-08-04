import argparse
import os

import numpy as np
import rosbag
import rospy
import tf2_py as tf2
from geometry_msgs.msg import TransformStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tqdm import tqdm
import pandas as pd
import open3d as o3d
import tf_conversions


# example: python rosbag_extract_newer_college.py --filename <cow_and_lady_dir>/data.bag --output-dir <cow_and_lady_dir>


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

    # save pose and point cloud data
    poses = []
    all_pts = []
    seq = 0
    for topic, msg_pc, time_stamp in tqdm(
        bag.read_messages(topics=[sensor_topic]),
        total=bag.get_message_count(sensor_topic),
        ncols=120,
    ):
        try:
            pose: TransformStamped = tf_buffer.lookup_transform_core("map", sensor_frame_id, msg_pc.header.stamp)
            poses.append(
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

        pts = list(point_cloud2.read_points(msg_pc, field_names=("x", "y", "z"), skip_nans=args.save_clean))
        pts = np.array(pts, dtype=np.float64)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd_file = os.path.join(ply_output_dir, f"{seq:04d}.ply")
        o3d.io.write_point_cloud(pcd_file, pcd, write_ascii=True)

        distances = np.linalg.norm(pts, axis=1)
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

        mat = transform_to_matrix(pose)
        pts = np.dot(mat[:3, :3], pts.T).T + mat[:3, 3]

        if args.global_pcd_downsample > 1:
            n = len(pts) // args.global_pcd_downsample
            indices = np.random.permutation(len(pts))[:n]
            pts = pts[indices]

        all_pts.append(pts)

        seq += 1

    poses = np.array(poses, dtype=np.float64)
    np.savetxt(os.path.join(output_dir, "poses.csv"), poses, delimiter=",")

    print(f"Saved {len(poses)} poses to {os.path.join(output_dir, 'poses.csv')}")
    print(f"Saved {len(all_pts)} point clouds to {ply_output_dir}")

    all_pts = np.concatenate(all_pts, axis=0)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_pts)
    pcd_file = os.path.join(output_dir, "all_points.ply")
    o3d.io.write_point_cloud(pcd_file, pcd, write_ascii=True)

    print(f"Saved {len(all_pts)} points to {pcd_file}")

    return


if __name__ == "__main__":
    main()
