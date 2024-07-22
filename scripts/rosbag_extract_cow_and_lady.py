import sys

python_minor_version = sys.version_info.minor
for path in [
    f"/opt/ros/noetic/lib/python3.{python_minor_version}/site-packages",
    "/opt/ros/noetic/lib/python3/dist-packages",
    f"/usr/lib/python3.{python_minor_version}/site-packages",
    "/usr/lib/python3/dist-packages",
]:
    if path not in sys.path:
        sys.path.append(path)

import argparse
import rospy
import rosbag
import tf2_py as tf2
import geometry_msgs
import std_msgs
import numpy as np
import tf_conversions
from sensor_msgs import point_cloud2
from tqdm import tqdm
import os
import cv2


def get_transform_map(seq, time_stamp):
    msg_tf = geometry_msgs.msg.TransformStamped()
    msg_tf.header = std_msgs.msg.Header()
    msg_tf.header.frame_id = "map"
    msg_tf.header.seq = seq
    msg_tf.header.stamp = time_stamp
    msg_tf.child_frame_id = "vicon"
    msg_tf.transform.translation.x = 0.0
    msg_tf.transform.translation.y = 0.0
    msg_tf.transform.translation.z = 0.0
    msg_tf.transform.rotation.x = 0.0
    msg_tf.transform.rotation.y = 0.0
    msg_tf.transform.rotation.z = 0.0
    msg_tf.transform.rotation.w = 1.0
    return msg_tf


T_V_C = np.array(
    [
        [0.971048, -0.120915, 0.206023, 0.00114049],
        [0.15701, 0.973037, -0.168959, 0.0450936],
        [-0.180038, 0.196415, 0.96385, 0.0430765],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
quaternion = tf_conversions.toTf(tf_conversions.fromMatrix(T_V_C))[1]


def get_transform_camera(seq, time_stamp):
    msg_tf = geometry_msgs.msg.TransformStamped()
    msg_tf.header = std_msgs.msg.Header()
    msg_tf.header.frame_id = "kinect"
    msg_tf.header.seq = seq
    msg_tf.header.stamp = time_stamp
    msg_tf.child_frame_id = "camera_rgb_optical_frame"
    msg_tf.transform.translation.x = T_V_C[0, 3]
    msg_tf.transform.translation.y = T_V_C[1, 3]
    msg_tf.transform.translation.z = T_V_C[2, 3]
    msg_tf.transform.rotation.x = quaternion[0]
    msg_tf.transform.rotation.y = quaternion[1]
    msg_tf.transform.rotation.z = quaternion[2]
    msg_tf.transform.rotation.w = quaternion[3]
    return msg_tf


def load_transform_stamped(msg_tf_raw):
    msg_tf = geometry_msgs.msg.TransformStamped()
    msg_tf.header = std_msgs.msg.Header()
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


def transform_to_matrix(msg_tf):
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


# example: python rosbag_extract_cow_and_lady.py --filename <cow_and_lady_dir>/data.bag --output-dir <cow_and_lady_dir>


def main():
    parser = argparse.ArgumentParser(description="Extract data from rosbag")
    parser.add_argument("--filename", type=str, help="rosbag file", default="data.bag")
    parser.add_argument("--output-dir", type=str, help="output directory", default=".")
    args = parser.parse_args()
    filename = args.filename
    output_dir = os.path.realpath(args.output_dir)
    color_output_dir = os.path.join(output_dir, "color")
    pcd_output_dir = os.path.join(output_dir, "pcd")
    if not os.path.exists(color_output_dir):
        os.makedirs(color_output_dir, exist_ok=True)
    if not os.path.exists(pcd_output_dir):
        os.makedirs(pcd_output_dir, exist_ok=True)

    bag = rosbag.Bag(filename, "r")
    # list all topics
    type_and_topic_info_list = bag.get_type_and_topic_info()
    max_topic_path_len = max([len(topic_path) for topic_path in type_and_topic_info_list.topics])
    for topic in type_and_topic_info_list.topics:
        msg_type = type_and_topic_info_list.topics[topic].msg_type
        topic = " " * (max_topic_path_len - len(topic)) + topic
        print(topic, msg_type, sep="    ")

    # print example frame_ids
    tf_topic = "/kinect/vrpn_client/estimated_transform"
    tf_msg = next(bag.read_messages(topics=[tf_topic]))[1]
    print(f"tf frame_id: {tf_msg.header.frame_id} child_frame_id: {tf_msg.child_frame_id}")
    pc_topic = "/camera/depth_registered/points"
    pc_msg = next(bag.read_messages(topics=[pc_topic]))[1]
    print(f"pc frame_id: {pc_msg.header.frame_id}")

    # extract data
    # get transform
    tf_buffer = tf2.BufferCore(rospy.Duration(1000000000))
    time_stamps = []
    header_time_stamps = []
    seqs = []
    poses = []
    cnt = 0
    for topic, msg, time_stamp in tqdm(
        bag.read_messages(topics=[tf_topic]),
        total=bag.get_message_count(tf_topic),
        ncols=120,
    ):
        msg_tf = load_transform_stamped(msg)
        time_stamps.append(time_stamp.to_nsec())
        header_time_stamps.append(msg.header.stamp.to_nsec())
        seqs.append(msg.header.seq)
        poses.append(
            [
                msg_tf.transform.translation.x,
                msg_tf.transform.translation.y,
                msg_tf.transform.translation.z,
                msg_tf.transform.rotation.x,
                msg_tf.transform.rotation.y,
                msg_tf.transform.rotation.z,
                msg_tf.transform.rotation.w,
            ]
        )
        tf_buffer.set_transform(msg_tf, "default_authority")
        if cnt == 0:  # for static transform, only set once
            tf_buffer.set_transform_static(
                get_transform_map(msg_tf.header.seq, time_stamp),
                "default_authority",
            )
            tf_buffer.set_transform_static(
                get_transform_camera(msg_tf.header.seq, time_stamp),
                "default_authority",
            )
        cnt += 1

    data_float32 = np.concatenate(
        [
            np.array(time_stamps).reshape(-1, 1).astype(np.float64),
            np.array(header_time_stamps).reshape(-1, 1).astype(np.float64),
            np.array(seqs).reshape(-1, 1).astype(np.float64),
            np.array(poses).reshape(-1, 7).astype(np.float64),
        ],
        axis=1,
    )
    order = np.argsort(data_float32[:, 1])  # sort by seq
    data_float32 = np.ascontiguousarray(data_float32[order, :])
    np.savetxt(os.path.join(output_dir, "tf.csv"), data_float32, delimiter=",")

    # get point cloud
    time_stamps = []
    header_time_stamps = []
    seqs = []
    poses = []
    for topic, msg, time_stamp in tqdm(
        bag.read_messages(topics=[pc_topic]),
        total=bag.get_message_count(pc_topic),
        ncols=120,
    ):
        try:
            pose = tf_buffer.lookup_transform_core("vicon", "kinect", msg.header.stamp)
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

        img_height = msg.height
        img_width = msg.width
        time_stamps.append(time_stamp.to_nsec())
        header_time_stamps.append(msg.header.stamp.to_nsec())
        seqs.append(msg.header.seq)

        # data_float32 = np.array(np.frombuffer(msg.data, dtype=np.float32).reshape((img_height, img_width, 8)))
        data_uint8 = np.array(np.frombuffer(msg.data, dtype=np.uint8).reshape((img_height, img_width, 32)))

        # follow erl::common Eigen::MatrixX<Eigen::Vector3f> format
        pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
        # pts = np.ascontiguousarray(data_float32[:, :, :3].transpose((1, 0, 2)))
        pcd_file = os.path.join(pcd_output_dir, f"{msg.header.seq}.pcd")
        with open(pcd_file, "wb") as f:
            f.write(
                np.array(
                    [pts.shape[1], pts.shape[0], pts.size, 3, pts.shape[1] * pts.shape[0]], dtype=np.int64
                ).tobytes()
            )
            f.write(pts.tobytes("C"))

        rgb = np.ascontiguousarray(data_uint8[:, :, 16:19])
        cv2.imwrite(os.path.join(color_output_dir, f"{msg.header.seq}.png"), rgb)

    data = np.concatenate(
        [
            np.array(seqs).reshape(-1, 1).astype(np.float64),
            np.array(time_stamps).reshape(-1, 1).astype(np.float64),
            np.array(header_time_stamps).reshape(-1, 1).astype(np.float64),
            np.array(poses).reshape(-1, 7).astype(np.float64),
        ],
        axis=1,
    )
    order = np.argsort(data[:, 0])  # sort by seq
    data = np.ascontiguousarray(data[order, :])
    # follow erl::common Eigen::MatrixXd format
    with open(os.path.join(output_dir, "poses.dat"), "wb") as f:
        f.write(np.array(data.size).astype(np.int64).tobytes())
        f.write(np.array(data.shape[::-1]).astype(np.int64).tobytes())  # for Eigen default column major
        f.write(data.tobytes("C"))
    np.savetxt(os.path.join(output_dir, "poses.csv"), data, delimiter=",")  # omit points


if __name__ == "__main__":
    main()
