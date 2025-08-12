import argparse
import os

import cv2
import numpy as np
import open3d as o3d
import rosbag
import rospy
import tf2_py as tf2
import tf_conversions
from geometry_msgs.msg import TransformStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tqdm import tqdm

# map <--> vicon: identity
# vicon <--> kinect: /kinect/vrpn_client/estimated_transform
# kinect <--> camera_rgb_optical_frame: T_V_C


def get_transform_map(seq: int, time_stamp: rospy.Time):
    msg_tf = TransformStamped()
    msg_tf.header = Header()
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
translation = (T_V_C[0, 3], T_V_C[1, 3], T_V_C[2, 3])

print(f"{'Frame Transformations':=^160}")
print(f"                      map <--> vicon: identity")
print(f"                   vicon <--> kinect: /kinect/vrpn_client/estimated_transform")
print(f"kinect <--> camera_rgb_optical_frame: {translation}, {quaternion}")
print(f"{'='*160}")


def get_transform_camera(seq, time_stamp):
    msg_tf = TransformStamped()
    msg_tf.header = Header()
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


def load_transform_stamped(msg_tf_raw: TransformStamped) -> TransformStamped:
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


# example: python rosbag_extract_cow_and_lady.py --filename <cow_and_lady_dir>/data.bag --output-dir <cow_and_lady_dir>


def main():
    parser = argparse.ArgumentParser(description="Extract data from rosbag")
    parser.add_argument("--filename", type=str, help="rosbag file", default="data.bag")
    parser.add_argument("--output-dir", type=str, help="output directory", default=".")
    args = parser.parse_args()
    filename = args.filename
    output_dir = os.path.realpath(args.output_dir)
    color_output_dir = os.path.join(output_dir, "color")
    depth_output_dir = os.path.join(output_dir, "depth")
    pcd_output_dir = os.path.join(output_dir, "pcd")
    ply_output_dir = os.path.join(output_dir, "ply")
    if not os.path.exists(color_output_dir):
        os.makedirs(color_output_dir, exist_ok=True)
    if not os.path.exists(depth_output_dir):
        os.makedirs(depth_output_dir, exist_ok=True)
    if not os.path.exists(pcd_output_dir):
        os.makedirs(pcd_output_dir, exist_ok=True)
    if not os.path.exists(ply_output_dir):
        os.makedirs(ply_output_dir, exist_ok=True)

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
    tf_msg: TransformStamped = next(bag.read_messages(topics=[tf_topic]))[1]
    print(f"tf frame_id: {tf_msg.header.frame_id} child_frame_id: {tf_msg.child_frame_id}")
    pc_topic = "/camera/depth_registered/points"
    pc_msg: PointCloud2 = next(bag.read_messages(topics=[pc_topic]))[1]
    print(f"pc frame_id: {pc_msg.header.frame_id}")

    # extract data
    # get transform
    # build tf2 buffer
    tf_buffer = tf2.BufferCore(rospy.Duration(1000000000))
    time_stamps = []
    header_time_stamps = []
    seqs = []
    poses = []
    if os.path.exists(os.path.join(output_dir, "tf.csv")):
        data_float32 = np.loadtxt(os.path.join(output_dir, "tf.csv"), delimiter=",")
        time_stamps = data_float32[:, 0].tolist()
        header_time_stamps = data_float32[:, 1].tolist()
        seqs: list[int] = data_float32[:, 2].astype(np.int64).tolist()
        poses = data_float32[:, 3:].tolist()

        tf_buffer.set_transform_static(
            get_transform_map(seqs[0], rospy.Time.from_sec(time_stamps[0] / 1e9)),
            "default_authority",
        )
        tf_buffer.set_transform_static(
            get_transform_camera(seqs[0], rospy.Time.from_sec(time_stamps[0] / 1e9)),
            "default_authority",
        )

        for time_stamp, header_time_stamp, seq, pose in tqdm(
            zip(time_stamps, header_time_stamps, seqs, poses), total=len(time_stamps), ncols=120
        ):
            msg_tf = TransformStamped()
            msg_tf.header = Header()
            msg_tf.header.frame_id = tf_msg.header.frame_id
            msg_tf.header.seq = seq
            msg_tf.header.stamp = rospy.Time.from_sec(header_time_stamp / 1e9)
            msg_tf.child_frame_id = tf_msg.child_frame_id
            msg_tf.transform.translation.x = pose[0]
            msg_tf.transform.translation.y = pose[1]
            msg_tf.transform.translation.z = pose[2]
            msg_tf.transform.rotation.x = pose[3]
            msg_tf.transform.rotation.y = pose[4]
            msg_tf.transform.rotation.z = pose[5]
            msg_tf.transform.rotation.w = pose[6]
            tf_buffer.set_transform(msg_tf, "default_authority")

    else:
        tf_buffer.set_transform_static(
            get_transform_map(tf_msg.header.seq, tf_msg.header.stamp),
            "default_authority",
        )
        tf_buffer.set_transform_static(
            get_transform_camera(tf_msg.header.seq, tf_msg.header.stamp),
            "default_authority",
        )

        for topic, msg_tf, time_stamp in tqdm(
            bag.read_messages(topics=[tf_topic]),
            total=bag.get_message_count(tf_topic),
            ncols=120,
        ):
            topic: str
            msg_tf: TransformStamped
            time_stamp: rospy.Time

            msg_tf = load_transform_stamped(msg_tf)
            time_stamps.append(time_stamp.to_nsec())
            header_time_stamps.append(msg_tf.header.stamp.to_nsec())
            seqs.append(msg_tf.header.seq)
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
    for topic, msg_pc, time_stamp in tqdm(
        bag.read_messages(topics=[pc_topic]),
        total=bag.get_message_count(pc_topic),
        ncols=120,
    ):
        msg_pc: PointCloud2

        try:
            pose: TransformStamped = tf_buffer.lookup_transform_core("vicon", "kinect", msg_pc.header.stamp)
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

        height = msg_pc.height
        width = msg_pc.width
        time_stamps.append(time_stamp.to_nsec())
        header_time_stamps.append(msg_pc.header.stamp.to_nsec())
        seqs.append(msg_pc.header.seq)

        data_uint8 = np.array(np.frombuffer(msg_pc.data, dtype=np.uint8).reshape((height, width, 32)))

        depth = list(point_cloud2.read_points(msg_pc, field_names=("z"), skip_nans=False))
        depth = np.array(depth, dtype=np.float32)
        depth = depth.reshape((height, width))
        depth[~np.isfinite(depth)] = 0.0
        depth = (depth * 1000.0).astype(np.uint16)  # convert to mm
        cv2.imwrite(os.path.join(depth_output_dir, f"{msg_pc.header.seq}.png"), depth)

        rgb = np.ascontiguousarray(data_uint8[:, :, 16:19])
        cv2.imwrite(os.path.join(color_output_dir, f"{msg_pc.header.seq}.png"), rgb)

        pts = list(point_cloud2.read_points(msg_pc, field_names=("x", "y", "z"), skip_nans=False))
        pts = np.array(pts, dtype=np.float32)
        mask = np.isfinite(pts).all(axis=1)
        pts_valid = pts[mask].astype(np.float64)
        rgb_valid = rgb.reshape(-1, 3)[mask].astype(np.float64) / 255.0
        ply_file = os.path.join(ply_output_dir, f"{msg_pc.header.seq}.ply")
        pcd_o3d = o3d.geometry.PointCloud()
        pcd_o3d.points = o3d.utility.Vector3dVector(pts_valid)
        pcd_o3d.colors = o3d.utility.Vector3dVector(rgb_valid)
        o3d.io.write_point_cloud(ply_file, pcd_o3d, write_ascii=False)

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
