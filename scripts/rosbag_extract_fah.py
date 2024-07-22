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
from tqdm import tqdm


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


def transform_to_xy_theta(msg_tf):
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
    x = mat[0, 3]
    y = mat[1, 3]
    theta = np.arctan2(mat[1, 0], mat[0, 0])
    return np.array([x, y, theta])


def main():
    parser = argparse.ArgumentParser(description="Extract data from rosbag")
    parser.add_argument(
        "--filename",
        default="/home/daizhirui/Data/long_succ4_till_no_path.bag",
        # required=True,
        type=str,
        help="rosbag file",
    )
    parser.add_argument("--output-filename", type=str, help="output file")
    args = parser.parse_args()
    filename = args.filename
    output_filename = args.output_filename
    bag = rosbag.Bag(filename, "r")
    # list all topics
    type_and_topic_info_list = bag.get_type_and_topic_info()
    max_topic_path_len = max([len(topic_path) for topic_path in type_and_topic_info_list.topics])
    for topic in type_and_topic_info_list.topics:
        msg_type = type_and_topic_info_list.topics[topic].msg_type
        topic = " " * (max_topic_path_len - len(topic)) + topic
        print(topic, msg_type, sep="    ")

    # get transform
    tf_buffer = tf2.BufferCore(rospy.Duration(1000000000))
    topics = ["/tf", "/tf_static"]
    static_frame_ids = dict()
    frame_ids = dict()
    for topic, msg, time_stamp in tqdm(
        bag.read_messages(topics=topics),
        ncols=80,
        desc="Loading TF",
        total=max(bag.get_message_count(topic) for topic in topics),
    ):
        if topic == "/tf_static":
            for msg_tf_raw in msg.transforms:
                static_frame_ids[msg_tf_raw.header.frame_id] = msg_tf_raw.child_frame_id
                msg_tf = load_transform_stamped(msg_tf_raw)
                tf_buffer.set_transform_static(msg_tf, "default_authority")
        else:
            for msg_tf_raw in msg.transforms:
                frame_ids[msg_tf_raw.header.frame_id] = msg_tf_raw.child_frame_id
                msg_tf = load_transform_stamped(msg_tf_raw)
                tf_buffer.set_transform(msg_tf, "default_authority")

    # get lidar data
    time_stamps = []
    header_time_stamps = []
    seqs = []
    lidar_angles = []
    lidar_ranges = []
    lidar_poses = []
    topic = "/front/scan"
    for topic, msg, time_stamp in tqdm(
        bag.read_messages(topics=[topic]),
        ncols=80,
        desc="Loading Lidar",
        total=bag.get_message_count(topic),
    ):
        # tqdm.write(f"msg.header.frame_id: {msg.header.frame_id}")
        try:
            pose = tf_buffer.lookup_transform_core("map", msg.header.frame_id, time_stamp)
            pose = transform_to_xy_theta(pose)
            lidar_poses.append(pose)
        except Exception as e:
            tqdm.write(str(e))
            break
        time_stamps.append(time_stamp.to_nsec())
        header_time_stamps.append(msg.header.stamp.to_nsec())
        seqs.append(msg.header.seq)
        lidar_angles.append(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment))
        lidar_ranges.append(np.array(msg.ranges))

    # save data
    data = np.concatenate(
        [
            np.array(seqs).reshape(-1, 1).astype(np.float64),
            np.array(time_stamps).reshape(-1, 1).astype(np.float64),
            np.array(header_time_stamps).reshape(-1, 1).astype(np.float64),
            np.array(lidar_poses).reshape(-1, 3).astype(np.float64),
            np.array(lidar_angles).astype(np.float64),
            np.array(lidar_ranges).astype(np.float64),
        ],
        axis=1,
    )
    order = np.argsort(data[:, 0])  # sort by seq
    data = np.ascontiguousarray(data[order, :])

    if output_filename is None:
        output_filename = filename[:-4]

    # np.savetxt(output_filename + ".csv", data, delimiter=",")
    with open(output_filename + ".dat", "wb") as f:
        f.write(np.array(data.size).astype(np.int64).tobytes())
        f.write(np.array(data.shape[::-1]).astype(np.int64).tobytes())  # for Eigen default column major
        f.write(data.tobytes())


if __name__ == "__main__":
    main()
