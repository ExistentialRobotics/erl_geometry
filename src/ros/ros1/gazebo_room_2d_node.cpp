#include "erl_geometry/gazebo_room_2d.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

static const std::filesystem::path kProjectRootDir = ERL_GEOMETRY_ROOT_DIR;
static const std::filesystem::path kDataDir = kProjectRootDir / "data";

class GazeboRoom2dNode {

    struct Setting {
        std::string laser_frame = "front_laser";
        std::string map_frame = "map";
        std::string laser_topic_name = "scan";
        std::string pose_topic_name = "pose";
        std::string path_topic_name = "path";
        double publish_rate = 10.0;  // Hz
    };

    Setting m_setting_;
    erl::geometry::GazeboRoom2D::TrainDataLoader m_data_loader_;
    ros::NodeHandle m_nh_;
    ros::Publisher m_pub_scan_;
    ros::Publisher m_pub_pose_;
    ros::Publisher m_pub_path_;
    ros::Timer m_timer_;
    tf2_ros::TransformBroadcaster m_broadcaster_;
    long m_frame_index_ = 0;
    sensor_msgs::LaserScan m_msg_scan_;
    geometry_msgs::PoseStamped m_msg_pose_;
    geometry_msgs::TransformStamped m_msg_transform_;
    nav_msgs::Path m_msg_path_;

public:
    GazeboRoom2dNode(ros::NodeHandle &nh)
        : m_data_loader_(kDataDir / "gazebo"),
          m_nh_(nh) {
        ROS_INFO(
            "Gazebo Room 2D: min=[%f, %f], max=[%f, %f]",
            erl::geometry::GazeboRoom2D::kMapMin[0],
            erl::geometry::GazeboRoom2D::kMapMin[1],
            erl::geometry::GazeboRoom2D::kMapMax[0],
            erl::geometry::GazeboRoom2D::kMapMax[1]);
        // Initialize the settings
        m_nh_.param("laser_frame", m_setting_.laser_frame, m_setting_.laser_frame);
        m_nh_.param("map_frame", m_setting_.map_frame, m_setting_.map_frame);
        m_nh_.param("publish_rate", m_setting_.publish_rate, m_setting_.publish_rate);
        m_msg_scan_.header.frame_id = m_setting_.laser_frame;      // reference frame for the scan
        m_msg_pose_.header.frame_id = m_setting_.map_frame;        // reference frame for the pose
        m_msg_transform_.header.frame_id = m_setting_.map_frame;   // parent frame for the transform
        m_msg_transform_.child_frame_id = m_setting_.laser_frame;  // child frame for the transform
        m_msg_path_.header.frame_id = m_setting_.map_frame;        // reference frame for the path
        m_msg_path_.poses.reserve(m_data_loader_.size());
        // Initialize the node
        m_pub_scan_ = m_nh_.advertise<sensor_msgs::LaserScan>(m_setting_.laser_topic_name, 1);
        m_pub_pose_ = m_nh_.advertise<geometry_msgs::PoseStamped>(m_setting_.pose_topic_name, 1);
        m_pub_path_ = m_nh_.advertise<nav_msgs::Path>(m_setting_.path_topic_name, 1);
        m_timer_ = m_nh_.createTimer(
            ros::Duration(1.0 / m_setting_.publish_rate),
            &GazeboRoom2dNode::CallbackTimer,
            this);
        ROS_INFO("Gazebo Room 2D Node started.");
    }

    void
    CallbackTimer(const ros::TimerEvent &) {
        if (m_frame_index_ >= m_data_loader_.size()) {
            ROS_WARN("%s: No more frames to publish.", m_nh_.getNamespace().c_str());
            return;
        }

        auto &frame = m_data_loader_[m_frame_index_];

        m_msg_transform_.transform.translation.x = frame.translation[0];
        m_msg_transform_.transform.translation.y = frame.translation[1];
        m_msg_transform_.transform.translation.z = 0.0;
        const double yaw = std::atan2(frame.rotation(1, 0), frame.rotation(0, 0));
        m_msg_transform_.transform.rotation.w = std::cos(yaw / 2.0);
        m_msg_transform_.transform.rotation.x = 0.0;
        m_msg_transform_.transform.rotation.y = 0.0;
        m_msg_transform_.transform.rotation.z = std::sin(yaw / 2.0);
        m_msg_transform_.header.stamp = ros::Time::now();
        m_msg_transform_.header.seq = m_frame_index_;
        m_broadcaster_.sendTransform(m_msg_transform_);

        m_msg_pose_.header.stamp = m_msg_transform_.header.stamp;
        m_msg_pose_.header.seq = m_frame_index_;
        m_msg_pose_.pose.position.x = m_msg_transform_.transform.translation.x;
        m_msg_pose_.pose.position.y = m_msg_transform_.transform.translation.y;
        m_msg_pose_.pose.position.z = 0.0;
        m_msg_pose_.pose.orientation = m_msg_transform_.transform.rotation;
        m_pub_pose_.publish(m_msg_pose_);

        m_msg_path_.header.stamp = m_msg_transform_.header.stamp;
        m_msg_path_.header.seq = m_frame_index_;
        m_msg_path_.poses.emplace_back(m_msg_pose_);
        m_pub_path_.publish(m_msg_path_);

        m_msg_scan_.header.stamp = m_msg_transform_.header.stamp;
        m_msg_scan_.header.seq = m_frame_index_++;
        m_msg_scan_.angle_min = frame.angles[0];
        m_msg_scan_.angle_max = frame.angles[269];
        m_msg_scan_.angle_increment = frame.angles[1] - frame.angles[0];
        m_msg_scan_.time_increment = 0.0;
        m_msg_scan_.scan_time = 0.0;
        m_msg_scan_.range_min = std::numeric_limits<float>::infinity();
        m_msg_scan_.range_max = -std::numeric_limits<float>::infinity();
        m_msg_scan_.ranges.resize(frame.ranges.size());
        m_msg_scan_.intensities.resize(frame.ranges.size(), 1.0f);
        for (long i = 0; i < frame.ranges.size(); ++i) {
            m_msg_scan_.ranges[i] = frame.ranges[i];
            m_msg_scan_.range_min = std::min(m_msg_scan_.range_min, m_msg_scan_.ranges[i]);
            m_msg_scan_.range_max = std::max(m_msg_scan_.range_max, m_msg_scan_.ranges[i]);
        }
        m_pub_scan_.publish(m_msg_scan_);
    }
};

int
main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_room_2d_node");
    ros::NodeHandle nh("~");
    GazeboRoom2dNode node(nh);
    ros::spin();
    return 0;
}
