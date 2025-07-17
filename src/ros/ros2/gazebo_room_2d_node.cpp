#include "erl_geometry/gazebo_room_2d.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <memory>

static const std::filesystem::path kProjectRootDir = ERL_GEOMETRY_ROOT_DIR;
static const std::filesystem::path kDataDir = kProjectRootDir / "data";

class GazeboRoom2dNode : public rclcpp::Node {

    erl::geometry::GazeboRoom2D::TrainDataLoader m_data_loader_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_pub_scan_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pub_pose_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_pub_path_;
    rclcpp::TimerBase::SharedPtr m_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_broadcaster_;
    long m_frame_index_ = 0;
    sensor_msgs::msg::LaserScan m_msg_scan_;
    geometry_msgs::msg::PoseStamped m_msg_pose_;
    geometry_msgs::msg::TransformStamped m_msg_transform_;
    nav_msgs::msg::Path m_msg_path_;

public:
    GazeboRoom2dNode()
        : Node("gazebo_room_2d_node"),
          m_data_loader_(kDataDir / "gazebo") {
        RCLCPP_INFO(
            this->get_logger(),
            "Gazebo Room 2D: min=[%f, %f], max=[%f, %f]",
            erl::geometry::GazeboRoom2D::kMapMin[0],
            erl::geometry::GazeboRoom2D::kMapMin[1],
            erl::geometry::GazeboRoom2D::kMapMax[0],
            erl::geometry::GazeboRoom2D::kMapMax[1]);

        std::string laser_frame = "front_laser";
        std::string map_frame = "map";
        std::string laser_topic_name = "scan";
        std::string pose_topic_name = "pose";
        std::string path_topic_name = "path";
        double publish_rate = 10.0;  // Hz

        // Declare parameters with default values
        this->declare_parameter("laser_frame", laser_frame);
        this->declare_parameter("map_frame", map_frame);
        this->declare_parameter("laser_topic_name", laser_topic_name);
        this->declare_parameter("pose_topic_name", pose_topic_name);
        this->declare_parameter("path_topic_name", path_topic_name);
        this->declare_parameter("publish_rate", publish_rate);

        // Get parameters
        laser_frame = this->get_parameter("laser_frame").as_string();
        map_frame = this->get_parameter("map_frame").as_string();
        laser_topic_name = this->get_parameter("laser_topic_name").as_string();
        pose_topic_name = this->get_parameter("pose_topic_name").as_string();
        path_topic_name = this->get_parameter("path_topic_name").as_string();
        publish_rate = this->get_parameter("publish_rate").as_double();

        // Initialize message headers
        m_msg_scan_.header.frame_id = laser_frame;      // reference frame for the scan
        m_msg_pose_.header.frame_id = map_frame;        // reference frame for the pose
        m_msg_transform_.header.frame_id = map_frame;   // parent frame for the transform
        m_msg_transform_.child_frame_id = laser_frame;  // child frame for the transform
        m_msg_path_.header.frame_id = map_frame;        // reference frame for the path
        m_msg_path_.poses.reserve(m_data_loader_.size());

        // Initialize publishers
        m_pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(laser_topic_name, 1);
        m_pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_name, 1);
        m_pub_path_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_name, 1);

        // Initialize transform broadcaster
        m_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Create timer
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);
        m_timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&GazeboRoom2dNode::CallbackTimer, this));

        RCLCPP_INFO(this->get_logger(), "Gazebo Room 2D Node started.");
    }

private:
    void
    CallbackTimer() {
        if (m_frame_index_ >= m_data_loader_.size()) {
            RCLCPP_WARN(this->get_logger(), "No more frames to publish.");
            return;
        }

        auto &frame = m_data_loader_[m_frame_index_];
        auto now = this->now();

        // Set transform
        m_msg_transform_.transform.translation.x = frame.translation[0];
        m_msg_transform_.transform.translation.y = frame.translation[1];
        m_msg_transform_.transform.translation.z = 0.0;
        const double yaw = std::atan2(frame.rotation(1, 0), frame.rotation(0, 0));
        m_msg_transform_.transform.rotation.w = std::cos(yaw / 2.0);
        m_msg_transform_.transform.rotation.x = 0.0;
        m_msg_transform_.transform.rotation.y = 0.0;
        m_msg_transform_.transform.rotation.z = std::sin(yaw / 2.0);
        m_msg_transform_.header.stamp = now;
        // Note: ROS2 doesn't have seq field, it's handled automatically
        m_broadcaster_->sendTransform(m_msg_transform_);

        // Set pose
        m_msg_pose_.header.stamp = now;
        m_msg_pose_.pose.position.x = m_msg_transform_.transform.translation.x;
        m_msg_pose_.pose.position.y = m_msg_transform_.transform.translation.y;
        m_msg_pose_.pose.position.z = 0.0;
        m_msg_pose_.pose.orientation = m_msg_transform_.transform.rotation;
        m_pub_pose_->publish(m_msg_pose_);

        // Set path
        m_msg_path_.header.stamp = now;
        m_msg_path_.poses.emplace_back(m_msg_pose_);
        m_pub_path_->publish(m_msg_path_);

        // Set laser scan
        m_msg_scan_.header.stamp = now;
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
        m_pub_scan_->publish(m_msg_scan_);

        m_frame_index_++;
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboRoom2dNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
