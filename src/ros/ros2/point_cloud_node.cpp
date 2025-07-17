#include <open3d/io/PointCloudIO.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <filesystem>
#include <memory>

class PointCloudNode : public rclcpp::Node {

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg_;

public:
    PointCloudNode()
        : Node("point_cloud_node") {
        // Declare parameters with default values
        this->declare_parameter("point_cloud_file", "/path/to/your/pointcloud.pcd");
        this->declare_parameter("frame_id", "map");
        this->declare_parameter("publish_normals", false);
        this->declare_parameter("publish_colors", true);

        // Get parameters
        std::string point_cloud_file = this->get_parameter("point_cloud_file").as_string();
        std::string frame_id = this->get_parameter("frame_id").as_string();
        bool publish_normals = this->get_parameter("publish_normals").as_bool();
        bool publish_colors = this->get_parameter("publish_colors").as_bool();

        // Validate file path
        if (point_cloud_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Point cloud file path is not specified.");
            rclcpp::shutdown();
            return;
        }
        if (!std::filesystem::exists(point_cloud_file)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Point cloud file does not exist: %s",
                point_cloud_file.c_str());
            rclcpp::shutdown();
            return;
        }

        // Load point cloud
        open3d::geometry::PointCloud point_cloud;
        if (!open3d::io::ReadPointCloud(point_cloud_file, point_cloud)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to read point cloud from %s",
                point_cloud_file.c_str());
            rclcpp::shutdown();
            return;
        }
        if (point_cloud.IsEmpty()) {
            RCLCPP_ERROR(this->get_logger(), "Point cloud is empty.");
            rclcpp::shutdown();
            return;
        }

        // Check color availability
        if (publish_colors && !point_cloud.HasColors()) {
            RCLCPP_WARN(
                this->get_logger(),
                "Point cloud does not have colors, but publish_colors is set to true.");
            publish_colors = false;  // Disable color publishing if not available
        }

        // Convert Open3D PointCloud to ROS2 PointCloud2 message
        auto msg_pcd = std::make_shared<sensor_msgs::msg::PointCloud2>();
        msg_pcd->header.frame_id = frame_id;
        msg_pcd->header.stamp = this->now();
        msg_pcd->height = 1;                          // unorganized point cloud
        msg_pcd->width = point_cloud.points_.size();  // number of points in the cloud
        msg_pcd->is_bigendian = false;
        msg_pcd->is_dense = false;

        // Setup point cloud fields
        msg_pcd->fields.resize(3);
        msg_pcd->fields[0].name = "x";
        msg_pcd->fields[0].offset = 0;
        msg_pcd->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg_pcd->fields[0].count = 1;
        msg_pcd->fields[1].name = "y";
        msg_pcd->fields[1].offset = 4;
        msg_pcd->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg_pcd->fields[1].count = 1;
        msg_pcd->fields[2].name = "z";
        msg_pcd->fields[2].offset = 8;
        msg_pcd->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg_pcd->fields[2].count = 1;

        uint32_t offset = 12;
        if (publish_colors && point_cloud.HasColors()) {
            msg_pcd->fields.resize(4);
            msg_pcd->fields[3].name = "rgb";
            msg_pcd->fields[3].offset = 12;
            msg_pcd->fields[3].datatype =
                sensor_msgs::msg::PointField::FLOAT32;  // RGB packed in a float
            msg_pcd->fields[3].count = 1;
            offset += 4;  // 4 bytes for RGB
        }

        uint32_t normal_offset = offset;  // for normals if needed
        if (publish_normals) {
            point_cloud.EstimateNormals();
            sensor_msgs::msg::PointField field = msg_pcd->fields.back();
            field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field.count = 1;

            msg_pcd->fields.resize(msg_pcd->fields.size() + 3);

            field.name = "normal_x";
            field.offset = offset;
            msg_pcd->fields.push_back(field);

            field.name = "normal_y";
            field.offset += 4;
            msg_pcd->fields.push_back(field);

            field.name = "normal_z";
            field.offset += 4;
            msg_pcd->fields.push_back(field);

            offset += 12;
        }

        msg_pcd->point_step = offset;  // total size of one point
        msg_pcd->row_step = msg_pcd->point_step * point_cloud.points_.size();
        msg_pcd->data.resize(msg_pcd->row_step);

        // Fill the PointCloud2 message with point data
        uint8_t* ptr = msg_pcd->data.data();
        for (size_t i = 0; i < point_cloud.points_.size(); ++i) {
            Eigen::Vector3f point = point_cloud.points_[i].cast<float>();
            // x, y, z
            std::memcpy(reinterpret_cast<void*>(ptr), reinterpret_cast<const void*>(&point[0]), 12);
            if (publish_colors && point_cloud.HasColors()) {
                Eigen::Vector3f color = point_cloud.colors_[i].cast<float>();
                ptr[12] = static_cast<uint8_t>(color[2] * 255);
                ptr[13] = static_cast<uint8_t>(color[1] * 255);
                ptr[14] = static_cast<uint8_t>(color[0] * 255);
                ptr[15] = 255;
            }
            if (publish_normals) {
                Eigen::Vector3f normal = point_cloud.normals_[i].cast<float>();
                std::memcpy(
                    reinterpret_cast<void*>(ptr + normal_offset),
                    reinterpret_cast<const void*>(&normal[0]),
                    12);
            }
            ptr += msg_pcd->point_step;  // move to the next point
        }

        // Create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 1);

        // Publish the point cloud once
        publisher_->publish(*msg_pcd);

        RCLCPP_INFO(
            this->get_logger(),
            "Published point cloud with %zu points",
            point_cloud.points_.size());

        // Store the message for republishing if needed
        point_cloud_msg_ = msg_pcd;

        // Create a timer to republish periodically (optional)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PointCloudNode::CallbackTimer, this));
    }

private:
    void
    CallbackTimer() {
        if (point_cloud_msg_) {
            point_cloud_msg_->header.stamp = this->now();
            publisher_->publish(*point_cloud_msg_);
        }
    }
};

int
main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
