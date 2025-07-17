#include <open3d/io/PointCloudIO.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>

#include <filesystem>

int
main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_node");
    ros::NodeHandle nh("~");

    // Load point cloud from a file
    std::string point_cloud_file = "/path/to/your/pointcloud.pcd";  // Update with your file path
    nh.param("point_cloud_file", point_cloud_file, point_cloud_file);
    if (point_cloud_file.empty()) {
        ROS_ERROR("Point cloud file path is not specified.");
        return -1;
    }
    if (!std::filesystem::exists(point_cloud_file)) {
        ROS_ERROR("Point cloud file does not exist: %s", point_cloud_file.c_str());
        return -1;
    }
    open3d::geometry::PointCloud point_cloud;
    if (!open3d::io::ReadPointCloud(point_cloud_file, point_cloud)) {
        ROS_ERROR("Failed to read point cloud from %s", point_cloud_file.c_str());
        return -1;
    }
    if (point_cloud.IsEmpty()) {
        ROS_ERROR("Point cloud is empty.");
        return -1;
    }

    std::string frame_id = "map";
    nh.param("frame_id", frame_id, frame_id);

    bool publish_normals = false;
    nh.param("publish_normals", publish_normals, publish_normals);

    bool publish_colors = true;
    nh.param("publish_colors", publish_colors, publish_colors);
    if (publish_colors && !point_cloud.HasColors()) {
        ROS_WARN("Point cloud does not have colors, but publish_colors is set to true.");
        publish_colors = false;  // Disable color publishing if not available
    }

    // Convert Open3D PointCloud to ROS PointCloud2 message
    sensor_msgs::PointCloud2 msg_pcd;
    msg_pcd.header.frame_id = frame_id;
    msg_pcd.header.stamp = ros::Time::now();
    msg_pcd.header.seq = 0;
    msg_pcd.height = 1;                          // unorganized point cloud
    msg_pcd.width = point_cloud.points_.size();  // number of points in the cloud
    msg_pcd.is_bigendian = false;
    msg_pcd.is_dense = false;

    // sensor_msgs::PointCloud2Modifier pcd_modifier(msg_pcd);
    // pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    // sensor_msgs::PointCloud2Iterator<float> iter_x(msg_pcd, "x");

    msg_pcd.fields.resize(3);
    msg_pcd.fields[0].name = "x";
    msg_pcd.fields[0].offset = 0;
    msg_pcd.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    msg_pcd.fields[0].count = 1;
    msg_pcd.fields[1].name = "y";
    msg_pcd.fields[1].offset = 4;
    msg_pcd.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    msg_pcd.fields[1].count = 1;
    msg_pcd.fields[2].name = "z";
    msg_pcd.fields[2].offset = 8;
    msg_pcd.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    msg_pcd.fields[2].count = 1;
    uint32_t offset = 12;
    if (publish_colors && point_cloud.HasColors()) {
        msg_pcd.fields.resize(4);
        msg_pcd.fields[3].name = "rgb";
        msg_pcd.fields[3].offset = 12;
        msg_pcd.fields[3].datatype = sensor_msgs::PointField::FLOAT32;  // RGB packed in a float
        msg_pcd.fields[3].count = 1;
        offset += 4;  // 4 bytes for RGB
    }
    uint32_t normal_offset = offset;  // for normals if needed
    if (publish_normals) {
        point_cloud.EstimateNormals();
        sensor_msgs::PointField field = msg_pcd.fields.back();
        field.datatype = sensor_msgs::PointField::FLOAT32;

        msg_pcd.fields.reserve(msg_pcd.fields.size() + 3);

        field.name = "normal_x";
        field.offset = offset;
        msg_pcd.fields.push_back(field);  // normal_x

        field.name = "normal_y";
        field.offset += 4;
        msg_pcd.fields.push_back(field);  // normal_y

        field.name = "normal_z";
        field.offset += 4;
        msg_pcd.fields.push_back(field);  // normal_z

        offset += 12;  // 3 normals * 4 bytes each
    }

    msg_pcd.point_step = offset;  // total size of one point
    msg_pcd.row_step = msg_pcd.point_step * point_cloud.points_.size();
    msg_pcd.data.resize(msg_pcd.row_step);

    // Fill the PointCloud2 message with point data
    uint8_t* ptr = msg_pcd.data.data();
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
        ptr += msg_pcd.point_step;  // move to the next point
    }

    // Publish the point cloud
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1, true);
    pub.publish(msg_pcd);
    ros::spin();  // Keep the node alive to publish the point cloud
    return 0;
}
