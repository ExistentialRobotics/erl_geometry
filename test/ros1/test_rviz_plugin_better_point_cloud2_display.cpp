#include <open3d/io/TriangleMeshIO.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <filesystem>

const static std::filesystem::path kProjectDir = ERL_GEOMETRY_ROOT_DIR;

class Node {

    ros::NodeHandle m_nh_;
    ros::Publisher m_pub_;
    ros::Timer m_timer_;
    sensor_msgs::PointCloud2 m_msg_;

public:
    Node(ros::NodeHandle& nh)
        : m_nh_(nh) {
        // Initialize the node
        m_pub_ = m_nh_.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);

        auto mesh = open3d::io::CreateMeshFromFile(kProjectDir / "data/bunny_z_up.ply");
        mesh->ComputeVertexNormals();
        m_msg_.header.frame_id = "map";
        m_msg_.height = 1;
        m_msg_.width = mesh->vertices_.size();
        m_msg_.fields.resize(6);
        m_msg_.fields[0].name = "x";
        m_msg_.fields[0].offset = 0;
        m_msg_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        m_msg_.fields[0].count = 1;  // number of elements per point
        m_msg_.fields[1].name = "y";
        m_msg_.fields[1].offset = 4;
        m_msg_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        m_msg_.fields[1].count = 1;
        m_msg_.fields[2].name = "z";
        m_msg_.fields[2].offset = 8;
        m_msg_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        m_msg_.fields[2].count = 1;
        m_msg_.fields[3].name = "normal_x";
        m_msg_.fields[3].offset = 12;
        m_msg_.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        m_msg_.fields[3].count = 1;
        m_msg_.fields[4].name = "normal_y";
        m_msg_.fields[4].offset = 16;
        m_msg_.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
        m_msg_.fields[4].count = 1;
        m_msg_.fields[5].name = "normal_z";
        m_msg_.fields[5].offset = 20;
        m_msg_.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
        m_msg_.fields[5].count = 1;
        m_msg_.is_bigendian = false;
        m_msg_.point_step = 24;  // size of one point in bytes
        m_msg_.row_step = m_msg_.point_step * m_msg_.width;
        m_msg_.data.resize(m_msg_.row_step);
        m_msg_.is_dense = true;
        char* data_ptr = reinterpret_cast<char*>(m_msg_.data.data());
        for (size_t i = 0; i < mesh->vertices_.size(); ++i) {
            const Eigen::Vector3f vertex = mesh->vertices_[i].cast<float>();
            const Eigen::Vector3f normal = mesh->vertex_normals_[i].cast<float>();
            std::memcpy(data_ptr, &vertex[0], sizeof(float) * 3);
            std::memcpy(data_ptr + 12, &normal[0], sizeof(float) * 3);
            data_ptr += m_msg_.point_step;
        }
        m_msg_.header.stamp = ros::Time::now();
        m_msg_.header.seq = 0;
        m_pub_.publish(m_msg_);
        m_timer_ = m_nh_.createTimer(ros::Duration(0.1), &Node::Callback, this);
        ROS_INFO("Published point cloud with %zu points", mesh->vertices_.size());
    }

private:
    void
    Callback(const ros::TimerEvent& /* event */) {
        m_msg_.header.stamp = ros::Time::now();
        m_msg_.header.seq++;
        m_pub_.publish(m_msg_);
    }
};

int
main(int argc, char** argv) {
    ros::init(argc, argv, "test_rviz_plugin_better_point_cloud2_display");
    ros::NodeHandle nh("~");
    Node node(nh);
    ros::spin();
    return 0;
}
