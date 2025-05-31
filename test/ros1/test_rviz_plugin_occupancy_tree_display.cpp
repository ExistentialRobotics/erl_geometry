#include "erl_common/eigen.hpp"
#include "erl_geometry/house_expo_map_lidar_2d.hpp"
#include "erl_geometry/lidar_2d.hpp"
#include "erl_geometry/lidar_3d.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/ros_msgs/occupancy_tree_msg.hpp"
#include "erl_geometry/utils.hpp"

#include <open3d/io/TriangleMeshIO.h>
#include <ros/ros.h>

const static std::filesystem::path kProjectDir = ERL_GEOMETRY_ROOT_DIR;
#define AZIMUTH_MIN         (-M_PI)
#define AZIMUTH_MAX         M_PI
#define ELEVATION_MIN       (-M_PI / 2)
#define ELEVATION_MAX       (M_PI / 2)
#define NUM_AZIMUTH_LINES   360
#define NUM_ELEVATION_LINES 181
#define OCTREE_RESOLUTION   0.1
#define QUADTREE_RESOLUTION 0.05

template<typename Dtype>
class TestOccupancyTreeGridDisplayNode3D {

    using Lidar3D = erl::geometry::Lidar3D<Dtype>;
    using Matrix3X = Eigen::Matrix3X<Dtype>;
    using MatrixX = Eigen::MatrixX<Dtype>;
    using Matrix4 = Eigen::Matrix4<Dtype>;
    using Matrix3 = Eigen::Matrix3<Dtype>;
    using Vector3 = Eigen::Vector3<Dtype>;
    using OccupancyOctree = erl::geometry::OccupancyOctree<Dtype>;

    ros::NodeHandle m_nh_;
    ros::Publisher m_pub_;
    ros::Timer m_timer_;
    std::shared_ptr<Lidar3D> m_lidar_ = nullptr;
    std::vector<Matrix4> m_path_3d_;
    std::shared_ptr<OccupancyOctree> m_octree_ = nullptr;
    int m_traj_index_ = 0;
    bool m_publish_binary_msg_ = false;

public:
    TestOccupancyTreeGridDisplayNode3D(ros::NodeHandle& nh)
        : m_nh_(nh) {

        m_nh_.param("publish_binary_msg", m_publish_binary_msg_, false);
        if (m_publish_binary_msg_) {
            ROS_INFO("Publish binary occupancy tree message");
        } else {
            ROS_INFO("Publish complete occupancy tree message");
        }

        // load data
        std::string mesh_file = kProjectDir / "data/house_expo_room_1451.ply";
        std::string traj_file = kProjectDir / "data/house_expo_room_1451.csv";

        auto mesh_legacy = open3d::io::CreateMeshFromFile(mesh_file);
        auto mesh = open3d::t::geometry::TriangleMesh::FromLegacy(*mesh_legacy);
        auto o3d_scene = std::make_shared<open3d::t::geometry::RaycastingScene>();
        o3d_scene->AddTriangles(mesh);

        auto lidar_setting = std::make_shared<typename Lidar3D::Setting>();
        lidar_setting->azimuth_min = AZIMUTH_MIN;
        lidar_setting->azimuth_max = AZIMUTH_MAX;
        lidar_setting->elevation_min = ELEVATION_MIN;
        lidar_setting->elevation_max = ELEVATION_MAX;
        lidar_setting->num_azimuth_lines = NUM_AZIMUTH_LINES;
        lidar_setting->num_elevation_lines = NUM_ELEVATION_LINES;
        m_lidar_ = std::make_shared<Lidar3D>(lidar_setting, o3d_scene);

        using namespace erl::common;
        m_path_3d_ = erl::geometry::ConvertPath2dTo3d<Dtype>(
            LoadEigenMatrixFromTextFile<Dtype>(traj_file, EigenTextFormat::kCsvFmt).transpose(),
            1.0);

        auto octree_setting = std::make_shared<typename OccupancyOctree::Setting>();
        octree_setting->resolution = OCTREE_RESOLUTION;
        octree_setting->log_odd_max = 10.0;
        octree_setting->SetProbabilityHit(0.95);   // log_odd_hit = 3
        octree_setting->SetProbabilityMiss(0.49);  // log_odd_miss = 0
        octree_setting->use_change_detection = true;
        m_octree_ = std::make_shared<OccupancyOctree>(octree_setting);

        // publish topic, ros timer
        m_pub_ = m_nh_.advertise<erl_geometry::OccupancyTreeMsg>("tree", 1, true);
        m_timer_ = m_nh_.createTimer(
            ros::Duration(0.01),
            &TestOccupancyTreeGridDisplayNode3D::CallbackTimer,
            this);
        ROS_INFO("TestOccupancyTreeGridDisplayNode3D initialized");
    }

private:
    void
    CallbackTimer(const ros::TimerEvent& /* event */) {
        if (m_traj_index_ >= static_cast<int>(m_path_3d_.size())) {
            ROS_WARN("Trajectory index out of range");
            return;
        }
        const Matrix4 pose = m_path_3d_[m_traj_index_++];
        const Matrix3 orientation = pose.template topLeftCorner<3, 3>();
        Vector3 sensor_origin = pose.template topRightCorner<3, 1>();
        MatrixX ranges;
        Matrix3X points;
        Eigen::MatrixX<Vector3> ray_directions = m_lidar_->GetRayDirectionsInFrame();
        {
            ERL_BLOCK_TIMER_MSG("Scan time");
            ranges = m_lidar_->Scan(orientation, sensor_origin);
            if (ranges.size() == 0) {
                ROS_WARN("No points scanned");
                return;
            }
            points.resize(3, ranges.size());
            long cnt_points = 0;
            for (long i = 0; i < ranges.rows(); ++i) {
                for (long j = 0; j < ranges.cols(); ++j) {
                    const Dtype& range = ranges(i, j);
                    if (!std::isfinite(range)) { continue; }
                    Vector3 point = sensor_origin + range * orientation * ray_directions(i, j);
                    points.col(cnt_points++) = point;
                }
            }
            points.conservativeResize(3, cnt_points);
        }
        {
            ERL_BLOCK_TIMER_MSG("Insert time");
            constexpr bool parallel = true;
            constexpr bool lazy_eval = true;
            constexpr bool discrete = true;
            m_octree_->ClearChangedKeys();
            m_octree_->InsertPointCloud(points, sensor_origin, -1, parallel, lazy_eval, discrete);
            if (lazy_eval) {
                m_octree_->UpdateInnerOccupancy();
                m_octree_->Prune();
            }
        }
        {
            ERL_BLOCK_TIMER_MSG("Publish time");
            using namespace erl_geometry;
            using namespace erl::geometry;
            OccupancyTreeMsg msg;
            bool ok = SaveToOccupancyTreeMsg<Dtype>(m_octree_, m_publish_binary_msg_, msg);
            if (!ok) {
                ROS_WARN("Failed to save occupancy tree to message");
                return;
            }
            msg.header.frame_id = "map";
            msg.header.stamp = ros::Time::now();
            msg.header.seq = m_traj_index_;
            m_pub_.publish(msg);
        }
    }
};

template<typename Dtype>
class TestOccupancyTreeGridDisplayNode2D {

    using Lidar2D = erl::geometry::Lidar2D;
    using Matrix2X = Eigen::Matrix2X<Dtype>;
    using Vector2 = Eigen::Vector2<Dtype>;
    using OccupancyQuadtree = erl::geometry::OccupancyQuadtree<Dtype>;

    ros::NodeHandle m_nh_;
    ros::Publisher m_pub_;
    ros::Timer m_timer_;
    std::shared_ptr<erl::geometry::HouseExpoMapLidar2D> m_map_ = nullptr;
    std::shared_ptr<OccupancyQuadtree> m_tree_ = nullptr;
    long m_traj_index_ = 0;
    bool m_publish_binary_msg_ = false;

public:
    TestOccupancyTreeGridDisplayNode2D(ros::NodeHandle& nh)
        : m_nh_(nh) {

        m_nh_.param("publish_binary_msg", m_publish_binary_msg_, false);
        if (m_publish_binary_msg_) {
            ROS_INFO("Publish binary occupancy tree message");
        } else {
            ROS_INFO("Publish complete occupancy tree message");
        }

        // load data
        std::string map_file = kProjectDir / "data/house_expo_room_1451.json";
        std::string traj_file = kProjectDir / "data/house_expo_room_1451.csv";
        m_map_ = std::make_shared<erl::geometry::HouseExpoMapLidar2D>(
            map_file,
            traj_file,
            0.2 /* wall_thickness */,
            std::make_shared<Lidar2D::Setting>(),
            false /* add_noise */,
            0.01 /* noise_std */);
        Eigen::Vector2d map_min = m_map_->GetMapMin();
        Eigen::Vector2d map_max = m_map_->GetMapMax();
        m_map_->Translate(-(map_min + map_max) / 2.0);

        // create occupancy quadtree
        auto quadtree_setting = std::make_shared<typename OccupancyQuadtree::Setting>();
        quadtree_setting->resolution = QUADTREE_RESOLUTION;
        quadtree_setting->log_odd_max = 10.0;
        quadtree_setting->SetProbabilityHit(0.95);   // log_odd_hit = 3
        quadtree_setting->SetProbabilityMiss(0.49);  // log_odd_miss = 0
        quadtree_setting->use_change_detection = true;
        m_tree_ = std::make_shared<OccupancyQuadtree>(quadtree_setting);

        // publish topic, ros timer
        m_pub_ = m_nh_.advertise<erl_geometry::OccupancyTreeMsg>("tree", 1, true);
        m_timer_ = m_nh_.createTimer(
            ros::Duration(0.1),
            &TestOccupancyTreeGridDisplayNode2D::CallbackTimer,
            this);
        ROS_INFO("TestOccupancyTreeGridDisplayNode2D initialized");
    }

private:
    void
    CallbackTimer(const ros::TimerEvent& /* event */) {
        if (m_traj_index_ >= m_map_->Size()) {
            ROS_WARN("Trajectory index out of range");
            return;
        }
        const auto& [rotation, translation, angles, ranges] = (*m_map_)[m_traj_index_++];
        Matrix2X points(2, ranges.size());
        {
            ERL_BLOCK_TIMER_MSG("Scan time");
            long cnt = 0;
            for (long k = 0; k < ranges.size(); ++k) {
                const Dtype& angle = angles[k];
                const Dtype& range = ranges[k];
                if (!std::isfinite(range)) { continue; }
                points.col(cnt) = translation.cast<Dtype>() +
                                  rotation.cast<Dtype>() *
                                      Vector2(std::cos(angle) * range, std::sin(angle) * range);
                ++cnt;
            }
            points.conservativeResize(2, cnt);
        }
        {
            ERL_BLOCK_TIMER_MSG("Insert time");
            constexpr bool parallel = true;
            constexpr bool lazy_eval = true;
            constexpr bool discrete = true;
            m_tree_->InsertPointCloud(
                points,
                translation.cast<Dtype>(),
                -1,
                parallel,
                lazy_eval,
                discrete);
            if (lazy_eval) {
                m_tree_->UpdateInnerOccupancy();
                m_tree_->Prune();
            }
        }
        {
            ERL_BLOCK_TIMER_MSG("Publish time");
            using namespace erl_geometry;
            using namespace erl::geometry;
            OccupancyTreeMsg msg;
            bool ok = SaveToOccupancyTreeMsg<Dtype>(m_tree_, m_publish_binary_msg_, msg);
            if (!ok) {
                ROS_WARN("Failed to save occupancy tree to message");
                return;
            }
            msg.header.frame_id = "map";
            msg.header.stamp = ros::Time::now();
            msg.header.seq = m_traj_index_;
            m_pub_.publish(msg);
        }
    }
};

struct EntryNode {
    std::shared_ptr<void> m_node_ = nullptr;

    EntryNode(ros::NodeHandle& nh) {
        bool is_double = false;
        nh.param("is_double", is_double, false);
        bool is_3d = true;
        nh.param("is_3d", is_3d, true);
        if (is_double) {
            if (is_3d) {
                m_node_ = std::make_shared<TestOccupancyTreeGridDisplayNode3D<double>>(nh);
            } else {
                m_node_ = std::make_shared<TestOccupancyTreeGridDisplayNode2D<double>>(nh);
            }
        } else {
            if (is_3d) {
                m_node_ = std::make_shared<TestOccupancyTreeGridDisplayNode3D<float>>(nh);
            } else {
                m_node_ = std::make_shared<TestOccupancyTreeGridDisplayNode2D<float>>(nh);
            }
        }
    }
};

int
main(int argc, char** argv) {
    ros::init(argc, argv, "test_occupancy_tree_grid_display_node");
    ros::NodeHandle nh("~");
    EntryNode node(nh);
    ros::spin();
    return 0;
}
