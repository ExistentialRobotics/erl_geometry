#include "erl_common/block_timer.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/lidar_3d.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/occupancy_octree_drawer.hpp"
#include "erl_geometry/open3d_visualizer_wrapper.hpp"
#include "erl_geometry/utils.hpp"

#include <open3d/geometry/VoxelGrid.h>
#include <open3d/io/TriangleMeshIO.h>

#include <filesystem>

// parameters
#define WINDOW_NAME          "OccupancyOctree_Build"
#define OCTREE_RESOLUTION    0.1
#define AZIMUTH_MIN          (-M_PI)
#define AZIMUTH_MAX          M_PI
#define ELEVATION_MIN        (-M_PI / 2)
#define ELEVATION_MAX        (M_PI / 2)
#define NUM_AZIMUTH_LINES    360
#define NUM_ELEVATION_LINES  181
#define ANIMATION_INTERVAL   2
#define MAX_POINT_CLOUD_SIZE 1000000
#define STRIDE               1

using Dtype = float;
using Lidar3D = erl::geometry::Lidar3D<Dtype>;
using AbstractOctree = erl::geometry::AbstractOctree<Dtype>;
using OccupancyOctree = erl::geometry::OccupancyOctree<Dtype>;
using OccupancyOctreeNode = erl::geometry::OccupancyOctreeNode;
using OccupancyOctreeDrawer = erl::geometry::OccupancyOctreeDrawer<OccupancyOctree>;
using Open3dVisualizerWrapper = erl::geometry::Open3dVisualizerWrapper;
using VectorX = Eigen::VectorX<Dtype>;
using Vector3 = Eigen::Vector3<Dtype>;
using MatrixX = Eigen::MatrixX<Dtype>;
using Matrix3 = Eigen::Matrix3<Dtype>;
using Matrix3X = Eigen::Matrix3X<Dtype>;
using Matrix4 = Eigen::Matrix4<Dtype>;

TEST(OccupancyOctree, BuildProfiling) {
    GTEST_PREPARE_OUTPUT_DIR();
    using namespace erl::common;

    std::string mesh_file = (gtest_src_dir / "../../data/house_expo_room_1451.ply").string();
    std::string traj_file = (gtest_src_dir / "../../data/house_expo_room_1451.csv").string();

    auto mesh_legacy = open3d::io::CreateMeshFromFile(mesh_file);
    auto mesh = open3d::t::geometry::TriangleMesh::FromLegacy(*mesh_legacy);
    auto o3d_scene = std::make_shared<open3d::t::geometry::RaycastingScene>();
    o3d_scene->AddTriangles(mesh);

    auto lidar_setting = std::make_shared<Lidar3D::Setting>();
    auto lidar = Lidar3D(lidar_setting, o3d_scene);

    Matrix3X traj_2d =
        LoadEigenMatrixFromTextFile<Dtype>(traj_file, EigenTextFormat::kCsvFmt).transpose();
    std::cout << traj_2d.rows() << " " << traj_2d.cols() << std::endl;
    std::vector<Matrix4> path_3d = erl::geometry::ConvertPath2dTo3d<Dtype>(traj_2d, 1.0);

    auto octree_setting = std::make_shared<OccupancyOctree::Setting>();
    octree_setting->resolution = 0.01;
    octree_setting->use_change_detection = true;
    // once hit, the cell will be occupied almost forever
    auto octree = std::make_shared<OccupancyOctree>(octree_setting);

    std::size_t num_cores = std::thread::hardware_concurrency();
    double max_duration = 600.0 * 32 / static_cast<double>(num_cores);
    double max_mean_duration = 520.0 * 32 / static_cast<double>(num_cores);

    std::size_t pose_idx = 0;
    Eigen::MatrixX<Vector3> ray_directions = lidar.GetRayDirectionsInFrame();
    int n = 10;
    double mean_duration = 0;
    for (int k = 0; k < 10; ++k) {
        const Matrix4 pose = path_3d[pose_idx].cast<Dtype>();
        pose_idx += 1;
        Matrix3 orientation = pose.topLeftCorner<3, 3>();
        Vector3 sensor_origin = pose.topRightCorner<3, 1>();

        MatrixX ranges = lidar.Scan(orientation, sensor_origin);

        Matrix3X points(3, ranges.size());
        long cnt_points = 0;
        for (long i = 0; i < ranges.rows(); ++i) {
            for (long j = 0; j < ranges.cols(); ++j) {
                const Dtype &range = ranges(i, j);
                if (std::isinf(range) || std::isnan(range)) { continue; }
                Vector3 point = sensor_origin + range * orientation * ray_directions(i, j);
                points.col(cnt_points++) = point;
            }
        }
        points.conservativeResize(3, cnt_points);

        auto t0 = std::chrono::high_resolution_clock::now();
        octree->ClearChangedKeys();
        octree->InsertPointCloud(points, sensor_origin, -1, false, true, true);
        octree->UpdateInnerOccupancy();
        octree->Prune();
        auto t1 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "Insert time: " << duration << " ms." << std::endl;
        EXPECT_LE(duration, max_duration);
        mean_duration += duration;
    }
    mean_duration /= n;
    std::cout << "Mean Insert time: " << mean_duration << " ms." << std::endl;
    EXPECT_LE(mean_duration, max_mean_duration);
    // 14900K: ~480ms
    // 13700K: ~550ms
}
