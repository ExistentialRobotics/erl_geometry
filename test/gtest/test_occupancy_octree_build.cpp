#include <filesystem>
#include <open3d/io/TriangleMeshIO.h>

#include "erl_common/test_helper.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/lidar_3d.hpp"
#include "erl_geometry/utils.hpp"
#include "erl_geometry/open3d_visualizer_wrapper.hpp"

static std::filesystem::path g_test_data_dir = std::filesystem::path(__FILE__).parent_path();
static std::string g_window_name = "OccupancyOctree_Build";

struct Options {
    std::string mesh_file = (g_test_data_dir / "house_expo_room_1451.ply").string();
    std::string traj_file = (g_test_data_dir / "house_expo_room_1451.csv").string();
    double octree_resolution = 0.05;
};

static Options g_options;

TEST(OccupancyOctree, Build) {
    GTEST_PREPARE_OUTPUT_DIR();
    using namespace erl::common;
    using namespace erl::geometry;

    auto mesh_legacy = open3d::io::CreateMeshFromFile(g_options.mesh_file);
    auto mesh = open3d::t::geometry::TriangleMesh::FromLegacy(*mesh_legacy);
    auto o3d_scene = std::make_shared<open3d::t::geometry::RaycastingScene>();
    o3d_scene->AddTriangles(mesh);

    auto lidar_setting = std::make_shared<Lidar3D::Setting>();
    auto lidar = Lidar3D(lidar_setting, o3d_scene);

    Eigen::MatrixXd traj_2d = LoadEigenMatrixFromTextFile<double>(g_options.traj_file, EigenTextFormat::kCsvFmt).transpose();
    std::cout << traj_2d.rows() << " " << traj_2d.cols() << std::endl;
    std::vector<Eigen::Matrix4d> path_3d = ConvertPath2dTo3d(traj_2d, 1.0);

    auto octree_setting = std::make_shared<OccupancyOctree::Setting>();
    octree_setting->resolution = g_options.octree_resolution;
    octree_setting->log_odd_max = 10.0;
    octree_setting->probability_hit = 0.95;  // log_odd_hit = 3
    octree_setting->probability_miss = 0.49;  // log_odd_miss = 0
    // once hit, the cell will be occupied almost forever
    auto octree = std::make_shared<OccupancyOctree>(octree_setting);
    Open3dVisualizerWrapper visualizer;
    visualizer.GetSetting()->window_name = g_window_name;
    auto drawer_setting = std::make_shared<OccupancyOctree::Drawer::Setting>();
    drawer_setting->area_min = mesh_legacy->GetMinBound();
    drawer_setting->area_max = mesh_legacy->GetMaxBound();
    drawer_setting->occupied_only = true;
    OccupancyOctree::Drawer drawer(drawer_setting);
    drawer.SetOctree(octree);

    std::size_t pose_idx = 0;
    Eigen::MatrixX<Eigen::Vector3d> ray_directions = lidar.GetRayDirectionsInFrame();

    while (pose_idx < path_3d.size()) {
        Eigen::Matrix4d &pose = path_3d[pose_idx++];
        Eigen::Matrix3d orientation = pose.topLeftCorner<3, 3>();
        Eigen::Vector3d sensor_origin = pose.topRightCorner<3, 1>();

        auto t0 = std::chrono::high_resolution_clock::now();
        Eigen::MatrixXd ranges = lidar.Scan(orientation, sensor_origin);
        auto t1 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "==== " << pose_idx << " ====" << std::endl  //
                  << "Scan time: " << duration << " ms." << std::endl;

        Eigen::Matrix3Xd points(3, ranges.size());
        long cnt_points = 0;
        for (long i = 0; i < ranges.rows(); ++i) {
            for (long j = 0; j < ranges.cols(); ++j) {
                double &range = ranges(i, j);
                if (std::isinf(range) || std::isnan(range)) { continue; }
                points.col(cnt_points++) = sensor_origin + range * orientation * ray_directions(i, j);
            }
        }
        points.conservativeResize(3, cnt_points);

        t0 = std::chrono::high_resolution_clock::now();
        octree->InsertPointCloud(points, sensor_origin, -1, false, false, true);
        t1 = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "Insert time: " << duration << " ms." << std::endl;
        std::cout << "Number of points: " << points.cols() << std::endl;
    }
    EXPECT_TRUE(octree->Write((test_output_dir / "house_expo_room_1451_3d.ot").string()));
    EXPECT_TRUE(octree->WriteBinary((test_output_dir / "house_expo_room_1451_3d.bt").string()));

    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries = OccupancyOctree::Drawer::GetBlankGeometries();
    drawer.DrawLeaves(geometries);
    visualizer.AddGeometries(geometries);
    visualizer.GetVisualizer()->UpdateGeometry();
    visualizer.Show();
}
