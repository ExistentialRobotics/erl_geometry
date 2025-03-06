#include "erl_common/test_helper.hpp"
#include "erl_geometry/lidar_3d.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/open3d_visualizer_wrapper.hpp"
#include "erl_geometry/utils.hpp"

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
using Open3dVisualizerWrapper = erl::geometry::Open3dVisualizerWrapper;
using VectorX = Eigen::VectorX<Dtype>;
using Vector3 = Eigen::Vector3<Dtype>;
using MatrixX = Eigen::MatrixX<Dtype>;
using Matrix3 = Eigen::Matrix3<Dtype>;
using Matrix3X = Eigen::Matrix3X<Dtype>;
using Matrix4 = Eigen::Matrix4<Dtype>;

TEST(OccupancyOctree, Build) {
    GTEST_PREPARE_OUTPUT_DIR();
    using namespace erl::common;

    std::string mesh_file = (gtest_src_dir / "../../data/house_expo_room_1451.ply").string();
    std::string traj_file = (gtest_src_dir / "../../data/house_expo_room_1451.csv").string();

    auto mesh_legacy = open3d::io::CreateMeshFromFile(mesh_file);
    auto mesh = open3d::t::geometry::TriangleMesh::FromLegacy(*mesh_legacy);
    auto o3d_scene = std::make_shared<open3d::t::geometry::RaycastingScene>();
    o3d_scene->AddTriangles(mesh);

    auto lidar_setting = std::make_shared<Lidar3D::Setting>();
    lidar_setting->azimuth_min = AZIMUTH_MIN;
    lidar_setting->azimuth_max = AZIMUTH_MAX;
    lidar_setting->elevation_min = ELEVATION_MIN;
    lidar_setting->elevation_max = ELEVATION_MAX;
    lidar_setting->num_azimuth_lines = NUM_AZIMUTH_LINES;
    lidar_setting->num_elevation_lines = NUM_ELEVATION_LINES;
    auto lidar = Lidar3D(lidar_setting, o3d_scene);

    Matrix3X traj_2d = LoadEigenMatrixFromTextFile<Dtype>(traj_file, EigenTextFormat::kCsvFmt).transpose();
    std::cout << traj_2d.rows() << " " << traj_2d.cols() << std::endl;
    std::vector<Matrix4> path_3d = erl::geometry::ConvertPath2dTo3d<Dtype>(traj_2d, 1.0);

    auto octree_setting = std::make_shared<OccupancyOctree::Setting>();
    octree_setting->resolution = OCTREE_RESOLUTION;
    octree_setting->log_odd_max = 10.0;
    octree_setting->SetProbabilityHit(0.95);   // log_odd_hit = 3
    octree_setting->SetProbabilityMiss(0.49);  // log_odd_miss = 0
    octree_setting->use_change_detection = true;
    // once hit, the cell will be occupied almost forever
    auto octree = std::make_shared<OccupancyOctree>(octree_setting);
    const auto visualizer_setting = std::make_shared<Open3dVisualizerWrapper::Setting>();
    visualizer_setting->window_name = WINDOW_NAME;
    visualizer_setting->mesh_show_back_face = false;
    Open3dVisualizerWrapper visualizer(visualizer_setting);
    auto point_cloud = std::make_shared<open3d::geometry::PointCloud>();
    auto line_set_traj = std::make_shared<open3d::geometry::LineSet>();
    auto line_set_rays = std::make_shared<open3d::geometry::LineSet>();
    auto voxel_grid = std::make_shared<open3d::geometry::VoxelGrid>();
    voxel_grid->voxel_size_ = octree_setting->resolution;
    visualizer.AddGeometries({point_cloud, line_set_traj, line_set_rays, voxel_grid});

    auto drawer_setting = std::make_shared<OccupancyOctree::Drawer::Setting>();
    drawer_setting->area_min = mesh_legacy->GetMinBound();
    drawer_setting->area_max = mesh_legacy->GetMaxBound();
    drawer_setting->occupied_only = true;
    OccupancyOctree::Drawer drawer(drawer_setting);
    drawer.SetOctree(octree);

    std::size_t pose_idx = 0;
    Eigen::MatrixX<Vector3> ray_directions = lidar.GetRayDirectionsInFrame();

    bool octree_saved = false;
    int animation_cnt = 0;
    double mean_scan_time = 0;
    double mean_insert_time = 0;
    auto callback = [&](Open3dVisualizerWrapper *wrapper, open3d::visualization::Visualizer *vis) {
        if (pose_idx >= path_3d.size()) {
            if (octree_saved) {
                ERL_WARN_ONCE("callback is still called after octree is saved.");
                return false;
            }
            EXPECT_TRUE(octree->Write((test_output_dir / "house_expo_room_1451_3d.ot").string()));
            EXPECT_TRUE(octree->WriteBinary((test_output_dir / "house_expo_room_1451_3d.bt").string()));
            octree_saved = true;
            wrapper->ClearGeometries();
            std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries = OccupancyOctree::Drawer::GetBlankGeometries();
            drawer.DrawLeaves(geometries);
            geometries.push_back(point_cloud);
            geometries.push_back(line_set_traj);
            wrapper->AddGeometries(geometries);
            vis->UpdateGeometry();
            wrapper->SetAnimationCallback(nullptr);  // stop calling this callback
            return false;
        }
        const auto t_start = std::chrono::high_resolution_clock::now();
        const Matrix4 &pose = path_3d[pose_idx].cast<Dtype>();
        pose_idx += STRIDE;
        const Matrix3 orientation = pose.topLeftCorner<3, 3>();
        Vector3 sensor_origin = pose.topRightCorner<3, 1>();

        std::cout << "==== " << pose_idx << " ====" << std::endl;
        double dt = 0;
        MatrixX ranges;
        {
            ERL_BLOCK_TIMER_MSG_TIME("Scan time", dt);
            ranges = lidar.Scan(orientation, sensor_origin);
        }
        mean_scan_time = (mean_scan_time * animation_cnt + dt) / (animation_cnt + 1);

        line_set_traj->points_.emplace_back(sensor_origin.cast<double>());
        if (line_set_traj->points_.size() > 1) { line_set_traj->lines_.emplace_back(line_set_traj->points_.size() - 2, line_set_traj->points_.size() - 1); }
        line_set_rays->points_.clear();
        line_set_rays->lines_.clear();
        line_set_rays->points_.emplace_back(sensor_origin.cast<double>());

        Matrix3X points(3, ranges.size());
        long cnt_points = 0;
        for (long i = 0; i < ranges.rows(); ++i) {
            for (long j = 0; j < ranges.cols(); ++j) {
                const Dtype &range = ranges(i, j);
                if (!std::isfinite(range)) { continue; }
                Vector3 point = sensor_origin + range * orientation * ray_directions(i, j);
                points.col(cnt_points++) = point;
                point_cloud->points_.emplace_back(point.cast<double>());
                line_set_rays->points_.emplace_back(point.cast<double>());
                line_set_rays->lines_.emplace_back(0, static_cast<long>(line_set_rays->points_.size()) - 1);
            }
        }
        points.conservativeResize(3, cnt_points);

        octree->ClearChangedKey();
        {
            {
                ERL_BLOCK_TIMER_MSG_TIME("Insert time", dt);
                // octree->InsertPointCloud(points, sensor_origin, -1, false, false, true);
                octree->InsertPointCloud(points, sensor_origin, -1, false, true, true);
            }
            octree->UpdateInnerOccupancy();
            octree->Prune();
        }
        mean_insert_time = (mean_insert_time * animation_cnt + dt) / (animation_cnt + 1);
        std::cout << "Number of points: " << points.cols() << std::endl;
        std::cout << "Mean scan time: " << mean_scan_time << " ms." << std::endl;
        std::cout << "Mean insert time: " << mean_insert_time << " ms." << std::endl;

        for (auto itr = octree->BeginChangedKey(); itr != octree->EndChangedKey(); ++itr) {
            Dtype x, y, z;
            octree->KeyToCoord(itr->first, x, y, z);
            Eigen::Vector3i voxel_index(
                std::floor(x / voxel_grid->voxel_size_),
                std::floor(y / voxel_grid->voxel_size_),
                std::floor(z / voxel_grid->voxel_size_));
            if (octree->IsNodeOccupied(octree->Search(itr->first))) {
                voxel_grid->AddVoxel(open3d::geometry::Voxel(voxel_index, Eigen::Vector3d(0.5, 0.5, 0.5)));
            } else {
                voxel_grid->RemoveVoxel(voxel_index);
            }
        }

        line_set_traj->PaintUniformColor({1, 0, 0});
        line_set_rays->PaintUniformColor({0, 1, 0});
        if (line_set_traj->lines_.empty()) { vis->ResetViewPoint(true); }
        if (point_cloud->points_.size() > MAX_POINT_CLOUD_SIZE) { point_cloud->points_.swap(point_cloud->RandomDownSample(0.5)->points_); }

        const auto t_end = std::chrono::high_resolution_clock::now();
        const auto duration_total = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        std::cout << "Callback time: " << duration_total << " ms." << std::endl;

        return animation_cnt++ % ANIMATION_INTERVAL == 0;
    };

    visualizer.SetAnimationCallback(callback);
    visualizer.Show();
}

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

    Matrix3X traj_2d = LoadEigenMatrixFromTextFile<Dtype>(traj_file, EigenTextFormat::kCsvFmt).transpose();
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
        octree->ClearChangedKey();
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
