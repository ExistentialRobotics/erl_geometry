#include <filesystem>
#include <open3d/io/TriangleMeshIO.h>

#include "erl_common/test_helper.hpp"
#include "erl_geometry/azimuth_elevation.hpp"
#include "erl_geometry/euler_angle.hpp"
#include "erl_geometry/point_occupancy_octree.hpp"
#include "erl_geometry/lidar_3d.hpp"
#include "erl_geometry/utils.hpp"
#include "erl_geometry/open3d_visualizer_wrapper.hpp"

using namespace erl::common;
using namespace erl::geometry;

struct UserData {
    double octree_resolution = 0.05;
    std::size_t stride = 5;

    std::shared_ptr<PointOccupancyOctree::Drawer::Setting> drawer_setting = nullptr;
    std::shared_ptr<Open3dVisualizerWrapper::Setting> visualizer_setting = nullptr;
    std::shared_ptr<PointOccupancyOctree> octree = nullptr;
    std::shared_ptr<PointOccupancyOctree::Drawer> drawer = nullptr;
    std::shared_ptr<Open3dVisualizerWrapper> visualizer = nullptr;
    std::shared_ptr<open3d::geometry::LineSet> rays = std::make_shared<open3d::geometry::LineSet>();
    std::shared_ptr<open3d::geometry::TriangleMesh> pos_mesh = nullptr;
    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries = PointOccupancyOctree::Drawer::GetBlankGeometries();

    // cast ray setting
    double azimuth_min = -M_PI;
    double azimuth_max = M_PI;
    double elevation_min = -M_PI / 2;
    double elevation_max = M_PI / 2;
    int num_azimuth_lines = 360;
    int num_elevation_lines = 181;
    bool ignore_unknown = false;
    double max_range = -1;

    Eigen::Matrix3Xd ray_directions;

    UserData() {
        drawer_setting = std::make_shared<PointOccupancyOctree::Drawer::Setting>();
        visualizer_setting = std::make_shared<Open3dVisualizerWrapper::Setting>();

        double d = (azimuth_max - azimuth_min) / num_azimuth_lines;
        Eigen::VectorXd azimuth_angles = Eigen::VectorXd::LinSpaced(num_azimuth_lines, azimuth_min, azimuth_max - d);
        Eigen::VectorXd elevation_angles = Eigen::VectorXd::LinSpaced(num_elevation_lines, elevation_min, elevation_max);
        ray_directions.resize(3, num_azimuth_lines * num_elevation_lines);
        int idx = 0;
        for (int i = 0; i < num_azimuth_lines; ++i) {
            for (int j = 0; j < num_elevation_lines; ++j) { ray_directions.col(idx++) << AzimuthElevationToDirection(azimuth_angles[i], elevation_angles[j]); }
        }
        geometries.emplace_back(rays);
        pos_mesh = open3d::geometry::TriangleMesh::CreateSphere(0.1);
        pos_mesh->PaintUniformColor({0, 0, 1});
        geometries.emplace_back(pos_mesh);
    }

    void
    ApplySettings() {
        ERL_ASSERTM(octree != nullptr, "octree is nullptr");
        octree->GetMetricMinMax(
            drawer_setting->area_min[0],
            drawer_setting->area_min[1],
            drawer_setting->area_min[2],
            drawer_setting->area_max[0],
            drawer_setting->area_max[1],
            drawer_setting->area_max[2]);
        drawer_setting->occupied_only = true;
        drawer = std::make_shared<PointOccupancyOctree::Drawer>(drawer_setting);
        visualizer_setting->x = (drawer_setting->area_min[0] + drawer_setting->area_max[0]) / 2.;
        visualizer_setting->y = (drawer_setting->area_min[1] + drawer_setting->area_max[1]) / 2.;
        visualizer_setting->z = (drawer_setting->area_min[2] + drawer_setting->area_max[2]) / 2.;
        visualizer = std::make_shared<Open3dVisualizerWrapper>(visualizer_setting);
        drawer->SetOctree(octree);
    }
};

static UserData g_user_data;

void
VisualizerUpdateCallback(Open3dVisualizerWrapper *visualizer, open3d::visualization::Visualizer *vis) {
    double &px = visualizer->GetSetting()->x;
    double &py = visualizer->GetSetting()->y;
    double &pz = visualizer->GetSetting()->z;
    double &roll = visualizer->GetSetting()->roll;
    double &pitch = visualizer->GetSetting()->pitch;
    double &yaw = visualizer->GetSetting()->yaw;

    auto t0 = std::chrono::high_resolution_clock::now();
    std::vector<long> hit_ray_indices;
    std::vector<Eigen::Vector3d> hit_positions;
    std::vector<const PointOccupancyOctreeNode *> hit_nodes;
    std::vector<uint32_t> node_depths;
    g_user_data.octree->CastRays(
        Eigen::Vector3d(px, py, pz),
        EulerToRotation3D(roll, pitch, yaw, EulerAngleOrder::kRxyz) * g_user_data.ray_directions,
        g_user_data.ignore_unknown,
        g_user_data.max_range,
        true,
        true,
        hit_ray_indices,
        hit_positions,
        hit_nodes,
        node_depths);
    auto t1 = std::chrono::high_resolution_clock::now();
    double duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
    ERL_INFO("ray casting takes %f ms", duration);

    g_user_data.rays->points_.clear();
    g_user_data.rays->lines_.clear();
    g_user_data.rays->points_.emplace_back(px, py, pz);
    for (auto &hit_node : hit_nodes) {
        auto &points = hit_node->GetPoints();
        for (auto &point: points) {
            g_user_data.rays->points_.emplace_back(point);
            g_user_data.rays->lines_.emplace_back(0, long(g_user_data.rays->points_.size()) - 1);
        }
    }
    if (g_user_data.rays->lines_.empty()) { g_user_data.rays->lines_.emplace_back(0, 0); }
    g_user_data.rays->PaintUniformColor({1, 0, 0});
    vis->UpdateGeometry(g_user_data.rays);

    Eigen::Vector3d translation = -g_user_data.pos_mesh->GetCenter();
    translation[0] += px;
    translation[1] += py;
    translation[2] += pz;
    g_user_data.pos_mesh->Translate(translation);
    vis->UpdateGeometry(g_user_data.pos_mesh);
    vis->UpdateRender();
}

TEST(PointOccupancyOctree, Build) {
    GTEST_PREPARE_OUTPUT_DIR();

    std::string mesh_file = (gtest_src_dir / "house_expo_room_1451.ply").string();
    std::string traj_file = (gtest_src_dir / "house_expo_room_1451.csv").string();

    auto mesh_legacy = open3d::io::CreateMeshFromFile(mesh_file);
    auto mesh = open3d::t::geometry::TriangleMesh::FromLegacy(*mesh_legacy);
    auto o3d_scene = std::make_shared<open3d::t::geometry::RaycastingScene>();
    o3d_scene->AddTriangles(mesh);

    auto octree_setting = std::make_shared<PointOccupancyOctree::Setting>();
    octree_setting->resolution = g_user_data.octree_resolution;
    octree_setting->log_odd_max = 10.0;
    octree_setting->probability_hit = 0.95;   // log_odd_hit = 3
    octree_setting->probability_miss = 0.49;  // log_odd_miss = 0
    octree_setting->max_num_points_per_node = 3;
    // once hit, the cell will be occupied almost forever
    auto octree = std::make_shared<PointOccupancyOctree>(octree_setting);

    g_user_data.drawer_setting->area_min = mesh_legacy->GetMinBound();
    g_user_data.drawer_setting->area_max = mesh_legacy->GetMaxBound();
    g_user_data.drawer_setting->occupied_only = true;

    std::filesystem::path ot_file = test_output_dir / "house_expo_room_1451_3d.ot";
    if (std::filesystem::exists(ot_file)) {
        octree->LoadData(ot_file.string());
    } else {
        std::size_t pose_idx = 0;
        auto lidar_setting = std::make_shared<Lidar3D::Setting>();
        auto lidar = Lidar3D(lidar_setting, o3d_scene);
        Eigen::MatrixX<Eigen::Vector3d> ray_directions = lidar.GetRayDirectionsInFrame();
        Eigen::MatrixXd traj_2d = LoadEigenMatrixFromTextFile<double>(traj_file, EigenTextFormat::kCsvFmt).transpose();
        std::cout << traj_2d.rows() << " " << traj_2d.cols() << std::endl;
        std::vector<Eigen::Matrix4d> path_3d = ConvertPath2dTo3d(traj_2d, 1.0);
        double mean_insert_time = 0;
        long cnt_insert = 0;
        while (pose_idx < path_3d.size()) {
            Eigen::Matrix4d &pose = path_3d[pose_idx];
            pose_idx += g_user_data.stride;
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
            mean_insert_time += duration;
            cnt_insert++;
            std::cout << "Insert time: " << duration << " ms." << std::endl;
            std::cout << "Number of points: " << points.cols() << std::endl;
        }
        mean_insert_time /= double(cnt_insert);
        std::cout << "Mean insert time: " << mean_insert_time << " ms." << std::endl;
        EXPECT_TRUE(octree->Write(ot_file.string()));
        EXPECT_TRUE(octree->WriteBinary((test_output_dir / "house_expo_room_1451_3d.bt").string()));
    }
    g_user_data.visualizer_setting->window_name = "PointOccupancyOctree_Build";
    g_user_data.octree = octree;
    g_user_data.ApplySettings();

    std::shared_ptr<open3d::geometry::PointCloud> point_cloud = std::make_shared<open3d::geometry::PointCloud>();
    auto it = octree->BeginLeaf();
    auto end = octree->EndLeaf();
    for (; it != end; ++it) {
        auto &node_points = it->GetPoints();
        point_cloud->points_.insert(point_cloud->points_.end(), node_points.begin(), node_points.end());
    }
    std::cout << "Number of points: " << point_cloud->points_.size() << std::endl;

    g_user_data.geometries.push_back(point_cloud);
    g_user_data.drawer->DrawLeaves(g_user_data.geometries);
    g_user_data.visualizer->AddGeometries(g_user_data.geometries);
    g_user_data.visualizer->SetUpdateCallback(VisualizerUpdateCallback);
    g_user_data.visualizer->Show();
}
