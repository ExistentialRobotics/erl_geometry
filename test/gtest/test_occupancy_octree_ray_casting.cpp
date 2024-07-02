#include "erl_common/angle_utils.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/euler_angle.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/open3d_visualizer_wrapper.hpp"

#include <open3d/geometry/LineSet.h>

using namespace erl::common;
using namespace erl::geometry;

struct UserData {
    std::shared_ptr<OccupancyOctree::Drawer::Setting> drawer_setting = std::make_shared<OccupancyOctree::Drawer::Setting>();
    std::shared_ptr<Open3dVisualizerWrapper::Setting> visualizer_setting = std::make_shared<Open3dVisualizerWrapper::Setting>();
    std::shared_ptr<OccupancyOctree> octree = nullptr;
    std::shared_ptr<OccupancyOctree::Drawer> drawer = nullptr;
    std::shared_ptr<Open3dVisualizerWrapper> visualizer = nullptr;
    std::shared_ptr<open3d::geometry::LineSet> rays = std::make_shared<open3d::geometry::LineSet>();
    std::shared_ptr<open3d::geometry::TriangleMesh> pos_mesh = nullptr;
    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries = OccupancyOctree::Drawer::GetBlankGeometries();

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
        const double d = (azimuth_max - azimuth_min) / num_azimuth_lines;
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
        drawer = std::make_shared<OccupancyOctree::Drawer>(drawer_setting);
        visualizer_setting->x = (drawer_setting->area_min[0] + drawer_setting->area_max[0]) / 2.;
        visualizer_setting->y = (drawer_setting->area_min[1] + drawer_setting->area_max[1]) / 2.;
        visualizer_setting->z = (drawer_setting->area_min[2] + drawer_setting->area_max[2]) / 2.;
        visualizer = std::make_shared<Open3dVisualizerWrapper>(visualizer_setting);
        drawer->SetOctree(octree);
    }
};

static UserData g_user_data;

bool
VisualizerUpdateCallback(const Open3dVisualizerWrapper *visualizer, open3d::visualization::Visualizer *vis) {
    const double px = visualizer->GetSetting()->x;
    const double py = visualizer->GetSetting()->y;
    const double pz = visualizer->GetSetting()->z;
    const double roll = visualizer->GetSetting()->roll;
    const double pitch = visualizer->GetSetting()->pitch;
    const double yaw = visualizer->GetSetting()->yaw;
    const Eigen::Matrix3d rotation = EulerToRotation3D(roll, pitch, yaw, EulerAngleOrder::kRxyz);
    Eigen::Matrix3Xd ray_directions = rotation * g_user_data.ray_directions;
    Eigen::Matrix3Xd end_points(3, ray_directions.cols());
    Eigen::VectorXb is_hit(ray_directions.cols());
    const auto t0 = std::chrono::high_resolution_clock::now();
#pragma omp parallel for default(none) shared(px, py, pz, ray_directions, end_points, is_hit, g_user_data)
    for (long i = 0; i < ray_directions.cols(); ++i) {
        const double &vx = ray_directions(0, i);
        const double &vy = ray_directions(1, i);
        const double &vz = ray_directions(2, i);
        double &ex = end_points(0, i);
        double &ey = end_points(1, i);
        double &ez = end_points(2, i);
        is_hit[i] = g_user_data.octree->CastRay(px, py, pz, vx, vy, vz, g_user_data.ignore_unknown, g_user_data.max_range, ex, ey, ez) != nullptr;
    }
    const auto t1 = std::chrono::high_resolution_clock::now();
    double duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
    ERL_INFO("ray casting takes {:f} ms", duration);
    g_user_data.rays->points_.clear();
    g_user_data.rays->lines_.clear();
    g_user_data.rays->points_.emplace_back(px, py, pz);
    for (long i = 0; i < ray_directions.cols(); ++i) {
        if (is_hit[i]) {
            g_user_data.rays->points_.emplace_back(end_points.col(i));
            g_user_data.rays->lines_.emplace_back(0, static_cast<long>(g_user_data.rays->points_.size()) - 1);
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
    return true;
}

TEST(OccupancyOctree, RayCasting) {
    GTEST_PREPARE_OUTPUT_DIR();

    g_user_data.visualizer_setting->window_name = "OccupancyOctree_RayCasting";
    g_user_data.octree = AbstractOctree::ReadAs<OccupancyOctree>((gtest_src_dir / "house_expo_room_1451_3d.ot").string());
    g_user_data.ApplySettings();

    g_user_data.drawer->DrawLeaves(g_user_data.geometries);
    g_user_data.visualizer->AddGeometries(g_user_data.geometries);
    g_user_data.visualizer->SetKeyboardCallback(VisualizerUpdateCallback);
    g_user_data.visualizer->Show();
}
