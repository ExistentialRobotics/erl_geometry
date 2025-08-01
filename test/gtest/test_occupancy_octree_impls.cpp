#include "erl_common/test_helper.hpp"
#include "erl_geometry/lidar_3d.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/utils.hpp"

#include <octomap/math/Vector3.h>
#include <octomap/OcTree.h>
#include <open3d/io/TriangleMeshIO.h>

#include <filesystem>

static std::filesystem::path g_test_data_dir =
    std::filesystem::path(ERL_GEOMETRY_ROOT_DIR) / "data";

struct Options {
    std::string mesh_file = (g_test_data_dir / "house_expo_room_1451.ply").string();
    std::string traj_file = (g_test_data_dir / "house_expo_room_1451.csv").string();
    std::shared_ptr<erl::geometry::OccupancyOctreeD::Setting> octree_setting = [] {
        auto setting = std::make_shared<erl::geometry::OccupancyOctreeD::Setting>();
        setting->resolution = 0.01;
        return setting;
    }();
};

static Options g_options;

TEST(OccupancyOctree, ErlImpl) {
    using namespace erl::common;
    using namespace erl::geometry;

    const auto mesh_legacy = open3d::io::CreateMeshFromFile(g_options.mesh_file);
    const auto mesh = open3d::t::geometry::TriangleMesh::FromLegacy(*mesh_legacy);
    const auto o3d_scene = std::make_shared<open3d::t::geometry::RaycastingScene>();
    o3d_scene->AddTriangles(mesh);

    const auto lidar_setting = std::make_shared<Lidar3Dd::Setting>();
    Lidar3Dd lidar(lidar_setting, o3d_scene);
    Eigen::MatrixX<Eigen::Vector3d> ray_directions = lidar.GetRayDirectionsInFrame();

    const Eigen::MatrixXd traj_2d =
        LoadEigenMatrixFromTextFile<double>(g_options.traj_file, EigenTextFormat::kCsvFmt)
            .transpose();
    std::vector<Eigen::Matrix4d> path_3d = ConvertPath2dTo3d<double>(traj_2d, 1.0);

    const auto erl_octree = std::make_shared<OccupancyOctreeD>(g_options.octree_setting);

    double dt_erl = 0;
    double dt_erl_discrete = 0;
    std::size_t pose_idx = 0;
    while (pose_idx < 4) {
        Eigen::Matrix4d &pose = path_3d[pose_idx++];
        Eigen::Matrix3d orientation = pose.topLeftCorner<3, 3>();
        Eigen::Vector3d sensor_origin = pose.topRightCorner<3, 1>();

        auto t0 = std::chrono::high_resolution_clock::now();
        Eigen::MatrixXd ranges = lidar.Scan(orientation, sensor_origin);
        auto t1 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "Scan time: " << duration << " ms." << std::endl;

        Eigen::Matrix3Xd points(3, ranges.size());
        long cnt_points = 0;
        for (long i = 0; i < ranges.rows(); ++i) {
            for (long j = 0; j < ranges.cols(); ++j) {
                const double &range = ranges(i, j);
                if (!std::isfinite(range)) { continue; }
                points.col(cnt_points++) =
                    sensor_origin + range * orientation * ray_directions(i, j);
            }
        }
        points.conservativeResize(3, cnt_points);

        t0 = std::chrono::high_resolution_clock::now();
        erl_octree->InsertPointCloud(points, sensor_origin, 0, -1, false, false, false);
        t1 = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "ERL insert time: " << duration << " ms." << std::endl;
        dt_erl += duration;

        t0 = std::chrono::high_resolution_clock::now();
        erl_octree->InsertPointCloud(points, sensor_origin, 0, -1, false, false, true);
        t1 = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "ERL discrete insert time: " << duration << " ms." << std::endl;
        dt_erl_discrete += duration;
    }
    dt_erl /= static_cast<double>(pose_idx);
    dt_erl_discrete /= static_cast<double>(pose_idx);
    std::cout << "===================\n"  //
              << "Average insert time\n"  //
              << "ERL:          " << dt_erl << " ms.\n"
              << "ERL discrete: " << dt_erl_discrete << " ms." << std::endl;
}

TEST(OccupancyOctree, ErlComputeUpdate) {
    using namespace erl::common;
    using namespace erl::geometry;

    const auto mesh_legacy = open3d::io::CreateMeshFromFile(g_options.mesh_file);
    const auto mesh = open3d::t::geometry::TriangleMesh::FromLegacy(*mesh_legacy);
    const auto o3d_scene = std::make_shared<open3d::t::geometry::RaycastingScene>();
    o3d_scene->AddTriangles(mesh);

    const auto lidar_setting = std::make_shared<Lidar3Dd::Setting>();
    Lidar3Dd lidar(lidar_setting, o3d_scene);
    Eigen::MatrixX<Eigen::Vector3d> ray_directions = lidar.GetRayDirectionsInFrame();

    const Eigen::MatrixXd traj_2d =
        LoadEigenMatrixFromTextFile<double>(g_options.traj_file, EigenTextFormat::kCsvFmt)
            .transpose();
    std::vector<Eigen::Matrix4d> path_3d = ConvertPath2dTo3d<double>(traj_2d, 1.0);

    const auto erl_octree = std::make_shared<OccupancyOctreeD>(g_options.octree_setting);

    double dt_erl = 0;
    double dt_erl_parallel = 0;
    double dt_erl_parallel_discrete = 0;
    std::size_t pose_idx = 0;
    while (pose_idx < 4) {
        Eigen::Matrix4d &pose = path_3d[pose_idx++];
        Eigen::Matrix3d orientation = pose.topLeftCorner<3, 3>();
        Eigen::Vector3d sensor_origin = pose.topRightCorner<3, 1>();

        auto t0 = std::chrono::high_resolution_clock::now();
        Eigen::MatrixXd ranges = lidar.Scan(orientation, sensor_origin);
        auto t1 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "Scan time: " << duration << " ms." << std::endl;

        Eigen::Matrix3Xd points(3, ranges.size());
        long cnt_points = 0;
        for (long i = 0; i < ranges.rows(); ++i) {
            for (long j = 0; j < ranges.cols(); ++j) {
                const double &range = ranges(i, j);
                if (!std::isfinite(range)) { continue; }
                points.col(cnt_points++) =
                    sensor_origin + range * orientation * ray_directions(i, j);
            }
        }
        points.conservativeResize(3, cnt_points);

        OctreeKeyVector free_cells, occupied_cells;
        t0 = std::chrono::high_resolution_clock::now();
        erl_octree->ComputeUpdateForPointCloud(
            points,
            sensor_origin,
            0,
            -1,
            false,
            free_cells,
            occupied_cells);
        t1 = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "ERL compute update time: " << duration << " ms." << std::endl;
        dt_erl += duration;

        t0 = std::chrono::high_resolution_clock::now();
        erl_octree->ComputeUpdateForPointCloud(
            points,
            sensor_origin,
            0,
            -1,
            true,
            free_cells,
            occupied_cells);
        t1 = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "ERL parallel compute update time: " << duration << " ms." << std::endl;
        dt_erl_parallel += duration;

        t0 = std::chrono::high_resolution_clock::now();
        erl_octree->ComputeDiscreteUpdateForPointCloud(
            points,
            sensor_origin,
            0,
            -1,
            true,
            free_cells,
            occupied_cells);
        t1 = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "ERL parallel compute update time: " << duration << " ms." << std::endl;
        dt_erl_parallel_discrete += duration;
    }
    dt_erl /= static_cast<double>(pose_idx);
    dt_erl_parallel /= static_cast<double>(pose_idx);
    dt_erl_parallel_discrete /= static_cast<double>(pose_idx);
    std::cout << "===================\n"  //
              << "Average time\n"         //
              << "ERL:                   " << dt_erl << " ms.\n"
              << "ERL parallel:          " << dt_erl_parallel << " ms.\n"
              << "ERL parallel discrete: " << dt_erl_parallel_discrete << " ms." << std::endl;
}

TEST(OccupancyOctree, OctomapImpl) {
    using namespace erl::common;
    using namespace erl::geometry;

    const auto mesh_legacy = open3d::io::CreateMeshFromFile(g_options.mesh_file);
    const auto mesh = open3d::t::geometry::TriangleMesh::FromLegacy(*mesh_legacy);
    const auto o3d_scene = std::make_shared<open3d::t::geometry::RaycastingScene>();
    o3d_scene->AddTriangles(mesh);

    const auto lidar_setting = std::make_shared<Lidar3Dd::Setting>();
    Lidar3Dd lidar(lidar_setting, o3d_scene);
    Eigen::MatrixX<Eigen::Vector3d> ray_directions = lidar.GetRayDirectionsInFrame();

    const Eigen::MatrixXd traj_2d =
        LoadEigenMatrixFromTextFile<double>(g_options.traj_file, EigenTextFormat::kCsvFmt)
            .transpose();
    std::cout << traj_2d.rows() << " " << traj_2d.cols() << std::endl;
    std::vector<Eigen::Matrix4d> path_3d = ConvertPath2dTo3d<double>(traj_2d, 1.0);

    const auto octomap_octree =
        std::make_shared<octomap::OcTree>(g_options.octree_setting->resolution);

    double dt_octomap = 0;
    double dt_octomap_discrete = 0;
    std::size_t pose_idx = 0;
    while (pose_idx < 4) {
        Eigen::Matrix4d &pose = path_3d[pose_idx++];
        Eigen::Matrix3d orientation = pose.topLeftCorner<3, 3>();
        Eigen::Vector3d sensor_origin = pose.topRightCorner<3, 1>();

        auto t0 = std::chrono::high_resolution_clock::now();
        Eigen::MatrixXd ranges = lidar.Scan(orientation, sensor_origin);
        auto t1 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "Scan time: " << duration << " ms." << std::endl;

        octomap::Pointcloud scan;
        for (long i = 0; i < ranges.rows(); ++i) {
            for (long j = 0; j < ranges.cols(); ++j) {
                const double &range = ranges(i, j);
                if (!std::isfinite(range)) { continue; }
                Eigen::Vector3d point = sensor_origin + range * orientation * ray_directions(i, j);
                scan.push_back(
                    static_cast<float>(point[0]),
                    static_cast<float>(point[1]),
                    static_cast<float>(point[2]));
            }
        }

        t0 = std::chrono::high_resolution_clock::now();
        octomap_octree->insertPointCloud(
            scan,
            octomath::Vector3(
                static_cast<float>(sensor_origin[0]),
                static_cast<float>(sensor_origin[1]),
                static_cast<float>(sensor_origin[2])),
            -1,
            false,
            false);
        t1 = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "Octomap insert time: " << duration << " ms." << std::endl;
        dt_octomap += duration;

        t0 = std::chrono::high_resolution_clock::now();
        octomap_octree->insertPointCloud(
            scan,
            octomath::Vector3(
                static_cast<float>(sensor_origin[0]),
                static_cast<float>(sensor_origin[1]),
                static_cast<float>(sensor_origin[2])),
            -1,
            false,
            true);
        t1 = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "Octomap discrete insert time: " << duration << " ms." << std::endl;
        dt_octomap_discrete += duration;
    }
    dt_octomap /= static_cast<double>(pose_idx);
    dt_octomap_discrete /= static_cast<double>(pose_idx);
    std::cout << "===================\n"  //
              << "Average insert time\n"  //
              << "Octomap:          " << dt_octomap << " ms.\n"
              << "Octomap discrete: " << dt_octomap_discrete << " ms." << std::endl;
}

TEST(OccupancyOctree, Octomap_ComputeUpdate) {
    using namespace erl::common;
    using namespace erl::geometry;

    auto mesh_legacy = open3d::io::CreateMeshFromFile(g_options.mesh_file);
    auto mesh = open3d::t::geometry::TriangleMesh::FromLegacy(*mesh_legacy);
    auto o3d_scene = std::make_shared<open3d::t::geometry::RaycastingScene>();
    o3d_scene->AddTriangles(mesh);

    auto lidar_setting = std::make_shared<Lidar3Dd::Setting>();
    auto lidar = Lidar3Dd(lidar_setting, o3d_scene);
    Eigen::MatrixX<Eigen::Vector3d> ray_directions = lidar.GetRayDirectionsInFrame();

    Eigen::MatrixXd traj_2d =
        LoadEigenMatrixFromTextFile<double>(g_options.traj_file, EigenTextFormat::kCsvFmt)
            .transpose();
    std::vector<Eigen::Matrix4d> path_3d = ConvertPath2dTo3d<double>(traj_2d, 1.0);

    auto octomap_octree = std::make_shared<octomap::OcTree>(g_options.octree_setting->resolution);

    double dt_octomap = 0;
    double dt_octomap_discrete = 0;
    std::size_t pose_idx = 0;
    while (pose_idx < 4) {
        Eigen::Matrix4d &pose = path_3d[pose_idx++];
        Eigen::Matrix3d orientation = pose.topLeftCorner<3, 3>();
        Eigen::Vector3d sensor_origin = pose.topRightCorner<3, 1>();

        auto t0 = std::chrono::high_resolution_clock::now();
        Eigen::MatrixXd ranges = lidar.Scan(orientation, sensor_origin);
        auto t1 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "Scan time: " << duration << " ms." << std::endl;

        octomap::Pointcloud scan;
        for (long i = 0; i < ranges.rows(); ++i) {
            for (long j = 0; j < ranges.cols(); ++j) {
                const double &range = ranges(i, j);
                if (!std::isfinite(range)) { continue; }
                Eigen::Vector3d point = sensor_origin + range * orientation * ray_directions(i, j);
                scan.push_back(
                    static_cast<float>(point[0]),
                    static_cast<float>(point[1]),
                    static_cast<float>(point[2]));
            }
        }

        octomath::Vector3 octo_sensor_origin;
        octo_sensor_origin.x() = static_cast<float>(sensor_origin[0]);
        octo_sensor_origin.y() = static_cast<float>(sensor_origin[1]);
        octo_sensor_origin.z() = static_cast<float>(sensor_origin[2]);
        octomap::KeySet free_cells, occupied_cells;
        t0 = std::chrono::high_resolution_clock::now();
        octomap_octree->computeUpdate(scan, octo_sensor_origin, free_cells, occupied_cells, -1);
        t1 = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "Octomap time: " << duration << " ms." << std::endl;
        dt_octomap += duration;

        t0 = std::chrono::high_resolution_clock::now();
        octomap_octree
            ->computeDiscreteUpdate(scan, octo_sensor_origin, free_cells, occupied_cells, -1);
        t1 = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "Octomap discrete time: " << duration << " ms." << std::endl;
        dt_octomap_discrete += duration;
    }
    dt_octomap /= static_cast<double>(pose_idx);
    dt_octomap_discrete /= static_cast<double>(pose_idx);
    std::cout << "===================\n"  //
              << "Average time\n"         //
              << "Octomap: " << dt_octomap << " ms.\n"
              << "Octomap discrete: " << dt_octomap_discrete << " ms." << std::endl;
}
