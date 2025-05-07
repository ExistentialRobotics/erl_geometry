#include "erl_common/angle_utils.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/lidar_3d.hpp"
#include "erl_geometry/lidar_frame_3d.hpp"

#include <boost/program_options.hpp>
#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/t/geometry/RaycastingScene.h>
#include <open3d/t/geometry/TriangleMesh.h>
#include <open3d/visualization/utility/DrawGeometry.h>
#include <open3d/visualization/visualizer/Visualizer.h>

std::filesystem::path g_gtest_dir = std::filesystem::path(__FILE__).parent_path();

struct Options {
    std::string window_name = "LidarFrame3D";
    std::string ply_file = (g_gtest_dir / "house_expo_room_1451.ply").string();
    // std::string ply_file = (gtest_dir / "replica-office-0.ply").string();
    double lidar_elevation_min = -30;
    double lidar_elevation_max = 30;
    int lidar_num_elevation_lines = 61;
    double init_x = std::numeric_limits<double>::infinity();
    double init_y = std::numeric_limits<double>::infinity();
    double init_z = std::numeric_limits<double>::infinity();
    double init_roll = std::numeric_limits<double>::infinity();
    double init_pitch = std::numeric_limits<double>::infinity();
    double init_yaw = std::numeric_limits<double>::infinity();
};

Options g_user_data;

TEST(LidarFrame3D, Basic) {
    using namespace erl::common;
    using namespace erl::geometry;
    std::cout << "ply_file: " << g_user_data.ply_file << std::endl
              << "lidar_elevation_min: " << g_user_data.lidar_elevation_min << std::endl
              << "lidar_elevation_max: " << g_user_data.lidar_elevation_max << std::endl
              << "lidar_num_elevation_lines: " << g_user_data.lidar_num_elevation_lines << std::endl
              << "init_x: " << g_user_data.init_x << std::endl
              << "init_y: " << g_user_data.init_y << std::endl
              << "init_z: " << g_user_data.init_z << std::endl
              << "init_roll: " << g_user_data.init_roll << std::endl
              << "init_pitch: " << g_user_data.init_pitch << std::endl
              << "init_yaw: " << g_user_data.init_yaw << std::endl;

    try {
        auto room_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        {
            open3d::io::ReadTriangleMeshOptions options;
            options.enable_post_processing = true;
            open3d::io::ReadTriangleMesh(g_user_data.ply_file, *room_mesh, options);
            room_mesh->ComputeTriangleNormals();
        }
        std::shared_ptr<open3d::geometry::TriangleMesh> position_sphere =
            open3d::geometry::TriangleMesh::CreateSphere(0.1);
        position_sphere->PaintUniformColor({0.0, 0.0, 1.0});
        auto lidar_rays_line_set = std::make_shared<open3d::geometry::LineSet>();
        auto lidar_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        auto surface_samples_line_set = std::make_shared<open3d::geometry::LineSet>();
        auto surface_samples_pos_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        auto surface_samples_end_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        auto region_samples_hpr_line_set = std::make_shared<open3d::geometry::LineSet>();
        auto region_samples_hpr_pos_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        auto region_samples_hpr_end_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        auto region_samples_vrs_line_set = std::make_shared<open3d::geometry::LineSet>();
        auto region_samples_vrs_pos_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        auto region_samples_vrs_end_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        auto along_ray_samples_line_set = std::make_shared<open3d::geometry::LineSet>();
        auto along_ray_samples_pos_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        auto along_ray_samples_end_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        bool surface_samples_ready = false;
        bool region_samples_hpr_ready = false;
        bool region_samples_vrs_ready = false;
        bool along_ray_samples_ready = false;
        bool show_sample_rays = true;
        bool show_sample_positions = true;
        bool show_sample_end_points = true;

        auto o3d_scene = std::make_shared<open3d::t::geometry::RaycastingScene>();
        o3d_scene->AddTriangles(open3d::t::geometry::TriangleMesh::FromLegacy(*room_mesh));

        auto lidar_3d_setting = std::make_shared<Lidar3Dd::Setting>();
        lidar_3d_setting->elevation_min = DegreeToRadian(g_user_data.lidar_elevation_min);
        lidar_3d_setting->elevation_max = DegreeToRadian(g_user_data.lidar_elevation_max);
        lidar_3d_setting->num_elevation_lines = g_user_data.lidar_num_elevation_lines;
        auto lidar_3d = std::make_shared<Lidar3Dd>(lidar_3d_setting);
        lidar_3d->AddMesh(room_mesh->vertices_, room_mesh->triangles_);

        double lidar_roll = 0.0;
        double lidar_pitch = 0.0;
        double lidar_yaw = 0.0;
        Eigen::Vector3d lidar_position = room_mesh->GetCenter();
        // Eigen::Vector3d lidar_position(6.61, 6.46, 1.0);

        if (std::isfinite(g_user_data.init_x)) { lidar_position[0] = g_user_data.init_x; }
        if (std::isfinite(g_user_data.init_y)) { lidar_position[1] = g_user_data.init_y; }
        if (std::isfinite(g_user_data.init_z)) { lidar_position[2] = g_user_data.init_z; }
        if (std::isfinite(g_user_data.init_roll)) {
            lidar_roll = DegreeToRadian(g_user_data.init_roll);
        }
        if (std::isfinite(g_user_data.init_pitch)) {
            lidar_pitch = DegreeToRadian(g_user_data.init_pitch);
        }
        if (std::isfinite(g_user_data.init_yaw)) {
            lidar_yaw = DegreeToRadian(g_user_data.init_yaw);
        }
        position_sphere->Translate(lidar_position);

        Eigen::MatrixX<Eigen::Vector3d> lidar_ray_dirs = lidar_3d->GetRayDirectionsInFrame();
        Eigen::VectorXd azimuths = lidar_3d->GetAzimuthAngles();
        Eigen::VectorXd elevations = lidar_3d->GetElevationAngles();

        bool show_lidar_rays = true;
        bool show_lidar_points = true;
        bool show_surface_samples = false;
        bool show_region_samples_hpr = false;
        bool show_region_samples_vrs = true;
        bool show_along_ray_samples = false;
        auto update_render = [&](open3d::visualization::Visualizer *vis) {
            auto render_tic = std::chrono::high_resolution_clock::now();
            if (show_lidar_rays || show_lidar_points || show_surface_samples ||
                show_region_samples_hpr || show_region_samples_vrs || show_along_ray_samples) {
                // r-zyx order
                Eigen::Quaterniond orientation =
                    Eigen::AngleAxisd(lidar_yaw, Eigen::Vector3d::UnitZ()) *    // yaw
                    Eigen::AngleAxisd(lidar_pitch, Eigen::Vector3d::UnitY()) *  // pitch
                    Eigen::AngleAxisd(lidar_roll, Eigen::Vector3d::UnitX());    // roll
                Eigen::Matrix3d rotation = orientation.matrix();
                Eigen::MatrixXd ranges = lidar_3d->Scan(rotation, lidar_position);
                long num_azimuths = ranges.rows();
                long num_elevations = ranges.cols();
                long max_num_valid_rays = num_azimuths * num_elevations;

                if (show_lidar_rays) {
                    lidar_rays_line_set->Clear();
                    lidar_rays_line_set->points_.reserve(max_num_valid_rays + 1);
                    lidar_rays_line_set->lines_.reserve(max_num_valid_rays);
                    lidar_rays_line_set->colors_.reserve(max_num_valid_rays);
                    lidar_rays_line_set->points_.push_back(lidar_position);
                }
                if (show_lidar_points) {
                    lidar_point_cloud->Clear();
                    lidar_point_cloud->points_.reserve(max_num_valid_rays);
                    lidar_point_cloud->colors_.reserve(max_num_valid_rays);
                }
                for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
                    long ray_idx_base = azimuth_idx * num_elevations;
                    for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
                        long ray_idx = ray_idx_base + elevation_idx;
                        const double &range = ranges(azimuth_idx, elevation_idx);
                        if (std::isinf(range)) { continue; }
                        Eigen::Vector3d end_pt = range * lidar_ray_dirs(azimuth_idx, elevation_idx);
                        end_pt = rotation * end_pt + lidar_position;
                        if (show_lidar_rays) {
                            lidar_rays_line_set->points_.push_back(end_pt);
                            lidar_rays_line_set->lines_.emplace_back(0, ray_idx + 1);
                            lidar_rays_line_set->colors_.emplace_back(0.0, 1.0, 0.0);
                        }
                        if (show_lidar_points) {
                            lidar_point_cloud->points_.push_back(end_pt);
                            lidar_point_cloud->colors_.emplace_back(1.0, 0.0, 0.0);
                        }
                    }
                }

                auto lidar_frame_3d_setting = std::make_shared<LidarFrame3Dd::Setting>();
                auto lidar_frame_3d = std::make_shared<LidarFrame3Dd>(lidar_frame_3d_setting);
                if (!surface_samples_ready || !region_samples_hpr_ready ||
                    !region_samples_vrs_ready || !along_ray_samples_ready) {
                    auto tic = std::chrono::high_resolution_clock::now();
                    lidar_frame_3d->UpdateRanges(rotation, lidar_position, ranges, false);
                    auto toc = std::chrono::high_resolution_clock::now();
                    std::cout << "lidar_frame_3d->Update takes "
                              << std::chrono::duration<double, std::milli>(toc - tic).count()
                              << " ms" << std::endl;
                }

                if (show_surface_samples && !surface_samples_ready) {
                    Eigen::Matrix3Xd sampled_positions, sampled_directions;
                    Eigen::VectorXd sampled_distances;
                    auto tic = std::chrono::high_resolution_clock::now();
                    lidar_frame_3d->SampleNearSurface(
                        10,
                        0.05,
                        1.0,
                        sampled_positions,
                        sampled_directions,
                        sampled_distances);
                    auto toc = std::chrono::high_resolution_clock::now();
                    std::cout
                        << "SampleNearSurface: "
                        << std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count()
                        << " ms" << std::endl;
                    long num_samples = sampled_positions.cols();
                    surface_samples_line_set->Clear();
                    surface_samples_line_set->points_.reserve(num_samples * 2);
                    surface_samples_line_set->lines_.reserve(num_samples);
                    surface_samples_line_set->colors_.reserve(num_samples);
                    surface_samples_pos_point_cloud->Clear();
                    surface_samples_pos_point_cloud->points_.reserve(num_samples);
                    surface_samples_pos_point_cloud->colors_.reserve(num_samples);
                    surface_samples_end_point_cloud->Clear();
                    surface_samples_end_point_cloud->points_.reserve(num_samples);
                    surface_samples_end_point_cloud->colors_.reserve(num_samples);
                    for (long i = 0; i < num_samples; ++i) {
                        Eigen::Vector3d end_point =
                            sampled_positions.col(i) +
                            sampled_directions.col(i) * sampled_distances[i];
                        surface_samples_line_set->points_.emplace_back(sampled_positions.col(i));
                        surface_samples_line_set->points_.emplace_back(end_point);
                        surface_samples_line_set->lines_.emplace_back(i * 2, i * 2 + 1);
                        if (sampled_distances[i] > 0) {
                            surface_samples_line_set->colors_.emplace_back(0.0, 0.0, 1.0);
                        } else {
                            surface_samples_line_set->colors_.emplace_back(1.0, 0.5, 0.0);
                        }
                        surface_samples_pos_point_cloud->points_.emplace_back(
                            sampled_positions.col(i));
                        surface_samples_pos_point_cloud->colors_.emplace_back(0.0, 1.0, 1.0);
                        surface_samples_end_point_cloud->points_.emplace_back(end_point);
                        surface_samples_end_point_cloud->colors_.emplace_back(1.0, 0.0, 0.0);
                    }
                    surface_samples_ready = true;
                }

                if (show_region_samples_hpr && !region_samples_hpr_ready) {
                    Eigen::Matrix3Xd sampled_positions, sampled_directions;
                    Eigen::VectorXd sampled_distances;
                    long num_positions = 20;
                    long num_near_surface_samples_per_ray = 5;
                    long num_along_ray_samples_per_ray = 10;
                    double max_in_obstacle_dist = 0.05;
                    std::cout << "num_samples: " << num_positions << std::endl
                              << "num_near_surface_samples_per_ray: "
                              << num_near_surface_samples_per_ray << std::endl
                              << "num_along_ray_samples_per_ray: " << num_along_ray_samples_per_ray
                              << std::endl
                              << "max_in_obstacle_dist: " << max_in_obstacle_dist << std::endl;
                    auto tic = std::chrono::high_resolution_clock::now();
                    lidar_frame_3d->SampleInRegionHpr(
                        num_positions,
                        num_near_surface_samples_per_ray,
                        num_along_ray_samples_per_ray,
                        max_in_obstacle_dist,
                        sampled_positions,
                        sampled_directions,
                        sampled_distances,
                        true);
                    auto toc = std::chrono::high_resolution_clock::now();
                    double t = std::chrono::duration<double, std::milli>(toc - tic).count();
                    std::cout << "SampleInRegionHpr: " << t << " ms" << std::endl
                              << "number of samples: " << sampled_positions.cols() << std::endl
                              << "time per sample: "
                              << t / static_cast<double>(sampled_positions.cols()) << " ms"
                              << std::endl;
                    region_samples_hpr_line_set->Clear();
                    region_samples_hpr_line_set->points_.reserve(sampled_positions.cols());
                    region_samples_hpr_line_set->lines_.reserve(sampled_positions.cols());
                    region_samples_hpr_line_set->colors_.reserve(sampled_positions.cols());
                    region_samples_hpr_pos_point_cloud->Clear();
                    region_samples_hpr_pos_point_cloud->points_.reserve(sampled_positions.cols());
                    region_samples_hpr_pos_point_cloud->colors_.reserve(sampled_positions.cols());
                    region_samples_hpr_end_point_cloud->Clear();
                    region_samples_hpr_end_point_cloud->points_.reserve(sampled_positions.cols());
                    region_samples_hpr_end_point_cloud->colors_.reserve(sampled_positions.cols());
                    for (long i = 0; i < sampled_positions.cols(); ++i) {
                        Eigen::Vector3d end_point =
                            sampled_positions.col(i) +
                            sampled_directions.col(i) * sampled_distances[i];
                        region_samples_hpr_line_set->points_.emplace_back(sampled_positions.col(i));
                        region_samples_hpr_line_set->points_.emplace_back(end_point);
                        region_samples_hpr_line_set->lines_.emplace_back(i * 2, i * 2 + 1);
                        if (sampled_distances[i] > 0) {
                            region_samples_hpr_line_set->colors_.emplace_back(0.5, 0.0, 1.0);
                        } else {
                            region_samples_hpr_line_set->colors_.emplace_back(1.0, 1.0, 0.0);
                        }
                        region_samples_hpr_pos_point_cloud->points_.emplace_back(
                            sampled_positions.col(i));
                        region_samples_hpr_pos_point_cloud->colors_.emplace_back(0.0, 1.0, 1.0);
                        region_samples_hpr_end_point_cloud->points_.emplace_back(end_point);
                        region_samples_hpr_end_point_cloud->colors_.emplace_back(1.0, 0.0, 0.0);
                    }
                    region_samples_hpr_ready = true;
                }

                if (show_region_samples_vrs && !region_samples_vrs_ready) {
                    Eigen::Matrix3Xd sampled_positions, sampled_directions;
                    Eigen::VectorXd sampled_distances;
                    long num_hit_points = 100;
                    long num_samples_per_azimuth_segment = 100;
                    long num_azimuth_segments = 36;
                    std::cout << "num_hit_points: " << num_hit_points << std::endl
                              << "num_samples_per_azimuth_segment: "
                              << num_samples_per_azimuth_segment << std::endl
                              << "num_azimuth_segments: " << num_azimuth_segments << std::endl;
                    auto tic = std::chrono::high_resolution_clock::now();
                    lidar_frame_3d->SampleInRegionVrs(
                        num_hit_points,
                        num_samples_per_azimuth_segment,
                        num_azimuth_segments,
                        sampled_positions,
                        sampled_directions,
                        sampled_distances,
                        true);
                    auto toc = std::chrono::high_resolution_clock::now();
                    double t = std::chrono::duration<double, std::milli>(toc - tic).count();
                    std::cout << "SampleInRegionVrs: " << t << " ms" << std::endl
                              << "number of samples: " << sampled_positions.cols() << std::endl
                              << "time per sample: "
                              << t / static_cast<double>(sampled_positions.cols()) << " ms"
                              << std::endl;
                    region_samples_vrs_line_set->Clear();
                    region_samples_vrs_line_set->points_.reserve(sampled_positions.cols());
                    region_samples_vrs_line_set->lines_.reserve(sampled_positions.cols());
                    region_samples_vrs_line_set->colors_.reserve(sampled_positions.cols());
                    region_samples_vrs_pos_point_cloud->Clear();
                    region_samples_vrs_pos_point_cloud->points_.reserve(sampled_positions.cols());
                    region_samples_vrs_pos_point_cloud->colors_.reserve(sampled_positions.cols());
                    region_samples_vrs_end_point_cloud->Clear();
                    region_samples_vrs_end_point_cloud->points_.reserve(sampled_positions.cols());
                    region_samples_vrs_end_point_cloud->colors_.reserve(sampled_positions.cols());
                    for (long i = 0; i < sampled_positions.cols(); ++i) {
                        Eigen::Vector3d end_point =
                            sampled_positions.col(i) +
                            sampled_directions.col(i) * sampled_distances[i];
                        region_samples_vrs_line_set->points_.emplace_back(sampled_positions.col(i));
                        region_samples_vrs_line_set->points_.emplace_back(end_point);
                        region_samples_vrs_line_set->lines_.emplace_back(i * 2, i * 2 + 1);
                        if (sampled_distances[i] > 0) {
                            region_samples_vrs_line_set->colors_.emplace_back(0.5, 0.0, 1.0);
                        } else {
                            region_samples_vrs_line_set->colors_.emplace_back(1.0, 1.0, 0.0);
                        }
                        region_samples_vrs_pos_point_cloud->points_.emplace_back(
                            sampled_positions.col(i));
                        region_samples_vrs_pos_point_cloud->colors_.emplace_back(0.0, 1.0, 1.0);
                        region_samples_vrs_end_point_cloud->points_.emplace_back(end_point);
                        region_samples_vrs_end_point_cloud->colors_.emplace_back(1.0, 0.0, 0.0);
                    }
                    region_samples_vrs_ready = true;
                }

                if (show_along_ray_samples && !along_ray_samples_ready) {
                    Eigen::Matrix3Xd sampled_positions, sampled_directions;
                    Eigen::VectorXd sampled_distances;
                    double range_step = 0.2;
                    double max_in_obstacle_dist = 0.05;
                    auto tic = std::chrono::high_resolution_clock::now();
                    lidar_frame_3d->SampleAlongRays(
                        range_step,
                        max_in_obstacle_dist,
                        1.0,
                        sampled_positions,
                        sampled_directions,
                        sampled_distances);
                    auto toc = std::chrono::high_resolution_clock::now();
                    std::cout
                        << "SampleAlongRays (fixed range step): "
                        << std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count()
                        << " ms" << std::endl;

                    long num_samples = sampled_positions.cols();
                    long num_hit_rays = lidar_frame_3d->GetNumHitRays();
                    long num_samples_per_ray = 5l;
                    num_samples += num_hit_rays * num_samples_per_ray;
                    along_ray_samples_line_set->Clear();
                    along_ray_samples_line_set->points_.reserve(num_samples * 2);
                    along_ray_samples_line_set->lines_.reserve(num_samples);
                    along_ray_samples_line_set->colors_.reserve(num_samples);
                    along_ray_samples_pos_point_cloud->Clear();
                    along_ray_samples_pos_point_cloud->points_.reserve(num_samples);
                    along_ray_samples_pos_point_cloud->colors_.reserve(num_samples);
                    along_ray_samples_end_point_cloud->Clear();
                    along_ray_samples_end_point_cloud->points_.reserve(num_samples);
                    along_ray_samples_end_point_cloud->colors_.reserve(num_samples);
                    num_samples = sampled_positions.cols();
                    for (long i = 0; i < num_samples; ++i) {
                        Eigen::Vector3d end_point =
                            sampled_positions.col(i) +
                            sampled_directions.col(i) * sampled_distances[i];
                        along_ray_samples_line_set->points_.emplace_back(sampled_positions.col(i));
                        along_ray_samples_line_set->points_.emplace_back(end_point);
                        along_ray_samples_line_set->lines_.emplace_back(i * 2, i * 2 + 1);
                        if (sampled_distances[i] > 0) {
                            along_ray_samples_line_set->colors_.emplace_back(0.0, 0.5, 1.0);
                        } else {
                            along_ray_samples_line_set->colors_.emplace_back(1.0, 0.0, 1.0);
                        }
                        along_ray_samples_pos_point_cloud->points_.emplace_back(
                            sampled_positions.col(i));
                        along_ray_samples_pos_point_cloud->colors_.emplace_back(0.0, 1.0, 1.0);
                        along_ray_samples_end_point_cloud->points_.emplace_back(end_point);
                        along_ray_samples_end_point_cloud->colors_.emplace_back(1.0, 0.0, 0.0);
                    }

                    tic = std::chrono::high_resolution_clock::now();
                    lidar_frame_3d->SampleAlongRays(
                        num_samples_per_ray,
                        max_in_obstacle_dist,
                        1.0,
                        sampled_positions,
                        sampled_directions,
                        sampled_distances);
                    toc = std::chrono::high_resolution_clock::now();
                    std::cout
                        << "SampleAlongRays (fixed num samples per ray): "
                        << std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count()
                        << " ms" << std::endl;
                    long collected_num_samples = num_samples;
                    num_samples = sampled_positions.cols();
                    for (long i = 0; i < num_samples; ++i) {
                        Eigen::Vector3d end_point =
                            sampled_positions.col(i) +
                            sampled_directions.col(i) * sampled_distances[i];
                        along_ray_samples_line_set->points_.emplace_back(sampled_positions.col(i));
                        along_ray_samples_line_set->points_.emplace_back(end_point);
                        along_ray_samples_line_set->lines_.emplace_back(
                            collected_num_samples + i * 2,
                            collected_num_samples + i * 2 + 1);
                        if (sampled_distances[i] > 0) {
                            along_ray_samples_line_set->colors_.emplace_back(0.0, 0.5, 1.0);
                        } else {
                            along_ray_samples_line_set->colors_.emplace_back(1.0, 0.0, 1.0);
                        }
                        along_ray_samples_pos_point_cloud->points_.emplace_back(
                            sampled_positions.col(i));
                        along_ray_samples_pos_point_cloud->colors_.emplace_back(0.0, 1.0, 1.0);
                        along_ray_samples_end_point_cloud->points_.emplace_back(end_point);
                        along_ray_samples_end_point_cloud->colors_.emplace_back(1.0, 0.0, 0.0);
                    }
                    along_ray_samples_ready = true;
                }
            }

            if (vis == nullptr) { return; }
            if (show_lidar_rays) { vis->UpdateGeometry(lidar_rays_line_set); }
            if (show_lidar_points) { vis->UpdateGeometry(lidar_point_cloud); }
            if (show_surface_samples) {
                if (show_sample_rays) { vis->UpdateGeometry(surface_samples_line_set); }
                if (show_sample_positions) { vis->UpdateGeometry(surface_samples_pos_point_cloud); }
                if (show_sample_end_points) {
                    vis->UpdateGeometry(surface_samples_end_point_cloud);
                }
            }
            if (show_region_samples_hpr) {
                if (show_sample_rays) { vis->UpdateGeometry(region_samples_hpr_line_set); }
                if (show_sample_positions) {
                    vis->UpdateGeometry(region_samples_hpr_pos_point_cloud);
                }
                if (show_sample_end_points) {
                    vis->UpdateGeometry(region_samples_hpr_end_point_cloud);
                }
            }
            if (show_region_samples_vrs) {
                if (show_sample_rays) { vis->UpdateGeometry(region_samples_vrs_line_set); }
                if (show_sample_positions) {
                    vis->UpdateGeometry(region_samples_vrs_pos_point_cloud);
                }
                if (show_sample_end_points) {
                    vis->UpdateGeometry(region_samples_vrs_end_point_cloud);
                }
            }
            if (show_along_ray_samples) {
                if (show_sample_rays) { vis->UpdateGeometry(along_ray_samples_line_set); }
                if (show_sample_positions) {
                    vis->UpdateGeometry(along_ray_samples_pos_point_cloud);
                }
                if (show_sample_end_points) {
                    vis->UpdateGeometry(along_ray_samples_end_point_cloud);
                }
            }
            auto render_toc = std::chrono::high_resolution_clock::now();
            std::cout << "update_render takes "
                      << std::chrono::duration<double, std::milli>(render_toc - render_tic).count()
                      << " ms" << std::endl;
        };
        update_render(nullptr);

        double angle_step = DegreeToRadian(0.1);
        double translation_step = 0.05;
        std::map<int, std::function<bool(open3d::visualization::Visualizer *)>> key_to_callback;
        key_to_callback[GLFW_KEY_F1] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (show_lidar_rays) {
                vis->RemoveGeometry(lidar_rays_line_set);
                std::cout << "lidar rays removed." << std::endl;
            } else {
                vis->AddGeometry(lidar_rays_line_set, false);
                std::cout << "lidar rays added." << std::endl;
            }
            show_lidar_rays = !show_lidar_rays;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_F2] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (show_lidar_points) {
                vis->RemoveGeometry(lidar_point_cloud);
                std::cout << "lidar points removed." << std::endl;
            } else {
                vis->AddGeometry(lidar_point_cloud, false);
                std::cout << "lidar points added." << std::endl;
            }
            show_lidar_points = !show_lidar_points;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_F3] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (show_surface_samples) {
                if (show_sample_rays) { vis->RemoveGeometry(surface_samples_line_set); }
                if (show_sample_positions) { vis->RemoveGeometry(surface_samples_pos_point_cloud); }
                if (show_sample_end_points) {
                    vis->RemoveGeometry(surface_samples_end_point_cloud);
                }
                std::cout << "surface samples removed." << std::endl;
            } else {
                if (show_sample_rays) { vis->AddGeometry(surface_samples_line_set, false); }
                if (show_sample_positions) {
                    vis->AddGeometry(surface_samples_pos_point_cloud, false);
                }
                if (show_sample_end_points) {
                    vis->AddGeometry(surface_samples_end_point_cloud, false);
                }
                std::cout << "surface samples added." << std::endl;
            }
            show_surface_samples = !show_surface_samples;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_F4] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (show_region_samples_hpr) {
                if (show_sample_rays) { vis->RemoveGeometry(region_samples_hpr_line_set); }
                if (show_sample_positions) {
                    vis->RemoveGeometry(region_samples_hpr_pos_point_cloud);
                }
                if (show_sample_end_points) {
                    vis->RemoveGeometry(region_samples_hpr_end_point_cloud);
                }
                std::cout << "region samples hpr removed." << std::endl;
            } else {
                if (show_sample_rays) { vis->AddGeometry(region_samples_hpr_line_set, false); }
                if (show_sample_positions) {
                    vis->AddGeometry(region_samples_hpr_pos_point_cloud, false);
                }
                if (show_sample_end_points) {
                    vis->AddGeometry(region_samples_hpr_end_point_cloud, false);
                }
                std::cout << "region samples hpr added." << std::endl;
            }
            show_region_samples_hpr = !show_region_samples_hpr;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_F5] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (show_region_samples_vrs) {
                if (show_sample_rays) { vis->RemoveGeometry(region_samples_vrs_line_set); }
                if (show_sample_positions) {
                    vis->RemoveGeometry(region_samples_vrs_pos_point_cloud);
                }
                if (show_sample_end_points) {
                    vis->RemoveGeometry(region_samples_vrs_end_point_cloud);
                }
                std::cout << "region samples vrs removed." << std::endl;
            } else {
                if (show_sample_rays) { vis->AddGeometry(region_samples_vrs_line_set, false); }
                if (show_sample_positions) {
                    vis->AddGeometry(region_samples_vrs_pos_point_cloud, false);
                }
                if (show_sample_end_points) {
                    vis->AddGeometry(region_samples_vrs_end_point_cloud, false);
                }
                std::cout << "region samples vrs added." << std::endl;
            }
            show_region_samples_vrs = !show_region_samples_vrs;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_F6] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (show_along_ray_samples) {
                if (show_sample_rays) { vis->RemoveGeometry(along_ray_samples_line_set); }
                if (show_sample_positions) {
                    vis->RemoveGeometry(along_ray_samples_pos_point_cloud);
                }
                if (show_sample_end_points) {
                    vis->RemoveGeometry(along_ray_samples_end_point_cloud);
                }
                std::cout << "along ray samples removed." << std::endl;
            } else {
                if (show_sample_rays) { vis->AddGeometry(along_ray_samples_line_set, false); }
                if (show_sample_positions) {
                    vis->AddGeometry(along_ray_samples_pos_point_cloud, false);
                }
                if (show_sample_end_points) {
                    vis->AddGeometry(along_ray_samples_end_point_cloud, false);
                }
                std::cout << "along ray samples added." << std::endl;
            }
            show_along_ray_samples = !show_along_ray_samples;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_F7] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (show_sample_rays) {
                if (show_surface_samples) { vis->RemoveGeometry(surface_samples_line_set); }
                if (show_region_samples_hpr) { vis->RemoveGeometry(region_samples_hpr_line_set); }
                if (show_region_samples_vrs) { vis->RemoveGeometry(region_samples_vrs_line_set); }
                if (show_along_ray_samples) { vis->RemoveGeometry(along_ray_samples_line_set); }
                std::cout << "sample rays removed." << std::endl;
            } else {
                if (show_surface_samples) { vis->AddGeometry(surface_samples_line_set, false); }
                if (show_region_samples_hpr) {
                    vis->AddGeometry(region_samples_hpr_line_set, false);
                }
                if (show_region_samples_vrs) {
                    vis->AddGeometry(region_samples_vrs_line_set, false);
                }
                if (show_along_ray_samples) { vis->AddGeometry(along_ray_samples_line_set, false); }
                std::cout << "sample rays added." << std::endl;
            }
            show_sample_rays = !show_sample_rays;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_F8] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (show_sample_positions) {
                if (show_surface_samples) { vis->RemoveGeometry(surface_samples_pos_point_cloud); }
                if (show_region_samples_hpr) {
                    vis->RemoveGeometry(region_samples_hpr_pos_point_cloud);
                }
                if (show_region_samples_vrs) {
                    vis->RemoveGeometry(region_samples_vrs_pos_point_cloud);
                }
                if (show_along_ray_samples) {
                    vis->RemoveGeometry(along_ray_samples_pos_point_cloud);
                }
                std::cout << "sample points removed." << std::endl;
            } else {
                if (show_surface_samples) {
                    vis->AddGeometry(surface_samples_pos_point_cloud, false);
                }
                if (show_region_samples_hpr) {
                    vis->AddGeometry(region_samples_hpr_pos_point_cloud, false);
                }
                if (show_region_samples_vrs) {
                    vis->AddGeometry(region_samples_vrs_pos_point_cloud, false);
                }
                if (show_along_ray_samples) {
                    vis->AddGeometry(along_ray_samples_pos_point_cloud, false);
                }
                std::cout << "sample points added." << std::endl;
            }
            show_sample_positions = !show_sample_positions;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_F9] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (show_sample_end_points) {
                if (show_surface_samples) { vis->RemoveGeometry(surface_samples_end_point_cloud); }
                if (show_region_samples_hpr) {
                    vis->RemoveGeometry(region_samples_hpr_end_point_cloud);
                }
                if (show_region_samples_vrs) {
                    vis->RemoveGeometry(region_samples_vrs_end_point_cloud);
                }
                if (show_along_ray_samples) {
                    vis->RemoveGeometry(along_ray_samples_end_point_cloud);
                }
                std::cout << "sample end points removed." << std::endl;
            } else {
                if (show_surface_samples) {
                    vis->AddGeometry(surface_samples_end_point_cloud, false);
                }
                if (show_region_samples_hpr) {
                    vis->AddGeometry(region_samples_hpr_end_point_cloud, false);
                }
                if (show_region_samples_vrs) {
                    vis->AddGeometry(region_samples_vrs_end_point_cloud, false);
                }
                if (show_along_ray_samples) {
                    vis->AddGeometry(along_ray_samples_end_point_cloud, false);
                }
                std::cout << "sample end points added." << std::endl;
            }
            show_sample_end_points = !show_sample_end_points;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_F10] = [&](open3d::visualization::Visualizer *vis) -> bool {
            auto scene = lidar_3d->GetScene();
            std::vector<std::pair<std::string, std::shared_ptr<open3d::geometry::LineSet>>>
                line_sets;
            if (show_surface_samples) {
                line_sets.emplace_back("surface_samples", surface_samples_line_set);
            }
            if (show_region_samples_hpr) {
                line_sets.emplace_back("region_samples_hpr", region_samples_hpr_line_set);
            }
            if (show_region_samples_vrs) {
                line_sets.emplace_back("region_samples_vrs", region_samples_vrs_line_set);
            }
            if (show_along_ray_samples) {
                line_sets.emplace_back("along_ray_samples", along_ray_samples_line_set);
            }
            for (auto &[sample_name, line_set]: line_sets) {  // verify samples are correct
                auto num_rays = static_cast<long>(line_set->lines_.size());
                open3d::core::Tensor rays({num_rays, 6}, open3d::core::Dtype::Float32);
                auto *rays_ptr = rays.GetDataPtr<float>();
                std::vector<double> ranges_to_check;
                ranges_to_check.reserve(num_rays);
                for (long i = 0; i < num_rays; ++i) {
                    long start_idx = line_set->lines_[i][0];
                    long end_idx = line_set->lines_[i][1];
                    Eigen::Vector3d &start_pt = line_set->points_[start_idx];
                    Eigen::Vector3d &end_pt = line_set->points_[end_idx];
                    long base = i * 6;
                    rays_ptr[base + 0] = static_cast<float>(start_pt[0]);
                    rays_ptr[base + 1] = static_cast<float>(start_pt[1]);
                    rays_ptr[base + 2] = static_cast<float>(start_pt[2]);
                    Eigen::Vector3d dir = end_pt - start_pt;
                    ranges_to_check.push_back(dir.norm());
                    dir /= ranges_to_check.back();
                    rays_ptr[base + 3] = static_cast<float>(dir[0]);
                    rays_ptr[base + 4] = static_cast<float>(dir[1]);
                    rays_ptr[base + 5] = static_cast<float>(dir[2]);
                }
                std::unordered_map<std::string, open3d::core::Tensor> cast_results =
                    scene->CastRays(rays);
                std::vector<float> ranges = cast_results.at("t_hit").ToFlatVector<float>();
                std::vector<uint32_t> geometry_ids =
                    cast_results.at("geometry_ids").ToFlatVector<uint32_t>();
                uint32_t invalid_id = open3d::t::geometry::RaycastingScene::INVALID_ID();
                long cnt_correct_100 = 0;
                long cnt_correct_50 = 0;
                long cnt_correct_25 = 0;
                long cnt_correct_10 = 0;
                long cnt_correct_5 = 0;
                long cnt_correct_1 = 0;
                long cnt_miss = 0;
                double diff_sum = 0;
                auto error_line_set_010 = std::make_shared<open3d::geometry::LineSet>();
                auto error_line_set_025 = std::make_shared<open3d::geometry::LineSet>();
                auto error_line_set_050 = std::make_shared<open3d::geometry::LineSet>();
                auto error_line_set_100 = std::make_shared<open3d::geometry::LineSet>();
                for (long i = 0; i < num_rays; ++i) {
                    if (geometry_ids[i] == invalid_id) {
                        ++cnt_miss;
                        continue;
                    }
                    double diff = std::abs(static_cast<double>(ranges[i]) - ranges_to_check[i]);
                    if (diff < 0.01) {
                        ++cnt_correct_1;
                        ++cnt_correct_5;
                        ++cnt_correct_10;
                        ++cnt_correct_25;
                        ++cnt_correct_50;
                        ++cnt_correct_100;
                    } else if (diff < 0.05) {
                        ++cnt_correct_5;
                        ++cnt_correct_10;
                        ++cnt_correct_25;
                        ++cnt_correct_50;
                        ++cnt_correct_100;
                    } else if (diff < 0.1) {
                        ++cnt_correct_10;
                        ++cnt_correct_25;
                        ++cnt_correct_50;
                        ++cnt_correct_100;
                    } else if (diff < 0.25) {
                        ++cnt_correct_25;
                        ++cnt_correct_50;
                        ++cnt_correct_100;
                        int start_idx = line_set->lines_[i][0];
                        int end_idx = line_set->lines_[i][1];
                        error_line_set_010->points_.emplace_back(line_set->points_[start_idx]);
                        error_line_set_010->points_.emplace_back(line_set->points_[end_idx]);
                        start_idx = static_cast<int>(error_line_set_010->points_.size() - 2);
                        end_idx = static_cast<int>(error_line_set_010->points_.size() - 1);
                        error_line_set_010->lines_.emplace_back(start_idx, end_idx);
                    } else if (diff < 0.5) {
                        ++cnt_correct_50;
                        ++cnt_correct_100;
                        int start_idx = line_set->lines_[i][0];
                        int end_idx = line_set->lines_[i][1];
                        error_line_set_025->points_.emplace_back(line_set->points_[start_idx]);
                        error_line_set_025->points_.emplace_back(line_set->points_[end_idx]);
                        start_idx = static_cast<int>(error_line_set_025->points_.size() - 2);
                        end_idx = static_cast<int>(error_line_set_025->points_.size() - 1);
                        error_line_set_025->lines_.emplace_back(start_idx, end_idx);
                    } else if (diff < 1.0) {
                        ++cnt_correct_100;
                        int start_idx = line_set->lines_[i][0];
                        int end_idx = line_set->lines_[i][1];
                        error_line_set_050->points_.emplace_back(line_set->points_[start_idx]);
                        error_line_set_050->points_.emplace_back(line_set->points_[end_idx]);
                        start_idx = static_cast<int>(error_line_set_050->points_.size() - 2);
                        end_idx = static_cast<int>(error_line_set_050->points_.size() - 1);
                        error_line_set_050->lines_.emplace_back(start_idx, end_idx);
                    } else {
                        int start_idx = line_set->lines_[i][0];
                        int end_idx = line_set->lines_[i][1];
                        error_line_set_100->points_.emplace_back(line_set->points_[start_idx]);
                        error_line_set_100->points_.emplace_back(line_set->points_[end_idx]);
                        start_idx = static_cast<int>(error_line_set_100->points_.size() - 2);
                        end_idx = static_cast<int>(error_line_set_100->points_.size() - 1);
                        error_line_set_100->lines_.emplace_back(start_idx, end_idx);
                    }

                    if (diff >= 0.01) { diff_sum += diff; }
                }
                error_line_set_010->PaintUniformColor({0.0, 1.0, 0.0});
                error_line_set_025->PaintUniformColor({1.0, 1.0, 0.0});
                error_line_set_050->PaintUniformColor({1.0, 0.5, 0.0});
                error_line_set_100->PaintUniformColor({1.0, 0.0, 0.0});
                if (!error_line_set_010->IsEmpty()) { vis->AddGeometry(error_line_set_010); }
                if (!error_line_set_025->IsEmpty()) { vis->AddGeometry(error_line_set_025); }
                if (!error_line_set_050->IsEmpty()) { vis->AddGeometry(error_line_set_050); }
                if (!error_line_set_100->IsEmpty()) { vis->AddGeometry(error_line_set_100); }
                vis->UpdateRender();
                double success_rate_100 =
                    static_cast<double>(cnt_correct_100) / static_cast<double>(num_rays) * 100;
                double success_rate_50 =
                    static_cast<double>(cnt_correct_50) / static_cast<double>(num_rays) * 100;
                double success_rate_25 =
                    static_cast<double>(cnt_correct_25) / static_cast<double>(num_rays) * 100;
                double success_rate_10 =
                    static_cast<double>(cnt_correct_10) / static_cast<double>(num_rays) * 100;
                double success_rate_5 =
                    static_cast<double>(cnt_correct_5) / static_cast<double>(num_rays) * 100;
                double success_rate_1 =
                    static_cast<double>(cnt_correct_1) / static_cast<double>(num_rays) * 100;
                double miss_rate = static_cast<double>(cnt_miss) / static_cast<double>(num_rays);
                double avg_diff =
                    diff_sum / static_cast<double>(num_rays - cnt_miss - cnt_correct_1);
                std::cout << "==========" << std::endl
                          << sample_name
                          << "     <=1.0     <=0.5    <=0.25     <=0.1    <=0.05    <=0.01"
                          << std::endl
                          << std::setw(static_cast<int>(sample_name.size())) << "success rate"
                          << std::setw(10) << success_rate_100 << std::setw(10) << success_rate_50
                          << std::setw(10) << success_rate_25 << std::setw(10) << success_rate_10
                          << std::setw(10) << success_rate_5 << std::setw(10) << success_rate_1
                          << std::endl
                          << "miss rate: " << miss_rate << std::endl
                          << "avg diff(>=0.01): " << avg_diff << std::endl;
            }
            return true;
        };
        key_to_callback[GLFW_KEY_J] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_roll -= angle_step;
            lidar_roll = WrapAnglePi(lidar_roll);
            std::cout << "xyz: " << lidar_position.transpose() << ", rpy: [" << lidar_roll << ", "
                      << lidar_pitch << ", " << lidar_yaw << "]" << std::endl;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_hpr_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_L] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_roll += angle_step;
            lidar_roll = WrapAnglePi(lidar_roll);
            std::cout << "xyz: " << lidar_position.transpose() << ", rpy: [" << lidar_roll << ", "
                      << lidar_pitch << ", " << lidar_yaw << "]" << std::endl;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_hpr_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_K] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_pitch -= angle_step;
            lidar_pitch = WrapAnglePi(lidar_pitch);
            std::cout << "xyz: " << lidar_position.transpose() << ", rpy: [" << lidar_roll << ", "
                      << lidar_pitch << ", " << lidar_yaw << "]" << std::endl;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_hpr_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_I] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_pitch += angle_step;
            lidar_pitch = WrapAnglePi(lidar_pitch);
            std::cout << "xyz: " << lidar_position.transpose() << ", rpy: [" << lidar_roll << ", "
                      << lidar_pitch << ", " << lidar_yaw << "]" << std::endl;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_hpr_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_U] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_yaw -= angle_step;
            lidar_yaw = WrapAnglePi(lidar_yaw);
            std::cout << "xyz: " << lidar_position.transpose() << ", rpy: [" << lidar_roll << ", "
                      << lidar_pitch << ", " << lidar_yaw << "]" << std::endl;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_hpr_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_O] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_yaw += angle_step;
            lidar_yaw = WrapAnglePi(lidar_yaw);
            std::cout << "xyz: " << lidar_position.transpose() << ", rpy: [" << lidar_roll << ", "
                      << lidar_pitch << ", " << lidar_yaw << "]" << std::endl;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_hpr_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_LEFT] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_position[0] -= translation_step;
            position_sphere->Translate(Eigen::Vector3d(-translation_step, 0.0, 0.0));
            std::cout << "xyz: " << lidar_position.transpose() << ", rpy: [" << lidar_roll << ", "
                      << lidar_pitch << ", " << lidar_yaw << "]" << std::endl;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_hpr_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_RIGHT] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_position[0] += translation_step;
            position_sphere->Translate(Eigen::Vector3d(translation_step, 0.0, 0.0));
            std::cout << "xyz: " << lidar_position.transpose() << ", rpy: [" << lidar_roll << ", "
                      << lidar_pitch << ", " << lidar_yaw << "]" << std::endl;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_hpr_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_DOWN] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_position[1] -= translation_step;
            position_sphere->Translate(Eigen::Vector3d(0.0, -translation_step, 0.0));
            std::cout << "xyz: " << lidar_position.transpose() << ", rpy: [" << lidar_roll << ", "
                      << lidar_pitch << ", " << lidar_yaw << "]" << std::endl;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_hpr_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_UP] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_position[1] += translation_step;
            position_sphere->Translate(Eigen::Vector3d(0.0, translation_step, 0.0));
            std::cout << "xyz: " << lidar_position.transpose() << ", rpy: [" << lidar_roll << ", "
                      << lidar_pitch << ", " << lidar_yaw << "]" << std::endl;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_hpr_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_PAGE_DOWN] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_position[2] -= translation_step;
            position_sphere->Translate(Eigen::Vector3d(0.0, 0.0, -translation_step));
            std::cout << "xyz: " << lidar_position.transpose() << ", rpy: [" << lidar_roll << ", "
                      << lidar_pitch << ", " << lidar_yaw << "]" << std::endl;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_hpr_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_PAGE_UP] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_position[2] += translation_step;
            position_sphere->Translate(Eigen::Vector3d(0.0, 0.0, translation_step));
            std::cout << "xyz: " << lidar_position.transpose() << ", rpy: [" << lidar_roll << ", "
                      << lidar_pitch << ", " << lidar_yaw << "]" << std::endl;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_hpr_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_H] = [&](open3d::visualization::Visualizer *vis) -> bool {
            vis->PrintVisualizerHelp();
            std::cout << "[H]: print help." << std::endl
                      << "[I]: increase lidar pitch." << std::endl
                      << "[K]: decrease lidar pitch." << std::endl
                      << "[J]: decrease lidar roll." << std::endl
                      << "[L]: increase lidar roll." << std::endl
                      << "[U]: decrease lidar yaw." << std::endl
                      << "[O]: increase lidar yaw." << std::endl
                      << "[Right arrow]: increase lidar x." << std::endl
                      << "[Left arrow]: decrease lidar x." << std::endl
                      << "[Up arrow]: increase lidar y." << std::endl
                      << "[Down arrow]: decrease lidar y." << std::endl
                      << "[Page up]: increase lidar z." << std::endl
                      << "[Page down]: decrease lidar z." << std::endl
                      << "[F1]: toggle lidar rays." << std::endl
                      << "[F2]: toggle lidar points." << std::endl
                      << "[F3]: toggle surface samples." << std::endl
                      << "[F4]: toggle region samples hpr." << std::endl
                      << "[F5]: toggle region samples vrs." << std::endl
                      << "[F6]: toggle along ray samples." << std::endl
                      << "[F7]: toggle sample rays." << std::endl
                      << "[F8]: toggle sample positions." << std::endl
                      << "[F9]: toggle sample end points" << std::endl
                      << "[F10]: verify samples" << std::endl;
            return true;
        };
        key_to_callback[GLFW_KEY_ESCAPE] = [&](open3d::visualization::Visualizer *vis) -> bool {
            std::cout << "visualizer closed." << std::endl;
            vis->Close();
            return true;
        };

        std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries = {
            room_mesh,
            position_sphere};
        if (show_lidar_rays) { geometries.emplace_back(lidar_rays_line_set); }
        if (show_lidar_points) { geometries.emplace_back(lidar_point_cloud); }
        if (show_surface_samples) {
            if (show_sample_rays) { geometries.emplace_back(surface_samples_line_set); }
            if (show_sample_positions) { geometries.emplace_back(surface_samples_pos_point_cloud); }
            if (show_sample_end_points) {
                geometries.emplace_back(surface_samples_end_point_cloud);
            }
        }
        if (show_region_samples_hpr) {
            if (show_sample_rays) { geometries.emplace_back(region_samples_hpr_line_set); }
            if (show_sample_positions) {
                geometries.emplace_back(region_samples_hpr_pos_point_cloud);
            }
            if (show_sample_end_points) {
                geometries.emplace_back(region_samples_hpr_end_point_cloud);
            }
        }
        if (show_region_samples_vrs) {
            if (show_sample_rays) { geometries.emplace_back(region_samples_vrs_line_set); }
            if (show_sample_positions) {
                geometries.emplace_back(region_samples_vrs_pos_point_cloud);
            }
            if (show_sample_end_points) {
                geometries.emplace_back(region_samples_vrs_end_point_cloud);
            }
        }
        if (show_along_ray_samples) {
            if (show_sample_rays) { geometries.emplace_back(along_ray_samples_line_set); }
            if (show_sample_positions) {
                geometries.emplace_back(along_ray_samples_pos_point_cloud);
            }
            if (show_sample_end_points) {
                geometries.emplace_back(along_ray_samples_end_point_cloud);
            }
        }
        open3d::visualization::DrawGeometriesWithKeyCallbacks(
            geometries,
            key_to_callback,
            "test LidarFrame3D");

    } catch (const std::exception &e) { std::cerr << e.what() << std::endl; }
}

int
main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    try {
        namespace po = boost::program_options;
        po::options_description desc;
        // clang-format off
        desc.add_options()
            ("help", "produce help message")
            ("ply-file", po::value<std::string>(&g_user_data.ply_file)->default_value(g_user_data.ply_file), "ply file path")
            ("lidar-elevation-min", po::value<double>(&g_user_data.lidar_elevation_min)->default_value(g_user_data.lidar_elevation_min), "lidar elevation min")
            ("lidar-elevation-max", po::value<double>(&g_user_data.lidar_elevation_max)->default_value(g_user_data.lidar_elevation_max), "lidar elevation max")
            ("lidar-num-elevation-lines", po::value<int>(&g_user_data.lidar_num_elevation_lines)->default_value(g_user_data.lidar_num_elevation_lines), "lidar num elevation lines")
            ("init-x", po::value<double>(&g_user_data.init_x)->default_value(g_user_data.init_x), "init x")
            ("init-y", po::value<double>(&g_user_data.init_y)->default_value(g_user_data.init_y), "init y")
            ("init-z", po::value<double>(&g_user_data.init_z)->default_value(g_user_data.init_z), "init z")
            ("init-roll", po::value<double>(&g_user_data.init_roll)->default_value(g_user_data.init_roll), "init roll")
            ("init-pitch", po::value<double>(&g_user_data.init_pitch)->default_value(g_user_data.init_pitch), "init pitch")
            ("init-yaw", po::value<double>(&g_user_data.init_yaw)->default_value(g_user_data.init_yaw), "init yaw");
        // clang-format on

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);

        if (vm.count("help")) {
            std::cout << "Usage: " << argv[0] << " [options] tree_bt_file" << std::endl
                      << desc << std::endl;
            return 0;
        }
        po::notify(vm);
    } catch (std::exception &e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    return RUN_ALL_TESTS();
}
