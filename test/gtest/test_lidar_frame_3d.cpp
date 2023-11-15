#include "erl_common/test_helper.hpp"
#include "erl_common/angle_utils.hpp"
#include "erl_geometry/lidar_3d.hpp"
#include "erl_geometry/lidar_frame_3d.hpp"

#include <open3d/geometry/TriangleMesh.h>
#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/t/geometry/TriangleMesh.h>
#include <open3d/t/geometry/RaycastingScene.h>
#include <open3d/visualization/visualizer/Visualizer.h>
#include <open3d/visualization/utility/DrawGeometry.h>

TEST(ERL_GEOMETRY, LidarFrame3D) {
    using namespace erl::common;
    using namespace erl::geometry;

    std::filesystem::path gtest_dir = __FILE__;
    gtest_dir = gtest_dir.parent_path();
    std::filesystem::path ply_path = gtest_dir / "replica-office-0.ply";
    std::cout << "ply_path: " << ply_path << std::endl;

    try {
        auto room_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        {
            open3d::io::ReadTriangleMeshOptions options;
            options.enable_post_processing = true;
            open3d::io::ReadTriangleMesh(ply_path.string(), *room_mesh, options);
            room_mesh->ComputeTriangleNormals();
        }
        auto lidar_rays_line_set = std::make_shared<open3d::geometry::LineSet>();
        auto lidar_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        auto surface_samples_line_set = std::make_shared<open3d::geometry::LineSet>();
        auto surface_samples_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        auto region_samples_line_set = std::make_shared<open3d::geometry::LineSet>();
        auto region_samples_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        auto along_ray_samples_line_set = std::make_shared<open3d::geometry::LineSet>();
        auto along_ray_samples_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        bool surface_samples_ready = false;
        bool region_samples_ready = false;
        bool along_ray_samples_ready = false;
        bool show_sample_rays = true;
        bool show_sample_points = true;

        auto o3d_scene = std::make_shared<open3d::t::geometry::RaycastingScene>();
        o3d_scene->AddTriangles(open3d::t::geometry::TriangleMesh::FromLegacy(*room_mesh));

        auto lidar_3d_setting = std::make_shared<Lidar3D::Setting>();
        auto lidar_3d = std::make_shared<Lidar3D>(lidar_3d_setting, o3d_scene);

        double lidar_roll = 0.0;
        double lidar_pitch = 0.0;
        double lidar_yaw = 0.0;
        Eigen::Vector3d lidar_position = room_mesh->GetCenter();
        Eigen::MatrixX<Eigen::Vector3d> lidar_ray_dirs = lidar_3d->GetRayDirectionsInFrame();
        Eigen::VectorXd azimuths = lidar_3d->GetAzimuthAngles();
        Eigen::VectorXd elevations = lidar_3d->GetElevationAngles();

        bool show_lidar_rays = true;
        bool show_lidar_points = true;
        bool show_surface_samples = false;
        bool show_region_samples = false;
        bool show_along_ray_samples = false;
        auto update_render = [&](open3d::visualization::Visualizer *vis) {
            auto render_tic = std::chrono::high_resolution_clock::now();
            if (show_lidar_rays || show_lidar_points || show_surface_samples || show_region_samples || show_along_ray_samples) {
                // r-zyx order
                Eigen::Quaterniond orientation = Eigen::AngleAxisd(lidar_yaw, Eigen::Vector3d::UnitZ()) *    // yaw
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
                        double &range = ranges(azimuth_idx, elevation_idx);
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

                auto lidar_frame_3d_setting = std::make_shared<LidarFrame3D::Setting>();
                auto lidar_frame_3d = std::make_shared<LidarFrame3D>(lidar_frame_3d_setting);
                if (!surface_samples_ready || !region_samples_ready || !along_ray_samples_ready) {
                    auto tic = std::chrono::high_resolution_clock::now();
                    lidar_frame_3d->Update(rotation, lidar_position, azimuths, elevations, ranges, false);
                    auto toc = std::chrono::high_resolution_clock::now();
                    std::cout << "lidar_frame_3d->Update takes " << std::chrono::duration<double, std::milli>(toc - tic).count() << " ms" << std::endl;
                }

                if (show_surface_samples && !surface_samples_ready) {
                    Eigen::Matrix3Xd sampled_positions, sampled_directions;
                    Eigen::VectorXd sampled_distances;
                    auto tic = std::chrono::high_resolution_clock::now();
                    lidar_frame_3d->SampleNearSurface(10, 0.05, sampled_positions, sampled_directions, sampled_distances);
                    auto toc = std::chrono::high_resolution_clock::now();
                    std::cout << "SampleNearSurface: " << std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count() << " ms" << std::endl;
                    long num_samples = sampled_positions.cols();
                    surface_samples_line_set->Clear();
                    surface_samples_line_set->points_.reserve(num_samples * 2);
                    surface_samples_line_set->lines_.reserve(num_samples);
                    surface_samples_line_set->colors_.reserve(num_samples);
                    surface_samples_point_cloud->Clear();
                    surface_samples_point_cloud->points_.reserve(num_samples);
                    surface_samples_point_cloud->colors_.reserve(num_samples);
                    for (long i = 0; i < num_samples; ++i) {
                        surface_samples_line_set->points_.emplace_back(sampled_positions.col(i));
                        surface_samples_line_set->points_.emplace_back(sampled_positions.col(i) + sampled_directions.col(i) * sampled_distances[i]);
                        surface_samples_line_set->lines_.emplace_back(i * 2, i * 2 + 1);
                        if (sampled_distances[i] > 0) {
                            surface_samples_line_set->colors_.emplace_back(0.0, 0.0, 1.0);
                        } else {
                            surface_samples_line_set->colors_.emplace_back(1.0, 0.5, 0.0);
                        }
                        surface_samples_point_cloud->points_.emplace_back(sampled_positions.col(i));
                        surface_samples_point_cloud->colors_.emplace_back(0.0, 1.0, 1.0);
                    }
                    surface_samples_ready = true;
                }

                if (show_region_samples && !region_samples_ready) {
                    Eigen::Matrix3Xd sampled_positions, sampled_directions;
                    Eigen::VectorXd sampled_distances;
                    long num_samples = lidar_3d_setting->num_azimuth_lines * lidar_3d_setting->num_elevation_lines * 2;
                    long num_samples_per_iter = num_samples / 100;
                    std::cout << "num_samples: " << num_samples << std::endl;
                    std::cout << "num_samples_per_iter: " << num_samples_per_iter << std::endl;
                    auto tic = std::chrono::high_resolution_clock::now();
                    lidar_frame_3d->SampleInRegion(num_samples, num_samples_per_iter, sampled_positions, sampled_directions, sampled_distances, true);
                    auto toc = std::chrono::high_resolution_clock::now();
                    std::cout << "SampleInRegion: " << std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count() << " ms" << std::endl;
                    region_samples_line_set->Clear();
                    region_samples_line_set->points_.reserve(num_samples * 2);
                    region_samples_line_set->lines_.reserve(num_samples);
                    region_samples_line_set->colors_.reserve(num_samples);
                    region_samples_point_cloud->Clear();
                    region_samples_point_cloud->points_.reserve(num_samples);
                    region_samples_point_cloud->colors_.reserve(num_samples);
                    for (long i = 0; i < num_samples; ++i) {
                        region_samples_line_set->points_.emplace_back(sampled_positions.col(i));
                        region_samples_line_set->points_.emplace_back(sampled_positions.col(i) + sampled_directions.col(i) * sampled_distances[i]);
                        region_samples_line_set->lines_.emplace_back(i * 2, i * 2 + 1);
                        if (sampled_distances[i] > 0) {
                            region_samples_line_set->colors_.emplace_back(0.5, 0.0, 1.0);
                        } else {
                            region_samples_line_set->colors_.emplace_back(1.0, 1.0, 0.0);
                        }
                        region_samples_point_cloud->points_.emplace_back(sampled_positions.col(i));
                        region_samples_point_cloud->colors_.emplace_back(0.0, 1.0, 1.0);
                    }
                    region_samples_ready = true;
                }

                if (show_along_ray_samples && !along_ray_samples_ready) {
                    Eigen::Matrix3Xd sampled_positions, sampled_directions;
                    Eigen::VectorXd sampled_distances;
                    double range_step = 0.2;
                    double max_in_obstacle_dist = 0.05;
                    auto tic = std::chrono::high_resolution_clock::now();
                    lidar_frame_3d->SampleAlongRays(range_step, max_in_obstacle_dist, sampled_positions, sampled_directions, sampled_distances);
                    auto toc = std::chrono::high_resolution_clock::now();
                    std::cout << "SampleAlongRays (fixed range step): " << std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count() << " ms" << std::endl;

                    long num_samples = sampled_positions.cols();
                    long num_hit_rays = lidar_frame_3d->GetNumHitRays();
                    long num_samples_per_ray = 5l;
                    num_samples += num_hit_rays * num_samples_per_ray;
                    along_ray_samples_line_set->Clear();
                    along_ray_samples_line_set->points_.reserve(num_samples * 2);
                    along_ray_samples_line_set->lines_.reserve(num_samples);
                    along_ray_samples_line_set->colors_.reserve(num_samples);
                    along_ray_samples_point_cloud->Clear();
                    along_ray_samples_point_cloud->points_.reserve(num_samples);
                    along_ray_samples_point_cloud->colors_.reserve(num_samples);
                    num_samples = sampled_positions.cols();
                    for (long i = 0; i < num_samples; ++i) {
                        along_ray_samples_line_set->points_.emplace_back(sampled_positions.col(i));
                        along_ray_samples_line_set->points_.emplace_back(sampled_positions.col(i) + sampled_directions.col(i) * sampled_distances[i]);
                        along_ray_samples_line_set->lines_.emplace_back(i * 2, i * 2 + 1);
                        if (sampled_distances[i] > 0) {
                            along_ray_samples_line_set->colors_.emplace_back(0.0, 0.5, 1.0);
                        } else {
                            along_ray_samples_line_set->colors_.emplace_back(1.0, 0.0, 1.0);
                        }
                        along_ray_samples_point_cloud->points_.emplace_back(sampled_positions.col(i));
                        along_ray_samples_point_cloud->colors_.emplace_back(0.0, 1.0, 1.0);
                    }

                    tic = std::chrono::high_resolution_clock::now();
                    lidar_frame_3d->SampleAlongRays(num_samples_per_ray, max_in_obstacle_dist, sampled_positions, sampled_directions, sampled_distances);
                    toc = std::chrono::high_resolution_clock::now();
                    std::cout << "SampleAlongRays (fixed num samples per ray): " << std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count() << " ms" << std::endl;
                    long collected_num_samples = num_samples;
                    num_samples = sampled_positions.cols();
                    for (long i = 0; i < num_samples; ++i) {
                        along_ray_samples_line_set->points_.emplace_back(sampled_positions.col(i));
                        along_ray_samples_line_set->points_.emplace_back(sampled_positions.col(i) + sampled_directions.col(i) * sampled_distances[i]);
                        along_ray_samples_line_set->lines_.emplace_back(collected_num_samples + i * 2, collected_num_samples + i * 2 + 1);
                        if (sampled_distances[i] > 0) {
                            along_ray_samples_line_set->colors_.emplace_back(0.0, 0.5, 1.0);
                        } else {
                            along_ray_samples_line_set->colors_.emplace_back(1.0, 0.0, 1.0);
                        }
                        along_ray_samples_point_cloud->points_.emplace_back(sampled_positions.col(i));
                        along_ray_samples_point_cloud->colors_.emplace_back(0.0, 1.0, 1.0);
                    }
                    along_ray_samples_ready = true;
                }
            }

            if (vis == nullptr) { return; }
            if (show_lidar_rays) { vis->UpdateGeometry(lidar_rays_line_set); }
            if (show_lidar_points) { vis->UpdateGeometry(lidar_point_cloud); }
            if (show_surface_samples) {
                if (show_sample_rays) { vis->UpdateGeometry(surface_samples_line_set); }
                if (show_sample_points) { vis->UpdateGeometry(surface_samples_point_cloud); }
            }
            if (show_region_samples) {
                if (show_sample_rays) { vis->UpdateGeometry(region_samples_line_set); }
                if (show_sample_points) { vis->UpdateGeometry(region_samples_point_cloud); }
            }
            if (show_along_ray_samples) {
                if (show_sample_rays) { vis->UpdateGeometry(along_ray_samples_line_set); }
                if (show_sample_points) { vis->UpdateGeometry(along_ray_samples_point_cloud); }
            }
            auto render_toc = std::chrono::high_resolution_clock::now();
            std::cout << "update_render takes " << std::chrono::duration<double, std::milli>(render_toc - render_tic).count() << " ms" << std::endl;
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
                if (show_sample_points) { vis->RemoveGeometry(surface_samples_point_cloud); }
                std::cout << "surface samples removed." << std::endl;
            } else {
                if (show_sample_rays) { vis->AddGeometry(surface_samples_line_set, false); }
                if (show_sample_points) { vis->AddGeometry(surface_samples_point_cloud, false); }
                std::cout << "surface samples added." << std::endl;
            }
            show_surface_samples = !show_surface_samples;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_F4] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (show_region_samples) {
                if (show_sample_rays) { vis->RemoveGeometry(region_samples_line_set); }
                if (show_sample_points) { vis->RemoveGeometry(region_samples_point_cloud); }
                std::cout << "region samples removed." << std::endl;
            } else {
                if (show_sample_rays) { vis->AddGeometry(region_samples_line_set, false); }
                if (show_sample_points) { vis->AddGeometry(region_samples_point_cloud, false); }
                std::cout << "region samples added." << std::endl;
            }
            show_region_samples = !show_region_samples;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_F5] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (show_along_ray_samples) {
                if (show_sample_rays) { vis->RemoveGeometry(along_ray_samples_line_set); }
                if (show_sample_points) { vis->RemoveGeometry(along_ray_samples_point_cloud); }
                std::cout << "along ray samples removed." << std::endl;
            } else {
                if (show_sample_rays) { vis->AddGeometry(along_ray_samples_line_set, false); }
                if (show_sample_points) { vis->AddGeometry(along_ray_samples_point_cloud, false); }
                std::cout << "along ray samples added." << std::endl;
            }
            show_along_ray_samples = !show_along_ray_samples;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_F6] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (show_sample_rays) {
                if (show_surface_samples) { vis->RemoveGeometry(surface_samples_line_set); }
                if (show_region_samples) { vis->RemoveGeometry(region_samples_line_set); }
                if (show_along_ray_samples) { vis->RemoveGeometry(along_ray_samples_line_set); }
                std::cout << "sample rays removed." << std::endl;
            } else {
                if (show_surface_samples) { vis->AddGeometry(surface_samples_line_set, false); }
                if (show_region_samples) { vis->AddGeometry(region_samples_line_set, false); }
                if (show_along_ray_samples) { vis->AddGeometry(along_ray_samples_line_set, false); }
                std::cout << "sample rays added." << std::endl;
            }
            show_sample_rays = !show_sample_rays;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_F7] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (show_sample_points) {
                if (show_surface_samples) { vis->RemoveGeometry(surface_samples_point_cloud); }
                if (show_region_samples) { vis->RemoveGeometry(region_samples_point_cloud); }
                if (show_along_ray_samples) { vis->RemoveGeometry(along_ray_samples_point_cloud); }
                std::cout << "sample points removed." << std::endl;
            } else {
                if (show_surface_samples) { vis->AddGeometry(surface_samples_point_cloud, false); }
                if (show_region_samples) { vis->AddGeometry(region_samples_point_cloud, false); }
                if (show_along_ray_samples) { vis->AddGeometry(along_ray_samples_point_cloud, false); }
                std::cout << "sample points added." << std::endl;
            }
            show_sample_points = !show_sample_points;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_J] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_roll -= angle_step;
            lidar_roll = ClipAngle(lidar_roll);
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_L] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_roll += angle_step;
            lidar_roll = ClipAngle(lidar_roll);
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_K] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_pitch -= angle_step;
            lidar_pitch = ClipAngle(lidar_pitch);
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_I] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_pitch += angle_step;
            lidar_pitch = ClipAngle(lidar_pitch);
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_U] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_yaw -= angle_step;
            lidar_yaw = ClipAngle(lidar_yaw);
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_O] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_yaw += angle_step;
            lidar_yaw = ClipAngle(lidar_yaw);
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_LEFT] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_position[0] -= translation_step;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_RIGHT] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_position[0] += translation_step;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_DOWN] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_position[1] -= translation_step;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_UP] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_position[1] += translation_step;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_PAGE_DOWN] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_position[2] -= translation_step;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_ready = false;
            along_ray_samples_ready = false;
            return true;
        };
        key_to_callback[GLFW_KEY_PAGE_UP] = [&](open3d::visualization::Visualizer *vis) -> bool {
            lidar_position[2] += translation_step;
            update_render(vis);  // notify vis to update
            surface_samples_ready = false;
            region_samples_ready = false;
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
                      << "[F4]: toggle region samples." << std::endl
                      << "[F5]: toggle along ray samples." << std::endl
                      << "[F6]: toggle sample rays." << std::endl
                      << "[F7]: toggle sample points." << std::endl;
            return true;
        };

        std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries = {room_mesh};
        if (show_lidar_rays) { geometries.emplace_back(lidar_rays_line_set); }
        if (show_lidar_points) { geometries.emplace_back(lidar_point_cloud); }
        if (show_surface_samples) {
            if (show_sample_rays) { geometries.emplace_back(surface_samples_line_set); }
            if (show_sample_points) { geometries.emplace_back(surface_samples_point_cloud); }
        }
        if (show_region_samples) {
            if (show_sample_rays) { geometries.emplace_back(region_samples_line_set); }
            if (show_sample_points) { geometries.emplace_back(region_samples_point_cloud); }
        }
        if (show_along_ray_samples) {
            if (show_sample_rays) { geometries.emplace_back(along_ray_samples_line_set); }
            if (show_sample_points) { geometries.emplace_back(along_ray_samples_point_cloud); }
        }
        open3d::visualization::DrawGeometriesWithKeyCallbacks(geometries, key_to_callback, "test LidarFrame3D");

    } catch (const std::exception &e) { std::cerr << e.what() << std::endl; }
}
