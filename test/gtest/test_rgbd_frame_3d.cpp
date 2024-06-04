#include "erl_common/test_helper.hpp"
#include "erl_geometry/rgbd_frame_3d.hpp"

#include <open3d/geometry/TriangleMesh.h>
#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/t/geometry/TriangleMesh.h>
#include <open3d/t/geometry/RaycastingScene.h>
#include <open3d/visualization/visualizer/Visualizer.h>
#include <open3d/visualization/utility/DrawGeometry.h>

TEST(ERL_GEOMETRY, RgbdFrame3D) {
    using namespace erl::common;
    using namespace erl::geometry;

    std::filesystem::path gtest_dir = __FILE__;
    gtest_dir = gtest_dir.parent_path();
    std::filesystem::path ply_path = gtest_dir / "replica-office-0.ply";
    std::filesystem::path depth_dir = gtest_dir / "replica_office_0_depth";
    int num_depth_files = 10;
    std::vector<std::string> depth_files;
    depth_files.reserve(10);
    for (int i = 0; i < num_depth_files; ++i) {
        std::stringstream ss;
        ss << "depth" << std::setw(6) << std::setfill('0') << i << ".png";
        depth_files.emplace_back(ss.str());
    }
    std::cout << "ply_path: " << ply_path << std::endl;
    std::cout << "depth_dir: " << depth_dir << std::endl;
    Eigen::MatrixXd traj_data = LoadEigenMatrixFromTextFile<double>((depth_dir / "traj.csv").string(), EigenTextFormat::kCsvFmt);

    try {
        int cur_depth_file = 0;
        auto room_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        {
            open3d::io::ReadTriangleMeshOptions options;
            options.enable_post_processing = true;
            open3d::io::ReadTriangleMesh(ply_path.string(), *room_mesh, options);
            room_mesh->ComputeTriangleNormals();
        }
        auto rgbd_rays_line_set = std::make_shared<open3d::geometry::LineSet>();
        auto rgbd_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
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

        bool show_rgbd_rays = true;
        bool show_rgbd_points = true;
        bool show_surface_samples = false;
        bool show_region_samples = false;
        bool show_along_ray_samples = false;
        bool down_sample = false;
        double down_sample_factor = 4.0;
        auto update_render = [&](open3d::visualization::Visualizer *vis) {
            auto render_tic = std::chrono::high_resolution_clock::now();
            if (show_rgbd_rays || show_rgbd_points || show_surface_samples || show_region_samples || show_along_ray_samples) {
                // r-zyx order
                Eigen::Matrix4d pose = traj_data.row(cur_depth_file).reshaped(4, 4).transpose();
                Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
                Eigen::Vector3d camera_position = pose.block<3, 1>(0, 3);
                auto rgbd_frame_3d_setting = std::make_shared<RgbdFrame3D::Setting>();
                auto rgbd_frame_3d = std::make_shared<RgbdFrame3D>(rgbd_frame_3d_setting);
                if (show_rgbd_rays || show_rgbd_points || !surface_samples_ready || !region_samples_ready || !along_ray_samples_ready) {
                    cv::Mat depth_img = cv::imread((depth_dir / depth_files[cur_depth_file]).string(), cv::IMREAD_UNCHANGED);
                    if (down_sample) {
                        double factor = 1 / down_sample_factor;
                        auto [new_height, new_width] = rgbd_frame_3d->Resize(factor);
                        cv::resize(depth_img, depth_img, cv::Size(new_width, new_height), 0, 0, cv::INTER_NEAREST);
                    }
                    depth_img.convertTo(depth_img, CV_64FC1);  // convert to double
                    Eigen::MatrixXd depth;
                    cv::cv2eigen(depth_img, depth);
                    std::cout << "rgbd image size: " << depth.rows() << " x " << depth.cols() << std::endl;
                    auto tic = std::chrono::high_resolution_clock::now();
                    rgbd_frame_3d->Update(rotation, camera_position, depth, false, false);  // depth not scaled yet, do not partition rays
                    auto toc = std::chrono::high_resolution_clock::now();
                    std::cout << "rgbd_frame_3d->Update takes " << std::chrono::duration<double, std::milli>(toc - tic).count() << " ms" << std::endl;
                }
                long num_azimuths = rgbd_frame_3d->GetNumAzimuthLines();
                long num_elevations = rgbd_frame_3d->GetNumElevationLines();
                long max_num_valid_rays = num_azimuths * num_elevations;

                if (show_rgbd_rays) {
                    rgbd_rays_line_set->Clear();
                    rgbd_rays_line_set->points_.reserve(max_num_valid_rays + 1);
                    rgbd_rays_line_set->lines_.reserve(max_num_valid_rays);
                    rgbd_rays_line_set->colors_.reserve(max_num_valid_rays);
                    rgbd_rays_line_set->points_.push_back(camera_position);
                }
                if (show_rgbd_points) {
                    rgbd_point_cloud->Clear();
                    rgbd_point_cloud->points_.reserve(max_num_valid_rays);
                    rgbd_point_cloud->colors_.reserve(max_num_valid_rays);
                }
                const Eigen::MatrixX<Eigen::Vector3d> &end_points_in_world = rgbd_frame_3d->GetEndPointsInWorld();
                const Eigen::MatrixXb &hit_mask = rgbd_frame_3d->GetHitMask();
                for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
                    long ray_idx_base = azimuth_idx * num_elevations;
                    for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
                        long ray_idx = ray_idx_base + elevation_idx;
                        if (!hit_mask(azimuth_idx, elevation_idx)) { continue; }
                        const Eigen::Vector3d &end_pt = end_points_in_world(azimuth_idx, elevation_idx);
                        if (show_rgbd_rays) {
                            rgbd_rays_line_set->points_.push_back(end_pt);
                            rgbd_rays_line_set->lines_.emplace_back(0, ray_idx + 1);
                            rgbd_rays_line_set->colors_.emplace_back(0.0, 1.0, 0.0);
                        }
                        if (show_rgbd_points) {
                            rgbd_point_cloud->points_.push_back(end_pt);
                            rgbd_point_cloud->colors_.emplace_back(1.0, 0.0, 0.0);
                        }
                    }
                }

                if (show_surface_samples && !surface_samples_ready) {
                    Eigen::Matrix3Xd sampled_positions, sampled_directions;
                    Eigen::VectorXd sampled_distances;
                    auto tic = std::chrono::high_resolution_clock::now();
                    rgbd_frame_3d->SampleNearSurface(10, 0.05, 1.0, sampled_positions, sampled_directions, sampled_distances);
                    auto toc = std::chrono::high_resolution_clock::now();
                    long num_samples = sampled_positions.cols();
                    std::cout << "============================" << std::endl
                              << "SampleNearSurface: " << std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count() << " ms" << std::endl
                              << num_samples << " surface samples" << std::endl;
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
                    long num_positions = 20;
                    long num_near_surface_samples_per_ray = 5;
                    long num_along_ray_samples_per_ray = 10;
                    double max_in_obstacle_dist = 0.05;
                    std::cout << "num_samples: " << num_positions << std::endl
                              << "num_near_surface_samples_per_ray: " << num_near_surface_samples_per_ray << std::endl
                              << "num_along_ray_samples_per_ray: " << num_along_ray_samples_per_ray << std::endl
                              << "max_in_obstacle_dist: " << max_in_obstacle_dist << std::endl;
                    auto tic = std::chrono::high_resolution_clock::now();
                    rgbd_frame_3d->SampleInRegionHpr(
                        num_positions,
                        num_near_surface_samples_per_ray,
                        num_along_ray_samples_per_ray,
                        max_in_obstacle_dist,
                        sampled_positions,
                        sampled_directions,
                        sampled_distances,
                        true);
                    auto toc = std::chrono::high_resolution_clock::now();
                    std::cout << "SampleInRegion: " << std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count() << " ms" << std::endl;
                    region_samples_line_set->Clear();
                    region_samples_line_set->points_.reserve(sampled_positions.cols() * 2);
                    region_samples_line_set->lines_.reserve(sampled_positions.cols());
                    region_samples_line_set->colors_.reserve(sampled_positions.cols());
                    region_samples_point_cloud->Clear();
                    region_samples_point_cloud->points_.reserve(sampled_positions.cols());
                    region_samples_point_cloud->colors_.reserve(sampled_positions.cols());
                    for (long i = 0; i < sampled_positions.cols(); ++i) {
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
                    rgbd_frame_3d->SampleAlongRays(range_step, max_in_obstacle_dist, 1.0, sampled_positions, sampled_directions, sampled_distances);
                    auto toc = std::chrono::high_resolution_clock::now();
                    long num_samples = sampled_positions.cols();
                    std::cout << "============================" << std::endl
                              << "SampleAlongRays (fixed range step): " << std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count() << " ms"
                              << std::endl
                              << num_samples << " samples" << std::endl;
                    long num_hit_rays = rgbd_frame_3d->GetNumHitRays();
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
                    rgbd_frame_3d->SampleAlongRays(num_samples_per_ray, max_in_obstacle_dist, 1.0, sampled_positions, sampled_directions, sampled_distances);
                    toc = std::chrono::high_resolution_clock::now();
                    long collected_num_samples = num_samples;
                    num_samples = sampled_positions.cols();
                    std::cout << "SampleAlongRays (fixed num samples per ray): " << std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count()
                              << " ms" << std::endl
                              << num_samples << " samples" << std::endl;
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
            if (show_rgbd_rays) { vis->UpdateGeometry(rgbd_rays_line_set); }
            if (show_rgbd_points) { vis->UpdateGeometry(rgbd_point_cloud); }
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

        std::map<int, std::function<bool(open3d::visualization::Visualizer *)>> key_to_callback;
        key_to_callback[GLFW_KEY_F1] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (show_rgbd_rays) {
                vis->RemoveGeometry(rgbd_rays_line_set);
                std::cout << "rgbd rays removed." << std::endl;
            } else {
                vis->AddGeometry(rgbd_rays_line_set, false);
                std::cout << "rgbd rays added." << std::endl;
            }
            show_rgbd_rays = !show_rgbd_rays;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_F2] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (show_rgbd_points) {
                vis->RemoveGeometry(rgbd_point_cloud);
                std::cout << "rgbd points removed." << std::endl;
            } else {
                vis->AddGeometry(rgbd_point_cloud, false);
                std::cout << "rgbd points added." << std::endl;
            }
            show_rgbd_points = !show_rgbd_points;
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
        key_to_callback[GLFW_KEY_F8] = [&](open3d::visualization::Visualizer *vis) -> bool {
            down_sample = !down_sample;
            std::cout << "down_sample: " << down_sample << std::endl << "down_sample_factor: " << down_sample_factor << std::endl;
            surface_samples_ready = false;
            region_samples_ready = false;
            along_ray_samples_ready = false;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_LEFT] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (cur_depth_file == 0) { return false; }
            --cur_depth_file;
            surface_samples_ready = false;
            region_samples_ready = false;
            along_ray_samples_ready = false;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_RIGHT] = [&](open3d::visualization::Visualizer *vis) -> bool {
            if (cur_depth_file == num_depth_files - 1) { return false; }
            ++cur_depth_file;
            surface_samples_ready = false;
            region_samples_ready = false;
            along_ray_samples_ready = false;
            update_render(vis);  // notify vis to update
            return true;
        };
        key_to_callback[GLFW_KEY_H] = [&](open3d::visualization::Visualizer *vis) -> bool {
            vis->PrintVisualizerHelp();
            std::cout << "[H]: print help." << std::endl
                      << "[Right arrow]: previous rgbd frame." << std::endl
                      << "[Left arrow]: next rgbd frame." << std::endl
                      << "[F1]: toggle rgbd rays." << std::endl
                      << "[F2]: toggle rgbd points." << std::endl
                      << "[F3]: toggle surface samples." << std::endl
                      << "[F4]: toggle region samples." << std::endl
                      << "[F5]: toggle along ray samples." << std::endl
                      << "[F6]: toggle sample rays." << std::endl
                      << "[F7]: toggle sample points." << std::endl
                      << "[F8]: down sample RGBD frame." << std::endl;
            return true;
        };

        std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries = {room_mesh};
        if (show_rgbd_rays) { geometries.emplace_back(rgbd_rays_line_set); }
        if (show_rgbd_points) { geometries.emplace_back(rgbd_point_cloud); }
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
