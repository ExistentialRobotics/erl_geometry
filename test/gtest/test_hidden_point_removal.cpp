#include "erl_common/test_helper.hpp"
#include "erl_geometry/hidden_point_removal.hpp"
#include "erl_geometry/occupancy_octree.hpp"

#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/t/geometry/RaycastingScene.h>
#include <open3d/t/geometry/TriangleMesh.h>
#include <open3d/visualization/utility/DrawGeometry.h>
#include <open3d/visualization/visualizer/Visualizer.h>

struct OctreeNode : public erl::geometry::OccupancyOctreeNode {
    std::size_t geometry_id = -1;
    std::size_t vertex_id = -1;

    explicit OctreeNode(
        const uint32_t depth = 0,
        const int child_index = -1,
        const float log_odds = 0)
        : OccupancyOctreeNode(depth, child_index, log_odds) {}
};

class Octree
    : public erl::geometry::
          OccupancyOctreeBase<double, OctreeNode, erl::geometry::OccupancyOctreeBaseSetting> {

public:
    using Super = OccupancyOctreeBase;
    using Setting = erl::geometry::OccupancyOctreeBaseSetting;

    Octree()
        : Octree(std::make_shared<Setting>()) {}

    explicit Octree(const std::shared_ptr<Setting> &setting)
        : OccupancyOctreeBase(setting) {}

protected:
    [[nodiscard]] std::shared_ptr<AbstractOctree>
    Create(const std::shared_ptr<erl::geometry::NdTreeSetting> &setting) const override {
        auto casted_setting = std::dynamic_pointer_cast<Setting>(setting);
        ERL_ASSERTM(casted_setting != nullptr, "Failed to cast setting to Octree::Setting.");
        return std::make_shared<Octree>(casted_setting);
    }
};

TEST(HiddenPointRemoval, Basic) {

    std::filesystem::path gtest_dir = __FILE__;
    gtest_dir = gtest_dir.parent_path();
    std::filesystem::path ply_path = gtest_dir / "bunny.ply";
    std::cout << "ply_path: " << ply_path << std::endl;

    enum class Mode {
        kOpen3DHiddenPointRemoval = 1,
        kErlHiddenPointRemoval,
        kOpen3DRaycastingScene,
        kOctomapRaytracing,
    };

    try {
        Mode mode = Mode::kErlHiddenPointRemoval;
        bool show_rays = true;

        auto camera_mesh = open3d::geometry::TriangleMesh::CreateSphere(0.005);

        Eigen::Vector3d default_mesh_color(0.5, 0.5, 0.5);
        auto bunny_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        {
            open3d::io::ReadTriangleMeshOptions option;
            option.enable_post_processing = true;
            open3d::io::ReadTriangleMesh(ply_path.string(), *bunny_mesh, option);
        }
        bunny_mesh->ComputeTriangleNormals();
        bunny_mesh->vertex_colors_.resize(bunny_mesh->vertices_.size(), default_mesh_color);

        auto point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        {
            open3d::io::ReadPointCloudOption option;
            option.remove_nan_points = true;
            option.remove_infinite_points = true;
            option.print_progress = true;
            open3d::io::ReadPointCloud(ply_path.string(), *point_cloud, option);
        }
        point_cloud->NormalizeNormals();

        Eigen::Matrix3Xd points(3, point_cloud->points_.size());
        Eigen::Vector3d camera_position(0, 0, 0);
        long point_cnt = 0;
        for (const Eigen::Vector3d &point: point_cloud->points_) {
            points.col(point_cnt++) = point;
            camera_position += point;
        }
        camera_position /= static_cast<double>(point_cnt);
        camera_mesh->Translate(camera_position);

        Eigen::Vector3d bunny_size = bunny_mesh->GetMaxBound() - bunny_mesh->GetMinBound();
        double bunny_scale = bunny_size.maxCoeff();
        Eigen::Vector3d bunny_position = bunny_mesh->GetCenter();

        auto rays = std::make_shared<open3d::geometry::LineSet>();
        auto visible_mesh = std::make_shared<open3d::geometry::TriangleMesh>();

        open3d::t::geometry::RaycastingScene scene;
        auto bunny_id =
            scene.AddTriangles(open3d::t::geometry::TriangleMesh::FromLegacy(*bunny_mesh));

        auto octree_setting = std::make_shared<Octree::Setting>();
        octree_setting->resolution = 0.001;
        Octree octree(octree_setting);
        for (std::size_t i = 0; i < bunny_mesh->vertices_.size(); ++i) {
            Eigen::Vector3d &vertex = bunny_mesh->vertices_[i];
            auto node = octree.UpdateNode(vertex[0], vertex[1], vertex[2], 20.0f, true);
            node->geometry_id = 0;
            node->vertex_id = i;
        }

        auto update_geometries = [&](open3d::visualization::Visualizer *vis = nullptr) {
            double r = (camera_position - bunny_position).norm();
            r = std::max(r, 0.001);
            double radius_scale = bunny_scale * r * r / 0.001;
            radius_scale = std::max(radius_scale, 10.0);

            switch (mode) {
                case Mode::kOpen3DHiddenPointRemoval: {
                    double radius = std::accumulate(  //
                        point_cloud->points_.begin(),
                        point_cloud->points_.end(),
                        0.0,
                        [&](const double acc, const Eigen::Vector3d &point) {
                            return std::max(acc, (point - camera_position).norm());
                        });
                    radius *= radius_scale;
                    std::cout << "point cloud center: "  //
                              << point_cloud->GetCenter().transpose() << std::endl;
                    std::cout << "camera position: " << camera_position.transpose() << std::endl;
                    std::cout << "radius: " << radius << std::endl;
                    std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
                    std::vector<std::size_t> visible_point_indices;
                    std::tie(mesh, visible_point_indices) =
                        point_cloud->HiddenPointRemoval(camera_position, radius);
                    visible_mesh->vertices_ = mesh->vertices_;
                    visible_mesh->triangles_ = mesh->triangles_;
                    rays->points_.resize(1 + mesh->vertices_.size());
                    rays->lines_.resize(mesh->vertices_.size());
                    rays->points_[0] = camera_position;
                    std::fill(
                        bunny_mesh->vertex_colors_.begin(),
                        bunny_mesh->vertex_colors_.end(),
                        default_mesh_color);  // reset the color
                    for (long i = 0; i < static_cast<long>(mesh->vertices_.size()); ++i) {
                        rays->points_[i + 1] = mesh->vertices_[i];
                        rays->lines_[i] = Eigen::Vector2i(0, i + 1);
                        bunny_mesh->vertex_colors_[visible_point_indices[i]] =
                            Eigen::Vector3d(1, 0, 0);
                    }
                    rays->colors_.resize(rays->lines_.size(), Eigen::Vector3d(0, 0, 1));
                    break;
                }
                case Mode::kErlHiddenPointRemoval: {
                    // compute visible points
                    std::vector<long> visible_point_indices;
                    erl::geometry::HiddenPointRemoval<double>(
                        points,
                        camera_position,
                        radius_scale,
                        visible_point_indices,
                        true);
                    // update rays, visible_mesh
                    rays->points_.resize(1 + visible_point_indices.size());
                    rays->lines_.resize(visible_point_indices.size());
                    rays->points_[0] = camera_position;
                    std::fill(
                        bunny_mesh->vertex_colors_.begin(),
                        bunny_mesh->vertex_colors_.end(),
                        default_mesh_color);  // reset the color
                    for (long i = 0; i < static_cast<long>(visible_point_indices.size()); ++i) {
                        rays->points_[i + 1] = points.col(visible_point_indices[i]);
                        rays->lines_[i] = Eigen::Vector2i(0, i + 1);
                        bunny_mesh->vertex_colors_[visible_point_indices[i]] =
                            Eigen::Vector3d(1, 0, 0);
                    }
                    rays->colors_.resize(rays->lines_.size(), Eigen::Vector3d(0, 0, 1));
                    break;
                }
                case Mode::kOpen3DRaycastingScene: {
                    std::vector<float> query_rays_data(6 * bunny_mesh->triangles_.size());
                    for (std::size_t i = 0; i < bunny_mesh->triangles_.size(); ++i) {
                        std::size_t ii = i * 6;
                        query_rays_data[ii + 0] = static_cast<float>(camera_position[0]);
                        query_rays_data[ii + 1] = static_cast<float>(camera_position[1]);
                        query_rays_data[ii + 2] = static_cast<float>(camera_position[2]);
                        Eigen::Vector3i triangle = bunny_mesh->triangles_[i];
                        Eigen::Vector3d pos = (bunny_mesh->vertices_[triangle[0]] +  //
                                               bunny_mesh->vertices_[triangle[1]] +  //
                                               bunny_mesh->vertices_[triangle[2]]) /
                                              3.0;
                        Eigen::Vector3d dir = pos - camera_position;
                        dir.normalize();
                        query_rays_data[ii + 3] = static_cast<float>(dir[0]);
                        query_rays_data[ii + 4] = static_cast<float>(dir[1]);
                        query_rays_data[ii + 5] = static_cast<float>(dir[2]);
                    }
                    auto num_rays = static_cast<long>(bunny_mesh->triangles_.size());
                    open3d::core::Tensor query_rays(
                        query_rays_data,
                        {num_rays, 6},
                        open3d::core::Dtype::Float32);
                    auto result = scene.CastRays(
                        query_rays,
                        static_cast<int>(std::thread::hardware_concurrency()));
                    auto ts_hit = result["t_hit"].ToFlatVector<float>();
                    auto geometry_ids = result["geometry_ids"].ToFlatVector<uint32_t>();
                    auto primitive_ids = result["primitive_ids"].ToFlatVector<uint32_t>();
                    std::fill(
                        bunny_mesh->vertex_colors_.begin(),
                        bunny_mesh->vertex_colors_.end(),
                        default_mesh_color);  // reset the color
                    auto invalid_id = open3d::t::geometry::RaycastingScene::INVALID_ID();
                    rays->points_.resize(1 + num_rays);
                    rays->lines_.resize(num_rays);
                    rays->points_[0] = camera_position;
                    for (long i = 0; i < num_rays; ++i) {
                        if (std::isinf(ts_hit[i])) { continue; }
                        if (geometry_ids[i] != bunny_id) { continue; }
                        auto primitive_id = primitive_ids[i];
                        if (primitive_id == invalid_id) { continue; }
                        Eigen::Vector3i triangle = bunny_mesh->triangles_[primitive_id];
                        bunny_mesh->vertex_colors_[triangle[0]] = Eigen::Vector3d(1, 0, 0);
                        bunny_mesh->vertex_colors_[triangle[1]] = Eigen::Vector3d(1, 0, 0);
                        bunny_mesh->vertex_colors_[triangle[2]] = Eigen::Vector3d(1, 0, 0);

                        rays->points_[i + 1] = bunny_mesh->vertices_[triangle[0]];
                        rays->lines_[i] = Eigen::Vector2i(0, i + 1);
                    }
                    rays->colors_.resize(rays->lines_.size(), Eigen::Vector3d(0, 0, 1));
                    break;
                }
                case Mode::kOctomapRaytracing: {
                    std::fill(
                        bunny_mesh->vertex_colors_.begin(),
                        bunny_mesh->vertex_colors_.end(),
                        default_mesh_color);  // reset the color
                    std::vector<Eigen::Vector3d> ends(bunny_mesh->vertices_.size());
                    std::vector<int> hits(bunny_mesh->vertices_.size(), -1);
#pragma omp parallel for default(none) shared(bunny_mesh, octree, ends, camera_position, hits)
                    for (std::size_t i = 0; i < bunny_mesh->vertices_.size(); ++i) {
                        Eigen::Vector3d dir = bunny_mesh->vertices_[i] - camera_position;
                        dir.normalize();
                        if (octree.CastRay(
                                camera_position[0],
                                camera_position[1],
                                camera_position[2],
                                dir[0],
                                dir[1],
                                dir[2],
                                true,
                                -1,
                                ends[i][0],
                                ends[i][1],
                                ends[i][2])) {
                            erl::geometry::OctreeKey key =
                                octree.CoordToKey(ends[i][0], ends[i][1], ends[i][2]);
                            if (auto node = octree.Search(key); node->geometry_id == 0) {
                                bunny_mesh->vertex_colors_[node->vertex_id] =
                                    Eigen::Vector3d(1, 0, 0);
                                hits[i] = static_cast<int>(node->vertex_id);
                            }
                        }
                    }
                    rays->points_.clear();
                    rays->lines_.clear();
                    rays->points_.reserve(bunny_mesh->vertices_.size() + 1);
                    rays->lines_.reserve(bunny_mesh->vertices_.size());
                    rays->points_.push_back(camera_position);
                    for (std::size_t i = 0; i < bunny_mesh->vertices_.size(); ++i) {
                        if (hits[i] <= -1) { continue; }
                        rays->lines_.emplace_back(0, rays->points_.size());
                        rays->points_.push_back(bunny_mesh->vertices_[hits[i]]);
                    }
                    rays->points_.shrink_to_fit();
                    rays->lines_.shrink_to_fit();
                    rays->colors_.resize(rays->lines_.size(), Eigen::Vector3d(0, 0, 1));
                    break;
                }
            }

            if (vis) {
                vis->UpdateGeometry(camera_mesh);
                vis->UpdateGeometry(bunny_mesh);
                if (show_rays) { vis->UpdateGeometry(rays); }
            }
        };
        update_geometries();

        // Open3D use GLFW key codes: https://www.glfw.org/docs/latest/group__keys.html
        double step = 0.001;
        std::map<int, std::function<bool(open3d::visualization::Visualizer *)>> key_to_callback;
        key_to_callback[GLFW_KEY_UP] = [&](open3d::visualization::Visualizer *vis) -> bool {
            std::cout << "Move camera up" << std::endl;
            camera_position[1] += step;  // y-axis is up
            for (auto &point: camera_mesh->vertices_) { point[1] += step; }
            update_geometries(vis);
            return true;
        };
        key_to_callback[GLFW_KEY_DOWN] = [&](open3d::visualization::Visualizer *vis) -> bool {
            std::cout << "Move camera down" << std::endl;
            camera_position[1] -= step;
            for (auto &point: camera_mesh->vertices_) { point[1] -= step; }
            update_geometries(vis);
            return true;
        };
        key_to_callback[GLFW_KEY_J] = [&](open3d::visualization::Visualizer *vis) -> bool {
            std::cout << "Move camera left" << std::endl;
            camera_position[0] -= step;
            for (auto &point: camera_mesh->vertices_) { point[0] -= step; }
            update_geometries(vis);
            return true;
        };
        key_to_callback[GLFW_KEY_L] = [&](open3d::visualization::Visualizer *vis) -> bool {
            std::cout << "Move camera right" << std::endl;
            camera_position[0] += step;
            for (auto &point: camera_mesh->vertices_) { point[0] += step; }
            update_geometries(vis);
            return true;
        };
        key_to_callback[GLFW_KEY_I] = [&](open3d::visualization::Visualizer *vis) -> bool {
            std::cout << "Move camera forward" << std::endl;
            camera_position[2] += step;
            for (auto &point: camera_mesh->vertices_) { point[2] += step; }
            update_geometries(vis);
            return true;
        };
        key_to_callback[GLFW_KEY_K] = [&](open3d::visualization::Visualizer *vis) -> bool {
            std::cout << "Move camera backward" << std::endl;
            camera_position[2] -= step;
            for (auto &point: camera_mesh->vertices_) { point[2] -= step; }
            update_geometries(vis);
            return true;
        };
        key_to_callback[GLFW_KEY_F2] = [&](open3d::visualization::Visualizer *vis) -> bool {
            std::cout << "Toggle rays" << std::endl;
            if (show_rays) {
                vis->RemoveGeometry(rays);
            } else {
                vis->AddGeometry(rays);
            }
            show_rays = !show_rays;
            update_geometries(vis);
            return true;
        };
        key_to_callback[GLFW_KEY_F3] = [&](open3d::visualization::Visualizer *vis) -> bool {
            std::cout << "Toggle mode" << std::endl;
            switch (mode) {
                case Mode::kOpen3DHiddenPointRemoval: {
                    mode = Mode::kErlHiddenPointRemoval;
                    std::cout << "Mode: kERL_HiddenPointRemoval" << std::endl;
                    break;
                }
                case Mode::kErlHiddenPointRemoval: {
                    mode = Mode::kOpen3DRaycastingScene;
                    std::cout << "Mode: kOpen3D_RaycastingScene" << std::endl;
                    break;
                }
                case Mode::kOpen3DRaycastingScene: {
                    mode = Mode::kOctomapRaytracing;
                    std::cout << "Mode: kOctomap_Raytracing" << std::endl;
                    break;
                }
                case Mode::kOctomapRaytracing: {
                    mode = Mode::kOpen3DHiddenPointRemoval;
                    std::cout << "Mode: kOpen3D_HiddenPointRemoval" << std::endl;
                    break;
                }
            }
            update_geometries(vis);
            return true;
        };

        std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries = {
            camera_mesh,
            bunny_mesh};
        if (show_rays) { geometries.push_back(rays); }
        open3d::visualization::DrawGeometriesWithKeyCallbacks(
            geometries,
            key_to_callback,
            "test hidden point removal",
            1920,
            1080);
    } catch (const std::exception &e) { std::cerr << e.what() << std::endl; }
}
