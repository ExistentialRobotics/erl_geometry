#include "erl_common/test_helper.hpp"
#include "erl_geometry/convex_hull.hpp"

#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/visualization/utility/DrawGeometry.h>
#include <open3d/visualization/visualizer/Visualizer.h>

TEST(ERL_GEOMETRY, ConvexHull) {

    std::filesystem::path gtest_dir = __FILE__;
    std::filesystem::path ply_path = gtest_dir.parent_path() / "bunny.ply";
    std::cout << "ply_path: " << ply_path << std::endl;

    try {
        // auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        // open3d::io::ReadTriangleMeshOptions options;
        // options.enable_post_processing = true;
        // open3d::io::ReadTriangleMesh(ply_path.string(), *mesh, options);
        // mesh->ComputeVertexNormals();
        // auto line_set = open3d::geometry::LineSet::CreateFromTriangleMesh(*mesh);
        // open3d::visualization::DrawGeometries({line_set});

        auto point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        open3d::io::ReadPointCloudOption option;
        option.remove_nan_points = true;
        option.remove_infinite_points = true;
        option.print_progress = true;
        open3d::io::ReadPointCloud(ply_path.string(), *point_cloud, option);
        point_cloud->NormalizeNormals();

        Eigen::Matrix3Xd points(3, point_cloud->points_.size());
        for (std::size_t i = 0; i < point_cloud->points_.size(); ++i) { points.col(static_cast<long>(i)) = point_cloud->points_[i]; }
        Eigen::Matrix3Xl mesh_triangles;
        Eigen::Matrix3Xd mesh_vertices;
        std::vector<long> hull_pt_map;
        erl::geometry::ConvexHull<Eigen::Matrix3Xd>(points, points.cols(), mesh_vertices, mesh_triangles, hull_pt_map);

        std::vector<Eigen::Vector3d> hull_vertices;
        hull_vertices.reserve(mesh_vertices.cols());
        for (long i = 0; i < mesh_vertices.cols(); ++i) { hull_vertices.emplace_back(mesh_vertices.col(i)); }
        std::vector<Eigen::Vector3i> hull_triangles;
        hull_triangles.reserve(mesh_triangles.cols());
        for (long i = 0; i < mesh_triangles.cols(); ++i) { hull_triangles.emplace_back(mesh_triangles.col(i).cast<int>()); }
        auto hull_mesh = std::make_shared<open3d::geometry::TriangleMesh>(hull_vertices, hull_triangles);

        erl::geometry::ConvexHull<Eigen::Matrix3Xd>(points, points.cols(), hull_pt_map);
        auto hull_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        hull_point_cloud->points_.reserve(hull_pt_map.size());
        for (auto &i: hull_pt_map) { hull_point_cloud->points_.emplace_back(point_cloud->points_[i]); }

        // // computed by Open3D
        // auto bunny_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        // open3d::io::ReadTriangleMeshOptions options;
        // options.enable_post_processing = true;
        // open3d::io::ReadTriangleMesh(ply_path.string(), *bunny_mesh, options);
        // auto hull_mesh2 = std::get<0>(point_cloud->ComputeConvexHull());

        bool show_bunny = true;
        bool show_hull = true;
        bool show_hull_point_cloud = true;
        int shown_geometries = 3;

        std::map<int, std::function<bool(open3d::visualization::Visualizer *)>> key_to_callback;
        key_to_callback[static_cast<int>('5')] = [&](open3d::visualization::Visualizer *vis) {
            if (show_bunny) {
                if (shown_geometries == 1) {
                    std::cout << "Cannot remove the last geometry." << std::endl;
                    return true;
                }
                vis->RemoveGeometry(point_cloud);
                shown_geometries -= 1;
            } else {
                vis->AddGeometry(point_cloud);
                shown_geometries += 1;
            }
            show_bunny = !show_bunny;
            return true;
        };
        key_to_callback[static_cast<int>('6')] = [&](open3d::visualization::Visualizer *vis) {
            if (show_hull) {
                if (shown_geometries == 1) {
                    std::cout << "Cannot remove the last geometry." << std::endl;
                    return true;
                }
                vis->RemoveGeometry(hull_mesh);
                shown_geometries -= 1;
            } else {
                vis->AddGeometry(hull_mesh);
                shown_geometries += 1;
            }
            show_hull = !show_hull;
            return true;
        };
        key_to_callback[static_cast<int>('7')] = [&](open3d::visualization::Visualizer *vis) {
            if (show_hull_point_cloud) {
                if (shown_geometries == 1) {
                    std::cout << "Cannot remove the last geometry." << std::endl;
                    return true;
                }
                vis->RemoveGeometry(hull_point_cloud);
                shown_geometries -= 1;
            } else {
                vis->AddGeometry(hull_point_cloud);
                shown_geometries += 1;
            }
            show_hull_point_cloud = !show_hull_point_cloud;
            return true;
        };

        open3d::visualization::DrawGeometriesWithKeyCallbacks({point_cloud, hull_mesh, hull_point_cloud}, key_to_callback, "test convex hull");

    } catch (std::exception &e) { std::cout << e.what() << std::endl; }
}
