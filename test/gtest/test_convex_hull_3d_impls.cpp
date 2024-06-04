#include "erl_common/test_helper.hpp"
#include "erl_geometry/convex_hull.hpp"

#include <boost/iterator/counting_iterator.hpp>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Extreme_points_traits_adapter_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Surface_mesh.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/visualization/utility/DrawGeometry.h>

#include <filesystem>
#include <iostream>

std::vector<std::size_t>
RunCgalImpl(const std::vector<Eigen::Vector3d> &points) {
    using K = CGAL::Exact_predicates_inexact_constructions_kernel;
    using Point_3 = K::Point_3;

    std::vector<Point_3> cgal_points;
    cgal_points.reserve(points.size());
    std::transform(points.begin(), points.end(), std::back_inserter(cgal_points), [](const Eigen::Vector3d &point) {
        return Point_3(point[0], point[1], point[2]);
    });
    std::vector<std::size_t> extreme_point_indices;
    CGAL::extreme_points_3(
        CGAL::make_range(boost::counting_iterator<std::size_t>(0), boost::counting_iterator<std::size_t>(cgal_points.size())),
        std::back_inserter(extreme_point_indices),
        CGAL::make_extreme_points_traits_adapter(CGAL::make_property_map(cgal_points)));
    return extreme_point_indices;
}

std::vector<long>
RunQhullImpl(const std::vector<Eigen::Vector3d> &points) {
    std::vector<long> indices;
    erl::geometry::ConvexHull<std::vector<Eigen::Vector3d>>(points, points.size(), indices, "Q3 Q5 Q8");
    return indices;
}

std::shared_ptr<open3d::geometry::TriangleMesh>
RunCgalImplMesh(const std::vector<Eigen::Vector3d> &points) {
    using K = CGAL::Exact_predicates_inexact_constructions_kernel;
    using Point_3 = K::Point_3;
    using Mesh = CGAL::Surface_mesh<Point_3>;

    std::vector<Point_3> cgal_points;
    cgal_points.reserve(points.size());
    std::transform(points.begin(), points.end(), std::back_inserter(cgal_points), [](const Eigen::Vector3d &point) {
        return Point_3(point[0], point[1], point[2]);
    });

    Mesh mesh;
    CGAL::convex_hull_3(cgal_points.begin(), cgal_points.end(), mesh);
    if (!CGAL::is_triangle_mesh(mesh)) { CGAL::Polygon_mesh_processing::triangulate_faces(mesh); }

    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;
    for (const auto v: mesh.vertices()) {
        auto point = mesh.point(v);
        vertices.emplace_back(point[0], point[1], point[2]);
    }
    for (const auto f: mesh.faces()) {
        const auto h = mesh.halfedge(f);
        auto v1 = mesh.source(h);
        auto v2 = mesh.target(h);
        auto v3 = mesh.target(mesh.next(h));
        triangles.emplace_back(v1.idx(), v2.idx(), v3.idx());
    }
    auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>(vertices, triangles);
    return o3d_mesh;
}

std::shared_ptr<open3d::geometry::TriangleMesh>
RunQhullImplMesh(const std::vector<Eigen::Vector3d> &points) {
    Eigen::Matrix3Xd mesh_vertices;
    Eigen::Matrix3Xl mesh_triangles;
    std::vector<long> indices;
    erl::geometry::ConvexHull<std::vector<Eigen::Vector3d>>(points, points.size(), mesh_vertices, mesh_triangles, indices, "Q3 Q5 Q8");

    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(mesh_vertices.cols());
    for (long i = 0; i < mesh_vertices.cols(); ++i) { vertices.emplace_back(mesh_vertices.col(i)); }
    std::vector<Eigen::Vector3i> triangles;
    triangles.reserve(mesh_triangles.cols());
    for (long i = 0; i < mesh_triangles.cols(); ++i) { triangles.emplace_back(mesh_triangles.col(i).cast<int>()); }
    auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>(vertices, triangles);
    return o3d_mesh;
}

TEST(ERL_GEOMETRY, ConvexHull3DImpls) {
    GTEST_PREPARE_OUTPUT_DIR();

    std::filesystem::path mesh_path = __FILE__;
    mesh_path = mesh_path.parent_path() / "bunny.ply";

    auto o3d_mesh = open3d::io::CreateMeshFromFile(mesh_path.string());
    std::vector<long> num_points = {static_cast<long>(1E3), static_cast<long>(1E4), static_cast<long>(1E5), static_cast<long>(1E6)};
    std::vector<std::pair<double, double>> timings;
    for (auto &num_point: num_points) {
        auto point_cloud = o3d_mesh->SamplePointsUniformly(num_point);
        ERL_INFO("num_points: {}", num_point);
        double t_cgal = erl::common::ReportTime<std::chrono::milliseconds>("CGAL", 5, false, [&] { RunCgalImpl(point_cloud->points_); });
        double t_qhull = erl::common::ReportTime<std::chrono::milliseconds>("QHull", 5, false, [&] { RunQhullImpl(point_cloud->points_); });
        timings.emplace_back(t_cgal, t_qhull);
    }
    std::cout << "Summary timing (ms):\n"
                 "num_points      CGAL     QHull     Ratio"
              << std::endl;
    for (std::size_t i = 0; i < num_points.size(); ++i) {
        std::cout << std::setfill(' ') << std::setw(10) << num_points[i]      //
                  << std::setfill(' ') << std::setw(10) << timings[i].first   //
                  << std::setfill(' ') << std::setw(10) << timings[i].second  //
                  << std::setfill(' ') << std::setw(10) << timings[i].first / timings[i].second << std::endl;
    }

    // check visualizations
    auto cgal_mesh = RunCgalImplMesh(o3d_mesh->vertices_);
    auto qhull_mesh = RunQhullImplMesh(o3d_mesh->vertices_);
    const double cgal_volume = cgal_mesh->GetVolume();
    const double qhull_volume = qhull_mesh->GetVolume();
    std::cout << "cgal_volume: " << cgal_volume << std::endl;
    std::cout << "qhull_volume: " << qhull_volume << std::endl;
    EXPECT_NEAR(cgal_volume, qhull_volume, 1E-6);

    cgal_mesh->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
    qhull_mesh->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
    o3d_mesh->PaintUniformColor(Eigen::Vector3d(0.0, 0.0, 1.0));
    auto o3d_mesh2 = std::make_shared<open3d::geometry::TriangleMesh>(*o3d_mesh);
    o3d_mesh2->Translate(Eigen::Vector3d(0.25, 0, 0));
    qhull_mesh->Translate(Eigen::Vector3d(0.25, 0, 0));
    open3d::visualization::DrawGeometries({cgal_mesh, o3d_mesh, qhull_mesh, o3d_mesh2});
}

/**
 * Intel Core i7-12700k:
 * Summary timing (ms):
 * num_points      CGAL     QHull     Ratio
 *       1000  0.383243  0.401766  0.953896
 *      10000   3.73169   2.56901   1.45258
 *     100000   34.4254   18.9784   1.81393
 *    1000000   327.108   143.306   2.28259
 * cgal_volume: 0.00124981
 * qhull_volume: 0.00124981
 *
 * Intel Core i9-14900k:
 * Summary timing (ms):
 * num_points      CGAL     QHull     Ratio
 *       1000  0.332376  0.349364  0.951375
 *      10000   3.04356   2.00392    1.5188
 *     100000   27.7558   14.2629   1.94601
 *    1000000   291.891   110.416   2.64355
 * cgal_volume: 0.00124981
 * qhull_volume: 0.00124981
 */
