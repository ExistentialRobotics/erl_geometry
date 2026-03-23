#include "erl_common/test_helper.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/occupancy_octree_drawer.hpp"

#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/LineSetIO.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/io/VoxelGridIO.h>
// #include <open3d/visualization/utility/DrawGeometry.h>

using Dtype = double;
using OccupancyOctree = erl::geometry::OccupancyOctree<Dtype>;
using OccupancyOctreeDrawer = erl::geometry::OccupancyOctreeDrawer<OccupancyOctree>;
using OctreeKey = erl::geometry::OctreeKey;
using Vector3 = Eigen::Vector3<Dtype>;
using Matrix3X = Eigen::Matrix3X<Dtype>;
using VectorX = Eigen::VectorX<Dtype>;

/// Convert a Frontier to an Open3D TriangleMesh with a given color.
static std::shared_ptr<open3d::geometry::TriangleMesh>
FrontierToMesh(const OccupancyOctree::Frontier &frontier, const Eigen::Vector3d &color) {
    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    mesh->vertices_.reserve(frontier.vertices.size());
    for (const auto &v: frontier.vertices) { mesh->vertices_.push_back(v); }
    mesh->triangles_.reserve(frontier.faces.size());
    for (const auto &f: frontier.faces) { mesh->triangles_.emplace_back(f[0], f[1], f[2]); }
    mesh->PaintUniformColor(color);
    mesh->ComputeTriangleNormals();
    mesh->ComputeVertexNormals();
    return mesh;
}

/// Build a tree with a spherical scan: free interior, occupied shell, unknown exterior.
static std::shared_ptr<OccupancyOctree>
BuildSphereTree(const Dtype resolution = 0.1) {
    auto setting = std::make_shared<OccupancyOctree::Setting>();
    setting->resolution = static_cast<float>(resolution);
    auto tree = std::make_shared<OccupancyOctree>(setting);

    // Generate points on a sphere using Fibonacci sphere sampling
    constexpr long n = 1000;
    Matrix3X points(3, n);
    const Vector3 sensor_origin(0., 0., 0.);

    const Dtype golden_ratio = (1.0 + std::sqrt(5.0)) / 2.0;
    for (long i = 0; i < n; ++i) {
        constexpr Dtype radius = 1.0;
        const Dtype theta = std::acos(1.0 - 2.0 * (static_cast<Dtype>(i) + 0.5) / n);
        const Dtype phi = 2.0 * M_PI * static_cast<Dtype>(i) / golden_ratio;
        points.col(i) <<                               //
            radius * std::sin(theta) * std::cos(phi),  //
            radius * std::sin(theta) * std::sin(phi),  //
            radius * std::cos(theta);
    }

    tree->InsertPointCloud(
        points,
        sensor_origin,
        /*min_range=*/0.0,
        /*max_range=*/-1.0,
        /*with_count=*/false,
        /*parallel=*/false,
        /*lazy_eval=*/false,
        /*discrete=*/false);

    return tree;
}

/// Compute total surface area of a frontier mesh.
static Dtype
FrontierArea(const OccupancyOctree::Frontier &frontier) {
    Dtype area = 0;
    for (const auto &face: frontier.faces) {
        const Vector3 &a = frontier.vertices[face[0]];
        const Vector3 &b = frontier.vertices[face[1]];
        const Vector3 &c = frontier.vertices[face[2]];
        area += (b - a).cross(c - a).norm() * 0.5;
    }
    return area;
}

TEST(OccupancyOctree, ExtractFrontiers_EmptyTree) {
    auto setting = std::make_shared<OccupancyOctree::Setting>();
    setting->resolution = 0.1;
    const OccupancyOctree tree(setting);

    auto frontiers = tree.ExtractFrontiers();
    EXPECT_TRUE(frontiers.empty());
}

TEST(OccupancyOctree, ExtractFrontiers_AllOccupied) {
    auto setting = std::make_shared<OccupancyOctree::Setting>();
    setting->resolution = 0.5;
    auto tree = std::make_shared<OccupancyOctree>(setting);

    // Mark a dense block as occupied — no free cells, so no frontiers.
    for (double x = -1.0; x <= 1.0; x += 0.5) {
        for (double y = -1.0; y <= 1.0; y += 0.5) {
            for (double z = -1.0; z <= 1.0; z += 0.5) {
                tree->UpdateNode(x, y, z, /*occupied=*/true, /*lazy_eval=*/false);
            }
        }
    }

    auto frontiers = tree->ExtractFrontiers();
    EXPECT_TRUE(frontiers.empty());
}

TEST(OccupancyOctree, ExtractFrontiers_SingleFreeCell) {
    auto setting = std::make_shared<OccupancyOctree::Setting>();
    setting->resolution = 1.0;
    auto tree = std::make_shared<OccupancyOctree>(setting);

    // A single free voxel surrounded by unknown space should produce
    // 6 faces * 2 triangles = 12 triangles total.
    tree->UpdateNode(0.0, 0.0, 0.0, /*occupied=*/false, /*lazy_eval=*/false);

    auto frontiers = tree->ExtractFrontiers();
    EXPECT_GT(frontiers.size(), 0u);

    // Count total triangles across all frontiers
    std::size_t total_triangles = 0;
    for (const auto &f: frontiers) { total_triangles += f.faces.size(); }
    EXPECT_EQ(total_triangles, 12u);
}

TEST(OccupancyOctree, ExtractFrontiers_SphereHasFrontiers) {
    auto tree = BuildSphereTree(0.1);
    auto frontiers = tree->ExtractFrontiers();

    ASSERT_GT(frontiers.size(), 0u);
    // The largest frontier should have substantial area
    EXPECT_GT(frontiers[0].faces.size(), 2u);
}

TEST(OccupancyOctree, ExtractFrontiers_SortedByArea) {
    auto tree = BuildSphereTree(0.1);
    auto frontiers = tree->ExtractFrontiers(/*min_num_triangles=*/1, /*sort_by_area=*/true);

    for (std::size_t i = 1; i < frontiers.size(); ++i) {
        EXPECT_GE(FrontierArea(frontiers[i - 1]), FrontierArea(frontiers[i]))
            << "Frontiers not sorted by area at index " << i;
    }
}

TEST(OccupancyOctree, ExtractFrontiers_UnsortedSameContent) {
    auto tree = BuildSphereTree(0.1);
    auto sorted = tree->ExtractFrontiers(/*min_num_triangles=*/1, /*sort_by_area=*/true);
    auto unsorted = tree->ExtractFrontiers(/*min_num_triangles=*/1, /*sort_by_area=*/false);

    ASSERT_EQ(sorted.size(), unsorted.size());

    std::vector<Dtype> sorted_areas;
    std::vector<Dtype> unsorted_areas;
    sorted_areas.reserve(sorted.size());
    unsorted_areas.reserve(unsorted.size());
    for (const auto &f: sorted) { sorted_areas.push_back(FrontierArea(f)); }
    for (const auto &f: unsorted) { unsorted_areas.push_back(FrontierArea(f)); }
    std::sort(sorted_areas.begin(), sorted_areas.end());
    std::sort(unsorted_areas.begin(), unsorted_areas.end());
    for (std::size_t i = 0; i < sorted_areas.size(); ++i) {
        EXPECT_NEAR(sorted_areas[i], unsorted_areas[i], 1e-12)
            << "Frontier area mismatch at index " << i;
    }
}

TEST(OccupancyOctree, ExtractFrontiers_MinTrianglesFiltering) {
    auto tree = BuildSphereTree(0.1);

    auto frontiers_all = tree->ExtractFrontiers(1);
    auto frontiers_large = tree->ExtractFrontiers(50);

    EXPECT_GE(frontiers_all.size(), frontiers_large.size());

    for (const auto &f: frontiers_large) { EXPECT_GE(f.faces.size(), 50u); }
}

TEST(OccupancyOctree, ExtractFrontiers_InAabb) {
    GTEST_PREPARE_OUTPUT_DIR();
    auto tree = BuildSphereTree(0.1);

    constexpr Dtype aabb_min_x = 0.0;
    constexpr Dtype aabb_min_y = 0.0;
    constexpr Dtype aabb_min_z = 0.0;
    constexpr Dtype aabb_max_x = 3.0;
    constexpr Dtype aabb_max_y = 3.0;
    constexpr Dtype aabb_max_z = 3.0;

    auto frontiers = tree->ExtractFrontiers(
        aabb_min_x,
        aabb_min_y,
        aabb_min_z,
        aabb_max_x,
        aabb_max_y,
        aabb_max_z);
    auto frontiers_all = tree->ExtractFrontiers();

    // ERL_INFO("Frontiers in AABB: {}, all frontiers: {}", frontiers.size(), frontiers_all.size());
    // for (std::size_t i = 0; i < frontiers.size(); ++i) {
    //     ERL_INFO(
    //         "  AABB frontier {}: {} vertices, {} triangles, area {:.4f}",
    //         i,
    //         frontiers[i].vertices.size(),
    //         frontiers[i].faces.size(),
    //         FrontierArea(frontiers[i]));
    // }
    // for (std::size_t i = 0; i < frontiers_all.size(); ++i) {
    //     ERL_INFO(
    //         "  All  frontier {}: {} vertices, {} triangles, area {:.4f}",
    //         i,
    //         frontiers_all[i].vertices.size(),
    //         frontiers_all[i].faces.size(),
    //         FrontierArea(frontiers_all[i]));
    // }

    // Save octree leaves
    auto drawer_setting = std::make_shared<OccupancyOctreeDrawer::Setting>();
    drawer_setting->area_min = tree->GetMetricMin();
    drawer_setting->area_max = tree->GetMetricMax();
    const OccupancyOctreeDrawer drawer(drawer_setting, tree);
    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries;
    drawer.DrawLeaves(geometries);
    if (!geometries.empty()) {
        if (auto voxel_grid =
                std::dynamic_pointer_cast<open3d::geometry::VoxelGrid>(geometries[0])) {
            open3d::io::WriteVoxelGrid(
                (test_output_dir / "voxel_grid.ply").string(),
                *voxel_grid,
                true);
        }
    }
    if (geometries.size() > 1) {
        if (auto line_set = std::dynamic_pointer_cast<open3d::geometry::LineSet>(geometries[1])) {
            open3d::io::WriteLineSet((test_output_dir / "line_set.ply").string(), *line_set, true);
        }
    }

    // Save AABB as a wireframe box
    auto aabb_box = open3d::geometry::TriangleMesh::CreateBox(
        aabb_max_x - aabb_min_x,
        aabb_max_y - aabb_min_y,
        aabb_max_z - aabb_min_z);
    aabb_box->Translate(Eigen::Vector3d(aabb_min_x, aabb_min_y, aabb_min_z));
    auto aabb_lineset = open3d::geometry::LineSet::CreateFromTriangleMesh(*aabb_box);
    aabb_lineset->PaintUniformColor({0.0, 1.0, 1.0});
    open3d::io::WriteTriangleMesh((test_output_dir / "aabb_box.ply").string(), *aabb_box, true);

    // Save all frontiers (full tree) in one color
    {
        open3d::geometry::TriangleMesh all_mesh;
        for (const auto &f: frontiers_all) {
            auto mesh = FrontierToMesh(f, {0.5, 0.5, 0.5});
            open3d::io::WriteTriangleMesh(
                (test_output_dir /
                 ("frontier_all_" + std::to_string(&f - &frontiers_all[0]) + ".ply"))
                    .string(),
                *mesh,
                true);
            all_mesh += *mesh;
        }
        open3d::io::WriteTriangleMesh(
            (test_output_dir / "frontiers_all.ply").string(),
            all_mesh,
            true);
    }

    // Save AABB-restricted frontiers with distinct colors
    {
        const Eigen::Vector3d colors[] = {
            {1.0, 0.0, 0.0},
            {0.0, 0.0, 1.0},
            {0.0, 0.8, 0.0},
            {1.0, 0.0, 1.0},
            {0.0, 0.8, 0.8},
            {1.0, 0.5, 0.0},
        };
        constexpr std::size_t num_colors = std::size(colors);
        open3d::geometry::TriangleMesh aabb_mesh;
        for (std::size_t fi = 0; fi < frontiers.size(); ++fi) {
            auto mesh = FrontierToMesh(frontiers[fi], colors[fi % num_colors]);
            open3d::io::WriteTriangleMesh(
                (test_output_dir / ("frontier_aabb_" + std::to_string(fi) + ".ply")).string(),
                *mesh,
                true);
            aabb_mesh += *mesh;
        }
        open3d::io::WriteTriangleMesh(
            (test_output_dir / "frontiers_in_aabb.ply").string(),
            aabb_mesh,
            true);
    }

    ERL_INFO("Saved visualization to {}", test_output_dir.string());

    EXPECT_LE(frontiers.size(), frontiers_all.size());
}

TEST(OccupancyOctree, ExtractFrontiers_Visualization) {
    GTEST_PREPARE_OUTPUT_DIR();
    auto tree = BuildSphereTree(0.1);
    auto frontiers = tree->ExtractFrontiers();
    // ERL_INFO("Extracted {} frontiers", frontiers.size());
    ASSERT_GT(frontiers.size(), 0u);

    // Distinct colors for each frontier
    const Eigen::Vector3d colors[] = {
        {1.0, 0.0, 0.0},  // red
        {0.0, 0.0, 1.0},  // blue
        {0.0, 0.8, 0.0},  // green
        {1.0, 0.0, 1.0},  // magenta
        {0.0, 0.8, 0.8},  // cyan
        {1.0, 0.5, 0.0},  // orange
        {0.5, 0.0, 1.0},  // purple
        {1.0, 1.0, 0.0},  // yellow
    };
    constexpr std::size_t num_colors = std::size(colors);

    // Build octree voxel visualization
    auto drawer_setting = std::make_shared<OccupancyOctreeDrawer::Setting>();
    drawer_setting->area_min = tree->GetMetricMin();
    drawer_setting->area_max = tree->GetMetricMax();
    drawer_setting->draw_free = true;
    drawer_setting->draw_occupied = false;
    OccupancyOctreeDrawer drawer(drawer_setting, tree);
    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries;
    drawer.DrawLeaves(geometries);
    if (!geometries.empty()) {
        if (auto voxel_grid =
                std::dynamic_pointer_cast<open3d::geometry::VoxelGrid>(geometries[0])) {
            open3d::io::WriteVoxelGrid(
                (test_output_dir / "voxel_grid.ply").string(),
                *voxel_grid,
                true);
        }
    }
    if (geometries.size() > 1) {
        if (auto line_set = std::dynamic_pointer_cast<open3d::geometry::LineSet>(geometries[1])) {
            open3d::io::WriteLineSet((test_output_dir / "line_set.ply").string(), *line_set, true);
        }
    }

    // Merge all frontier meshes into a single mesh for export
    open3d::geometry::TriangleMesh combined_mesh;
    for (std::size_t fi = 0; fi < frontiers.size(); ++fi) {
        auto mesh = FrontierToMesh(frontiers[fi], colors[fi % num_colors]);
        combined_mesh += *mesh;
    }
    open3d::io::WriteTriangleMesh(
        (test_output_dir / "frontiers.ply").string(),
        combined_mesh,
        true);

    // Also save each frontier individually for inspection
    for (std::size_t fi = 0; fi < std::min(frontiers.size(), num_colors); ++fi) {
        auto mesh = FrontierToMesh(frontiers[fi], colors[fi % num_colors]);
        open3d::io::WriteTriangleMesh(
            (test_output_dir / ("frontier_" + std::to_string(fi) + ".ply")).string(),
            *mesh,
            true);
    }

    ERL_INFO("Saved {} frontier meshes to {}", frontiers.size(), test_output_dir.string());

    // open3d::visualization::DrawGeometries(
    //     {
    //         geometries[0],
    //         geometries[1],
    //         std::make_shared<open3d::geometry::TriangleMesh>(combined_mesh),
    //     },
    //     /*window_name*/ "Open3D",
    //     /*width*/ 640,
    //     /*height*/ 480,
    //     /*left*/ 50,
    //     /*top*/ 50,
    //     /*point_show_normal*/ false,
    //     /*mesh_show_wireframe*/ false,
    //     /*mesh_show_back_face*/ true);
}

TEST(OccupancyOctree, ExtractFrontiers_TriangleNormals) {
    GTEST_PREPARE_OUTPUT_DIR();
    auto setting = std::make_shared<OccupancyOctree::Setting>();
    setting->resolution = 1.0;
    auto tree = std::make_shared<OccupancyOctree>(setting);

    tree->UpdateNode(0.0, 0.0, 0.0, /*occupied=*/false, /*lazy_eval=*/false);

    // Save octree leaves
    auto drawer_setting = std::make_shared<OccupancyOctreeDrawer::Setting>();
    drawer_setting->area_min = tree->GetMetricMin();
    drawer_setting->area_max = tree->GetMetricMax();
    const OccupancyOctreeDrawer drawer(drawer_setting, tree);
    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries;
    drawer.DrawLeaves(geometries);
    if (!geometries.empty()) {
        if (auto voxel_grid =
                std::dynamic_pointer_cast<open3d::geometry::VoxelGrid>(geometries[0])) {
            open3d::io::WriteVoxelGrid(
                (test_output_dir / "voxel_grid.ply").string(),
                *voxel_grid,
                true);
        }
    }
    if (geometries.size() > 1) {
        if (auto line_set = std::dynamic_pointer_cast<open3d::geometry::LineSet>(geometries[1])) {
            open3d::io::WriteLineSet((test_output_dir / "line_set.ply").string(), *line_set, true);
        }
    }

    auto frontiers = tree->ExtractFrontiers();
    ASSERT_GT(frontiers.size(), 0u);

    // Save frontier mesh
    open3d::geometry::TriangleMesh combined_mesh;
    for (const auto &f: frontiers) { combined_mesh += *FrontierToMesh(f, {1.0, 0.0, 0.0}); }
    open3d::io::WriteTriangleMesh(
        (test_output_dir / "frontiers.ply").string(),
        combined_mesh,
        true);
    ERL_INFO("Saved visualization to {}", test_output_dir.string());

    // For a single free cell, all triangle normals should point outward (away from cell center).
    // Compute the actual cell center from the bounding box of all frontier vertices.
    Vector3 vmin = frontiers[0].vertices[0];
    Vector3 vmax = frontiers[0].vertices[0];
    for (const auto &frontier: frontiers) {
        for (const auto &v: frontier.vertices) {
            vmin = vmin.cwiseMin(v);
            vmax = vmax.cwiseMax(v);
        }
    }
    const Vector3 cell_center = (vmin + vmax) / 2.0;
    // ERL_INFO("Cell center: ({}, {}, {})", cell_center.x(), cell_center.y(), cell_center.z());

    for (const auto &frontier: frontiers) {
        for (const auto &face: frontier.faces) {
            const Vector3 &a = frontier.vertices[face[0]];
            const Vector3 &b = frontier.vertices[face[1]];
            const Vector3 &c = frontier.vertices[face[2]];

            const Vector3 normal = (b - a).cross(c - a);
            const Vector3 centroid = (a + b + c) / 3.0;
            const Vector3 outward = centroid - cell_center;

            EXPECT_GT(normal.dot(outward), 0.0)
                << "Triangle normal points inward at centroid " << centroid.transpose()
                << ", normal " << normal.transpose() << ", cell_center " << cell_center.transpose();
        }
    }
}
