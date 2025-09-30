#include "erl_common/block_timer.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/semi_sparse_octree.hpp"
#include "erl_geometry/semi_sparse_octree_drawer.hpp"

#include <open3d/geometry/PointCloud.h>
#include <open3d/visualization/utility/DrawGeometry.h>

void
CheckTreeStructure(const std::shared_ptr<erl::geometry::SemiSparseOctreeD> &tree) {
    auto &parents = tree->GetParents();
    auto &children = tree->GetChildren();
    auto &voxels = tree->GetVoxels();
    auto &vertices = tree->GetVertices();
    auto &vertex_keys = tree->GetVertexKeys();

    using namespace erl::geometry;
    for (auto it = tree->BeginTree(), end = tree->EndTree(); it != end; ++it) {
        long node_index = it->GetNodeIndex();
        OctreeKey voxel_key = it.GetIndexKey();
        ASSERT_EQ(voxels(0, node_index), voxel_key[0]);
        ASSERT_EQ(voxels(1, node_index), voxel_key[1]);
        ASSERT_EQ(voxels(2, node_index), voxel_key[2]);
        ASSERT_EQ(voxels(3, node_index), 1 << (tree->GetTreeDepth() - it->GetDepth()));

        if (!it->HasAnyChild()) {
            for (int i = 0; i < 8; ++i) { ASSERT_EQ(children(i, node_index), -1); }
        } else {
            for (int i = 0; i < 8; ++i) {
                auto child = it->GetChild<SemiSparseOctreeNode>(i);
                ASSERT_EQ(children(i, node_index), child == nullptr ? -1 : child->GetNodeIndex());
            }
        }

        auto child_idx = it->GetChildIndex();
        auto parent_node_index = parents[node_index];
        if (parent_node_index >= 0) {
            ASSERT_EQ(children(child_idx, parent_node_index), node_index);
        }

        OctreeKey vertex_key;
        const uint32_t level = tree->GetTreeDepth() - it->GetDepth();
        for (int i = 0; i < 8; ++i) {
            OctreeKey::ComputeVertexKey(i, level, voxel_key, vertex_key);
            long vertex_index = vertices(i, node_index);
            ASSERT_EQ(vertex_keys[vertex_index][0], vertex_key[0]);
            ASSERT_EQ(vertex_keys[vertex_index][1], vertex_key[1]);
            ASSERT_EQ(vertex_keys[vertex_index][2], vertex_key[2]);
        }
    }
}

TEST(SemiSparseOctree, Build) {
    using namespace erl::common;
    using namespace erl::geometry;

    using TreeDrawer = SemiSparseOctreeDrawer<SemiSparseOctreeD>;

    auto setting = std::make_shared<SemiSparseNdTreeSetting>();
    setting->tree_depth = 8;
    setting->semi_sparse_depth = 2;
    setting->resolution = 0.05;
    setting->init_voxel_num = 1000;
    auto tree = std::make_shared<SemiSparseOctreeD>(setting);

    CheckTreeStructure(tree);

    auto &vertices = tree->GetVertices();
    auto &vertex_keys = tree->GetVertexKeys();

    constexpr double radius = 3.0;
    auto drawer_setting = std::make_shared<TreeDrawer::Setting>();
    drawer_setting->area_min.setConstant(-3.0 * radius);
    drawer_setting->area_max.setConstant(3.0 * radius);
    TreeDrawer drawer(drawer_setting, tree);
    auto pcd_vertices = std::make_shared<open3d::geometry::PointCloud>();
    auto pcd_voxel_centers = std::make_shared<open3d::geometry::PointCloud>();
    auto drawer_callback = [&](const TreeDrawer * /*drawer*/,
                               std::vector<std::shared_ptr<open3d::geometry::Geometry>> &geometries,
                               SemiSparseOctreeD::LeafInAabbIterator &it) {
        long node_index = it->GetNodeIndex();
        auto &vertex_indices = vertices.col(node_index);
        for (auto &vertex_index: vertex_indices) {
            ERL_ASSERTM(vertex_index >= 0, "invalid vertex index: {}", vertex_index);
            auto p = tree->KeyToVertexCoord(vertex_keys[vertex_index]);
            pcd_vertices->points_.emplace_back(p);
        }
        auto voxel_center = it.GetCenter();
        pcd_voxel_centers->points_.emplace_back(voxel_center);

        if (it->GetDepth() != setting->tree_depth) { return; }

        auto boxes = std::dynamic_pointer_cast<open3d::geometry::VoxelGrid>(geometries[0]);
        ERL_ASSERTM(boxes, "the first element of geometries should be a triangle mesh.");

        const double x = it.GetX();
        const double y = it.GetY();
        const double z = it.GetZ();

        Eigen::Vector3i voxel_index(
            std::floor((x - boxes->origin_[0]) / boxes->voxel_size_),   // x
            std::floor((y - boxes->origin_[1]) / boxes->voxel_size_),   // y
            std::floor((z - boxes->origin_[2]) / boxes->voxel_size_));  // z
        boxes->AddVoxel(open3d::geometry::Voxel(voxel_index, {1.0, 0.0, 0.0}));
    };
    drawer.SetDrawLeafCallback(drawer_callback);

    constexpr int n_azimuths = 720;
    constexpr int n_elevations = 720;
    Eigen::VectorXd azimuths = Eigen::VectorXd::LinSpaced(n_azimuths, -M_PI, M_PI);
    Eigen::VectorXd elevations = Eigen::VectorXd::LinSpaced(n_elevations, -M_PI / 4, M_PI / 4);
    std::vector<Eigen::Vector3d> points;
    points.reserve(n_azimuths * n_elevations);
    for (int i = 0; i < n_azimuths; ++i) {
        for (int j = 0; j < n_elevations; ++j) {
            double x = radius * std::cos(elevations[j]) * std::cos(azimuths[i]);
            double y = radius * std::cos(elevations[j]) * std::sin(azimuths[i]);
            double z = radius * std::sin(elevations[j]);
            points.emplace_back(x, y, z);
        }
    }

    auto node_indices = tree->InsertPoints(&points[0][0], static_cast<long>(points.size()));
    CheckTreeStructure(tree);

    // save points
    SaveBinaryFile(
        "semi_sparse_octree_points.bin",
        &points[0][0],
        static_cast<std::streamsize>(points.size() * 3));

    // save children
    SaveBinaryFile(
        "semi_sparse_octree_children.bin",
        tree->GetChildren().data(),
        tree->GetChildren().size());

    // save node_indices
    SaveBinaryFile("semi_sparse_octree_node_indices.bin", node_indices.data(), node_indices.size());
    Eigen::VectorXl found_node_indices;
    double dt;
    {
        ERL_BLOCK_TIMER_MSG_TIME("Find voxel indices", dt);
        found_node_indices =
            tree->FindVoxelIndices(&points[0][0], static_cast<long>(points.size()), true);
    }
    dt /= static_cast<double>(points.size());
    std::cout << "Average time per query: " << dt * 1e3 << " us." << std::endl;
    for (long i = 0; i < node_indices.size(); ++i) {
        ASSERT_EQ(node_indices[i], found_node_indices[i]);
    }

    auto geometries = TreeDrawer::GetBlankGeometries();
    drawer.DrawLeaves(geometries);

    pcd_voxel_centers->PaintUniformColor({0.0, 0.0, 0.0});
    open3d::visualization::DrawGeometries(
        {geometries[0],
         geometries[1],
         pcd_vertices,
         pcd_voxel_centers,
         open3d::geometry::TriangleMesh::CreateCoordinateFrame(1)});
}
