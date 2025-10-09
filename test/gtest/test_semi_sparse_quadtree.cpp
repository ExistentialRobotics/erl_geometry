#include "erl_common/block_timer.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/semi_sparse_quadtree.hpp"
#include "erl_geometry/semi_sparse_quadtree_drawer.hpp"

#include <open3d/geometry/PointCloud.h>
#include <open3d/visualization/utility/DrawGeometry.h>

void
CheckTreeStructure(const std::shared_ptr<erl::geometry::SemiSparseQuadtreeD> &tree) {
    auto &parents = tree->GetParents();
    auto &children = tree->GetChildren();
    auto &voxels = tree->GetVoxels();
    auto &vertices = tree->GetVertices();
    auto &vertex_keys = tree->GetVertexKeys();

    using namespace erl::geometry;
    for (auto it = tree->BeginTree(), end = tree->EndTree(); it != end; ++it) {
        long node_index = it->GetNodeIndex();
        QuadtreeKey voxel_key = it.GetIndexKey();
        ASSERT_EQ(voxels(0, node_index), voxel_key[0]);
        ASSERT_EQ(voxels(1, node_index), voxel_key[1]);
        ASSERT_EQ(voxels(2, node_index), 1 << (tree->GetTreeDepth() - it->GetDepth()));

        if (!it->HasAnyChild()) {
            for (int i = 0; i < 4; ++i) { ASSERT_EQ(children(i, node_index), -1); }
        } else {
            for (int i = 0; i < 4; ++i) {
                auto child = it->GetChild<SemiSparseQuadtreeNode>(i);
                ASSERT_EQ(children(i, node_index), child == nullptr ? -1 : child->GetNodeIndex());
            }
        }

        auto child_idx = it->GetChildIndex();
        auto parent_node_index = parents[node_index];
        if (parent_node_index >= 0) {
            ASSERT_EQ(children(child_idx, parent_node_index), node_index);
        }

        QuadtreeKey vertex_key;
        const uint32_t level = tree->GetTreeDepth() - it->GetDepth();
        for (int i = 0; i < 4; ++i) {
            QuadtreeKey::ComputeVertexKey(i, level, voxel_key, vertex_key);
            long vertex_index = vertices(i, node_index);
            ASSERT_EQ(vertex_keys[vertex_index][0], vertex_key[0]);
            ASSERT_EQ(vertex_keys[vertex_index][1], vertex_key[1]);
        }
    }
}

TEST(SemiSparseQuadtree, Build) {
    using namespace erl::common;
    using namespace erl::geometry;

    using TreeDrawer = SemiSparseQuadtreeDrawer<SemiSparseQuadtreeD>;

    auto setting = std::make_shared<SemiSparseNdTreeSetting>();
    setting->tree_depth = 8;
    setting->semi_sparse_depth = 2;
    setting->resolution = 0.05;
    setting->init_voxel_num = 1000;
    auto tree = std::make_shared<SemiSparseQuadtreeD>(setting);

    CheckTreeStructure(tree);

    auto &vertices = tree->GetVertices();
    auto &vertex_keys = tree->GetVertexKeys();

    constexpr double radius = 3.0;
    auto drawer_setting = std::make_shared<TreeDrawer::Setting>();
    drawer_setting->resolution = 0.025;
    drawer_setting->area_min.setConstant(-3.0 * radius);
    drawer_setting->area_max.setConstant(3.0 * radius);
    TreeDrawer drawer(drawer_setting, tree);
    auto drawer_callback =
        [&](const TreeDrawer *self, cv::Mat &mat, SemiSparseQuadtreeD::LeafInAabbIterator &it) {
            long node_index = it->GetNodeIndex();
            auto &vertex_indices = vertices.col(node_index);
            for (long i = 0; i < 4; ++i) {
                const long &vertex_index = vertex_indices[i];
                ERL_ASSERTM(vertex_index >= 0, "invalid vertex index: {}", vertex_index);
                auto p = tree->KeyToVertexCoord(vertex_keys[vertex_index]);
                auto pixel = self->GetPixelCoordsForPositions<double>(p, true);
                cv::circle(mat, {pixel(0, 0), pixel(1, 0)}, 1, {0, 0, 255}, -1);
            }

            if (it->GetDepth() != setting->tree_depth) { return; }

            Eigen::Vector2f voxel_center = it.GetCenter().cast<float>();
            const auto voxel_half_size = static_cast<float>(it.GetNodeSize()) * 0.5f;
            Eigen::Matrix2f area;
            // clang-format off
            area << voxel_center[0] - voxel_half_size, voxel_center[0] + voxel_half_size,
                    voxel_center[1] - voxel_half_size, voxel_center[1] + voxel_half_size;
            // clang-format on
            auto pixels = self->GetPixelCoordsForPositions<float>(area, true);
            cv::rectangle(
                mat,
                {pixels(0, 0), pixels(1, 0)},
                {pixels(0, 1), pixels(1, 1)},
                {255, 0, 0},
                -1);
        };
    drawer.SetDrawLeafCallback(drawer_callback);

    constexpr int n_angles = 720;
    Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(n_angles, -M_PI, M_PI);
    std::vector<Eigen::Vector2d> points;
    points.reserve(n_angles);
    for (int i = 0; i < n_angles; ++i) {
        double x = radius * std::cos(angles[i]);
        double y = radius * std::sin(angles[i]);
        points.emplace_back(x, y);
    }
    // std::cout << points[0] << std::endl;

    auto node_indices = tree->InsertPoints(&points[0][0], static_cast<long>(points.size()));
    CheckTreeStructure(tree);

    // save points
    SaveBinaryFile<double>(
        "semi_sparse_quadtree_points.bin",
        points.data()->data(),
        static_cast<std::streamsize>(points.size() * 2));

    // save children
    SaveBinaryFile<long>(
        "semi_sparse_quadtree_children.bin",
        tree->GetChildren().data(),
        tree->GetChildren().size());

    // save node_indices
    SaveBinaryFile<long>(
        "semi_sparse_quadtree_node_indices.bin",
        node_indices.data(),
        node_indices.size());
    Eigen::VectorXl found_node_indices;
    {
        ERL_BLOCK_TIMER_MSG("Find voxel indices");
        found_node_indices =
            tree->FindVoxelIndices(&points[0][0], static_cast<long>(points.size()), true);
    }
    for (long i = 0; i < node_indices.size(); ++i) {
        ASSERT_EQ(node_indices[i], found_node_indices[i]);
    }

    cv::Mat img;
    drawer.DrawLeaves(img);
    cv::imshow("semi_sparse_quadtree", img);
    cv::waitKey(0);
}
