#include "erl_common/test_helper.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/occupancy_octree_drawer.hpp"

#define SPHERE_RADIUS      0.75
#define NUM_AZIMUTHS       720
#define NUM_ELEVATIONS     180
#define SENSOR_ORIGIN_X    1.0
#define SENSOR_ORIGIN_Y    0.0
#define SENSOR_ORIGIN_Z    0.0
#define VISUALIZER_SECONDS 1

using Dtype = float;
using AbstractOctree = erl::geometry::AbstractOctree<Dtype>;
using OccupancyOctree = erl::geometry::OccupancyOctree<Dtype>;
using TreeSerializer = erl::common::Serialization<OccupancyOctree>;
using OccupancyOctreeNode = erl::geometry::OccupancyOctreeNode;
using Open3dVisualizerWrapper = erl::geometry::Open3dVisualizerWrapper;
using OctreeKey = erl::geometry::OctreeKey;
using VectorX = Eigen::VectorX<Dtype>;
using Vector3 = Eigen::Vector3<Dtype>;
using MatrixX = Eigen::MatrixX<Dtype>;
using Matrix3X = Eigen::Matrix3X<Dtype>;

TEST(OccupancyOctree, IO) {
    OccupancyOctree tree;
    EXPECT_EQ(tree.GetSize(), 0);
    EXPECT_TRUE(TreeSerializer::Write("empty.bt", [&](std::ostream& s) -> bool {
        return tree.WriteBinary(s);
    }));
    EXPECT_TRUE(TreeSerializer::Write("empty.ot", tree));

    OccupancyOctree read_tree_bt;
    EXPECT_TRUE(TreeSerializer::Read("empty.bt", [&](std::istream& s) -> bool {
        return read_tree_bt.ReadBinary(s);
    }));
    EXPECT_EQ(read_tree_bt.GetSize(), 0);
    EXPECT_TRUE(tree == read_tree_bt);

    OccupancyOctree read_tree_ot;
    EXPECT_TRUE(TreeSerializer::Read("empty.ot", read_tree_ot));
    EXPECT_TRUE(tree == read_tree_ot);
}

TEST(OccupancyOctree, InsertPointCloud) {
    GTEST_PREPARE_OUTPUT_DIR();

    auto tree_setting = std::make_shared<OccupancyOctree::Setting>();
    tree_setting->resolution = 0.05;
    auto tree = std::make_shared<OccupancyOctree>(tree_setting);

    constexpr long num_azimuths = NUM_AZIMUTHS;
    constexpr long num_elevations = NUM_ELEVATIONS;
    VectorX azimuths = VectorX::LinSpaced(num_azimuths, -M_PI, M_PI);
    VectorX elevations = VectorX::LinSpaced(num_elevations, -M_PI / 2, M_PI / 2);

    long n = num_azimuths * num_elevations;
    Matrix3X points(3, n);
    Vector3 sensor_origin(SENSOR_ORIGIN_X, SENSOR_ORIGIN_Y, SENSOR_ORIGIN_Z);

    for (long i = 0, k = 0; i < num_elevations; ++i) {
        for (long j = 0; j < num_azimuths; ++j, ++k) {
            constexpr Dtype radius = SPHERE_RADIUS;
            // clang-format off
            points.col(k) << std::cos(elevations[i]) * std::cos(azimuths[j]) * radius + sensor_origin.x(),
                             std::cos(elevations[i]) * std::sin(azimuths[j]) * radius + sensor_origin.y(),
                             std::sin(elevations[i]) * radius + sensor_origin.z();
            // clang-format on
        }
    }

    Dtype max_range = -1.;
    bool parallel = false;
    bool lazy_eval = false;
    bool discretize = false;

    {
        std::size_t node_cnt = 0;
        for (auto itr = tree->BeginTree(); itr != tree->EndTree(); ++itr) { ++node_cnt; }
        EXPECT_EQ(node_cnt, 0);
        EXPECT_EQ(tree->GetSize(), node_cnt);
    }

    erl::common::ReportTime<std::chrono::milliseconds>(test_info->name(), 1, true, [&] {
        tree->InsertPointCloud(points, sensor_origin, max_range, parallel, lazy_eval, discretize);
    });

    {
        std::size_t node_cnt = 0;
        for (auto itr = tree->BeginTree(); itr != tree->EndTree(); ++itr) { ++node_cnt; }
        if (std::is_same_v<Dtype, float>) {
            EXPECT_EQ(node_cnt, 7361);
        } else {
            EXPECT_EQ(node_cnt, 7359);
        }
        EXPECT_EQ(tree->GetSize(), node_cnt);
    }
    auto tree_ot_file = test_output_dir / "sphere.ot";
    EXPECT_TRUE(TreeSerializer::Write(tree_ot_file, *tree));
    OccupancyOctree read_tree_ot;
    EXPECT_TRUE(TreeSerializer::Read(tree_ot_file, read_tree_ot));
    EXPECT_EQ(tree->GetSize(), read_tree_ot.GetSize());
    EXPECT_TRUE(*tree == read_tree_ot);

    // test occupancy node deep copy (shallow copy is not allowed unless it is managed by
    // shared_ptr)
    auto cloned_node = std::shared_ptr<OccupancyOctreeNode>(
        reinterpret_cast<OccupancyOctreeNode*>(tree->GetRoot()->Clone()));  // deep copy
    EXPECT_TRUE(*cloned_node == *tree->GetRoot());
    cloned_node->GetChild<OccupancyOctreeNode>(1)->AddLogOdds(0.1);
    EXPECT_FALSE(*cloned_node == *tree->GetRoot());

    auto tree_bt_file = test_output_dir / "sphere.bt";
    EXPECT_TRUE(TreeSerializer::Write(tree_bt_file, [&](std::ostream& s) -> bool {
        return tree->WriteBinary(s);
    }));
    OccupancyOctree read_tree_bt;
    EXPECT_TRUE(TreeSerializer::Read(tree_bt_file, [&](std::istream& s) -> bool {
        return read_tree_bt.ReadBinary(s);
    }));
    EXPECT_EQ(tree->GetSize(), read_tree_bt.GetSize());

    using OctreeDrawer = erl::geometry::OccupancyOctreeDrawer<OccupancyOctree>;
    auto setting = std::make_shared<OctreeDrawer::Setting>();
    OctreeDrawer drawer(setting, tree);
    Open3dVisualizerWrapper visualizer;
    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries;
    drawer.DrawLeaves(geometries);
    visualizer.AddGeometries(geometries);
    visualizer.Show(VISUALIZER_SECONDS);
}

TEST(OccupancyOctree, InsertPointCloudRays) {
    GTEST_PREPARE_OUTPUT_DIR();

    auto tree_setting = std::make_shared<OccupancyOctree::Setting>();
    tree_setting->resolution = 0.05;
    auto tree = std::make_shared<OccupancyOctree>(tree_setting);

    constexpr long num_azimuths = NUM_AZIMUTHS;
    constexpr long num_elevations = NUM_ELEVATIONS;
    VectorX azimuths = VectorX::LinSpaced(num_azimuths, -M_PI, M_PI);
    VectorX elevations = VectorX::LinSpaced(num_elevations, -M_PI / 2, M_PI / 2);

    long n = num_azimuths * num_elevations;
    Matrix3X points(3, n);
    Vector3 sensor_origin(SENSOR_ORIGIN_X, SENSOR_ORIGIN_Y, SENSOR_ORIGIN_Z);

    for (long i = 0, k = 0; i < num_elevations; ++i) {
        for (long j = 0; j < num_azimuths; ++j, ++k) {
            constexpr Dtype radius = SPHERE_RADIUS;
            // clang-format off
            points.col(k) << std::cos(elevations[i]) * std::cos(azimuths[j]) * radius + sensor_origin.x(),
                             std::cos(elevations[i]) * std::sin(azimuths[j]) * radius + sensor_origin.y(),
                             std::sin(elevations[i]) * radius + sensor_origin.z();
            // clang-format on
        }
    }

    Dtype max_range = -1.;
    bool parallel = false;
    bool lazy_eval = false;
    erl::common::ReportTime<std::chrono::milliseconds>(test_info->name(), 1, true, [&] {
        tree->InsertPointCloudRays(points, sensor_origin, max_range, parallel, lazy_eval);
    });

    EXPECT_TRUE(TreeSerializer::Write("sphere.ot", *tree));
    OccupancyOctree read_tree_ot;
    EXPECT_TRUE(TreeSerializer::Read("sphere.ot", read_tree_ot));
    EXPECT_EQ(tree->GetSize(), read_tree_ot.GetSize());
    EXPECT_TRUE(*tree == read_tree_ot);
    EXPECT_TRUE(TreeSerializer::Write("sphere.bt", [&](std::ostream& s) -> bool {
        return tree->WriteBinary(s);
    }));
    OccupancyOctree read_tree_bt;
    EXPECT_TRUE(TreeSerializer::Read("sphere.bt", [&](std::istream& s) -> bool {
        return read_tree_bt.ReadBinary(s);
    }));
    EXPECT_EQ(tree->GetSize(), read_tree_bt.GetSize());

    using OctreeDrawer = erl::geometry::OccupancyOctreeDrawer<OccupancyOctree>;
    auto setting = std::make_shared<OctreeDrawer::Setting>();
    OctreeDrawer drawer(setting, tree);
    Open3dVisualizerWrapper visualizer;
    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries;
    drawer.DrawLeaves(geometries);
    visualizer.AddGeometries(geometries);
    visualizer.Show(VISUALIZER_SECONDS);
}

TEST(OccupancyOctree, InsertRay) {
    GTEST_PREPARE_OUTPUT_DIR();
    auto tree_setting = std::make_shared<OccupancyOctree::Setting>();
    tree_setting->resolution = 0.05;
    auto tree = std::make_shared<OccupancyOctree>(tree_setting);

    constexpr long num_azimuths = NUM_AZIMUTHS;
    constexpr long num_elevations = NUM_ELEVATIONS;
    VectorX azimuths = VectorX::LinSpaced(num_azimuths, -M_PI, M_PI);
    VectorX elevations = VectorX::LinSpaced(num_elevations, -M_PI / 2, M_PI / 2);

    long n = num_azimuths * num_elevations;
    Matrix3X points(3, n);
    Vector3 sensor_origin(SENSOR_ORIGIN_X, SENSOR_ORIGIN_Y, SENSOR_ORIGIN_Z);

    for (long i = 0, k = 0; i < num_elevations; ++i) {
        for (long j = 0; j < num_azimuths; ++j, ++k) {
            constexpr Dtype radius = SPHERE_RADIUS;
            // clang-format off
            points.col(k) << std::cos(elevations[i]) * std::cos(azimuths[j]) * radius + sensor_origin.x(),
                             std::cos(elevations[i]) * std::sin(azimuths[j]) * radius + sensor_origin.y(),
                             std::sin(elevations[i]) * radius + sensor_origin.z();
            // clang-format on
        }
    }

    Dtype max_range = -1.;
    bool lazy_eval = false;
    erl::common::ReportTime<std::chrono::milliseconds>(test_info->name(), 1, true, [&] {
        for (int i = 0; i < points.cols(); ++i) {
            tree->InsertRay(
                sensor_origin[0],
                sensor_origin[1],
                sensor_origin[2],
                points(0, i),
                points(1, i),
                points(2, i),
                max_range,
                lazy_eval);
        }
    });

    EXPECT_TRUE(TreeSerializer::Write("sphere.ot", *tree));
    OccupancyOctree read_tree_ot;
    EXPECT_TRUE(TreeSerializer::Read("sphere.ot", read_tree_ot));
    EXPECT_EQ(tree->GetSize(), read_tree_ot.GetSize());
    EXPECT_TRUE(*tree == read_tree_ot);

    EXPECT_TRUE(TreeSerializer::Write("sphere.bt", [&](std::ostream& s) -> bool {
        return tree->WriteBinary(s);
    }));
    OccupancyOctree read_tree_bt;
    EXPECT_TRUE(TreeSerializer::Read("sphere.bt", [&](std::istream& s) -> bool {
        return read_tree_bt.ReadBinary(s);
    }));
    EXPECT_EQ(tree->GetSize(), read_tree_bt.GetSize());

    using OctreeDrawer = erl::geometry::OccupancyOctreeDrawer<OccupancyOctree>;
    auto setting = std::make_shared<OctreeDrawer::Setting>();
    OctreeDrawer drawer(setting, tree);
    Open3dVisualizerWrapper visualizer;
    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries;
    drawer.DrawLeaves(geometries);
    visualizer.AddGeometries(geometries);
    visualizer.Show(VISUALIZER_SECONDS);
}

TEST(OccupancyOctree, CoordsAndKey) {
    const auto tree_setting = std::make_shared<OccupancyOctree::Setting>();
    tree_setting->resolution = 0.05;
    const OccupancyOctree tree(tree_setting);
    constexpr Dtype x = 0;
    constexpr Dtype y = 0;
    constexpr Dtype z = 0;
    OctreeKey key;
    EXPECT_TRUE(tree.CoordToKeyChecked(x, y, z, key));
    Dtype x_inv = 0, y_inv = 0, z_inv = 0;
    tree.KeyToCoord(key, x_inv, y_inv, z_inv);
    EXPECT_FLOAT_EQ(0.025, x_inv);
    EXPECT_FLOAT_EQ(0.025, y_inv);
    EXPECT_FLOAT_EQ(0.025, z_inv);

    const uint64_t morton_code = key.ToMortonCode();
    const OctreeKey key_from_morton_code(morton_code);
    EXPECT_EQ(key, key_from_morton_code);
}

TEST(OccupancyOctree, Prune) {
    Dtype resolution = 0.01;
    auto tree_setting = std::make_shared<OccupancyOctree::Setting>();
    tree_setting->resolution = resolution;
    auto tree = std::make_shared<OccupancyOctree>(tree_setting);

    // after pruning, an empty tree is still empty
    EXPECT_EQ(tree->GetSize(), 0);
    tree->Prune();
    EXPECT_EQ(tree->GetSize(), 0);

    // a single occupied cell should be found
    Dtype x = -0.05, y = -0.02, z = 0.05;
    OctreeKey single_key;
    EXPECT_TRUE(tree->CoordToKeyChecked(x, y, z, single_key));
    bool occupied = true;
    bool lazy_eval = false;
    auto single_node = tree->UpdateNode(single_key, occupied, lazy_eval);
    EXPECT_TRUE(single_node != nullptr);
    EXPECT_EQ(single_node, tree->Search(single_key));

    // neighboring nodes should not exist
    OctreeKey neighbor_key;
    for (neighbor_key[2] = single_key[2] - 1; neighbor_key[2] <= single_key[2] + 1;
         ++neighbor_key[2]) {
        for (neighbor_key[1] = single_key[1] - 1; neighbor_key[1] <= single_key[1] + 1;
             ++neighbor_key[1]) {
            for (neighbor_key[0] = single_key[0] - 1; neighbor_key[0] <= single_key[0] + 1;
                 ++neighbor_key[0]) {
                if (neighbor_key != single_key) {
                    auto node = tree->Search(neighbor_key);
                    EXPECT_TRUE(node == nullptr);
                } else {
                    auto node = tree->Search(neighbor_key);
                    EXPECT_TRUE(node != nullptr);
                    EXPECT_EQ(single_node, node);
                }
            }
        }
    }

    // prune should do nothing
    tree->Prune();
    for (neighbor_key[2] = single_key[2] - 1; neighbor_key[2] <= single_key[2] + 1;
         ++neighbor_key[2]) {
        for (neighbor_key[1] = single_key[1] - 1; neighbor_key[1] <= single_key[1] + 1;
             ++neighbor_key[1]) {
            for (neighbor_key[0] = single_key[0] - 1; neighbor_key[0] <= single_key[0] + 1;
                 ++neighbor_key[0]) {
                if (neighbor_key != single_key) {
                    auto node = tree->Search(neighbor_key);
                    EXPECT_TRUE(node == nullptr);
                } else {
                    auto node = tree->Search(neighbor_key);
                    EXPECT_TRUE(node != nullptr);
                    EXPECT_EQ(single_node, node);
                }
            }
        }
    }

    // node + 1 branch of depth 16
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), 17);

    // create diagonal neighbor in same parent node
    OctreeKey diagonal_key;
    diagonal_key[0] = single_key[0] - 1;
    diagonal_key[1] = single_key[1] + 1;
    diagonal_key[2] = single_key[2] - 1;
    auto diagonal_node = tree->UpdateNode(diagonal_key, occupied, lazy_eval);

    EXPECT_TRUE(diagonal_node);
    for (neighbor_key[2] = single_key[2] - 1; neighbor_key[2] <= single_key[2] + 1;
         ++neighbor_key[2]) {
        for (neighbor_key[1] = single_key[1] - 1; neighbor_key[1] <= single_key[1] + 1;
             ++neighbor_key[1]) {
            for (neighbor_key[0] = single_key[0] - 1; neighbor_key[0] <= single_key[0] + 1;
                 ++neighbor_key[0]) {
                if (neighbor_key == single_key) {
                    auto node = tree->Search(neighbor_key);
                    EXPECT_TRUE(node != nullptr);
                    EXPECT_EQ(single_node, node);
                } else if (neighbor_key == diagonal_key) {
                    auto node = tree->Search(diagonal_key);
                    EXPECT_TRUE(node != nullptr);
                    EXPECT_EQ(diagonal_node, node);
                } else {
                    auto node = tree->Search(neighbor_key);
                    EXPECT_TRUE(node == nullptr);
                }
            }
        }
    }

    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), 18);

    // prune should do nothing
    tree->Prune();
    for (neighbor_key[2] = single_key[2] - 1; neighbor_key[2] <= single_key[2] + 1;
         ++neighbor_key[2]) {
        for (neighbor_key[1] = single_key[1] - 1; neighbor_key[1] <= single_key[1] + 1;
             ++neighbor_key[1]) {
            for (neighbor_key[0] = single_key[0] - 1; neighbor_key[0] <= single_key[0] + 1;
                 ++neighbor_key[0]) {
                if (neighbor_key == single_key) {
                    auto node = tree->Search(neighbor_key);
                    EXPECT_TRUE(node != nullptr);
                    EXPECT_EQ(single_node, node);
                } else if (neighbor_key == diagonal_key) {
                    auto node = tree->Search(diagonal_key);
                    EXPECT_TRUE(node != nullptr);
                    EXPECT_EQ(diagonal_node, node);
                } else {
                    auto node = tree->Search(neighbor_key);
                    EXPECT_TRUE(node == nullptr);
                }
            }
        }
    }
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), 18);

    // fill the complete quadrant, should auto-prune
    tree->UpdateNode(
        OctreeKey(single_key[0], single_key[1] + 1, single_key[2]),
        occupied,
        lazy_eval);
    EXPECT_EQ(tree->GetSize(), 19);
    tree->UpdateNode(
        OctreeKey(single_key[0] - 1, single_key[1] + 1, single_key[2]),
        occupied,
        lazy_eval);
    EXPECT_EQ(tree->GetSize(), 20);
    tree->UpdateNode(
        OctreeKey(single_key[0] - 1, single_key[1], single_key[2]),
        occupied,
        lazy_eval);
    EXPECT_EQ(tree->GetSize(), 21);
    tree->UpdateNode(
        OctreeKey(single_key[0], single_key[1], single_key[2] - 1),
        occupied,
        lazy_eval);
    EXPECT_EQ(tree->GetSize(), 22);
    tree->UpdateNode(
        OctreeKey(single_key[0], single_key[1] + 1, single_key[2] - 1),
        occupied,
        lazy_eval);
    EXPECT_EQ(tree->GetSize(), 23);
    // should trigger auto-pruning
    auto pruned_node = tree->UpdateNode(
        OctreeKey(single_key[0] - 1, single_key[1], single_key[2] - 1),
        occupied,
        lazy_eval);
    EXPECT_EQ(tree->GetSize(), 16);

    // all queries should now end up at the same parent node
    auto node1 = tree->Search(single_key);
    auto node2 = tree->Search(diagonal_key);
    EXPECT_EQ(node1, node2);
    EXPECT_EQ(pruned_node, node1);

    // test larger volume pruning
    for (int i = 0, failed = 0; i <= 31 && !failed; ++i) {
        for (int j = 0; j <= 31 && !failed; ++j) {
            for (int k = 0; k <= 31 && !failed; ++k) {
                auto node = tree->UpdateNode(  //
                    static_cast<Dtype>(i) * resolution + 0.001f,
                    static_cast<Dtype>(j) * resolution + 0.001f,
                    static_cast<Dtype>(k) * resolution + 0.001f,
                    occupied,
                    lazy_eval);
                EXPECT_TRUE(node != nullptr);
                EXPECT_TRUE(tree->IsNodeOccupied(node));
                if (node == nullptr || !tree->IsNodeOccupied(node)) { failed = 1; }
            }
        }
    }
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), 27);

    tree->Expand();  // increase 32x32x32 + 16x16x16 + 8x8x8 + 4x4x4 + 2x2x2 and 2x2x2 nodes (37456)
    EXPECT_EQ(tree->GetSize(), 37483);

    tree->Prune();
    EXPECT_EQ(tree->GetSize(), 27);

    // test expansion
    for (int i = 0, failed = 0; i <= 31 && !failed; ++i) {
        for (int j = 0; j <= 31 && !failed; ++j) {
            for (int k = 0; k <= 31 && !failed; ++k) {
                auto node = tree->Search(
                    static_cast<Dtype>(i) * resolution + 0.001f,
                    static_cast<Dtype>(j) * resolution + 0.001f,
                    static_cast<Dtype>(k) * resolution + 0.001f);
                EXPECT_TRUE(node != nullptr);
                EXPECT_TRUE(tree->IsNodeOccupied(node));
                if (node == nullptr || !tree->IsNodeOccupied(node)) { failed = 1; }
            }
        }
    }

    // update a single cell, which should auto-expand
    EXPECT_TRUE(tree->CoordToKeyChecked(0.0, 0.0, 0.0, single_key));
    EXPECT_TRUE(tree->UpdateNode(single_key, occupied, lazy_eval));
    for (int i = 0, failed = 0; i <= 31 && !failed; ++i) {
        for (int j = 0; j <= 31 && !failed; ++j) {
            for (int k = 0; k <= 31 && !failed; ++k) {
                auto node = tree->Search(
                    static_cast<Dtype>(i) * resolution + 0.001f,
                    static_cast<Dtype>(j) * resolution + 0.001f,
                    static_cast<Dtype>(k) * resolution + 0.001f);
                EXPECT_TRUE(node != nullptr);
                EXPECT_TRUE(tree->IsNodeOccupied(node));
                if (node == nullptr || !tree->IsNodeOccupied(node)) { failed = 1; }
            }
        }
    }
    EXPECT_EQ(tree->GetSize(), 67);  // 32: level 4, increase 8x5 nodes
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());

    // delete, expand, prune of single node
    std::size_t init_size = tree->GetSize();
    auto new_node = tree->UpdateNode(-0.2, -0.2, -0.2, occupied, lazy_eval);
    EXPECT_TRUE(new_node != nullptr);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), init_size + 16);

    // find parent of newly inserted node
    unsigned int search_depth = tree->GetTreeDepth() - 1;
    OctreeKey parent_key = tree->CoordToKey(-0.2, -0.2, -0.2);
    auto* parent_node = const_cast<OccupancyOctreeNode*>(tree->Search(parent_key, search_depth));
    EXPECT_TRUE(parent_node != nullptr);
    EXPECT_TRUE(parent_node->HasAnyChild());
    // only one child exists
    EXPECT_TRUE(parent_node->GetChild<OccupancyOctreeNode>(0) != nullptr);
    EXPECT_TRUE(parent_node->GetChild<OccupancyOctreeNode>(1) == nullptr);
    EXPECT_TRUE(parent_node->GetChild<OccupancyOctreeNode>(2) == nullptr);
    EXPECT_TRUE(parent_node->GetChild<OccupancyOctreeNode>(3) == nullptr);
    EXPECT_TRUE(parent_node->GetChild<OccupancyOctreeNode>(4) == nullptr);
    EXPECT_TRUE(parent_node->GetChild<OccupancyOctreeNode>(5) == nullptr);
    EXPECT_TRUE(parent_node->GetChild<OccupancyOctreeNode>(6) == nullptr);
    EXPECT_TRUE(parent_node->GetChild<OccupancyOctreeNode>(7) == nullptr);

    // add another new node
    init_size = tree->GetSize();
    auto new_node_2 = tree->CreateNodeChild(parent_node, 3);
    EXPECT_TRUE(new_node_2 != nullptr);
    EXPECT_EQ(parent_node->GetChild<OccupancyOctreeNode>(3), new_node_2);
    new_node_2->SetLogOdds(0.123);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    tree->Prune();
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), init_size + 1);

    // delete them
    tree->DeleteNodeChild(parent_node, 0, parent_key);
    tree->DeleteNodeChild(parent_node, 3, parent_key);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), init_size - 1);
    tree->Prune();
    EXPECT_EQ(tree->GetSize(), init_size - 1);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());

    tree->ExpandNode(parent_node);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), init_size + 7);

    tree->PruneNode(parent_node);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), init_size - 1);

    EXPECT_TRUE(TreeSerializer::Write("prune.ot", *tree));
}

TEST(OccupancyOctree, CopyTree) {  // shallow copy
    OccupancyOctree tree1;
    EXPECT_TRUE(TreeSerializer::Read("prune.ot", tree1));
    const OccupancyOctree tree2 = tree1;
    EXPECT_EQ(  // the setting pointers should be the same
        tree1.GetSetting<OccupancyOctree::Setting>(),
        tree2.GetSetting<OccupancyOctree::Setting>());
    EXPECT_TRUE(tree1 == tree2);  // the content should be the same
    auto itr_tree1 = tree1.BeginTree();
    auto itr_tree2 = tree2.BeginTree();
    for (; itr_tree1 != tree1.EndTree(); ++itr_tree1, ++itr_tree2) {
        EXPECT_EQ(*itr_tree1, *itr_tree2);    // the pointers should be the same
        EXPECT_EQ(**itr_tree1, **itr_tree2);  // the content should be the same
    }
}

TEST(OccupancyOctree, CloneTree) {  // deep copy
    OccupancyOctree tree1;
    EXPECT_TRUE(TreeSerializer::Read("prune.ot", tree1));
    const auto tree2 = std::reinterpret_pointer_cast<OccupancyOctree>(tree1.Clone());
    EXPECT_NE(
        tree1.GetSetting<OccupancyOctree::Setting>(),
        tree2->GetSetting<OccupancyOctree::Setting>());  // the setting pointers should be different
    EXPECT_TRUE(tree1 == *tree2);                        // the content should be the same
    auto itr_tree1 = tree1.BeginTree();
    auto itr_tree2 = tree2->BeginTree();
    for (; itr_tree1 != tree1.EndTree(); ++itr_tree1, ++itr_tree2) {
        EXPECT_NE(*itr_tree1, *itr_tree2);    // the pointers should be different
        EXPECT_EQ(**itr_tree1, **itr_tree2);  // the content should be the same
    }
}

TEST(OccupancyOctree, DeleteTree) {
    OccupancyOctree tree;
    EXPECT_TRUE(TreeSerializer::Read("prune.ot", tree));
    tree.Clear();
    EXPECT_EQ(tree.GetSize(), 0);
    EXPECT_EQ(tree.ComputeNumberOfNodes(), tree.GetSize());
}

TEST(OccupancyOctree, DeleteNodeChild) {
    OccupancyOctree tree;
    EXPECT_TRUE(TreeSerializer::Read("prune.ot", tree));
    const uint32_t cnt_node_deleted =
        tree.DeleteNodeChild(tree.GetRoot().get(), 0, tree.GetTreeCenterKey());
    EXPECT_GT(cnt_node_deleted, 0);
}

TEST(OccupancyOctree, DeleteNode) {
    OccupancyOctree tree;
    EXPECT_TRUE(TreeSerializer::Read("prune.ot", tree));
    const uint32_t cnt_node_deleted = tree.DeleteNode(-0.2, -0.2, -0.2);
    EXPECT_GT(cnt_node_deleted, 0);
}

TEST(OccupancyOctree, Iterator) {
    // iterate over an empty tree
    auto tree_setting = std::make_shared<OccupancyOctree::Setting>();
    tree_setting->resolution = 0.05;
    auto tree = std::make_shared<OccupancyOctree>(tree_setting);
    EXPECT_EQ(tree->GetSize(), 0);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), 0);

    std::size_t num_iterated_nodes = 0;
    auto t_it = tree->BeginTree();
    auto t_end = tree->EndTree();
    EXPECT_EQ(t_it, t_end);
    for (; t_it != t_end; ++t_it) { num_iterated_nodes++; }
    EXPECT_EQ(num_iterated_nodes, 0);

    auto l_it = tree->BeginLeaf();
    auto l_end = tree->EndLeaf();
    for (; l_it != l_end; ++l_it) { num_iterated_nodes++; }
    EXPECT_EQ(num_iterated_nodes, 0);

    for (auto lb_it = tree->BeginLeafInAabb(-1., -1., -1., 1., 1., 1.),
              lb_end = tree->EndLeafInAabb();
         lb_it != lb_end;
         ++lb_it) {
        num_iterated_nodes++;
    }
    EXPECT_EQ(num_iterated_nodes, 0);

    // iterate over a non-empty tree
    std::string file = "OccupancyOctree/InsertPointCloud/sphere.ot";
    EXPECT_TRUE(std::filesystem::exists(file))
        << "File " << file << " does not exist. Run OccupancyOctree/InsertPointCloud first.";
    OccupancyOctree tree_ot;
    EXPECT_TRUE(TreeSerializer::Read(file, tree_ot));
    std::size_t num_iterated_leaf_nodes = 0;
    std::size_t num_iterated_occupied_leaf_nodes = 0;
    l_it = tree_ot.BeginLeaf();
    l_end = tree_ot.EndLeaf();
    for (; l_it != l_end; ++l_it) {
        num_iterated_leaf_nodes++;
        if (tree_ot.IsNodeOccupied(*l_it)) { num_iterated_occupied_leaf_nodes++; }
    }
    EXPECT_EQ(tree_ot.ComputeNumberOfLeafNodes(), num_iterated_leaf_nodes);

    std::size_t occupied_leaf_node_count = 0;
    std::vector<const OccupancyOctreeNode*> stack;
    stack.emplace_back(tree_ot.GetRoot().get());
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();

        if (!node->HasAnyChild()) {
            if (tree_ot.IsNodeOccupied(node)) { occupied_leaf_node_count++; }
            continue;
        }

        for (int i = 0; i < 8; ++i) {
            if (auto child = node->GetChild<OccupancyOctreeNode>(i); child != nullptr) {
                stack.emplace_back(child);
            }
        }
    }
    EXPECT_EQ(num_iterated_occupied_leaf_nodes, occupied_leaf_node_count);
}

TEST(OccupancyOctree, RayCasting) {
    std::string file = "OccupancyOctree/InsertPointCloud/sphere.ot";
    EXPECT_TRUE(std::filesystem::exists(file))
        << "File " << file << " does not exist. Run OccupancyOctree/InsertPointCloud first.";

    OccupancyOctree tree;
    EXPECT_TRUE(TreeSerializer::Read(file, tree));

    ERL_INFO("Casting rays in sphere ...");
    unsigned int hit = 0;
    unsigned int miss = 0;
    unsigned int unknown = 0;
    Dtype mean_dist = 0;
    const auto tree_setting = std::make_shared<OccupancyOctree::Setting>();
    tree_setting->resolution = 0.05;
    auto sampled_surface = std::make_shared<OccupancyOctree>(tree_setting);
    VectorX azimuths = VectorX::LinSpaced(NUM_AZIMUTHS, -M_PI, M_PI);
    VectorX elevations = VectorX::LinSpaced(NUM_ELEVATIONS, -M_PI / 2, M_PI / 2);

    long n = NUM_AZIMUTHS * NUM_ELEVATIONS;
    Matrix3X dirs(3, n);
    for (long i = 0, k = 0; i < NUM_ELEVATIONS; ++i) {
        for (long j = 0; j < NUM_AZIMUTHS; ++j, ++k) {
            // clang-format off
            dirs.col(k) << std::cos(elevations[i]) * std::cos(azimuths[j]),
                           std::cos(elevations[i]) * std::sin(azimuths[j]),
                           std::sin(elevations[i]);
            // clang-format on
        }
    }

    for (int i = 0; i < n; ++i) {
        constexpr bool ignore_unknown = false;
        constexpr Dtype max_range = 6;

        if (Dtype ex = 0, ey = 0, ez = 0;  //
            tree.CastRay(
                SENSOR_ORIGIN_X,
                SENSOR_ORIGIN_Y,
                SENSOR_ORIGIN_Z,
                dirs(0, i),
                dirs(1, i),
                dirs(2, i),
                ignore_unknown,
                max_range,
                ex,
                ey,
                ez)) {
            hit++;
            Dtype dx = ex - SENSOR_ORIGIN_X;
            Dtype dy = ey - SENSOR_ORIGIN_Y;
            Dtype dz = ez - SENSOR_ORIGIN_Z;
            mean_dist += std::sqrt(dx * dx + dy * dy + dz * dz);
            sampled_surface->UpdateNode(ex, ey, ez, true, false);
        } else {
            if (tree.Search(ex, ey, ez)) {
                miss++;  // miss: max_range is reached
            } else {
                unknown++;  // hit unknown
            }
        }
    }

    EXPECT_TRUE(TreeSerializer::Write("sphere_sampled.bt", [&](std::ostream& s) -> bool {
        return sampled_surface->WriteBinary(s);
    }));
    EXPECT_EQ(hit, n);
    EXPECT_EQ(miss, 0);
    EXPECT_EQ(unknown, 0);
    mean_dist /= static_cast<Dtype>(hit);
    EXPECT_NEAR(mean_dist, SPHERE_RADIUS, 0.02);
}
