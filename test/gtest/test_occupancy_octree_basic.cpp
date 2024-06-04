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

using namespace erl::geometry;

TEST(OccupancyOctree, IO) {
    OccupancyOctree tree;
    EXPECT_EQ(tree.GetSize(), 0);
    EXPECT_TRUE(tree.WriteBinary("empty.bt"));
    EXPECT_TRUE(tree.Write("empty.ot"));

    OccupancyOctree empty_read_tree;
    EXPECT_TRUE(empty_read_tree.ReadBinary("empty.bt"));
    EXPECT_EQ(empty_read_tree.GetSize(), 0);
    EXPECT_TRUE(tree == empty_read_tree);

    const auto read_tree_abstract = AbstractOctree::Read("empty.ot");
    EXPECT_TRUE(read_tree_abstract != nullptr);
    const auto occupancy_octree = std::dynamic_pointer_cast<OccupancyOctree>(read_tree_abstract);
    EXPECT_TRUE(occupancy_octree != nullptr);
    EXPECT_EQ(occupancy_octree->GetSize(), 0);
    EXPECT_TRUE(tree == *occupancy_octree);
}

TEST(OccupancyOctree, InsertPointCloud) {
    GTEST_PREPARE_OUTPUT_DIR();

    auto tree_setting = std::make_shared<OccupancyOctree::Setting>();
    tree_setting->resolution = 0.05;
    auto tree = std::make_shared<OccupancyOctree>(tree_setting);

    constexpr long num_azimuths = NUM_AZIMUTHS;
    constexpr long num_elevations = NUM_ELEVATIONS;
    Eigen::VectorXd azimuths = Eigen::VectorXd::LinSpaced(num_azimuths, -M_PI, M_PI);
    Eigen::VectorXd elevations = Eigen::VectorXd::LinSpaced(num_elevations, -M_PI / 2, M_PI / 2);

    long n = num_azimuths * num_elevations;
    Eigen::Matrix3Xd points(3, n);
    Eigen::Vector3d sensor_origin(SENSOR_ORIGIN_X, SENSOR_ORIGIN_Y, SENSOR_ORIGIN_Z);

    for (long i = 0, k = 0; i < num_elevations; ++i) {
        for (long j = 0; j < num_azimuths; ++j, ++k) {
            constexpr double radius = SPHERE_RADIUS;
            // clang-format off
            points.col(k) << std::cos(elevations[i]) * std::cos(azimuths[j]) * radius + sensor_origin.x(),
                             std::cos(elevations[i]) * std::sin(azimuths[j]) * radius + sensor_origin.y(),
                             std::sin(elevations[i]) * radius + sensor_origin.z();
            // clang-format on
        }
    }

    double max_range = -1.;
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
        EXPECT_EQ(node_cnt, 7359);
        EXPECT_EQ(tree->GetSize(), node_cnt);
    }

    EXPECT_TRUE(tree->Write((test_output_dir / "sphere.ot").string()));  // not pruned, log odds tree
    auto read_abstract_tree = AbstractOctree::Read((test_output_dir / "sphere.ot").string());
    EXPECT_TRUE(read_abstract_tree != nullptr);
    auto casted_tree = std::dynamic_pointer_cast<OccupancyOctree>(read_abstract_tree);
    EXPECT_TRUE(casted_tree != nullptr);
    EXPECT_EQ(tree->GetSize(), casted_tree->GetSize());
    EXPECT_TRUE(*tree == *casted_tree);

    // test occupancy node deep copy (shallow copy is not allowed unless it is managed by shared_ptr)
    auto cloned_node = std::shared_ptr<OccupancyOctreeNode>(reinterpret_cast<OccupancyOctreeNode*>(tree->GetRoot()->Clone()));  // deep copy
    EXPECT_TRUE(*cloned_node == *tree->GetRoot());
    cloned_node->GetChild<OccupancyOctreeNode>(1)->AddLogOdds(0.1);
    EXPECT_FALSE(*cloned_node == *tree->GetRoot());

    EXPECT_TRUE(tree->WriteBinary((test_output_dir / "sphere.bt").string()));  // pruned, binary tree
    auto read_tree = std::make_shared<OccupancyOctree>();
    EXPECT_TRUE(read_tree->ReadBinary((test_output_dir / "sphere.bt").string()));
    EXPECT_EQ(tree->GetSize(), read_tree->GetSize());

    auto setting = std::make_shared<OccupancyOctree::Drawer::Setting>();
    OccupancyOctree::Drawer drawer(setting, tree);
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
    Eigen::VectorXd azimuths = Eigen::VectorXd::LinSpaced(num_azimuths, -M_PI, M_PI);
    Eigen::VectorXd elevations = Eigen::VectorXd::LinSpaced(num_elevations, -M_PI / 2, M_PI / 2);

    long n = num_azimuths * num_elevations;
    Eigen::Matrix3Xd points(3, n);
    Eigen::Vector3d sensor_origin(SENSOR_ORIGIN_X, SENSOR_ORIGIN_Y, SENSOR_ORIGIN_Z);

    for (long i = 0, k = 0; i < num_elevations; ++i) {
        for (long j = 0; j < num_azimuths; ++j, ++k) {
            constexpr double radius = SPHERE_RADIUS;
            // clang-format off
            points.col(k) << std::cos(elevations[i]) * std::cos(azimuths[j]) * radius + sensor_origin.x(),
                             std::cos(elevations[i]) * std::sin(azimuths[j]) * radius + sensor_origin.y(),
                             std::sin(elevations[i]) * radius + sensor_origin.z();
            // clang-format on
        }
    }

    double max_range = -1.;
    bool parallel = false;
    bool lazy_eval = false;
    erl::common::ReportTime<std::chrono::milliseconds>(test_info->name(), 1, true, [&] {
        tree->InsertPointCloudRays(points, sensor_origin, max_range, parallel, lazy_eval);
    });

    EXPECT_TRUE(tree->Write((test_output_dir / "sphere.ot").string()));  // not pruned, log odds tree
    auto read_abstract_tree = AbstractOctree::Read((test_output_dir / "sphere.ot").string());
    EXPECT_TRUE(read_abstract_tree != nullptr);
    auto casted_tree = std::dynamic_pointer_cast<OccupancyOctree>(read_abstract_tree);
    EXPECT_TRUE(casted_tree != nullptr);
    EXPECT_TRUE(*tree == *casted_tree);

    EXPECT_TRUE(tree->WriteBinary((test_output_dir / "sphere.bt").string()));  // pruned, binary tree
    auto read_tree_setting = std::make_shared<OccupancyOctree::Setting>();
    read_tree_setting->resolution = 0.1;
    auto read_tree = std::make_shared<OccupancyOctree>(read_tree_setting);
    EXPECT_TRUE(read_tree->ReadBinary((test_output_dir / "sphere.bt").string()));
    EXPECT_EQ(tree->GetSize(), read_tree->GetSize());

    auto setting = std::make_shared<OccupancyOctree::Drawer::Setting>();
    OccupancyOctree::Drawer drawer(setting, tree);
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
    Eigen::VectorXd azimuths = Eigen::VectorXd::LinSpaced(num_azimuths, -M_PI, M_PI);
    Eigen::VectorXd elevations = Eigen::VectorXd::LinSpaced(num_elevations, -M_PI / 2, M_PI / 2);

    long n = num_azimuths * num_elevations;
    Eigen::Matrix3Xd points(3, n);
    Eigen::Vector3d sensor_origin(SENSOR_ORIGIN_X, SENSOR_ORIGIN_Y, SENSOR_ORIGIN_Z);

    for (long i = 0, k = 0; i < num_elevations; ++i) {
        for (long j = 0; j < num_azimuths; ++j, ++k) {
            constexpr double radius = SPHERE_RADIUS;
            // clang-format off
            points.col(k) << std::cos(elevations[i]) * std::cos(azimuths[j]) * radius + sensor_origin.x(),
                             std::cos(elevations[i]) * std::sin(azimuths[j]) * radius + sensor_origin.y(),
                             std::sin(elevations[i]) * radius + sensor_origin.z();
            // clang-format on
        }
    }

    double max_range = -1.;
    bool lazy_eval = false;
    erl::common::ReportTime<std::chrono::milliseconds>(test_info->name(), 1, true, [&] {
        for (int i = 0; i < points.cols(); ++i) {
            tree->InsertRay(sensor_origin[0], sensor_origin[1], sensor_origin[2], points(0, i), points(1, i), points(2, i), max_range, lazy_eval);
        }
    });

    EXPECT_TRUE(tree->Write((test_output_dir / "sphere.ot").string()));  // not pruned, log odds tree
    auto read_abstract_tree = AbstractOctree::Read((test_output_dir / "sphere.ot").string());
    EXPECT_TRUE(read_abstract_tree != nullptr);
    auto casted_tree = std::dynamic_pointer_cast<OccupancyOctree>(read_abstract_tree);
    EXPECT_TRUE(casted_tree != nullptr);
    EXPECT_TRUE(*tree == *casted_tree);

    EXPECT_TRUE(tree->WriteBinary((test_output_dir / "sphere.bt").string()));  // pruned, binary tree
    auto read_tree_setting = std::make_shared<OccupancyOctree::Setting>();
    read_tree_setting->resolution = 0.1;
    auto read_tree = std::make_shared<OccupancyOctree>(read_tree_setting);
    EXPECT_TRUE(read_tree->ReadBinary((test_output_dir / "sphere.bt").string()));
    EXPECT_EQ(tree->GetSize(), read_tree->GetSize());

    auto setting = std::make_shared<OccupancyOctree::Drawer::Setting>();
    OccupancyOctree::Drawer drawer(setting, tree);
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
    constexpr double x = 0;
    constexpr double y = 0;
    constexpr double z = 0;
    OctreeKey key;
    EXPECT_TRUE(tree.CoordToKeyChecked(x, y, z, key));
    double x_inv = 0, y_inv = 0, z_inv = 0;
    tree.KeyToCoord(key, x_inv, y_inv, z_inv);
    EXPECT_FLOAT_EQ(0.025, x_inv);
    EXPECT_FLOAT_EQ(0.025, y_inv);
    EXPECT_FLOAT_EQ(0.025, z_inv);
}

TEST(OccupancyOctree, Prune) {
    double resolution = 0.01;
    auto tree_setting = std::make_shared<OccupancyOctree::Setting>();
    tree_setting->resolution = resolution;
    auto tree = std::make_shared<OccupancyOctree>(tree_setting);

    // after pruning, empty tree is still empty
    EXPECT_EQ(tree->GetSize(), 0);
    tree->Prune();
    EXPECT_EQ(tree->GetSize(), 0);

    // single occupied cell should be found
    double x = -0.05, y = -0.02, z = 0.05;
    OctreeKey single_key;
    EXPECT_TRUE(tree->CoordToKeyChecked(x, y, z, single_key));
    bool occupied = true;
    bool lazy_eval = false;
    auto single_node = tree->UpdateNode(single_key, occupied, lazy_eval);
    EXPECT_TRUE(single_node != nullptr);
    EXPECT_EQ(single_node, tree->Search(single_key));

    // neighboring nodes should not exist
    OctreeKey neighbor_key;
    for (neighbor_key[2] = single_key[2] - 1; neighbor_key[2] <= single_key[2] + 1; ++neighbor_key[2]) {
        for (neighbor_key[1] = single_key[1] - 1; neighbor_key[1] <= single_key[1] + 1; ++neighbor_key[1]) {
            for (neighbor_key[0] = single_key[0] - 1; neighbor_key[0] <= single_key[0] + 1; ++neighbor_key[0]) {
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
    for (neighbor_key[2] = single_key[2] - 1; neighbor_key[2] <= single_key[2] + 1; ++neighbor_key[2]) {
        for (neighbor_key[1] = single_key[1] - 1; neighbor_key[1] <= single_key[1] + 1; ++neighbor_key[1]) {
            for (neighbor_key[0] = single_key[0] - 1; neighbor_key[0] <= single_key[0] + 1; ++neighbor_key[0]) {
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
    for (neighbor_key[2] = single_key[2] - 1; neighbor_key[2] <= single_key[2] + 1; ++neighbor_key[2]) {
        for (neighbor_key[1] = single_key[1] - 1; neighbor_key[1] <= single_key[1] + 1; ++neighbor_key[1]) {
            for (neighbor_key[0] = single_key[0] - 1; neighbor_key[0] <= single_key[0] + 1; ++neighbor_key[0]) {
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
    for (neighbor_key[2] = single_key[2] - 1; neighbor_key[2] <= single_key[2] + 1; ++neighbor_key[2]) {
        for (neighbor_key[1] = single_key[1] - 1; neighbor_key[1] <= single_key[1] + 1; ++neighbor_key[1]) {
            for (neighbor_key[0] = single_key[0] - 1; neighbor_key[0] <= single_key[0] + 1; ++neighbor_key[0]) {
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
    tree->UpdateNode(OctreeKey(single_key[0], single_key[1] + 1, single_key[2]), occupied, lazy_eval);
    EXPECT_EQ(tree->GetSize(), 19);
    tree->UpdateNode(OctreeKey(single_key[0] - 1, single_key[1] + 1, single_key[2]), occupied, lazy_eval);
    EXPECT_EQ(tree->GetSize(), 20);
    tree->UpdateNode(OctreeKey(single_key[0] - 1, single_key[1], single_key[2]), occupied, lazy_eval);
    EXPECT_EQ(tree->GetSize(), 21);
    tree->UpdateNode(OctreeKey(single_key[0], single_key[1], single_key[2] - 1), occupied, lazy_eval);
    EXPECT_EQ(tree->GetSize(), 22);
    tree->UpdateNode(OctreeKey(single_key[0], single_key[1] + 1, single_key[2] - 1), occupied, lazy_eval);
    EXPECT_EQ(tree->GetSize(), 23);
    // should trigger auto-pruning
    auto pruned_node = tree->UpdateNode(OctreeKey(single_key[0] - 1, single_key[1], single_key[2] - 1), occupied, lazy_eval);
    EXPECT_EQ(tree->GetSize(), 16);

    // all queries should now end up at same parent node
    auto node1 = tree->Search(single_key);
    auto node2 = tree->Search(diagonal_key);
    EXPECT_EQ(node1, node2);
    EXPECT_EQ(pruned_node, node1);

    // test larger volume pruning
    for (int i = 0, failed = 0; i <= 31 && !failed; ++i) {
        for (int j = 0; j <= 31 && !failed; ++j) {
            for (int k = 0; k <= 31 && !failed; ++k) {
                auto node = tree->UpdateNode(  //
                    static_cast<double>(i) * resolution + 0.001,
                    static_cast<double>(j) * resolution + 0.001,
                    static_cast<double>(k) * resolution + 0.001,
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
                    static_cast<double>(i) * resolution + 0.001,
                    static_cast<double>(j) * resolution + 0.001,
                    static_cast<double>(k) * resolution + 0.001);
                EXPECT_TRUE(node != nullptr);
                EXPECT_TRUE(tree->IsNodeOccupied(node));
                if (node == nullptr || !tree->IsNodeOccupied(node)) { failed = 1; }
            }
        }
    }

    // update a single cell, should auto-expand
    EXPECT_TRUE(tree->CoordToKeyChecked(0.0, 0.0, 0.0, single_key));
    EXPECT_TRUE(tree->UpdateNode(single_key, occupied, lazy_eval));
    for (int i = 0, failed = 0; i <= 31 && !failed; ++i) {
        for (int j = 0; j <= 31 && !failed; ++j) {
            for (int k = 0; k <= 31 && !failed; ++k) {
                auto node = tree->Search(
                    static_cast<double>(i) * resolution + 0.001,
                    static_cast<double>(j) * resolution + 0.001,
                    static_cast<double>(k) * resolution + 0.001);
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

    EXPECT_TRUE(tree->Write("prune.ot"));
}

TEST(OccupancyOctree, CopyTree) {  // shallow copy
    const auto tree1 = AbstractOctree::ReadAs<OccupancyOctree>("prune.ot");
    const OccupancyOctree tree2 = *tree1;
    EXPECT_EQ(tree1->GetSetting<OccupancyOctree::Setting>(), tree2.GetSetting<OccupancyOctree::Setting>());  // the setting pointers should be the same
    EXPECT_TRUE(*tree1 == tree2);                                                                            // the content should be the same
    auto itr_tree1 = tree1->BeginTree();
    auto itr_tree2 = tree2.BeginTree();
    for (; itr_tree1 != tree1->EndTree(); ++itr_tree1, ++itr_tree2) {
        EXPECT_EQ(*itr_tree1, *itr_tree2);    // the pointers should be the same
        EXPECT_EQ(**itr_tree1, **itr_tree2);  // the content should be the same
    }
}

TEST(OccupancyOctree, CloneTree) {  // deep copy
    const auto tree1 = AbstractOctree::ReadAs<OccupancyOctree>("prune.ot");
    const auto tree2 = std::reinterpret_pointer_cast<OccupancyOctree>(tree1->Clone());
    EXPECT_NE(tree1->GetSetting<OccupancyOctree::Setting>(), tree2->GetSetting<OccupancyOctree::Setting>());  // the setting pointers should be different
    EXPECT_TRUE(*tree1 == *tree2);                                                                            // the content should be the same
    auto itr_tree1 = tree1->BeginTree();
    auto itr_tree2 = tree2->BeginTree();
    for (; itr_tree1 != tree1->EndTree(); ++itr_tree1, ++itr_tree2) {
        EXPECT_NE(*itr_tree1, *itr_tree2);    // the pointers should be different
        EXPECT_EQ(**itr_tree1, **itr_tree2);  // the content should be the same
    }
}

TEST(OccupancyOctree, DeleteTree) {
    const auto abstract_tree = AbstractOctree::Read("prune.ot");
    EXPECT_TRUE(abstract_tree != nullptr);
    const auto tree = std::dynamic_pointer_cast<OccupancyOctree>(abstract_tree);
    EXPECT_TRUE(tree != nullptr);

    tree->Clear();
    EXPECT_EQ(tree->GetSize(), 0);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
}

TEST(OccupancyOctree, DeleteNodeChild) {
    const auto abstract_tree = AbstractOctree::Read("prune.ot");
    EXPECT_TRUE(abstract_tree != nullptr);
    const auto tree = std::dynamic_pointer_cast<OccupancyOctree>(abstract_tree);
    EXPECT_TRUE(tree != nullptr);

    const uint32_t cnt_node_deleted = tree->DeleteNodeChild(tree->GetRoot().get(), 0, tree->GetTreeCenterKey());
    EXPECT_GT(cnt_node_deleted, 0);
}

TEST(OccupancyOctree, DeleteNode) {
    const auto tree = AbstractOctree::ReadAs<OccupancyOctree>("prune.ot");
    EXPECT_TRUE(tree != nullptr);
    const uint32_t cnt_node_deleted = tree->DeleteNode(-0.2, -0.2, -0.2);
    EXPECT_GT(cnt_node_deleted, 0);
}

TEST(OccupancyOctree, Iterator) {
    // iterate over empty tree
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

    for (auto lb_it = tree->BeginLeafInAabb(-1., -1., -1., 1., 1., 1.), lb_end = tree->EndLeafInAabb(); lb_it != lb_end; ++lb_it) { num_iterated_nodes++; }
    EXPECT_EQ(num_iterated_nodes, 0);

    // iterate over non-empty tree
    std::string file = "OccupancyOctree/InsertPointCloud/sphere.ot";
    EXPECT_TRUE(std::filesystem::exists(file)) << "File " << file << " does not exist. Run OccupancyOctree/InsertPointCloud first.";
    auto abstract_tree = AbstractOctree::Read(file);
    EXPECT_TRUE(abstract_tree != nullptr);
    tree = std::dynamic_pointer_cast<OccupancyOctree>(abstract_tree);
    EXPECT_TRUE(tree != nullptr);
    std::size_t num_iterated_leaf_nodes = 0;
    std::size_t num_iterated_occupied_leaf_nodes = 0;
    l_it = tree->BeginLeaf();
    l_end = tree->EndLeaf();
    for (; l_it != l_end; ++l_it) {
        num_iterated_leaf_nodes++;
        if (tree->IsNodeOccupied(*l_it)) { num_iterated_occupied_leaf_nodes++; }
    }
    EXPECT_EQ(tree->ComputeNumberOfLeafNodes(), num_iterated_leaf_nodes);

    std::size_t occupied_leaf_node_count = 0;
    std::vector<const OccupancyOctreeNode*> stack;
    stack.emplace_back(tree->GetRoot().get());
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();

        if (!node->HasAnyChild()) {
            if (tree->IsNodeOccupied(node)) { occupied_leaf_node_count++; }
            continue;
        }

        for (int i = 0; i < 8; ++i) {
            if (auto child = node->GetChild<OccupancyOctreeNode>(i); child != nullptr) { stack.emplace_back(child); }
        }
    }
    EXPECT_EQ(num_iterated_occupied_leaf_nodes, occupied_leaf_node_count);
}

TEST(OccupancyOctree, RayCasting) {
    std::string file = "OccupancyOctree/InsertPointCloud/sphere.ot";
    EXPECT_TRUE(std::filesystem::exists(file)) << "File " << file << " does not exist. Run OccupancyOctree/InsertPointCloud first.";

    auto abstract_tree = AbstractOctree::Read(file);
    EXPECT_TRUE(abstract_tree != nullptr);
    auto tree = std::dynamic_pointer_cast<OccupancyOctree>(abstract_tree);
    EXPECT_TRUE(tree != nullptr);

    ERL_INFO("Casting rays in sphere ...");
    unsigned int hit = 0;
    unsigned int miss = 0;
    unsigned int unknown = 0;
    double mean_dist = 0;
    const auto tree_setting = std::make_shared<OccupancyOctree::Setting>();
    tree_setting->resolution = 0.05;
    auto sampled_surface = std::make_shared<OccupancyOctree>(tree_setting);
    Eigen::VectorXd azimuths = Eigen::VectorXd::LinSpaced(NUM_AZIMUTHS, -M_PI, M_PI);
    Eigen::VectorXd elevations = Eigen::VectorXd::LinSpaced(NUM_ELEVATIONS, -M_PI / 2, M_PI / 2);

    long n = NUM_AZIMUTHS * NUM_ELEVATIONS;
    Eigen::Matrix3Xd dirs(3, n);
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
        constexpr double max_range = 6;

        if (double ex = 0, ey = 0, ez = 0;  //
            tree->CastRay(SENSOR_ORIGIN_X, SENSOR_ORIGIN_Y, SENSOR_ORIGIN_Z, dirs(0, i), dirs(1, i), dirs(2, i), ignore_unknown, max_range, ex, ey, ez)) {
            hit++;
            double dx = ex - SENSOR_ORIGIN_X;
            double dy = ey - SENSOR_ORIGIN_Y;
            double dz = ez - SENSOR_ORIGIN_Z;
            mean_dist += std::sqrt(dx * dx + dy * dy + dz * dz);
            sampled_surface->UpdateNode(ex, ey, ez, true, false);
        } else {
            if (tree->Search(ex, ey, ez)) {
                // miss: max_range is reached
                miss++;
            } else {
                // hit unknown
                unknown++;
            }
        }
    }

    EXPECT_TRUE(sampled_surface->WriteBinary("sphere_sampled.bt"));
    EXPECT_EQ(hit, n);
    EXPECT_EQ(miss, 0);
    EXPECT_EQ(unknown, 0);
    mean_dist /= static_cast<double>(hit);
    EXPECT_NEAR(mean_dist, SPHERE_RADIUS, 0.02);
}
