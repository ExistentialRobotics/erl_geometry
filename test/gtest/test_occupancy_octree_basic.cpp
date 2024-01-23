#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/occupancy_octree_drawer.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_common/angle_utils.hpp"

#define SPHERE_RADIUS   0.75
#define NUM_AZIMUTHS    720
#define NUM_ELEVATIONS  180
#define SENSOR_ORIGIN_X 1.0
#define SENSOR_ORIGIN_Y 0.0
#define SENSOR_ORIGIN_Z 0.0

TEST(OccupancyOctree, IO) {
    erl::geometry::OccupancyOctree tree(0.1);
    EXPECT_EQ(tree.GetSize(), 0);
    EXPECT_TRUE(tree.WriteBinary("empty.bt"));
    EXPECT_TRUE(tree.Write("empty.ot"));

    erl::geometry::OccupancyOctree empty_read_tree(0.2);
    EXPECT_TRUE(empty_read_tree.ReadBinary("empty.bt"));
    EXPECT_EQ(empty_read_tree.GetSize(), 0);
    EXPECT_TRUE(tree == empty_read_tree);

    auto read_tree_abstract = erl::geometry::AbstractOctree::Read("empty.ot");
    EXPECT_TRUE(read_tree_abstract != nullptr);
    auto occupancy_octree = std::dynamic_pointer_cast<erl::geometry::OccupancyOctree>(read_tree_abstract);
    EXPECT_TRUE(occupancy_octree != nullptr);
    EXPECT_EQ(occupancy_octree->GetSize(), 0);
    EXPECT_TRUE(tree == *occupancy_octree);
}

TEST(OccupancyOctree, InsertPointCloud) {
    auto tree = std::make_shared<erl::geometry::OccupancyOctree>(0.05);

    double radius = SPHERE_RADIUS;
    long num_azimuths = NUM_AZIMUTHS;
    long num_elevations = NUM_ELEVATIONS;
    Eigen::VectorXd azimuths = Eigen::VectorXd::LinSpaced(num_azimuths, -M_PI, M_PI);
    Eigen::VectorXd elevations = Eigen::VectorXd::LinSpaced(num_elevations, -M_PI / 2, M_PI / 2);

    long n = num_azimuths * num_elevations;
    Eigen::Matrix3Xd points(3, n);
    Eigen::Vector3d sensor_origin(SENSOR_ORIGIN_X, SENSOR_ORIGIN_Y, SENSOR_ORIGIN_Z);

    for (long i = 0, k = 0; i < num_elevations; ++i) {
        for (long j = 0; j < num_azimuths; ++j, ++k) {
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
    erl::common::ReportTime<std::chrono::milliseconds>("InsertPointCloud", 1, true, [&]() {
        tree->InsertPointCloud(points, sensor_origin, max_range, parallel, lazy_eval, discretize);
    });

    EXPECT_TRUE(tree->Write("sphere.ot"));  // not pruned, log odds tree
    auto read_abstract_tree = erl::geometry::AbstractOctree::Read("sphere.ot");
    EXPECT_TRUE(read_abstract_tree != nullptr);
    auto casted_tree = std::dynamic_pointer_cast<erl::geometry::OccupancyOctree>(read_abstract_tree);
    EXPECT_TRUE(casted_tree != nullptr);
    EXPECT_TRUE(*tree == *casted_tree);

    EXPECT_TRUE(tree->WriteBinary("sphere.bt"));  // pruned, binary tree
    auto read_tree = std::make_shared<erl::geometry::OccupancyOctree>(0.1);
    EXPECT_TRUE(read_tree->ReadBinary("sphere.bt"));
    EXPECT_EQ(tree->GetSize(), read_tree->GetSize());

    auto setting = std::make_shared<erl::geometry::OccupancyOctree::Drawer::Setting>();
    erl::geometry::OccupancyOctree::Drawer drawer(setting, tree);
    erl::geometry::Open3dVisualizerWrapper visualizer;
    drawer.DrawLeaves(visualizer.GetVisualizer().get());
    visualizer.Show(10);
}

TEST(OccupancyOctree, InsertRay) {
    auto tree = std::make_shared<erl::geometry::OccupancyOctree>(0.05);

    double radius = SPHERE_RADIUS;
    long num_azimuths = NUM_AZIMUTHS;
    long num_elevations = NUM_ELEVATIONS;
    Eigen::VectorXd azimuths = Eigen::VectorXd::LinSpaced(num_azimuths, -M_PI, M_PI);
    Eigen::VectorXd elevations = Eigen::VectorXd::LinSpaced(num_elevations, -M_PI / 2, M_PI / 2);

    long n = num_azimuths * num_elevations;
    Eigen::Matrix3Xd points(3, n);
    Eigen::Vector3d sensor_origin(SENSOR_ORIGIN_X, SENSOR_ORIGIN_Y, SENSOR_ORIGIN_Z);

    for (long i = 0, k = 0; i < num_elevations; ++i) {
        for (long j = 0; j < num_azimuths; ++j, ++k) {
            // clang-format off
            points.col(k) << std::cos(elevations[i]) * std::cos(azimuths[j]) * radius + sensor_origin.x(),
                             std::cos(elevations[i]) * std::sin(azimuths[j]) * radius + sensor_origin.y(),
                             std::sin(elevations[i]) * radius + sensor_origin.z();
            // clang-format on
        }
    }

    double max_range = -1.;
    bool lazy_eval = false;
    erl::common::ReportTime<std::chrono::milliseconds>("InsertRay", 1, true, [&]() {
        for (int i = 0; i < points.cols(); ++i) {
            tree->InsertRay(sensor_origin[0], sensor_origin[1], sensor_origin[2], points(0, i), points(1, i), points(2, i), max_range, lazy_eval);
        }
    });

    EXPECT_TRUE(tree->Write("sphere.ot"));  // not pruned, log odds tree
    auto read_abstract_tree = erl::geometry::AbstractOctree::Read("sphere.ot");
    EXPECT_TRUE(read_abstract_tree != nullptr);
    auto casted_tree = std::dynamic_pointer_cast<erl::geometry::OccupancyOctree>(read_abstract_tree);
    EXPECT_TRUE(casted_tree != nullptr);
    EXPECT_TRUE(*tree == *casted_tree);

    EXPECT_TRUE(tree->WriteBinary("sphere.bt"));  // pruned, binary tree
    auto read_tree = std::make_shared<erl::geometry::OccupancyOctree>(0.1);
    EXPECT_TRUE(read_tree->ReadBinary("sphere.bt"));
    EXPECT_EQ(tree->GetSize(), read_tree->GetSize());

    auto setting = std::make_shared<erl::geometry::OccupancyOctree::Drawer::Setting>();
    erl::geometry::OccupancyOctree::Drawer drawer(setting, tree);
    erl::geometry::Open3dVisualizerWrapper visualizer;
    drawer.DrawLeaves(visualizer.GetVisualizer().get());
    visualizer.Show(2);
}

TEST(OccupancyOctree, CoordsAndKey) {
    erl::geometry::OccupancyOctree tree(0.05);
    double x = 0, y = 0, z = 0;
    erl::geometry::OctreeKey key;
    EXPECT_TRUE(tree.CoordToKeyChecked(x, y, z, key));
    double x_inv = 0, y_inv = 0, z_inv = 0;
    tree.KeyToCoord(key, x_inv, y_inv, z_inv);
    EXPECT_FLOAT_EQ(0.025, x_inv);
    EXPECT_FLOAT_EQ(0.025, y_inv);
    EXPECT_FLOAT_EQ(0.025, z_inv);
}

TEST(OccupancyOctree, Prune) {
    double resolution = 0.01;
    auto tree = std::make_shared<erl::geometry::OccupancyOctree>(resolution);

    // constexpr bool visualize = true;
    // auto setting = std::make_shared<erl::geometry::OccupancyOctree::Drawer::Setting>();
    // setting->area_min = Eigen::Vector3d(-0.5, -0.5, -0.5);
    // setting->area_max = Eigen::Vector3d(0.5, 0.5, 0.5);
    // erl::geometry::OccupancyOctree::Drawer drawer(setting, tree);
    // erl::geometry::Open3dVisualizerWrapper visualizer;

    // after pruning, empty tree is still empty
    EXPECT_EQ(tree->GetSize(), 0);
    tree->Prune();
    EXPECT_EQ(tree->GetSize(), 0);

    // single occupied cell should be found
    double x = -0.05, y = -0.02, z = 0.05;
    // double x = 0.0, y = 0.0, z = 0.0;
    erl::geometry::OctreeKey single_key;
    EXPECT_TRUE(tree->CoordToKeyChecked(x, y, z, single_key));
    bool occupied = true;
    bool lazy_eval = false;
    auto single_node = tree->UpdateNode(single_key, occupied, lazy_eval);
    EXPECT_TRUE(single_node != nullptr);
    EXPECT_EQ(single_node, tree->Search(single_key));

    // neighboring nodes should not exist
    erl::geometry::OctreeKey neighbor_key;
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
    erl::geometry::OctreeKey diagonal_key;
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
    tree->UpdateNode(erl::geometry::OctreeKey(single_key[0], single_key[1] + 1, single_key[2]), occupied, lazy_eval);
    EXPECT_EQ(tree->GetSize(), 19);
    tree->UpdateNode(erl::geometry::OctreeKey(single_key[0] - 1, single_key[1] + 1, single_key[2]), occupied, lazy_eval);
    EXPECT_EQ(tree->GetSize(), 20);
    tree->UpdateNode(erl::geometry::OctreeKey(single_key[0] - 1, single_key[1], single_key[2]), occupied, lazy_eval);
    EXPECT_EQ(tree->GetSize(), 21);
    tree->UpdateNode(erl::geometry::OctreeKey(single_key[0], single_key[1], single_key[2] - 1), occupied, lazy_eval);
    EXPECT_EQ(tree->GetSize(), 22);
    tree->UpdateNode(erl::geometry::OctreeKey(single_key[0], single_key[1] + 1, single_key[2] - 1), occupied, lazy_eval);
    EXPECT_EQ(tree->GetSize(), 23);
    // should trigger auto-pruning
    auto pruned_node = tree->UpdateNode(erl::geometry::OctreeKey(single_key[0] - 1, single_key[1], single_key[2] - 1), occupied, lazy_eval);
    EXPECT_EQ(tree->GetSize(), 16);

    // all queries should now end up at same parent node
    auto node1 = tree->Search(single_key);
    auto node2 = tree->Search(diagonal_key);
    EXPECT_EQ(node1, node2);
    EXPECT_EQ(pruned_node, node1);

    // test larger volume pruning
    for (int i = 0; i <= 31; ++i) {
        for (int j = 0; j <= 31; ++j) {
            for (int k = 0; k <= 31; ++k) {
                auto node = tree->UpdateNode(  //
                    double(i) * resolution + 0.001,
                    double(j) * resolution + 0.001,
                    double(k) * resolution + 0.001,
                    occupied,
                    lazy_eval);
                EXPECT_TRUE(node != nullptr);
                EXPECT_TRUE(tree->IsNodeOccupied(node));
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
    for (int i = 0; i <= 31; ++i) {
        for (int j = 0; j <= 31; ++j) {
            for (int k = 0; k <= 31; ++k) {
                auto node = tree->Search(double(i) * resolution + 0.001, double(j) * resolution + 0.001, double(k) * resolution + 0.001);
                EXPECT_TRUE(node != nullptr);
                EXPECT_TRUE(tree->IsNodeOccupied(node));
            }
        }
    }

    // update a single cell, should auto-expand
    EXPECT_TRUE(tree->CoordToKeyChecked(0.0, 0.0, 0.0, single_key));
    EXPECT_TRUE(tree->UpdateNode(single_key, occupied, lazy_eval));
    for (int i = 0; i <= 31; ++i) {
        for (int j = 0; j <= 31; ++j) {
            for (int k = 0; k <= 31; ++k) {
                auto node = tree->Search(double(i) * resolution + 0.001, double(j) * resolution + 0.001, double(k) * resolution + 0.001);
                EXPECT_TRUE(node != nullptr);
                EXPECT_TRUE(tree->IsNodeOccupied(node));
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
    // if (visualize) {
    //     drawer.DrawTree(visualizer.GetVisualizer().get());
    //     visualizer.Show();
    // }

    // find parent of newly inserted node
    unsigned int search_depth = tree->GetTreeDepth() - 1;
    auto parent_node = tree->Search(-0.2, -0.2, -0.2, search_depth);
    EXPECT_TRUE(parent_node != nullptr);
    EXPECT_TRUE(parent_node->HasAnyChild());
    // only one child exists
    EXPECT_TRUE(parent_node->GetChild(0) != nullptr);
    EXPECT_TRUE(parent_node->GetChild(1) == nullptr);
    EXPECT_TRUE(parent_node->GetChild(2) == nullptr);
    EXPECT_TRUE(parent_node->GetChild(3) == nullptr);
    EXPECT_TRUE(parent_node->GetChild(4) == nullptr);
    EXPECT_TRUE(parent_node->GetChild(5) == nullptr);
    EXPECT_TRUE(parent_node->GetChild(6) == nullptr);
    EXPECT_TRUE(parent_node->GetChild(7) == nullptr);

    // add another new node
    init_size = tree->GetSize();
    auto new_node_2 = tree->CreateNodeChild(parent_node, 3);
    EXPECT_TRUE(new_node_2 != nullptr);
    EXPECT_EQ(parent_node->GetChild(3), new_node_2);
    new_node_2->SetLogOdds(0.123);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    tree->Prune();
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), init_size + 1);

    // delete them
    tree->DeleteNodeChild(parent_node, 0);
    tree->DeleteNodeChild(parent_node, 3);
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

TEST(OccupancyOctree, DeleteTree) {
    auto abstract_tree = erl::geometry::AbstractOctree::Read("prune.ot");
    EXPECT_TRUE(abstract_tree != nullptr);
    auto tree = std::dynamic_pointer_cast<erl::geometry::OccupancyOctree>(abstract_tree);
    EXPECT_TRUE(tree != nullptr);

    tree->Clear();
    EXPECT_EQ(tree->GetSize(), 0);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
}

TEST(OccupancyOctree, Iterator) {
    // iterate over empty tree
    auto tree = std::make_shared<erl::geometry::OccupancyOctree>(0.05);
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

    auto lb_it = tree->BeginLeafInAabb(-1., -1., -1., 1., 1., 1.);
    auto lb_end = tree->EndLeafInAabb();
    for (; lb_it != lb_end; ++lb_it) { num_iterated_nodes++; }
    EXPECT_EQ(num_iterated_nodes, 0);

    // iterate over non-empty tree
    auto abstract_tree = erl::geometry::AbstractOctree::Read("sphere.ot");
    EXPECT_TRUE(abstract_tree != nullptr);
    tree = std::dynamic_pointer_cast<erl::geometry::OccupancyOctree>(abstract_tree);
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
    std::vector<std::shared_ptr<const erl::geometry::OccupancyOctreeNode>> stack;
    stack.emplace_back(tree->GetRoot());
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();

        if (!node->HasAnyChild()) {
            if (tree->IsNodeOccupied(node)) { occupied_leaf_node_count++; }
            continue;
        }

        for (int i = 0; i < 8; ++i) {
            auto child = node->GetChild<erl::geometry::OccupancyOctreeNode>(i);
            if (child != nullptr) { stack.emplace_back(child); }
        }
    }
    EXPECT_EQ(num_iterated_occupied_leaf_nodes, occupied_leaf_node_count);
}

TEST(OccupancyOctree, RayCasting) {
    auto abstract_tree = erl::geometry::AbstractOctree::Read("sphere.ot");
    EXPECT_TRUE(abstract_tree != nullptr);
    auto tree = std::dynamic_pointer_cast<erl::geometry::OccupancyOctree>(abstract_tree);
    EXPECT_TRUE(tree != nullptr);

    // auto setting = std::make_shared<erl::geometry::OccupancyOctree::Drawer::Setting>();
    // erl::geometry::OccupancyOctree::Drawer drawer(setting, tree);
    // drawer.DrawLeaves("read_sphere.png");

    ERL_INFO("Casting rays in sphere ...");
    unsigned int hit = 0;
    unsigned int miss = 0;
    unsigned int unknown = 0;
    double mean_dist = 0;
    auto sampled_surface = std::make_shared<erl::geometry::OccupancyOctree>(0.05);
    double sx = SENSOR_ORIGIN_X, sy = SENSOR_ORIGIN_Y, sz = SENSOR_ORIGIN_Z;
    bool ignore_unknown = false;
    double max_range = 6;

    long num_azimuths = NUM_AZIMUTHS;
    long num_elevations = NUM_ELEVATIONS;
    Eigen::VectorXd azimuths = Eigen::VectorXd::LinSpaced(num_azimuths, -M_PI, M_PI);
    Eigen::VectorXd elevations = Eigen::VectorXd::LinSpaced(num_elevations, -M_PI / 2, M_PI / 2);

    long n = num_azimuths * num_elevations;
    Eigen::Matrix3Xd dirs(3, n);
    for (long i = 0, k = 0; i < num_elevations; ++i) {
        for (long j = 0; j < num_azimuths; ++j, ++k) {
            // clang-format off
            dirs.col(k) << std::cos(elevations[i]) * std::cos(azimuths[j]),
                           std::cos(elevations[i]) * std::sin(azimuths[j]),
                           std::sin(elevations[i]);
            // clang-format on
        }
    }

    for (int i = 0; i < n; ++i) {
        double ex = 0, ey = 0, ez = 0;
        if (tree->CastRay(sx, sy, sz, dirs(0, i), dirs(1, i), dirs(2, i), ignore_unknown, max_range, ex, ey, ez)) {
            hit++;
            double dx = ex - sx;
            double dy = ey - sy;
            double dz = ez - sz;
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
    mean_dist /= double(hit);
    EXPECT_NEAR(mean_dist, SPHERE_RADIUS, 0.02);
}
