#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"
#include "erl_common/test_helper.hpp"

using namespace erl::geometry;

TEST(OccupancyQuadtree, IO) {
    OccupancyQuadtree tree;
    EXPECT_EQ(tree.GetSize(), 0);
    EXPECT_TRUE(tree.WriteBinary("empty.bt"));
    EXPECT_TRUE(tree.Write("empty.ot"));

    OccupancyQuadtree empty_read_tree;
    EXPECT_TRUE(empty_read_tree.ReadBinary("empty.bt"));
    EXPECT_EQ(empty_read_tree.GetSize(), 0);
    EXPECT_TRUE(tree == empty_read_tree);

    auto read_tree_abstract = AbstractQuadtree::Read("empty.ot");
    EXPECT_TRUE(read_tree_abstract != nullptr);
    auto occupancy_quadtree = std::dynamic_pointer_cast<OccupancyQuadtree>(read_tree_abstract);
    EXPECT_TRUE(occupancy_quadtree != nullptr);
    EXPECT_EQ(occupancy_quadtree->GetSize(), 0);
    EXPECT_TRUE(tree == *occupancy_quadtree);
}

TEST(OccupancyQuadtree, InsertPointCloud) {
    GTEST_PREPARE_OUTPUT_DIR();

    auto tree_setting = std::make_shared<OccupancyQuadtree::Setting>();
    tree_setting->resolution = 0.05;
    auto tree = std::make_shared<OccupancyQuadtree>(tree_setting);

    long n = 720;
    Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(n, -M_PI, M_PI);
    Eigen::Matrix2Xd points(2, n);
    Eigen::Vector2d sensor_origin(1., 0);

    for (long i = 0; i < n; ++i) {
        // clang-format off
        points.col(i) << std::cos(angles[i]) * 2 + sensor_origin.x(),
                         std::sin(angles[i]) * 2 + sensor_origin.y();
        // clang-format on
    }
    double max_range = -1.;
    bool parallel = false;
    bool lazy_eval = false;
    bool discretize = false;
    erl::common::ReportTime<std::chrono::milliseconds>("InsertPointCloud", 1, true, [&]() {
        tree->InsertPointCloud(points, sensor_origin, max_range, parallel, lazy_eval, discretize);
    });

    auto setting = std::make_shared<OccupancyQuadtree::Drawer::Setting>();
    setting->area_min << -3, -3;
    setting->area_max << 4, 4;
    setting->resolution = 0.01;
    setting->border_color = cv::Scalar(255, 0, 0);
    OccupancyQuadtree::Drawer drawer(setting, tree);
    drawer.DrawLeaves("test_insert_point_cloud_by_point_cloud.png");

    EXPECT_TRUE(tree->WriteBinary("circle.bt"));  // pruned, binary tree
    EXPECT_TRUE(tree->Write("circle.ot"));        // not pruned, log odds tree

    auto read_tree = std::make_shared<OccupancyQuadtree>();
    EXPECT_TRUE(read_tree->ReadBinary("circle.bt"));
    EXPECT_EQ(tree->GetSize(), read_tree->GetSize());

    auto read_abstract_tree = AbstractQuadtree::Read("circle.ot");
    EXPECT_TRUE(read_abstract_tree != nullptr);
    auto casted_tree = std::dynamic_pointer_cast<OccupancyQuadtree>(read_abstract_tree);
    EXPECT_TRUE(casted_tree != nullptr);
    EXPECT_TRUE(*tree == *casted_tree);
}

TEST(OccupancyQuadtree, InsertRay) {
    GTEST_PREPARE_OUTPUT_DIR();

    auto tree_setting = std::make_shared<OccupancyQuadtree::Setting>();
    tree_setting->resolution = 0.05;
    auto tree = std::make_shared<OccupancyQuadtree>(tree_setting);

    long n = 180;
    Eigen::Matrix2Xd points(2, 4 * n);
    double d = std::sqrt(2);
    Eigen::VectorXd a = Eigen::VectorXd::LinSpaced(n, -d, d);
    points.row(0).segment(0, n) = a;
    points.row(1).segment(0, n).setConstant(d);
    points.row(0).segment(n, n).setConstant(d);
    points.row(1).segment(n, n) = a.reverse();
    points.row(0).segment(2 * n, n) = a.reverse();
    points.row(1).segment(2 * n, n).setConstant(-d);
    points.row(0).segment(3 * n, n).setConstant(-d);
    points.row(1).segment(3 * n, n) = a;
    Eigen::Vector2d sensor_origin(0., 0);

    double max_range = -1;
    bool lazy_eval = false;
    erl::common::ReportTime<std::chrono::milliseconds>("InsertRay", 1, true, [&]() {
        for (int i = 0; i < points.cols(); ++i) { tree->InsertRay(sensor_origin[0], sensor_origin[1], points(0, i), points(1, i), max_range, lazy_eval); }
    });

    auto setting = std::make_shared<OccupancyQuadtree::Drawer::Setting>();
    setting->area_min << -3, -3;
    setting->area_max << 4, 4;
    setting->resolution = 0.01;
    setting->border_color = cv::Scalar(255, 0, 0);
    OccupancyQuadtree::Drawer drawer(setting, tree);
    drawer.DrawLeaves("test_insert_point_cloud_by_ray.png");

    EXPECT_TRUE(tree->WriteBinary("square.bt"));
    EXPECT_TRUE(tree->Write("square.ot"));

    auto read_tree = std::make_shared<OccupancyQuadtree>();
    EXPECT_TRUE(read_tree->ReadBinary("square.bt"));
    EXPECT_EQ(tree->GetSize(), read_tree->GetSize());

    auto read_abstract_tree = AbstractQuadtree::Read("square.ot");
    EXPECT_TRUE(read_abstract_tree != nullptr);
    auto casted_tree = std::dynamic_pointer_cast<OccupancyQuadtree>(read_abstract_tree);
    EXPECT_TRUE(casted_tree != nullptr);
    EXPECT_TRUE(*tree == *casted_tree);
}

TEST(OccupancyQuadtree, CoordsAndKey) {
    auto tree_setting = std::make_shared<OccupancyQuadtree::Setting>();
    tree_setting->resolution = 0.05;
    OccupancyQuadtree tree(tree_setting);
    double x = 0, y = 0;
    QuadtreeKey key;
    EXPECT_TRUE(tree.CoordToKeyChecked(x, y, key));
    double x_inv = 0, y_inv = 0;
    tree.KeyToCoord(key, x_inv, y_inv);
    EXPECT_FLOAT_EQ(0.025, x_inv);
    EXPECT_FLOAT_EQ(0.025, y_inv);
}

TEST(OccupancyQuadtree, Prune) {
    double resolution = 0.01;
    auto tree_setting = std::make_shared<OccupancyQuadtree::Setting>();
    tree_setting->resolution = resolution;
    auto tree = std::make_shared<OccupancyQuadtree>(tree_setting);

    // after pruning, empty tree is still empty
    EXPECT_EQ(tree->GetSize(), 0);
    tree->Prune();
    EXPECT_EQ(tree->GetSize(), 0);

    // single occupied cell should be found
    double x = -0.05, y = -0.02;
    QuadtreeKey single_key;
    EXPECT_TRUE(tree->CoordToKeyChecked(x, y, single_key));
    bool occupied = true;
    bool lazy_eval = false;
    auto single_node = tree->UpdateNode(single_key, occupied, lazy_eval);
    EXPECT_TRUE(single_node != nullptr);
    EXPECT_EQ(single_node, tree->Search(single_key));

    // neighboring nodes should not exist
    QuadtreeKey neighbor_key;
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

    // prune should do nothing
    tree->Prune();
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

    // node + 1 branch of depth 16
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), 17);

    // create diagonal neighbor in same parent node
    QuadtreeKey diagonal_key;
    diagonal_key[0] = single_key[0] - 1;
    diagonal_key[1] = single_key[1] + 1;
    auto diagonal_node = tree->UpdateNode(diagonal_key, occupied, lazy_eval);
    EXPECT_TRUE(diagonal_node);
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

    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), 18);

    // prune should do nothing
    tree->Prune();
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
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), 18);

    // fill the complete quadrant, should auto-prune
    tree->UpdateNode(QuadtreeKey(single_key[0], single_key[1] + 1), occupied, lazy_eval);
    EXPECT_EQ(tree->GetSize(), 19);
    auto pruned_node = tree->UpdateNode(QuadtreeKey(single_key[0] - 1, single_key[1]), occupied, lazy_eval);  // should trigger auto-pruning
    EXPECT_EQ(tree->GetSize(), 16);

    // all queries should now end up at same parent node
    auto node1 = tree->Search(single_key);
    auto node2 = tree->Search(diagonal_key);
    EXPECT_EQ(node1, node2);
    EXPECT_EQ(pruned_node, node1);

    // test larger volume pruning
    for (int i = 0; i <= 31; ++i) {
        for (int j = 0; j <= 31; ++j) {
            auto node = tree->UpdateNode(static_cast<double>(i) * resolution + 0.001, static_cast<double>(j) * resolution + 0.001, occupied, lazy_eval);
            EXPECT_TRUE(node != nullptr);
            EXPECT_TRUE(tree->IsNodeOccupied(node));
        }
    }
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), 27);
    tree->Expand();  // increase 32x32 + 16x16 + 8x8 + 4x4 + 2x2 and 2x2 nodes (1368)
    EXPECT_EQ(tree->GetSize(), 1395);
    tree->Prune();
    EXPECT_EQ(tree->GetSize(), 27);
    // test expansion
    for (int i = 0; i <= 31; ++i) {
        for (int j = 0; j <= 31; ++j) {
            auto node = tree->Search(static_cast<double>(i) * resolution + 0.001, static_cast<double>(j) * resolution + 0.001);
            EXPECT_TRUE(node != nullptr);
            EXPECT_TRUE(tree->IsNodeOccupied(node));
        }
    }

    // update a single cell, should auto-expand
    EXPECT_TRUE(tree->CoordToKeyChecked(0.1, 0.1, single_key));
    EXPECT_TRUE(tree->UpdateNode(single_key, occupied, lazy_eval));
    for (int i = 0; i <= 31; ++i) {
        for (int j = 0; j <= 31; ++j) {
            auto node = tree->Search(static_cast<double>(i) * resolution + 0.001, static_cast<double>(j) * resolution + 0.001);
            EXPECT_TRUE(node != nullptr);
            EXPECT_TRUE(tree->IsNodeOccupied(node));
        }
    }
    EXPECT_EQ(tree->GetSize(), 47);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());

    // delete, expand, prune of single node
    std::size_t init_size = tree->GetSize();
    auto new_node = tree->UpdateNode(-2., -2., occupied, lazy_eval);
    EXPECT_TRUE(new_node != nullptr);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), init_size + 8);

    // find parent of newly inserted node
    unsigned int search_depth = tree->GetTreeDepth() - 1;
    auto parent_node = const_cast<OccupancyQuadtreeNode *>(tree->Search(-2., -2., search_depth));
    EXPECT_TRUE(parent_node != nullptr);
    EXPECT_TRUE(parent_node->HasAnyChild());
    // only one child exists
    EXPECT_TRUE(parent_node->GetChild<OccupancyQuadtreeNode>(0) != nullptr);
    EXPECT_TRUE(parent_node->GetChild<OccupancyQuadtreeNode>(1) == nullptr);
    EXPECT_TRUE(parent_node->GetChild<OccupancyQuadtreeNode>(2) == nullptr);
    EXPECT_TRUE(parent_node->GetChild<OccupancyQuadtreeNode>(3) == nullptr);

    // add another new node
    init_size = tree->GetSize();
    auto new_node_2 = tree->CreateNodeChild(parent_node, 3);
    EXPECT_TRUE(new_node_2 != nullptr);
    EXPECT_EQ(parent_node->GetChild<OccupancyQuadtreeNode>(3), new_node_2);
    new_node_2->SetLogOdds(0.123);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    tree->Prune();
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), init_size + 1);

    // delete them
    tree->DeleteNodeChild(parent_node, 0, tree->CoordToKey(-2, -2, 0));
    tree->DeleteNodeChild(parent_node, 3, tree->CoordToKey(-2, -2, 0));
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), init_size - 1);
    tree->Prune();
    EXPECT_EQ(tree->GetSize(), init_size - 1);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());

    tree->ExpandNode(parent_node);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), init_size + 3);

    tree->PruneNode(parent_node);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), init_size - 1);

    EXPECT_TRUE(tree->Write("prune.ot"));
}

TEST(OccupancyQuadtree, DeleteTree) {
    auto abstract_tree = AbstractQuadtree::Read("prune.ot");
    EXPECT_TRUE(abstract_tree != nullptr);
    auto tree = std::dynamic_pointer_cast<OccupancyQuadtree>(abstract_tree);
    EXPECT_TRUE(tree != nullptr);

    tree->Clear();
    EXPECT_EQ(tree->GetSize(), 0);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
}

TEST(OccupancyQuadtree, Iterator) {
    // iterate over empty tree
    auto tree_setting = std::make_shared<OccupancyQuadtree::Setting>();
    tree_setting->resolution = 0.05;
    auto tree = std::make_shared<OccupancyQuadtree>(tree_setting);
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

    auto lb_it = tree->BeginLeafInAabb(-1., -1., 1., 1.);
    auto lb_end = tree->EndLeafInAabb();
    for (; lb_it != lb_end; ++lb_it) { num_iterated_nodes++; }
    EXPECT_EQ(num_iterated_nodes, 0);

    // iterate over non-empty tree
    auto abstract_tree = AbstractQuadtree::Read("circle.ot");
    EXPECT_TRUE(abstract_tree != nullptr);
    tree = std::dynamic_pointer_cast<OccupancyQuadtree>(abstract_tree);
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
    std::vector<const OccupancyQuadtreeNode *> stack;
    stack.emplace_back(tree->GetRoot().get());
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();

        if (!node->HasAnyChild()) {
            if (tree->IsNodeOccupied(node)) { occupied_leaf_node_count++; }
            continue;
        }

        for (int i = 0; i < 4; ++i) {
            auto child = node->GetChild<OccupancyQuadtreeNode>(i);
            if (child != nullptr) { stack.emplace_back(child); }
        }
    }
    EXPECT_EQ(num_iterated_occupied_leaf_nodes, occupied_leaf_node_count);
}

TEST(OccupancyQuadtree, RayCasting) {
    auto abstract_tree = AbstractQuadtree::Read("circle.ot");
    EXPECT_TRUE(abstract_tree != nullptr);
    auto tree = std::dynamic_pointer_cast<OccupancyQuadtree>(abstract_tree);
    EXPECT_TRUE(tree != nullptr);

    auto setting = std::make_shared<OccupancyQuadtree::Drawer::Setting>();
    setting->area_min << -4, -4;
    setting->area_max << 4, 4;
    setting->resolution = 0.01;
    setting->border_color = cv::Scalar(255, 0, 0);
    OccupancyQuadtree::Drawer drawer(setting, tree);
    drawer.DrawLeaves("read_circle.png");

    ERL_INFO("Casting rays in circle ...");
    unsigned int hit = 0;
    unsigned int miss = 0;
    unsigned int unknown = 0;
    double mean_dist = 0;
    auto tree_setting = std::make_shared<OccupancyQuadtree::Setting>();
    tree_setting->resolution = 0.05;
    auto sampled_surface = std::make_shared<OccupancyQuadtree>(tree_setting);
    double sx = 1.0, sy = 0.;
    bool ignore_unknown = false;
    double max_range = 6;
    int n = 10000;
    for (int i = 0; i < n; ++i) {
        double angle = static_cast<double>(i) / 1000 * 2 * M_PI - M_PI;
        double vx = std::cos(angle);
        double vy = std::sin(angle);
        double ex = 0, ey = 0;
        if (tree->CastRay(sx, sy, vx, vy, ignore_unknown, max_range, ex, ey)) {
            hit++;
            double dx = ex - sx;
            double dy = ey - sy;
            mean_dist += std::sqrt(dx * dx + dy * dy);
            sampled_surface->UpdateNode(ex, ey, true, false);
        } else {
            if (tree->Search(ex, ey)) {
                // miss: max_range is reached
                miss++;
            } else {
                // hit unknown
                unknown++;
            }
        }
    }

    EXPECT_TRUE(sampled_surface->WriteBinary("circle_sampled.bt"));
    EXPECT_EQ(hit, n);
    EXPECT_EQ(miss, 0);
    EXPECT_EQ(unknown, 0);
    mean_dist /= static_cast<double>(hit);
    EXPECT_NEAR(mean_dist, 2.0, 0.01);
}
