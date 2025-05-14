#include "erl_common/test_helper.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"

using Dtype = double;
using AbstractQuadtree = erl::geometry::AbstractQuadtree<Dtype>;
using OccupancyQuadtree = erl::geometry::OccupancyQuadtree<Dtype>;
using TreeSerializer = erl::common::Serialization<OccupancyQuadtree>;
using OccupancyQuadtreeNode = erl::geometry::OccupancyQuadtreeNode;
using QuadtreeKey = erl::geometry::QuadtreeKey;
using OccupancyQuadtreeDrawer = erl::geometry::OccupancyQuadtreeDrawer<OccupancyQuadtree>;
using VectorX = Eigen::VectorX<Dtype>;
using Vector2 = Eigen::Vector2<Dtype>;
using Matrix2X = Eigen::Matrix2X<Dtype>;

TEST(OccupancyQuadtree, IO) {
    OccupancyQuadtree tree;
    EXPECT_EQ(tree.GetSize(), 0);
    EXPECT_TRUE(TreeSerializer::Write("empty.bt", [&](std::ofstream &s) -> bool {
        return tree.WriteBinary(s);
    }));
    EXPECT_TRUE(TreeSerializer::Write("empty.ot", &tree));

    OccupancyQuadtree read_tree_bt;
    EXPECT_TRUE(TreeSerializer::Read("empty.bt", [&](std::ifstream &s) -> bool {
        return read_tree_bt.ReadBinary(s);
    }));
    EXPECT_EQ(read_tree_bt.GetSize(), 0);
    EXPECT_TRUE(tree == read_tree_bt);

    OccupancyQuadtree read_tree_ot;
    EXPECT_TRUE(TreeSerializer::Read("empty.ot", &read_tree_ot));
    EXPECT_EQ(read_tree_ot.GetSize(), 0);
    EXPECT_TRUE(tree == read_tree_ot);
}

TEST(OccupancyQuadtree, InsertPointCloud) {
    GTEST_PREPARE_OUTPUT_DIR();

    auto tree_setting = std::make_shared<OccupancyQuadtree::Setting>();
    tree_setting->resolution = 0.05;
    auto tree = std::make_shared<OccupancyQuadtree>(tree_setting);

    long n = 720;
    VectorX angles = VectorX::LinSpaced(n, -M_PI, M_PI);
    Matrix2X points(2, n);
    Vector2 sensor_origin(1., 0);

    for (long i = 0; i < n; ++i) {
        // clang-format off
        points.col(i) << std::cos(angles[i]) * 2 + sensor_origin.x(),
                         std::sin(angles[i]) * 2 + sensor_origin.y();
        // clang-format on
    }
    Dtype max_range = -1.;
    bool parallel = false;
    bool lazy_eval = false;
    bool discretize = false;
    erl::common::ReportTime<std::chrono::milliseconds>("InsertPointCloud", 1, true, [&] {
        tree->InsertPointCloud(points, sensor_origin, max_range, parallel, lazy_eval, discretize);
    });

    auto setting = std::make_shared<OccupancyQuadtreeDrawer::Setting>();
    setting->area_min << -3, -3;
    setting->area_max << 4, 4;
    setting->resolution = 0.01;
    setting->border_color = cv::Scalar(255, 0, 0);
    OccupancyQuadtreeDrawer drawer(setting, tree);
    drawer.DrawLeaves("test_insert_point_cloud_by_point_cloud.png");

    // pruned, binary tree
    EXPECT_TRUE(TreeSerializer::Write("circle.bt", [&](std::ofstream &s) -> bool {
        return tree->WriteBinary(s, true);
    }));
    EXPECT_TRUE(TreeSerializer::Write("circle.ot", tree));

    auto read_tree_bt = std::make_shared<OccupancyQuadtree>();
    EXPECT_TRUE(TreeSerializer::Read("circle.bt", [&](std::ifstream &s) -> bool {
        return read_tree_bt->ReadBinary(s);
    }));
    EXPECT_EQ(tree->GetSize(), read_tree_bt->GetSize());

    auto read_tree_ot = std::make_shared<OccupancyQuadtree>();
    EXPECT_TRUE(TreeSerializer::Read("circle.ot", read_tree_ot));
    EXPECT_EQ(*tree, *read_tree_ot);
}

TEST(OccupancyQuadtree, InsertRay) {
    GTEST_PREPARE_OUTPUT_DIR();

    auto tree_setting = std::make_shared<OccupancyQuadtree::Setting>();
    tree_setting->resolution = 0.05;
    auto tree = std::make_shared<OccupancyQuadtree>(tree_setting);

    long n = 180;
    Eigen::Matrix2Xd points(2, 4 * n);
    Dtype d = std::sqrt(2.0f);
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

    Dtype max_range = -1;
    bool lazy_eval = false;
    erl::common::ReportTime<std::chrono::milliseconds>("InsertRay", 1, true, [&] {
        for (int i = 0; i < points.cols(); ++i) {
            tree->InsertRay(
                sensor_origin[0],
                sensor_origin[1],
                points(0, i),
                points(1, i),
                max_range,
                lazy_eval);
        }
    });

    auto setting = std::make_shared<OccupancyQuadtreeDrawer::Setting>();
    setting->area_min << -3, -3;
    setting->area_max << 4, 4;
    setting->resolution = 0.01;
    setting->border_color = cv::Scalar(255, 0, 0);
    OccupancyQuadtreeDrawer drawer(setting, tree);
    drawer.DrawLeaves("test_insert_point_cloud_by_ray.png");

    EXPECT_TRUE(TreeSerializer::Write("square.bt", [&](std::ofstream &s) -> bool {
        return tree->WriteBinary(s, true);
    }));
    EXPECT_TRUE(TreeSerializer::Write("square.ot", tree));

    auto read_tree_bt = std::make_shared<OccupancyQuadtree>();
    EXPECT_TRUE(TreeSerializer::Read("square.bt", [&](std::ifstream &s) -> bool {
        return read_tree_bt->ReadBinary(s);
    }));
    EXPECT_EQ(tree->GetSize(), read_tree_bt->GetSize());

    auto read_tree_ot = std::make_shared<OccupancyQuadtree>();
    EXPECT_TRUE(TreeSerializer::Read("square.ot", read_tree_ot));
    EXPECT_EQ(*tree, *read_tree_ot);
}

TEST(OccupancyQuadtree, CoordsAndKey) {
    const auto tree_setting = std::make_shared<OccupancyQuadtree::Setting>();
    tree_setting->resolution = 0.05;
    OccupancyQuadtree tree(tree_setting);
    constexpr Dtype x = 0;
    constexpr Dtype y = 0;
    QuadtreeKey key;
    EXPECT_TRUE(tree.CoordToKeyChecked(x, y, key));
    Dtype x_inv = 0, y_inv = 0;
    tree.KeyToCoord(key, x_inv, y_inv);
    EXPECT_FLOAT_EQ(0.025, x_inv);
    EXPECT_FLOAT_EQ(0.025, y_inv);
    QuadtreeKey key_inv = tree.CoordToKey(x_inv, y_inv);
    EXPECT_EQ(key, key_inv);

    tree_setting->resolution = 0.1;
    tree.ApplySetting();
    key[0] = 32888;  // this is a key but will not appear exactly at depth 14
    key[1] = 32760;
    tree.KeyToCoord(key, 14, x_inv, y_inv);  // get the center of the cell at depth 14
    EXPECT_FLOAT_EQ(12.2, x_inv);
    EXPECT_FLOAT_EQ(-0.6, y_inv);
    key = tree.CoordToKey(x_inv, y_inv, 14);  // different key
    EXPECT_EQ(key[0], 32890);
    EXPECT_EQ(key[1], 32762);

    const uint64_t morton_code = key.ToMortonCode();
    const QuadtreeKey key_from_morton_code(morton_code);
    EXPECT_EQ(key, key_from_morton_code);
}

TEST(OccupancyQuadtree, Prune) {
    Dtype resolution = 0.01f;
    auto tree_setting = std::make_shared<OccupancyQuadtree::Setting>();
    tree_setting->resolution = static_cast<float>(resolution);
    auto tree = std::make_shared<OccupancyQuadtree>(tree_setting);

    // after pruning, the empty tree is still empty
    EXPECT_EQ(tree->GetSize(), 0);
    tree->Prune();
    EXPECT_EQ(tree->GetSize(), 0);

    // a single occupied cell should be found
    Dtype x = -0.04f, y = -0.02f;
    QuadtreeKey single_key;
    EXPECT_TRUE(tree->CoordToKeyChecked(x, y, single_key));
    bool occupied = true;
    bool lazy_eval = false;
    auto single_node = tree->UpdateNode(single_key, occupied, lazy_eval);
    EXPECT_TRUE(single_node != nullptr);
    EXPECT_EQ(single_node, tree->Search(single_key));
    EXPECT_EQ(single_node->GetChildIndex(), 0);  // make share it is the first child.

    // neighboring nodes should not exist
    QuadtreeKey neighbor_key;
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

    // prune should do nothing
    tree->Prune();
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

    // node + 1 branch of depth 16
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), 17);

    // create diagonal neighbor in same parent node
    QuadtreeKey diagonal_key;
    diagonal_key[0] = single_key[0] + 1;
    diagonal_key[1] = single_key[1] + 1;
    auto diagonal_node = tree->UpdateNode(diagonal_key, occupied, lazy_eval);
    EXPECT_TRUE(diagonal_node);
    EXPECT_EQ(diagonal_node->GetChildIndex(), 3);
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

    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), 18);

    // prune should do nothing
    tree->Prune();
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
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), 18);

    // fill the complete quadrant, should auto-prune
    tree->UpdateNode(QuadtreeKey(single_key[0], single_key[1] + 1), occupied, lazy_eval);
    EXPECT_EQ(tree->GetSize(), 19);
    auto pruned_node = tree->UpdateNode(
        QuadtreeKey(single_key[0] + 1, single_key[1]),
        occupied,
        lazy_eval);  // should trigger auto-pruning
    EXPECT_EQ(tree->GetSize(), 16);

    // all queries should now end up at the same parent node
    auto node1 = tree->Search(single_key);
    auto node2 = tree->Search(diagonal_key);
    EXPECT_EQ(node1, node2);
    EXPECT_EQ(pruned_node, node1);

    // test larger volume pruning
    for (int i = 0; i <= 31; ++i) {
        for (int j = 0; j <= 31; ++j) {
            auto node = tree->UpdateNode(
                static_cast<Dtype>(i) * resolution + 0.001,
                static_cast<Dtype>(j) * resolution + 0.001,
                occupied,
                lazy_eval);
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
            auto node = tree->Search(
                static_cast<Dtype>(i) * resolution + 0.001,
                static_cast<Dtype>(j) * resolution + 0.001);
            EXPECT_TRUE(node != nullptr);
            EXPECT_TRUE(tree->IsNodeOccupied(node));
        }
    }

    // update a single cell, which should auto-expand
    EXPECT_TRUE(tree->CoordToKeyChecked(0.1, 0.1, single_key));
    EXPECT_TRUE(tree->UpdateNode(single_key, occupied, lazy_eval));
    for (int i = 0; i <= 31; ++i) {
        for (int j = 0; j <= 31; ++j) {
            auto node = tree->Search(
                static_cast<Dtype>(i) * resolution + 0.001,
                static_cast<Dtype>(j) * resolution + 0.001);
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
    EXPECT_EQ(new_node->GetChildIndex(), 3);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), init_size + 8);

    // find parent of newly inserted node
    unsigned int search_depth = tree->GetTreeDepth() - 1;
    auto parent_node = const_cast<OccupancyQuadtreeNode *>(tree->Search(-2., -2., search_depth));
    EXPECT_TRUE(parent_node != nullptr);
    EXPECT_TRUE(parent_node->HasAnyChild());
    // only one child exists
    EXPECT_EQ(parent_node->GetNumChildren(), 1);
    int cnt_children = 0;
    for (int i = 0; i < 4; ++i) {
        if (parent_node->GetChild<OccupancyQuadtreeNode>(i) == nullptr) { continue; }
        ++cnt_children;
    }
    EXPECT_EQ(cnt_children, 1);
    EXPECT_EQ(parent_node->GetChild<OccupancyQuadtreeNode>(new_node->GetChildIndex()), new_node);

    // add another new node
    init_size = tree->GetSize();
    auto new_node_2 = tree->CreateNodeChild(parent_node, 1);
    EXPECT_TRUE(new_node_2 != nullptr);
    EXPECT_EQ(parent_node->GetChild<OccupancyQuadtreeNode>(1), new_node_2);
    new_node_2->SetLogOdds(0.123);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    tree->Prune();
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
    EXPECT_EQ(tree->GetSize(), init_size + 1);

    // delete them
    tree->DeleteNodeChild(parent_node, 1, tree->CoordToKey(-2, -2, 0));
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

    EXPECT_TRUE(TreeSerializer::Write("prune.ot", tree));
}

TEST(OccupancyQuadtree, DeleteTree) {
    auto tree = std::make_shared<OccupancyQuadtree>();
    EXPECT_TRUE(TreeSerializer::Read("prune.ot", tree));
    tree->Clear();
    EXPECT_EQ(tree->GetSize(), 0);
    EXPECT_EQ(tree->ComputeNumberOfNodes(), tree->GetSize());
}

TEST(OccupancyQuadtree, Iterator) {
    // iterate over an empty tree
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

    for (auto lb_it = tree->BeginLeafInAabb(-1., -1., 1., 1.), lb_end = tree->EndLeafInAabb();
         lb_it != lb_end;
         ++lb_it) {
        num_iterated_nodes++;
    }
    EXPECT_EQ(num_iterated_nodes, 0);

    // iterate over a non-empty tree
    tree = std::make_shared<OccupancyQuadtree>();
    EXPECT_TRUE(TreeSerializer::Read("circle.ot", tree));
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
            if (auto child = node->GetChild<OccupancyQuadtreeNode>(i); child != nullptr) {
                stack.emplace_back(child);
            }
        }
    }
    EXPECT_EQ(num_iterated_occupied_leaf_nodes, occupied_leaf_node_count);
}

TEST(OccupancyQuadtree, RayCasting) {
    // auto abstract_tree = AbstractQuadtree::Read("circle.ot");
    // EXPECT_TRUE(abstract_tree != nullptr);
    // auto tree = std::dynamic_pointer_cast<OccupancyQuadtree>(abstract_tree);
    // EXPECT_TRUE(tree != nullptr);
    auto tree = std::make_shared<OccupancyQuadtree>();
    EXPECT_TRUE(TreeSerializer::Read("circle.ot", tree));

    auto setting = std::make_shared<OccupancyQuadtreeDrawer::Setting>();
    setting->area_min << -4, -4;
    setting->area_max << 4, 4;
    setting->resolution = 0.01;
    setting->border_color = cv::Scalar(255, 0, 0);
    OccupancyQuadtreeDrawer drawer(setting, tree);
    drawer.DrawLeaves("read_circle.png");

    ERL_INFO("Casting rays in circle ...");
    unsigned int hit = 0;
    unsigned int miss = 0;
    unsigned int unknown = 0;
    Dtype mean_dist = 0;
    auto tree_setting = std::make_shared<OccupancyQuadtree::Setting>();
    tree_setting->resolution = 0.05;
    auto sampled_surface = std::make_shared<OccupancyQuadtree>(tree_setting);

    int n = 10000;
    for (int i = 0; i < n; ++i) {
        Dtype angle = static_cast<Dtype>(i) / 1000 * 2 * M_PI - M_PI;
        Dtype vx = std::cos(angle);
        Dtype vy = std::sin(angle);

        constexpr Dtype sx = 1.0, sy = 0.;
        constexpr bool ignore_unknown = false;
        constexpr Dtype max_range = 6;

        if (Dtype ex = 0, ey = 0;
            tree->CastRay(sx, sy, vx, vy, ignore_unknown, max_range, ex, ey)) {
            hit++;
            Dtype dx = ex - sx;
            Dtype dy = ey - sy;
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

    EXPECT_TRUE(TreeSerializer::Write("circle_sampled.bt", [&](std::ofstream &s) -> bool {
        return sampled_surface->WriteBinary(s, true);
    }));

    // EXPECT_TRUE(sampled_surface->WriteBinary("circle_sampled.bt"));
    EXPECT_EQ(hit, n);
    EXPECT_EQ(miss, 0);
    EXPECT_EQ(unknown, 0);
    mean_dist /= static_cast<Dtype>(hit);
    EXPECT_NEAR(mean_dist, 2.0, 0.01);
}
