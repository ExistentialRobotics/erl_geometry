#include <boost/heap/d_ary_heap.hpp>
// MUST INCLUDE BOOST HEADER BEFORE EIGEN HEADER WHEN USING LAPACK

#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <utility>

#include "erl_common/string_utils.hpp"
#include "erl_geometry/incremental_quadtree.hpp"
#include "erl_geometry/utils.hpp"

namespace erl::geometry {

    // std::shared_ptr<IncrementalQuadtree>
    // IncrementalQuadtree::GetRoot() {
    //     return (*m_root_ptr_)->GetSharedPtr();
    // }
    //
    // std::shared_ptr<IncrementalQuadtree>
    // IncrementalQuadtree::GetCluster() {
    //     if (m_cluster_ == nullptr) { return nullptr; }
    //     return m_cluster_->GetSharedPtr();
    // }
    //
    // std::shared_ptr<IncrementalQuadtree>
    // IncrementalQuadtree::GetParent() {
    //     if (m_parent_ == nullptr) { return nullptr; }
    //     return m_parent_->GetSharedPtr();
    // }

    std::shared_ptr<IncrementalQuadtree>
    IncrementalQuadtree::Insert(const std::shared_ptr<Node> &node, std::shared_ptr<IncrementalQuadtree> &new_root) {  // NOLINT(misc-no-recursion)
        if (node == nullptr) { throw std::invalid_argument("node is nullptr"); }
        new_root = GetRoot();
        // std::shared_ptr<IncrementalQuadtree> node_inserted = nullptr;

        // std::vector<std::shared_ptr<IncrementalQuadtree>> tree_stack;
        // tree_stack.push_back(shared_from_this());
        // std::shared_ptr<IncrementalQuadtree> root = IsRoot() ? shared_from_this() : GetRoot();
        std::shared_ptr<IncrementalQuadtree> tree = shared_from_this();
        std::vector<std::shared_ptr<IncrementalQuadtree>> split_trees;

        while (tree) {
            // auto tree = tree_stack.back();
            // tree_stack.pop_back();

            // node is out of the area
            if (!tree->m_area_.contains(node->position)) {
                // the node is out of the area of current tree, try its parent
                if (tree->IsRoot()) {  // this tree is the root, try to expand it
                    if (!tree->IsExpandable()) {
                        throw std::runtime_error(common::AsString(
                            "IncrementalQuadtree size ",
                            tree->m_area_.half_sizes[0],
                            " exceeds the maximum limit: ",
                            tree->m_setting_->max_half_area_size));
                    }
                    auto &np = node->position;
                    auto &cp = tree->m_area_.center;
                    // determine child type of this tree relative to the new root that will be created.
                    //  NW  |  NE
                    // -----------
                    //  SW  |  SE
                    auto my_child_type =
                        Children::Type((np.y() < cp.y() ? int(Children::Type::kNorthWest) : int(Children::Type::kSouthWest)) + (np.x() > cp.x() ? 0 : 1));
                    tree = tree->Expand(my_child_type);
                    new_root = tree;
                } else {  // this tree is not the root, try its parent
                    tree = tree->GetParent();
                }
                continue;
            }

            // node is in the area
            if (tree->IsLeaf()) {
                if (tree->IsInCluster()) {
                    bool too_close = false;
                    if (tree->m_node_container_->Insert(node, too_close)) { return tree; }  // insertion should happen here only
                    if (too_close) { return nullptr; }                                      // node is too close to other nodes in the cluster

                    // the cluster is full, try to split it
                    if (!tree->CreateChildren()) { return nullptr; }  // this cluster is already the smallest

                    // move nodes to the children
                    auto node_types = tree->m_node_container_->GetNodeTypes();
                    for (auto &node_type: node_types) {
                        auto begin = tree->m_node_container_->Begin(node_type);
                        auto end = tree->m_node_container_->End(node_type);
                        std::for_each(begin, end, [&](const std::shared_ptr<Node> &n) {  // NOLINT(misc-no-recursion)
                            std::any_of(
                                tree->m_children_.vector.begin(),
                                tree->m_children_.vector.end(),
                                [&n, &new_root](std::shared_ptr<IncrementalQuadtree> &child) -> bool {  // NOLINT(misc-no-recursion)
                                    // Don't call child->Insert if the child does not contain n's position. Otherwise, call-stack overflow will occur.
                                    // The current tree stack is not for this node, so we call child->Insert to start a new tree stack for this node.
                                    if (child->m_area_.contains(n->position)) { return child->Insert(n, new_root) != nullptr; }
                                    return false;
                                });
                        });
                    }
                    tree->m_node_container_->Clear();
                } else {
                    tree->CreateChildren();  // create children for the current tree
                    split_trees.push_back(tree);
                }
            }

            // Insert node to any descendent that accepts it
            if (!std::any_of(tree->m_children_.vector.begin(), tree->m_children_.vector.end(), [&](std::shared_ptr<IncrementalQuadtree> &child) -> bool {
                    if (child->m_area_.contains(node->position)) {
                        // tree_stack.push_back(child);
                        tree = child;
                        return true;
                    }
                    return false;
                })) {
                // may come here due to numerical error
                std::vector<std::shared_ptr<Node>> nodes;
                for (auto &split_tree: split_trees) {
                    nodes.clear();
                    split_tree->CollectNodes(nodes);
                    if (nodes.empty()) { split_tree->DeleteChildren(); }
                }
                // cluster_inserted = nullptr;
                return nullptr;
            }
        }

        // not accepted, return nullptr
        // cluster_inserted = nullptr;
        return nullptr;
    }

    bool
    IncrementalQuadtree::Remove(const std::shared_ptr<const Node> &node) {
        static bool warned = false;
        if (IsRoot()) {
            warned = true;
        } else if (!warned) {
            ERL_WARN("called by a non-root IncrementalQuadtree, there may be some leftover of empty sub-trees.");
            warned = true;
        }

        if (!m_area_.contains(node->position)) { return false; }

        if (!IsEmpty(node->type) && m_node_container_->Remove(node)) { return true; }

        if (IsLeaf()) { return false; }

        auto removed = std::any_of(m_children_.vector.begin(), m_children_.vector.end(), [&](std::shared_ptr<IncrementalQuadtree> &child) -> bool {
            return child->Remove(node);
        });

        if (removed && std::all_of(m_children_.vector.begin(), m_children_.vector.end(), [&](const std::shared_ptr<IncrementalQuadtree> &child) -> bool {
                return child->IsLeaf() && child->IsEmpty();
            })) {
            DeleteChildren();
        }

        if (IsRoot()) { warned = false; }

        return removed;
    }

    void
    IncrementalQuadtree::CollectTrees(
        const std::function<bool(const std::shared_ptr<const IncrementalQuadtree> &tree)> &qualify,
        std::vector<std::shared_ptr<const IncrementalQuadtree>> &trees) const {

        std::vector<std::shared_ptr<const IncrementalQuadtree>> tree_stack;
        tree_stack.reserve(100);
        tree_stack.push_back(shared_from_this());

        while (!tree_stack.empty()) {
            auto tree = tree_stack.back();
            tree_stack.pop_back();

            if (qualify(tree)) { trees.push_back(tree); }
            if (tree->IsLeaf()) { continue; }
            // keep the same order as the recursive version.
            for (auto child = tree->m_children_.vector.rbegin(); child != tree->m_children_.vector.rend(); ++child) { tree_stack.push_back(*child); }
            // for (auto &child: tree->m_children_.vector) { tree_stack.push_back(child); }
        }

        // auto ptr = shared_from_this();
        // if (qualify(ptr)) { trees.push_back(ptr); }
        // if (IsLeaf()) { return; }
        // for (auto &child: m_children_.vector) { child->CollectTrees(qualify, trees); }
    }

    void
    IncrementalQuadtree::CollectNonEmptyClusters(const Aabb2D &area, std::vector<std::shared_ptr<IncrementalQuadtree>> &clusters) {

        if (!m_area_.intersects(area)) { return; }  // this tree does not intersect with the area

        std::vector<std::shared_ptr<IncrementalQuadtree>> tree_stack;
        tree_stack.reserve(100);
        tree_stack.push_back(shared_from_this());

        while (!tree_stack.empty()) {
            auto tree = tree_stack.back();
            tree_stack.pop_back();

            if (tree->IsInCluster()) {  // top-level cluster, don't go deeper to sub-clusters
                // non-leaf cluster that must contain nodes or a non-empty cluster leaf
                if (!tree->IsLeaf() || !tree->IsEmpty()) { clusters.push_back(tree); }
            } else if (!tree->IsLeaf()) {  // go deeper to find the cluster!
                for (auto child = tree->m_children_.vector.rbegin(); child != tree->m_children_.vector.rend(); ++child) {
                    // keep the same order as the recursive version.
                    if ((*child)->m_area_.intersects(area)) { tree_stack.push_back(*child); }
                }
            }
        }
    }

    void
    IncrementalQuadtree::CollectNonEmptyClusters(
        const Aabb2D &area,
        std::vector<std::shared_ptr<IncrementalQuadtree>> &clusters,
        std::vector<double> &square_distances) {

        if (!m_area_.intersects(area)) { return; }  // this tree does not intersect with the area

        std::vector<std::shared_ptr<IncrementalQuadtree>> tree_stack;
        tree_stack.reserve(100);
        tree_stack.push_back(shared_from_this());

        while (!tree_stack.empty()) {
            auto tree = tree_stack.back();
            tree_stack.pop_back();

            if (tree->IsInCluster()) {  // top-level cluster, don't go deeper to sub-clusters
                // non-leaf cluster that must contain nodes or a non-empty cluster leaf
                if (!tree->IsLeaf() || !tree->IsEmpty()) {
                    square_distances.push_back((tree->m_area_.center - area.center).squaredNorm());
                    clusters.push_back(tree);
                }
            } else if (!tree->IsLeaf()) {  // go deeper to find the cluster!
                // keep the same order as the recursive version.
                for (auto child = tree->m_children_.vector.rbegin(); child != tree->m_children_.vector.rend(); ++child) {
                    if ((*child)->m_area_.intersects(area)) { tree_stack.push_back(*child); }
                }
            }
        }
    }

    void
    IncrementalQuadtree::CollectClustersWithData(
        const Aabb2D &area,
        std::vector<std::shared_ptr<IncrementalQuadtree>> &clusters,
        std::vector<double> &square_distances) {

        if (!m_area_.intersects(area)) { return; }  // this tree does not intersect with the area

        std::vector<std::shared_ptr<IncrementalQuadtree>> tree_stack;
        tree_stack.reserve(100);
        tree_stack.push_back(shared_from_this());

        while (!tree_stack.empty()) {
            auto tree = tree_stack.back();
            tree_stack.pop_back();

            if (tree->IsInCluster()) {  // top-level cluster, don't go deeper to sub-clusters
                // non-leaf cluster that must contain nodes or a non-empty cluster leaf
                if (tree->m_data_ptr_ != nullptr) {
                    square_distances.push_back((tree->m_area_.center - area.center).squaredNorm());
                    clusters.push_back(tree);
                }
            } else if (!tree->IsLeaf()) {  // go deeper to find the cluster!
                // keep the same order as the recursive version.
                for (auto child = tree->m_children_.vector.rbegin(); child != tree->m_children_.vector.rend(); ++child) {
                    if ((*child)->m_area_.intersects(area)) { tree_stack.push_back(*child); }
                }
            }
        }
    }

    void
    IncrementalQuadtree::CollectNodes(std::vector<std::shared_ptr<Node>> &nodes) const {

        std::vector<std::shared_ptr<const IncrementalQuadtree>> tree_stack;
        tree_stack.reserve(100);
        tree_stack.push_back(shared_from_this());

        while (!tree_stack.empty()) {
            auto tree = tree_stack.back();
            tree_stack.pop_back();

            if (!tree->IsEmpty()) { tree->m_node_container_->CollectNodes(nodes); }
            if (tree->IsLeaf()) { continue; }
            // keep the same order as the recursive version.
            for (auto child = tree->m_children_.vector.rbegin(); child != tree->m_children_.vector.rend(); ++child) { tree_stack.push_back(*child); }
        }
    }

    void
    IncrementalQuadtree::RayTracing(
        const Eigen::Ref<const Eigen::Vector2d> &ray_origin,
        const Eigen::Ref<const Eigen::Vector2d> &ray_direction,
        double hit_distance_threshold,
        double &ray_travel_distance,
        std::shared_ptr<Node> &hit_node) const {

        Eigen::Vector2d r_inv = ray_direction.cwiseInverse();
        auto self = shared_from_this();
        double dist, dist2;
        bool intersected;
        ComputeIntersectionBetweenRayAndAabb2D(ray_origin, r_inv, self->m_area_.min(), self->m_area_.max(), dist, dist2, intersected);
        if (!intersected) { return; }  // the ray does not intersect with the tree's area, won't check any descendants.

        struct Compare {
            bool
            operator()(
                const std::pair<double, std::shared_ptr<const IncrementalQuadtree>> &a,
                const std::pair<double, std::shared_ptr<const IncrementalQuadtree>> &b) const {
                return a.first > b.first;
            }
        };

        std::priority_queue<
            std::pair<double, std::shared_ptr<const IncrementalQuadtree>>,
            std::vector<std::pair<double, std::shared_ptr<const IncrementalQuadtree>>>,
            Compare>
            tree_queue;  // the top element has the smallest distance to make sure the closest tree is checked first.
        tree_queue.emplace(dist, self);
        ray_travel_distance = std::numeric_limits<double>::infinity();
        hit_node = nullptr;

        while (!tree_queue.empty()) {
            auto tree = tree_queue.top().second;
            tree_queue.pop();

            // the ray intersects with the tree's area
            if (tree->IsLeaf()) {
                if (!tree->IsEmpty()) {
                    auto container_nodes = tree->m_node_container_->CollectNodes();  // collect all nodes in the container
                    for (auto &node: container_nodes) {
                        double dx = node->position[0] - ray_origin[0];
                        double dy = node->position[1] - ray_origin[1];
                        // bool in_front = (dx * ray_direction[0] + dy * ray_direction[1]) > 0;
                        dist = dx * ray_direction[0] + dy * ray_direction[1];
                        bool in_front = (dist > 0);
                        double vertical_dist = std::abs(dx * ray_direction[1] - dy * ray_direction[0]);
                        if (in_front && vertical_dist < hit_distance_threshold) {
                            // dist = std::sqrt(dx * dx + dy * dy);
                            if (dist < ray_travel_distance) {
                                ray_travel_distance = dist;
                                hit_node = node;
                            }
                        }  // the node is hit
                    }
                    if (hit_node != nullptr) { return; }  // return if any node is hit, no need to check the rest of the trees
                }
            } else {
                // keep the same order as the recursive version.
                for (auto child = tree->m_children_.vector.rbegin(); child != tree->m_children_.vector.rend(); ++child) {
                    ComputeIntersectionBetweenRayAndAabb2D(ray_origin, r_inv, (*child)->m_area_.min(), (*child)->m_area_.max(), dist, dist2, intersected);
                    if (intersected && dist >= 0) {
                        tree_queue.emplace(dist, *child);
                        continue;
                    }
                    // not intersected but this child is in front of the ray and very close to the ray
                    auto &child_area_center = (*child)->m_area_.center;
                    double &half_size = (*child)->m_area_.half_sizes[0];
                    double dx = child_area_center[0] - ray_origin[0];
                    double dy = child_area_center[1] - ray_origin[1];
                    bool in_front = (dx * ray_direction[0] + dy * ray_direction[1]) > 0;
                    double vertical_dist = std::abs(dx * ray_direction[1] - dy * ray_direction[0]);
                    if (in_front && vertical_dist < hit_distance_threshold + half_size) { tree_queue.emplace(std::sqrt(dx * dx + dy * dy), *child); }
                }
            }
        }
    }

    void
    IncrementalQuadtree::RayTracing(
        const Eigen::Ref<const Eigen::Matrix2Xd> &ray_origins,
        const Eigen::Ref<const Eigen::Matrix2Xd> &ray_directions,
        double hit_distance_threshold,
        int num_threads,
        std::vector<double> &ray_travel_distances,
        std::vector<std::shared_ptr<Node>> &hit_nodes) const {

        ERL_DEBUG_ASSERT(ray_origins.cols() == ray_directions.cols(), "The number of ray origins and ray directions must be the same.");
        ray_travel_distances.resize(ray_origins.cols());
        hit_nodes.resize(ray_origins.cols());

        long num_rays = ray_origins.cols();

#pragma omp parallel for default(none) shared(num_rays, ray_origins, ray_directions, hit_distance_threshold, ray_travel_distances, hit_nodes) \
    num_threads(num_threads)
        for (int i = 0; i < num_rays; ++i) {
            RayTracing(ray_origins.col(i), ray_directions.col(i), hit_distance_threshold, ray_travel_distances[i], hit_nodes[i]);
        }
    }

    void
    IncrementalQuadtree::RayTracing(
        int node_type,
        const Eigen::Ref<const Eigen::Vector2d> &ray_origin,
        const Eigen::Ref<const Eigen::Vector2d> &ray_direction,
        double hit_distance_threshold,
        double &ray_travel_distance,
        std::shared_ptr<Node> &hit_node) const {

        Eigen::Vector2d r_inv = ray_direction.cwiseInverse();
        auto self = shared_from_this();
        double dist, dist2;
        bool intersected;
        ComputeIntersectionBetweenRayAndAabb2D(ray_origin, r_inv, self->m_area_.min(), self->m_area_.max(), dist, dist2, intersected);
        if (!intersected) { return; }  // the ray does not intersect with the tree's area, won't check any descendants.

        struct Compare {
            bool
            operator()(
                const std::pair<double, std::shared_ptr<const IncrementalQuadtree>> &a,
                const std::pair<double, std::shared_ptr<const IncrementalQuadtree>> &b) const {
                return a.first > b.first;
            }
        };

        std::priority_queue<
            std::pair<double, std::shared_ptr<const IncrementalQuadtree>>,
            std::vector<std::pair<double, std::shared_ptr<const IncrementalQuadtree>>>,
            Compare>
            tree_queue;  // the top element has the smallest distance to make sure the closest tree is checked first.
        tree_queue.emplace(dist, self);
        ray_travel_distance = std::numeric_limits<double>::infinity();
        hit_node = nullptr;

        while (!tree_queue.empty()) {
            auto tree = tree_queue.top().second;
            tree_queue.pop();

            // the ray intersects with the tree's area
            if (tree->IsLeaf()) {
                if (!tree->IsEmpty()) {
                    auto container_nodes = tree->m_node_container_->CollectNodesOfType(node_type);  // collect all nodes in the container
                    for (auto &node: container_nodes) {
                        double dx = node->position[0] - ray_origin[0];
                        double dy = node->position[1] - ray_origin[1];
                        // bool in_front = (dx * ray_direction[0] + dy * ray_direction[1]) > 0;
                        dist = dx * ray_direction[0] + dy * ray_direction[1];
                        bool in_front = dist > 0;
                        double vertical_dist = std::abs(dx * ray_direction[1] - dy * ray_direction[0]);
                        if (in_front && vertical_dist < hit_distance_threshold) {
                            // dist = std::sqrt(dx * dx + dy * dy);
                            if (dist < ray_travel_distance) {
                                ray_travel_distance = dist;
                                hit_node = node;
                            }
                        }  // the node is hit
                    }
                    if (hit_node != nullptr) { return; }  // return if any node is hit, no need to check the rest of the trees
                }
            } else {
                // keep the same order as the recursive version.
                for (auto child = tree->m_children_.vector.rbegin(); child != tree->m_children_.vector.rend(); ++child) {
                    ComputeIntersectionBetweenRayAndAabb2D(ray_origin, r_inv, (*child)->m_area_.min(), (*child)->m_area_.max(), dist, dist2, intersected);
                    if (intersected && dist >= 0) {
                        tree_queue.emplace(dist, *child);
                        continue;
                    }
                    // not intersected but this child is in front of the ray and very close to the ray
                    auto &child_area_center = (*child)->m_area_.center;
                    double &half_size = (*child)->m_area_.half_sizes[0];
                    double dx = child_area_center[0] - ray_origin[0];
                    double dy = child_area_center[1] - ray_origin[1];
                    bool in_front = (dx * ray_direction[0] + dy * ray_direction[1]) > 0;
                    double vertical_dist = std::abs(dx * ray_direction[1] - dy * ray_direction[0]);
                    if (in_front && vertical_dist <= hit_distance_threshold + half_size) { tree_queue.emplace(std::sqrt(dx * dx + dy * dy), *child); }
                }
            }
        }
    }

    void
    IncrementalQuadtree::RayTracing(
        int node_type,
        const Eigen::Ref<const Eigen::Matrix2Xd> &ray_origins,
        const Eigen::Ref<const Eigen::Matrix2Xd> &ray_directions,
        double hit_distance_threshold,
        int num_threads,
        std::vector<double> &ray_travel_distances,
        std::vector<std::shared_ptr<Node>> &hit_nodes) const {

        ERL_DEBUG_ASSERT(ray_origins.cols() == ray_directions.cols(), "The number of ray origins and ray directions must be the same.");
        ray_travel_distances.resize(ray_origins.cols());
        hit_nodes.resize(ray_origins.cols());

        long num_rays = ray_origins.cols();

#pragma omp parallel for default(none) shared(num_rays, node_type, ray_origins, ray_directions, hit_distance_threshold, ray_travel_distances, hit_nodes) \
    num_threads(num_threads)
        for (int i = 0; i < num_rays; ++i) {
            RayTracing(node_type, ray_origins.col(i), ray_directions.col(i), hit_distance_threshold, ray_travel_distances[i], hit_nodes[i]);
        }
    }

    cv::Mat
    IncrementalQuadtree::Plot(
        const std::shared_ptr<const common::GridMapInfo<2>> &grid_map_info,
        const std::vector<int> &node_types,
        std::unordered_map<int, cv::Scalar> node_type_colors,
        std::unordered_map<int, int> node_type_radius,
        const cv::Scalar &bg_color,
        const cv::Scalar &area_rect_color,
        int area_rect_thickness,
        const cv::Scalar &tree_data_color,
        int tree_data_radius,
        const std::function<void(cv::Mat &, std::vector<std::shared_ptr<Node>> &)> &plot_node_data) {

        // a blank image with white background
        cv::Mat image(grid_map_info->Height(), grid_map_info->Width(), CV_8UC3, bg_color);
        // check if the image is created successfully or not
        if (!image.data) { throw std::runtime_error("Failed to allocate image buffer."); }

        std::vector<std::shared_ptr<IncrementalQuadtree>> tree_stack;
        tree_stack.reserve(100);
        tree_stack.push_back(shared_from_this());
        std::vector<cv::Point2i> tree_data_positions;

        std::vector<std::shared_ptr<Node>> node_data;
        // std::unordered_map<int, std::vector<cv::Point2i>> node_type_pixels;
        // for (auto &kType: node_types) { node_type_pixels[kType] = std::vector<cv::Point2i>(); }

        while (!tree_stack.empty()) {
            auto tree = tree_stack.back();
            tree_stack.pop_back();

            Eigen::Vector2i bottom_left =
                grid_map_info->MeterToPixelForPoints(tree->m_area_.corner(Eigen::AlignedBox2d::BottomLeft)).array().round().cast<int>();
            Eigen::Vector2i top_right = grid_map_info->MeterToPixelForPoints(tree->m_area_.corner(Eigen::AlignedBox2d::TopRight)).array().round().cast<int>();
            cv::rectangle(
                image,
                {bottom_left.x(), bottom_left.y()},
                {top_right.x(), top_right.y()},
                area_rect_color,
                area_rect_thickness,
                cv::LineTypes::LINE_8);

            if (tree->m_node_container_ != nullptr) {
                for (auto type: node_types) { tree->m_node_container_->CollectNodesOfType(type, node_data); }
            }

            if (!tree->IsLeaf()) {
                for (auto &child: tree->m_children_.vector) { tree_stack.push_back(child); }
            }

            if (tree->m_data_ptr_ != nullptr) {
                Eigen::Vector2i pixel = grid_map_info->MeterToPixelForPoints(tree->m_area_.center).array().round().cast<int>();
                tree_data_positions.emplace_back(pixel.x(), pixel.y());
            }
        }

        // for (auto &[node_type, node_pixels]: node_type_pixels) {
        //     for (auto &pixel: node_pixels) { cv::circle(image, pixel, node_type_radius[node_type], node_type_colors[node_type], cv::FILLED); }
        // }
        for (auto &node: node_data) {
            Eigen::Vector2i pixel = grid_map_info->MeterToPixelForPoints(node->position);
            cv::circle(image, cv::Point2i(pixel.x(), pixel.y()), node_type_radius[node->type], node_type_colors[node->type], cv::FILLED);
        }
        if (plot_node_data != nullptr) { plot_node_data(image, node_data); }
        for (auto &p: tree_data_positions) { cv::circle(image, p, tree_data_radius, tree_data_color, cv::FILLED); }

        return image;
    }

    void
    IncrementalQuadtree::Print(std::ostream &os) const {
        static std::vector<std::string> indents;
        static auto print_indent = [&]() -> std::string {
            std::stringstream ss;
            for (auto &indent: indents) { ss << indent; }
            return ss.str();
        };
        std::vector<std::shared_ptr<const IncrementalQuadtree>> tree_stack;
        tree_stack.reserve(100);
        std::vector<int> level_stack;
        level_stack.reserve(100);

        int level = 0;
        indents.clear();
        tree_stack.push_back(shared_from_this());
        level_stack.push_back(level);
        std::vector<std::shared_ptr<Node>> nodes;

        while (!tree_stack.empty()) {
            auto tree = tree_stack.back();
            tree_stack.pop_back();
            level = level_stack.back();
            level_stack.pop_back();

            auto &kArea = tree->GetArea();
            auto &c = kArea.center;

            nodes.clear();
            tree->CollectNodes(nodes);

            os << print_indent() << Children::GetTypeName(tree->m_child_type_) << ": center = [" << c.x() << ", " << c.y()
               << "], half size = " << kArea.half_sizes[0] << ", is leaf: " << tree->IsLeaf() << ", has SurfaceData: " << (tree->m_data_ptr_ != nullptr)
               << ", number of nodes = " << nodes.size() << std::endl;

            // print nodes
            if (tree->m_node_container_ != nullptr) {
                indents.pop_back();
                if (tree->m_child_type_ == Children::Type::kSouthEast) {
                    indents.emplace_back("    ");
                } else {
                    indents.emplace_back("│   ");
                }

                auto node_types = tree->m_node_container_->GetNodeTypes();
                for (auto &node_type: node_types) {
                    auto begin = tree->m_node_container_->Begin(node_type);
                    auto end = tree->m_node_container_->End(node_type);
                    for (auto itr = begin; itr < end; itr++) {
                        auto &node = *itr;
                        auto np = node->position;
                        os << print_indent() << (itr != end - 1 ? "├──* " : "└──* ") << tree->m_node_container_->GetNodeTypeName(node->type) << " Node"
                           << std::setw(2) << std::distance(begin, itr) << ": position = [" << np.x() << ", " << np.y() << ']' << std::endl;
                        if (node->node_data != nullptr) { os << print_indent() << "    └──* " << *(node->node_data) << std::endl; }
                    }
                }
                indents.pop_back();
                indents.emplace_back("├── ");
            }

            // add child m_nodes_
            if (!tree->IsLeaf()) {
                level++;
                for (int i = 3; i >= 0; --i) {
                    tree_stack.push_back(tree->m_children_.vector[i]);
                    level_stack.push_back(level);
                }

                if (!indents.empty()) { indents.pop_back(); }
                if (tree->m_child_type_ == Children::Type::kSouthEast) {
                    indents.emplace_back("    ");
                } else if (level > 1) {
                    indents.emplace_back("│   ");
                }
                indents.emplace_back("├── ");
            }

            if (!level_stack.empty() && level > level_stack.back()) {
                auto n = level - level_stack.back();
                for (int i = 0; i <= n; ++i) { indents.pop_back(); }
                indents.emplace_back("├── ");
            }

            if ((!tree_stack.empty()) && (tree_stack.back()->m_child_type_ == Children::Type::kSouthEast)) {
                indents.pop_back();
                indents.emplace_back("└── ");
            }
        }
    }

    IncrementalQuadtree::IncrementalQuadtree(
        std::shared_ptr<Setting> setting,
        Aabb2D area,
        const std::function<std::shared_ptr<NodeContainer>()> &node_container_constructor,
        const std::shared_ptr<IncrementalQuadtree *> &root)
        : m_setting_(std::move(setting)),
          m_area_(std::move(area)),
          m_node_container_constructor_(node_container_constructor) {

        ERL_ASSERTM(m_setting_->min_half_area_size > 0, "min_half_area_size should be larger than 0.");
        ERL_ASSERTM(m_setting_->cluster_half_area_size > 0, "cluster_half_area_size should be larger than 0.");
        ERL_ASSERTM(m_setting_->min_half_area_size < m_setting_->cluster_half_area_size, "min_half_area_size should be smaller than cluster_half_area_size.");

        if (root == nullptr) {  // root node
            m_root_ptr_ = std::make_shared<IncrementalQuadtree *>(this);
        } else {
            m_root_ptr_ = root;
        }
        if (IsCluster()) { m_cluster_ = this; }
        if (IsInCluster()) { m_node_container_ = m_node_container_constructor_(); }
    }

    /**
     * Expand the tree from the current node (must be the root), such that the area covered by the whole tree is four times of the old one.
     * @param current_root_child_type
     * @note this method can only be called by the root node safely, may return nullptr if the tree already reaches the maximum GetSize.
     */
    std::shared_ptr<IncrementalQuadtree>
    IncrementalQuadtree::Expand(Children::Type current_root_child_type) {
        ERL_DEBUG("Expand the tree.");
        ERL_DEBUG_ASSERT(IsRoot(), "*this must be the root node of the tree.");
        ERL_DEBUG_ASSERT(
            current_root_child_type == Children::Type::kNorthWest || current_root_child_type == Children::Type::kNorthEast ||
                current_root_child_type == Children::Type::kSouthWest || current_root_child_type == Children::Type::kSouthEast,
            "current_root_child_type must be one of kNorthWest, kNorthEast, kSouthWest, kSouthEast. Not {}.\n",
            Children::GetTypeName(current_root_child_type));

        if (!IsExpandable()) { return nullptr; }

        auto &c = m_area_.center;
        auto l = m_area_.half_sizes[0];
        // NW=0 : c.x + l, c.y - l
        // NE=1 : c.x - l, c.y - l
        // SW=2 : c.x + l, c.y + l
        // SE=3 : c.x - l, c.y + l
        double &&x = int(current_root_child_type) % 2 ? c.x() - l : c.x() + l;
        double &&y = int(current_root_child_type) < 2 ? c.y() - l : c.y() + l;
        std::shared_ptr<IncrementalQuadtree> new_root =
            std::shared_ptr<IncrementalQuadtree>(new IncrementalQuadtree(m_setting_, Aabb2D({x, y}, l * 2.), m_node_container_constructor_, m_root_ptr_));
        *m_root_ptr_ = new_root.get();
        m_parent_ = new_root.get();
        new_root->CreateChildren();
        new_root->ReplaceChild(shared_from_this(), current_root_child_type);
        m_child_type_ = current_root_child_type;
        return new_root;
    }

    void
    IncrementalQuadtree::ReplaceChild(const std::shared_ptr<IncrementalQuadtree> &child, Children::Type child_type) {
        ERL_DEBUG_ASSERT(child != nullptr, "child should not be nullptr.");
        ERL_DEBUG_ASSERT(
            child_type == Children::Type::kNorthWest || child_type == Children::Type::kNorthEast || child_type == Children::Type::kSouthWest ||
                child_type == Children::Type::kSouthEast,
            "child_type must be one of kNorthWest, kNorthEast, kSouthWest, kSouthEast. Not {}.\n",
            Children::GetTypeName(child_type));
        ERL_DEBUG_ASSERT(m_children_[static_cast<int>(child_type)] != nullptr, "No existing child to replace.");
        ERL_DEBUG_ASSERT(
            std::fabs(child->m_area_.half_sizes[0] * 2 - m_area_.half_sizes[0]) < 1e-6,
            "incompatible child half length: {:f} for this parent of half length: {:f}.\n",
            child->m_area_.half_sizes[0],
            m_area_.half_sizes[0]);
        m_children_[static_cast<int>(child_type)] = child;
    }

    bool
    IncrementalQuadtree::CreateChildren() {

        if (!IsLeaf()) { return true; }  // already created

        if (IsSubdividable()) {
            // auto l = m_area_.half_sizes[0] / 2.0 + 1.e-20;
            auto l = m_area_.half_sizes[0] / 2.0;
            auto &c = m_area_.center;
            m_children_.NorthWest() =
                Create(m_setting_, Aabb2D({c.x() - l, c.y() + l}, l), m_node_container_constructor_, m_root_ptr_, this, Children::Type::kNorthWest);
            m_children_.NorthEast() =
                Create(m_setting_, Aabb2D({c.x() + l, c.y() + l}, l), m_node_container_constructor_, m_root_ptr_, this, Children::Type::kNorthEast);
            m_children_.SouthWest() =
                Create(m_setting_, Aabb2D({c.x() - l, c.y() - l}, l), m_node_container_constructor_, m_root_ptr_, this, Children::Type::kSouthWest);
            m_children_.SouthEast() =
                Create(m_setting_, Aabb2D({c.x() + l, c.y() - l}, l), m_node_container_constructor_, m_root_ptr_, this, Children::Type::kSouthEast);
            m_is_leaf_ = false;

            if (m_cluster_ != nullptr) {
                m_children_.NorthWest()->m_cluster_ = m_cluster_;
                m_children_.NorthEast()->m_cluster_ = m_cluster_;
                m_children_.SouthWest()->m_cluster_ = m_cluster_;
                m_children_.SouthEast()->m_cluster_ = m_cluster_;
            }

            return true;
        }

        return false;  // cannot divide this tree further
    }

    void
    IncrementalQuadtree::DeleteChildren() {
        m_children_.Reset();
        m_is_leaf_ = true;
    }
}  // namespace erl::geometry
