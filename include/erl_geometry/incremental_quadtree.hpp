#pragma once

#include "aabb.hpp"
#include "node.hpp"
#include "node_container.hpp"

#include "erl_common/grid_map_info.hpp"
#include "erl_common/yaml.hpp"

#include <functional>
#include <unordered_map>
#include <utility>

#include <opencv2/core.hpp>

namespace erl::geometry {

    class IncrementalQuadtree : public std::enable_shared_from_this<IncrementalQuadtree> {  // NOTE: MUST BE PUBLIC INHERITANCE

    public:
        // structure for storing sub-IncrementalQuadtree
        struct Children {
            std::vector<std::shared_ptr<IncrementalQuadtree>> vector{4, nullptr};

            enum class Type { kNorthWest = 0, kNorthEast = 1, kSouthWest = 2, kSouthEast = 3, kRoot = 4 };

            static const char *
            GetTypeName(const Type &type) {
                static const char *names[] = {"kNorthWest", "kNorthEast", "kSouthWest", "kSouthEast", "kRoot"};
                return names[static_cast<int>(type)];
            }

            Children() = default;

            void
            Reset() {
                vector.resize(4, nullptr);
            }

            std::shared_ptr<IncrementalQuadtree> &
            operator[](const int i) {
                return vector[i];
            }

            std::shared_ptr<IncrementalQuadtree> &
            NorthWest() {
                return vector[0];
            }

            [[nodiscard]] std::shared_ptr<IncrementalQuadtree>
            NorthWest() const {
                return vector[0];
            }

            std::shared_ptr<IncrementalQuadtree> &
            NorthEast() {
                return vector[1];
            }

            [[nodiscard]] std::shared_ptr<IncrementalQuadtree>
            NorthEast() const {
                return vector[1];
            }

            std::shared_ptr<IncrementalQuadtree> &
            SouthWest() {
                return vector[2];
            }

            [[nodiscard]] std::shared_ptr<IncrementalQuadtree>
            SouthWest() const {
                return vector[2];
            }

            std::shared_ptr<IncrementalQuadtree> &
            SouthEast() {
                return vector[3];
            }

            [[nodiscard]] std::shared_ptr<IncrementalQuadtree>
            SouthEast() const {
                return vector[3];
            }

            ~Children() { Reset(); }
        };

        // structure for holding the parameters
        struct Setting : public common::Yamlable<Setting> {

            /**
             * The top-most IncrementalQuadtree that is no greater than this GetSize is called cluster. This IncrementalQuadtree and its descendents (sub
             * clusters) stores inserted m_nodes_. IncrementalQuadtree larger than this GetSize will be subdivided before the node inserted to its descendent.
             * This affects the spacial resolution and the searching performance. If this GetSize is too large, it takes more time to find nearest neighbors
             * because too many m_nodes_ are collected to do distance comparison. If this GetSize if too small, the tree volume will be too small. Note that
             * clusterHalfAreaSize must be larger than minHalfAreaSize if they are both positive. If non-positive, this will be ignored.
             */
            double cluster_half_area_size = 0.8;
            // If the whole tree is larger than this, it will not be allowed to expand. If non-positive, this will be ignored.
            double max_half_area_size = 150;
            // IncrementalQuadtree whose area is smaller than this cannot be further subdivided. If non-positive, this will be ignored.
            double min_half_area_size = 0.2;
        };

    private:
        std::shared_ptr<Setting> m_setting_;
        Aabb2D m_area_;
        Children::Type m_child_type_ = Children::Type::kRoot;
        std::shared_ptr<NodeContainer> m_node_container_;
        std::function<std::shared_ptr<NodeContainer>()> m_node_container_constructor_;
        bool m_is_leaf_ = true;
        std::shared_ptr<void> m_data_ptr_ = nullptr;  // attached data
        // here we use raw pointer for root, cluster and parent to avoid circular reference
        // the memory that the raw pointer points to is managed by shared_ptr
        std::shared_ptr<IncrementalQuadtree *> m_root_ptr_ = nullptr;
        IncrementalQuadtree *m_cluster_ = nullptr;
        IncrementalQuadtree *m_parent_ = nullptr;

    public:
        Children m_children_;

        /**
         * @brief Construct a new IncrementalQuadtree object
         * @param setting
         * @param area
         * @param node_container_constructor
         * @return
         */
        [[nodiscard]] static std::shared_ptr<IncrementalQuadtree>
        Create(std::shared_ptr<Setting> setting, const Aabb2D &area, const std::function<std::shared_ptr<NodeContainer>()> &node_container_constructor) {
            auto tree = std::shared_ptr<IncrementalQuadtree>(new IncrementalQuadtree(std::move(setting), area, node_container_constructor, nullptr));
            return tree;
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] std::shared_ptr<IncrementalQuadtree>
        GetRoot() const {
            return (*m_root_ptr_)->shared_from_this();
        }

        [[nodiscard]] std::shared_ptr<IncrementalQuadtree>
        GetCluster() const {
            if (m_cluster_ == nullptr) { return nullptr; }
            return m_cluster_->shared_from_this();
        }

        std::shared_ptr<IncrementalQuadtree>
        GetParent() const {
            if (m_parent_ == nullptr) { return nullptr; }
            return m_parent_->shared_from_this();
        }

        [[nodiscard]] const Aabb2D &
        GetArea() const {
            return m_area_;
        }

        [[nodiscard]] Children::Type
        GetChildType() const {
            return m_child_type_;
        }

        [[nodiscard]] const Children &
        GetChildren() const {
            return m_children_;
        }

        template<typename T>
        std::shared_ptr<T>
        GetData() const {
            return std::static_pointer_cast<T>(m_data_ptr_);
        }

        void
        SetData(std::shared_ptr<void> ptr) {
            m_data_ptr_ = std::move(ptr);
        }

        [[nodiscard]] bool
        IsRoot() const {
            return m_parent_ == nullptr;
        }

        [[nodiscard]] bool
        IsLeaf() const {
            return m_is_leaf_;
        }

        [[nodiscard]] bool
        IsEmpty(const int type) const {
            return (m_node_container_ == nullptr) || (m_node_container_->Empty(type));
        }

        [[nodiscard]] bool
        IsEmpty() const {
            return (m_node_container_ == nullptr) || (m_node_container_->Empty());
        }

        [[nodiscard]] bool
        IsCluster() const {
            const double &kL = m_area_.half_sizes[0];
            return (kL <= m_setting_->cluster_half_area_size) && (kL * 2 >= m_setting_->cluster_half_area_size);
        }

        [[nodiscard]] bool
        IsInCluster() const {
            // m_area_ is a square.
            // return (m_area_.half_sizes[0] <= m_setting_->cluster_half_area_size) || (m_setting_->cluster_half_area_size < 0);
            // return m_cluster_ != nullptr;
            return m_area_.half_sizes[0] <= m_setting_->cluster_half_area_size;
        }

        [[nodiscard]] bool
        IsExpandable() const {
            return (m_area_.half_sizes[0] <= m_setting_->max_half_area_size) || (m_setting_->max_half_area_size < 0);
        }

        [[nodiscard]] bool
        IsSubdividable() const {
            return (m_area_.half_sizes[0] >= m_setting_->min_half_area_size) || (m_setting_->min_half_area_size < 0);
        }

        /**
         * @param node
         * @param new_root
         * @return If node is inserted, a IncrementalQuadtree of top level in the call stack will be returned. Otherwise, nullptr. So, if you call this method
         * from the root, the (new) root IncrementalQuadtree will be returned.
         */
        std::shared_ptr<IncrementalQuadtree>
        Insert(const std::shared_ptr<Node> &node, std::shared_ptr<IncrementalQuadtree> &new_root);

        bool
        Remove(const std::shared_ptr<const Node> &node);

        void
        CollectTrees(
            const std::function<bool(const std::shared_ptr<const IncrementalQuadtree> &tree)> &qualify,
            std::vector<std::shared_ptr<const IncrementalQuadtree>> &trees) const;

        void
        CollectNonEmptyClusters(const Aabb2D &area, std::vector<std::shared_ptr<IncrementalQuadtree>> &clusters);

        void
        CollectNonEmptyClusters(const Aabb2D &area, std::vector<std::shared_ptr<IncrementalQuadtree>> &clusters, std::vector<double> &square_distances);

        void
        CollectClustersWithData(const Aabb2D &area, std::vector<std::shared_ptr<IncrementalQuadtree>> &clusters, std::vector<double> &square_distances);

        void
        CollectNodes(std::vector<std::shared_ptr<Node>> &nodes) const;

        void
        CollectNodesOfType(const int type, std::vector<std::shared_ptr<Node>> &nodes) const {
            std::vector<std::shared_ptr<const IncrementalQuadtree>> tree_stack;
            tree_stack.reserve(100);
            tree_stack.push_back(shared_from_this());

            while (!tree_stack.empty()) {
                const auto tree = tree_stack.back();
                tree_stack.pop_back();

                if (!tree->IsEmpty(type)) { tree->m_node_container_->CollectNodesOfType(type, nodes); }
                if (!tree->IsLeaf()) {
                    // keep the same order as the recursive version.
                    for (auto child = tree->m_children_.vector.rbegin(); child != tree->m_children_.vector.rend(); ++child) { tree_stack.push_back(*child); }
                }
            }
        }

        void
        CollectNodesOfTypeInArea(const int type, const Aabb2D &area, std::vector<std::shared_ptr<Node>> &nodes) const {
            // This method should not be recursive as the overhead of function call is too large.
            std::vector<std::shared_ptr<const IncrementalQuadtree>> tree_stack;
            tree_stack.reserve(100);
            tree_stack.push_back(shared_from_this());

            while (!tree_stack.empty()) {
                const auto tree = tree_stack.back();
                tree_stack.pop_back();

                if (!tree->m_area_.intersects(area)) { continue; }
                if (!tree->IsEmpty(type)) { tree->m_node_container_->CollectNodesOfTypeInAabb2D(type, area, nodes); }
                if (!tree->IsLeaf()) {
                    // keep the same order as the recursive version.
                    for (auto child = tree->m_children_.vector.rbegin(); child != tree->m_children_.vector.rend(); ++child) { tree_stack.push_back(*child); }
                }
            }
        }

        /**
         * find the nearest node hit by the ray.
         *
         * @param ray_origin              ray origin
         * @param ray_direction           ray direction
         * @param hit_distance_threshold  if the distance between the ray origin and the node is larger than this threshold, the node will not be hit.
         * @param ray_travel_distance     output of the average distance between the ray origin and the nodes hit by the ray.
         * @param hit_node                output of the node hit by the ray.
         * @refitem http://webhome.cs.uvic.ca/~blob/courses/305/notes/pdf/Ray%20Tracing%20with%20Spatial%20Hierarchies.pdf
         */
        void
        RayTracing(
            const Eigen::Ref<const Eigen::Vector2d> &ray_origin,
            const Eigen::Ref<const Eigen::Vector2d> &ray_direction,
            double hit_distance_threshold,
            double &ray_travel_distance,
            std::shared_ptr<Node> &hit_node) const;

        void
        RayTracing(
            const Eigen::Ref<const Eigen::Matrix2Xd> &ray_origins,
            const Eigen::Ref<const Eigen::Matrix2Xd> &ray_directions,
            double hit_distance_threshold,
            int num_threads,
            std::vector<double> &ray_travel_distances,
            std::vector<std::shared_ptr<Node>> &hit_nodes) const;

        /**
         * collect nodes of specific type in the first leave that is hit by the ray and has nodes of the specific type.
         *
         * @param node_type                    node type
         * @param ray_origin              ray origin
         * @param ray_direction           ray direction
         * @param hit_distance_threshold  if the distance between the ray origin and the node is larger than this threshold, the node will not be hit.
         * @param ray_travel_distance     output of the average distance between the ray origin and the nodes hit by the ray.
         * @param hit_node                output of the node hit by the ray.
         */
        void
        RayTracing(
            int node_type,
            const Eigen::Ref<const Eigen::Vector2d> &ray_origin,
            const Eigen::Ref<const Eigen::Vector2d> &ray_direction,
            double hit_distance_threshold,
            double &ray_travel_distance,
            std::shared_ptr<Node> &hit_node) const;

        void
        RayTracing(
            int node_type,
            const Eigen::Ref<const Eigen::Matrix2Xd> &ray_origins,
            const Eigen::Ref<const Eigen::Matrix2Xd> &ray_directions,
            double hit_distance_threshold,
            int num_threads,
            std::vector<double> &ray_travel_distances,
            std::vector<std::shared_ptr<Node>> &hit_nodes) const;

        std::vector<int>
        GetNodeTypes() const {
            if (m_node_container_ == nullptr) { return {}; }
            return m_node_container_->GetNodeTypes();
        }

        cv::Mat
        Plot(
            const std::shared_ptr<const common::GridMapInfo<2>> &grid_map_info,
            const std::vector<int> &node_types,
            std::unordered_map<int, cv::Scalar> node_type_colors,
            std::unordered_map<int, int> node_type_radius,
            const cv::Scalar &bg_color = {255, 255, 255},
            const cv::Scalar &area_rect_color = {0, 0, 0},
            int area_rect_thickness = 1,
            const cv::Scalar &tree_data_color = {255, 0, 0},
            int tree_data_radius = 1,
            const std::function<void(cv::Mat &, std::vector<std::shared_ptr<Node>> &)> &plot_node_data = nullptr);

        void
        Print(std::ostream &os) const;

    private:
        [[nodiscard]] static std::shared_ptr<IncrementalQuadtree>
        Create(
            std::shared_ptr<Setting> setting,
            const Aabb2D &area,
            const std::function<std::shared_ptr<NodeContainer>()> &node_container_constructor,
            const std::shared_ptr<IncrementalQuadtree *> &root,
            IncrementalQuadtree *parent,
            const Children::Type child_type) {

            auto tree = std::shared_ptr<IncrementalQuadtree>(new IncrementalQuadtree(std::move(setting), area, node_container_constructor, root));
            tree->m_parent_ = parent;
            tree->m_child_type_ = child_type;
            return tree;
        }

        IncrementalQuadtree(
            std::shared_ptr<Setting> setting,
            Aabb2D area,
            const std::function<std::shared_ptr<NodeContainer>()> &node_container_constructor,
            const std::shared_ptr<IncrementalQuadtree *> &root);

        std::shared_ptr<IncrementalQuadtree>
        Expand(Children::Type current_root_child_type);

        void
        ReplaceChild(const std::shared_ptr<IncrementalQuadtree> &child, Children::Type child_type);

        bool
        CreateChildren();

        void
        DeleteChildren();
    };
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::IncrementalQuadtree::Setting> {
    static Node
    encode(const erl::geometry::IncrementalQuadtree::Setting &setting) {
        Node node;
        node["cluster_half_area_size"] = setting.cluster_half_area_size;
        node["max_half_area_size"] = setting.max_half_area_size;
        node["min_half_area_size"] = setting.min_half_area_size;
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::IncrementalQuadtree::Setting &setting) {
        if (!node.IsMap()) { return false; }
        setting.cluster_half_area_size = node["cluster_half_area_size"].as<double>();
        setting.max_half_area_size = node["max_half_area_size"].as<double>();
        setting.min_half_area_size = node["min_half_area_size"].as<double>();
        return true;
    }
};  // namespace YAML
