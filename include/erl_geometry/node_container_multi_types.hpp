#pragma once

#include "erl_common/yaml.hpp"
#include "node_container.hpp"

namespace erl::geometry {

    class NodeContainerMultiTypes : public NodeContainer {

    public:
        struct Setting : public common::Yamlable<Setting> {
            int num_node_types = 1;
            std::vector<int> node_type_capacity = {1};
            std::vector<double> node_type_min_squared_distance = {0.04};

            Setting() = default;

            explicit Setting(int num_node_types, int capacity_per_type = 1, double min_squared_distance = 0.04)
                : num_node_types(num_node_types),
                  node_type_capacity(num_node_types, capacity_per_type),
                  node_type_min_squared_distance(num_node_types, min_squared_distance) {}
        };

    private:
        std::shared_ptr<Setting> m_setting_;
        std::vector<std::vector<std::shared_ptr<Node>>> m_nodes_;
        std::size_t m_num_nodes_ = 0;

    public:
        inline static std::shared_ptr<NodeContainerMultiTypes>
        Create(const std::shared_ptr<Setting> &setting) {
            return std::shared_ptr<NodeContainerMultiTypes>(new NodeContainerMultiTypes(setting));
        }

        [[nodiscard]] inline std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] inline std::vector<int>
        GetNodeTypes() const override {
            std::vector<int> node_types(m_setting_->num_node_types);
            std::iota(node_types.begin(), node_types.end(), 0);
            return node_types;
        }

        [[nodiscard]] inline std::string
        GetNodeTypeName(int type) const override {
            ERL_ASSERTM(type >= 0 && type < m_setting_->num_node_types, "Invalid node type.");
            return std::to_string(type);
        }

        [[nodiscard]] inline std::size_t
        Capacity() const override {
            return std::reduce(m_setting_->node_type_capacity.begin(), m_setting_->node_type_capacity.end(), 0);
        }

        [[nodiscard]] inline std::size_t
        Capacity(int type) const override {
            ERL_DEBUG_ASSERT(type >= 0 && type < m_setting_->num_node_types, "Invalid node type.");
            return m_setting_->node_type_capacity[type];
        }

        [[nodiscard]] inline std::size_t
        Size() const override {
            return m_num_nodes_;
        }

        [[nodiscard]] inline std::size_t
        Size(int type) const override {
            ERL_DEBUG_ASSERT(type >= 0 && type < m_setting_->num_node_types, "Invalid node type.");
            return m_nodes_[type].size();
        }

        inline void
        Clear() override {
            Reset();
        }

        inline std::vector<std::shared_ptr<Node>>::iterator
        Begin(int type) override {
            ERL_DEBUG_ASSERT(type >= 0 && type < m_setting_->num_node_types, "Invalid node type.");
            return m_nodes_[type].begin();
        }

        [[nodiscard]] inline std::vector<std::shared_ptr<Node>>::const_iterator
        Begin(int type) const override {
            ERL_DEBUG_ASSERT(type >= 0 && type < m_setting_->num_node_types, "Invalid node type.");
            return m_nodes_[type].begin();
        }

        inline std::vector<std::shared_ptr<Node>>::iterator
        End(int type) override {
            ERL_DEBUG_ASSERT(type >= 0 && type < m_setting_->num_node_types, "Invalid node type.");
            return m_nodes_[type].end();
        }

        [[nodiscard]] inline std::vector<std::shared_ptr<Node>>::const_iterator
        End(int type) const override {
            ERL_DEBUG_ASSERT(type >= 0 && type < m_setting_->num_node_types, "Invalid node type.");
            return m_nodes_[type].end();
        }

        inline void
        CollectNodes(std::vector<std::shared_ptr<Node>> &out) const override {
            if (!m_num_nodes_) { return; }
            out.reserve(out.size() + m_num_nodes_);
            for (auto nodes: m_nodes_) { out.insert(out.end(), nodes.begin(), nodes.end()); }
        }

        inline void
        CollectNodesOfType(int type, std::vector<std::shared_ptr<Node>> &out) const override {
            ERL_DEBUG_ASSERT(type >= 0 && type < m_setting_->num_node_types, "Invalid node type.");
            auto &nodes_of_type = m_nodes_[type];
            out.insert(out.end(), nodes_of_type.begin(), nodes_of_type.end());
        }

        inline void
        CollectNodesOfTypeInAabb2D(int type, const Aabb2D &area, std::vector<std::shared_ptr<Node>> &nodes) const override {
            ERL_DEBUG_ASSERT(type >= 0 && type < m_setting_->num_node_types, "Invalid node type.");
            auto &nodes_of_type = m_nodes_[type];
            if (nodes_of_type.empty()) { return; }
            nodes.reserve(nodes.size() + nodes_of_type.size());
            for (auto &node: nodes_of_type) {
                ERL_DEBUG_ASSERT(node->position.size() == 2, "Node position must be 2D to use this method.");
                if (area.contains(node->position)) { nodes.push_back(node); }
            }
        }

        inline void
        CollectNodesOfTypeInAabb3D(int type, const Aabb3D &area, std::vector<std::shared_ptr<Node>> &nodes) const override {
            ERL_DEBUG_ASSERT(type >= 0 && type < m_setting_->num_node_types, "Invalid node type.");
            auto &nodes_of_type = m_nodes_[type];
            if (nodes_of_type.empty()) { return; }
            nodes.reserve(nodes.size() + nodes_of_type.size());
            for (auto &node: nodes_of_type) {
                ERL_DEBUG_ASSERT(node->position.size() == 3, "Node position must be 3D to use this method.");
                if (area.contains(node->position)) { nodes.push_back(node); }
            }
        }

        inline bool
        Insert(const std::shared_ptr<Node> &node, bool &too_close) override {
            auto &node_type = node->type;
            ERL_ASSERTM(node_type >= 0 && node_type < m_setting_->num_node_types, "Invalid node type.");

            too_close = false;
            auto &nodes_of_type = m_nodes_[node_type];
            auto &min_squared_distance = m_setting_->node_type_min_squared_distance[node_type];

            // must compute too_close before checking capacity, otherwise the node may be inserted into another container.
            if (!nodes_of_type.empty()) {  // not empty
                for (auto &node_of_type: nodes_of_type) {
                    if ((node_of_type->position - node->position).squaredNorm() < min_squared_distance) {
                        too_close = true;
                        return false;
                    }
                }
            }

            if (nodes_of_type.size() >= std::size_t(m_setting_->node_type_capacity[node_type])) { return false; }
            nodes_of_type.push_back(node);
            m_num_nodes_++;
            return true;
        }

        inline bool
        Remove(const std::shared_ptr<const geometry::Node> &node) override {
            auto &node_type = node->type;
            ERL_DEBUG_ASSERT(node_type >= 0 && node_type < m_setting_->num_node_types, "Invalid node type.");

            auto &nodes_of_type = m_nodes_[node_type];
            if (nodes_of_type.empty()) { return false; }

            auto begin = nodes_of_type.begin();
            auto end = nodes_of_type.end();

            for (auto itr = begin; itr < end; ++itr) {
                if (**itr != *node) { continue; }
                *itr = *(--end);
                nodes_of_type.pop_back();
                m_num_nodes_--;
                return true;
            }

            return false;
        }

    private:
        inline explicit NodeContainerMultiTypes(const std::shared_ptr<Setting> &setting)
            : m_setting_(setting) {
            Reset();
        }

        inline void
        Reset() {
            ERL_DEBUG_ASSERT(m_setting_ != nullptr, "Invalid setting.");
            ERL_DEBUG_ASSERT(m_setting_->num_node_types > 0, "Invalid number of node types.");
            m_nodes_.resize(m_setting_->num_node_types);
            for (int type = 0; type < m_setting_->num_node_types; ++type) {
                ERL_DEBUG_ASSERT(m_setting_->node_type_capacity[type] > 0, "Invalid node type capacity.");
                ERL_DEBUG_ASSERT(m_setting_->node_type_min_squared_distance[type] >= 0, "Invalid node type min squared distance.");
                m_nodes_[type].clear();
                m_nodes_[type].reserve(m_setting_->node_type_capacity[type]);
            }
            m_num_nodes_ = 0;
        }
    };
}  // namespace erl::geometry

namespace YAML {

    template<>
    struct convert<erl::geometry::NodeContainerMultiTypes::Setting> {
        inline static Node
        encode(const erl::geometry::NodeContainerMultiTypes::Setting &setting) {
            Node node;
            node["num_node_types"] = setting.num_node_types;
            node["node_type_capacity"] = setting.node_type_capacity;
            node["node_type_min_squared_distance"] = setting.node_type_min_squared_distance;
            return node;
        }

        inline static bool
        decode(const Node &node, erl::geometry::NodeContainerMultiTypes::Setting &setting) {
            if (!node.IsMap()) { return false; }
            setting.num_node_types = node["num_node_types"].as<int>();
            setting.node_type_capacity = node["node_type_capacity"].as<std::vector<int>>();
            setting.node_type_min_squared_distance = node["node_type_min_squared_distance"].as<std::vector<double>>();
            ERL_ASSERTM(int(setting.node_type_capacity.size()) == setting.num_node_types, "node_type_capacity.size() != num_node_types");
            ERL_ASSERTM(
                int(setting.node_type_min_squared_distance.size()) == setting.num_node_types,
                "node_type_min_squared_distance.size() != num_node_types");
            return true;
        }
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::NodeContainerMultiTypes::Setting &setting) {
        out << BeginMap;
        out << Key << "num_node_types" << Value << setting.num_node_types;
        out << Key << "node_type_capacity" << Value << setting.node_type_capacity;
        out << Key << "node_type_min_squared_distance" << Value << setting.node_type_min_squared_distance;
        out << EndMap;
        return out;
    }
}  // namespace YAML
