#pragma once

#include "erl_common/yaml.hpp"
#include "node_container.hpp"

namespace erl::geometry {

    class NodeContainerSingleType : public NodeContainer {

    public:
        struct Setting : public common::Yamlable<Setting> {
            int capacity = 1;
            double min_squared_distance = 0.04;

            Setting() = default;

            Setting(int capacity, double min_squared_distance)
                : capacity(capacity),
                  min_squared_distance(min_squared_distance) {}
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::vector<std::shared_ptr<geometry::Node>> m_nodes_ = {};

    public:
        inline static std::shared_ptr<NodeContainerSingleType>
        Create(const std::shared_ptr<Setting> &setting) {
            return std::shared_ptr<NodeContainerSingleType>(new NodeContainerSingleType(setting));
        }

        [[nodiscard]] inline std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] inline std::vector<int>
        GetNodeTypes() const override {
            return {0};
        }

        [[nodiscard]] inline std::string
        GetNodeTypeName(int type) const override {
            ERL_ASSERTM(type == 0, "Invalid node type.");
            return std::to_string(type);
        }

        [[nodiscard]] inline std::size_t
        Capacity() const override {
            return m_setting_->capacity;
        }

        [[nodiscard]] inline std::size_t
        Capacity(int type) const override {
            ERL_ASSERTM(type == 0, "Invalid node type.");
            return m_setting_->capacity;
        }

        [[nodiscard]] inline std::size_t
        Size() const override {
            return m_nodes_.size();
        }

        [[nodiscard]] inline std::size_t
        Size(int type) const override {
            ERL_ASSERTM(type == 0, "Invalid node type.");
            return m_nodes_.size();
        }

        inline void
        Clear() override {
            Reset();
        }

        inline std::vector<std::shared_ptr<geometry::Node>>::iterator
        Begin(int type) override {
            if (type == 0) { return m_nodes_.begin(); }
            throw std::runtime_error("Invalid node type.");
        }

        [[nodiscard]] inline std::vector<std::shared_ptr<geometry::Node>>::const_iterator
        Begin(int type) const override {
            if (type == 0) { return m_nodes_.begin(); }
            throw std::runtime_error("Invalid node type.");
        }

        inline std::vector<std::shared_ptr<geometry::Node>>::iterator
        End(int type) override {
            if (type == 0) { return m_nodes_.end(); }
            throw std::runtime_error("Invalid node type.");
        }

        [[nodiscard]] inline std::vector<std::shared_ptr<geometry::Node>>::const_iterator
        End(int type) const override {
            if (type == 0) { return m_nodes_.end(); }
            throw std::runtime_error("Invalid node type.");
        }

        inline void
        CollectNodes(std::vector<std::shared_ptr<geometry::Node>> &out) const override {
            out.insert(out.end(), m_nodes_.begin(), m_nodes_.end());
        }

        inline void
        CollectNodesOfType(int type, std::vector<std::shared_ptr<geometry::Node>> &out) const override {
            if (type == 0) {
                out.insert(out.end(), m_nodes_.begin(), m_nodes_.end());
            } else {
                throw std::runtime_error("Invalid node type.\n");
            }
        }

        inline void
        CollectNodesOfTypeInAabb2D(int type, const geometry::Aabb2D &area, std::vector<std::shared_ptr<geometry::Node>> &nodes) const override {
            ERL_ASSERTM(type == 0, "Invalid node type.\n");

            for (auto &kNode: m_nodes_) {
                ERL_ASSERTM(kNode->position.size() == 2, "Invalid node position dimension: %ld. Expecting 2.\n", kNode->position.size());
                if (area.contains(kNode->position)) { nodes.push_back(kNode); }
            }
        }

        inline void
        CollectNodesOfTypeInAabb3D(int type, const geometry::Aabb3D &area, std::vector<std::shared_ptr<geometry::Node>> &nodes) const override {
            ERL_ASSERTM(type == 0, "Invalid node type.\n");

            for (auto &kNode: m_nodes_) {
                ERL_ASSERTM(kNode->position.size() == 3, "Invalid node position dimension: %ld. Expecting 3.\n", kNode->position.size());
                if (kNode->type == type && area.contains(kNode->position)) { nodes.push_back(kNode); }
            }
        }

        inline bool
        Insert(const std::shared_ptr<geometry::Node> &node, bool &too_close) override {
            too_close = false;

            // must compute too_close before checking capacity, otherwise the node may be inserted into another container.
            if (!m_nodes_.empty()) {  // not empty
                for (const auto &stored_node: m_nodes_) {
                    if ((stored_node->position - node->position).squaredNorm() < m_setting_->min_squared_distance) {
                        too_close = true;
                        return false;
                    }
                }
            }

            if (m_nodes_.size() >= std::size_t(m_setting_->capacity)) { return false; }
            m_nodes_.push_back(node);
            return true;
        }

        inline bool
        Remove(const std::shared_ptr<const geometry::Node> &node) override {
            if (m_nodes_.empty()) { return false; }

            auto begin = m_nodes_.begin();
            auto end = m_nodes_.end();

            for (auto itr = begin; itr < end; ++itr) {
                if (**itr != *node) { continue; }
                *itr = *(--end);
                m_nodes_.pop_back();
                return true;
            }

            return false;
        }

    private:
        explicit NodeContainerSingleType(std::shared_ptr<Setting> setting)
            : m_setting_(std::move(setting)) {
            Reset();
        }

        inline void
        Reset() {
            ERL_ASSERTM(m_setting_ != nullptr, "Setting is null");
            ERL_ASSERTM(m_setting_->capacity > 0, "Capacity must be greater than 0");
            ERL_ASSERTM(m_setting_->min_squared_distance > 0, "Min squared distance must be greater than 0");
            m_nodes_.clear();
            m_nodes_.reserve(this->m_setting_->capacity);
        }
    };
}  // namespace erl::geometry

namespace YAML {

    template<>
    struct convert<erl::geometry::NodeContainerSingleType::Setting> {
        inline static Node
        encode(const erl::geometry::NodeContainerSingleType::Setting &setting) {
            Node node;
            node["capacity"] = setting.capacity;
            node["min_squared_distance"] = setting.min_squared_distance;
            return node;
        }

        inline static bool
        decode(const Node &node, erl::geometry::NodeContainerSingleType::Setting &setting) {
            if (!node.IsMap()) { return false; }
            setting.capacity = node["capacity"].as<int>();
            setting.min_squared_distance = node["min_squared_distance"].as<double>();
            return true;
        }
    };

//    inline Emitter &
//    operator<<(Emitter &out, const erl::geometry::NodeContainerSingleType::Setting &setting) {
//        out << BeginMap;
//        out << Key << "capacity" << Value << setting.capacity;
//        out << Key << "min_squared_distance" << Value << setting.min_squared_distance;
//        out << EndMap;
//        return out;
//    }
}  // namespace YAML
