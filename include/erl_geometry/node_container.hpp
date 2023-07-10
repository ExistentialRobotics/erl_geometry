#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "aabb.hpp"
#include "node.hpp"

namespace erl::geometry {
    struct NodeContainer {

        virtual ~NodeContainer() = default;

        [[nodiscard]] virtual std::vector<int>
        GetNodeTypes() const = 0;

        [[nodiscard]] virtual std::string
        GetNodeTypeName(int type) const = 0;

        [[nodiscard]] virtual std::size_t
        Size() const = 0;  // number of m_nodes_ stored in this container

        [[nodiscard]] virtual std::size_t
        Size(int type) const = 0;

        [[nodiscard]] virtual std::size_t
        Capacity() const = 0;

        [[nodiscard]] virtual std::size_t
        Capacity(int type) const = 0;

        [[nodiscard]] inline bool
        Empty() const {
            return Size() == 0;
        }

        [[nodiscard]] inline bool
        Empty(int type) const {
            return Size(type) == 0;
        }

        [[nodiscard]] inline bool
        Full() const {
            return Size() >= Capacity();
        }

        [[nodiscard]] inline bool
        Full(int type) const {
            return Size(type) >= Capacity(type);
        }

        virtual void
        Clear() = 0;

        virtual std::vector<std::shared_ptr<Node>>::iterator
        Begin(int type) = 0;

        [[nodiscard]] virtual std::vector<std::shared_ptr<Node>>::const_iterator
        Begin(int type) const = 0;

        virtual std::vector<std::shared_ptr<Node>>::iterator
        End(int type) = 0;

        [[nodiscard]] virtual std::vector<std::shared_ptr<Node>>::const_iterator
        End(int type) const = 0;

        virtual void
        CollectNodes(std::vector<std::shared_ptr<Node>> &out) const = 0;

        [[nodiscard]] std::vector<std::shared_ptr<Node>>
        CollectNodes() const {
            std::vector<std::shared_ptr<Node>> out;
            CollectNodes(out);
            return out;
        }

        virtual void
        CollectNodesOfType(int type, std::vector<std::shared_ptr<Node>> &out) const = 0;

        [[nodiscard]] std::vector<std::shared_ptr<Node>>
        CollectNodesOfType(int type) const {
            std::vector<std::shared_ptr<Node>> out;
            CollectNodesOfType(type, out);
            return out;
        }

        virtual void
        CollectNodesOfTypeInAabb2D(int type, const Aabb2D &area, std::vector<std::shared_ptr<Node>> &nodes) const = 0;

        [[nodiscard]] std::vector<std::shared_ptr<Node>>
        CollectNodesOfTypeInAabb2D(int type, const Aabb2D &area) const {
            std::vector<std::shared_ptr<Node>> out;
            CollectNodesOfTypeInAabb2D(type, area, out);
            return out;
        }

        virtual void
        CollectNodesOfTypeInAabb3D(int type, const Aabb3D &area, std::vector<std::shared_ptr<Node>> &nodes) const = 0;

        [[nodiscard]] std::vector<std::shared_ptr<Node>>
        CollectNodesOfTypeInAabb3D(int type, const Aabb3D &area) const {
            std::vector<std::shared_ptr<Node>> out;
            CollectNodesOfTypeInAabb3D(type, area, out);
            return out;
        }

        virtual bool
        Insert(const std::shared_ptr<Node> &node, bool &too_close) = 0;

        virtual bool
        Remove(const std::shared_ptr<const Node> &node) = 0;
    };
}  // namespace erl::geometry
