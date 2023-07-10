#pragma once

#include <memory>
#include <ostream>

#include "erl_common/eigen.hpp"

namespace erl::geometry {
    struct NodeData {

        virtual ~NodeData() = default;

        virtual void
        Print(std::ostream &os) const = 0;

        virtual bool
        operator==(const NodeData &other) const = 0;
    };

    std::ostream &
    operator<<(std::ostream &os, const NodeData &node_data);

    struct Node {
        int type;                                       // defined by successors of NodeContainer
        Eigen::VectorXd position;                       // node position to determine where to store in the tree
        std::shared_ptr<NodeData> node_data = nullptr;  // attached data

        Node(int type, Eigen::VectorXd position, std::shared_ptr<NodeData> data_ptr = nullptr)
            : type(type),
              position(std::move(position)),
              node_data(std::move(data_ptr)) {}

        virtual ~Node() = default;

        template<typename T>
        std::shared_ptr<T>
        GetData() {
            return std::dynamic_pointer_cast<T>(node_data);
        }

        virtual inline bool
        operator==(const Node &other) const {
            return type == other.type && position == other.position;
        }

        inline bool
        operator!=(const Node &other) const {
            return !(*this == other);
        }
    };

}  // namespace erl::geometry
