#include "erl_geometry/node.hpp"

namespace erl::geometry {

    std::ostream &
    operator<<(std::ostream &os, const NodeData &node_data) {
        node_data.Print(os);
        return os;
    }

}  // namespace erl::geometry
