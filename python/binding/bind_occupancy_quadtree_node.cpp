#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_quadtree_node.hpp"
#include "erl_geometry/pybind11_occupancy_quadtree.hpp"

void
BindOccupancyQuadtreeNode(const py::module &m) {
    using namespace erl::geometry;
    BindOccupancyQuadtreeNode<OccupancyQuadtreeNode, AbstractQuadtreeNode>(m, "OccupancyQuadtreeNode");
}
