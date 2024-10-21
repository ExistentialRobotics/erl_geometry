#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_octree_node.hpp"
#include "erl_geometry/pybind11_occupancy_octree.hpp"

void
BindOccupancyOctreeNode(const py::module& m) {
    using namespace erl::geometry;
    BindOccupancyOctreeNode<OccupancyOctreeNode, AbstractOctreeNode>(m, "OccupancyOctreeNode");
}
