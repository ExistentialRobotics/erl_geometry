#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/pybind11_occupancy_octree.hpp"

void
BindOccupancyOctree(const py::module& m) {
    using namespace erl::geometry;
    BindOccupancyOctree<OccupancyOctreeD, OccupancyOctreeNode>(m, "OccupancyOctreeD");
    BindOccupancyOctree<OccupancyOctreeF, OccupancyOctreeNode>(m, "OccupancyOctreeF");
}
