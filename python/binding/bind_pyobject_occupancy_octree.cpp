#include "pyobject_occupancy_octree.hpp"

#include "erl_common/pybind11.hpp"
#include "erl_geometry/pybind11_occupancy_octree.hpp"

void
BindPyObjectOccupancyOctree(const py::module &m) {
    using namespace erl::geometry;
    BindOccupancyOctree<PyObjectOccupancyOctreeD, PyObjectOccupancyOctreeNode>(m, "PyObjectOccupancyOctreeD");
    BindOccupancyOctree<PyObjectOccupancyOctreeF, PyObjectOccupancyOctreeNode>(m, "PyObjectOccupancyOctreeF");
}
