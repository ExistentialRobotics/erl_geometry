#include "pyobject_occupancy_quadtree.hpp"

#include "erl_common/pybind11.hpp"
#include "erl_geometry/pybind11_occupancy_quadtree.hpp"

void
BindPyObjectOccupancyQuadtree(const py::module &m) {
    using namespace erl::geometry;
    BindOccupancyQuadtree<PyObjectOccupancyQuadtree, PyObjectOccupancyQuadtreeNode>(m, "PyObjectOccupancyQuadtree");
}
