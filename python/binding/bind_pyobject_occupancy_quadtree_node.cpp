#include "pyobject_occupancy_quadtree.hpp"

#include "erl_geometry/pybind11_occupancy_quadtree.hpp"

void
BindPyObjectOccupancyQuadtreeNode(const py::module &m) {
    using namespace erl::geometry;
    BindOccupancyQuadtreeNode<PyObjectOccupancyQuadtreeNode, AbstractQuadtreeNode>(m, "PyObjectOccupancyQuadtreeNode")
        .def_property("py_object", &PyObjectOccupancyQuadtreeNode::GetPyObject, &PyObjectOccupancyQuadtreeNode::SetPyObject);
}
