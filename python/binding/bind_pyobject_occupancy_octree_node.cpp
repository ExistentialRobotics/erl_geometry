#include "pyobject_occupancy_octree.hpp"

#include "erl_geometry/pybind11_occupancy_octree.hpp"

void
BindPyObjectOccupancyOctreeNode(const py::module &m) {
    using namespace erl::geometry;
    BindOccupancyOctreeNode<PyObjectOccupancyOctreeNode, AbstractOctreeNode>(m, "PyObjectOccupancyOctreeNode")
        .def_property("py_object", &PyObjectOccupancyOctreeNode::GetPyObject, &PyObjectOccupancyOctreeNode::SetPyObject);
}
