#include "erl_common/pybind11.hpp"
#include "erl_geometry/pybind11_occupancy_octree.hpp"
#include "erl_geometry/surface_mapping_octree_node.hpp"

void
BindSurfaceMappingOctreeNode(const py::module& m) {
    using namespace erl::geometry;
    auto node = BindOccupancyOctreeNode<SurfaceMappingOctreeNode, OccupancyOctreeNode>(m, "SurfaceMappingOctreeNode");
    py::class_<SurfaceMappingOctreeNode::SurfaceData, std::shared_ptr<SurfaceMappingOctreeNode::SurfaceData>>(node, "SurfaceData")
        .def_readwrite("position", &SurfaceMappingOctreeNode::SurfaceData::position)
        .def_readwrite("normal", &SurfaceMappingOctreeNode::SurfaceData::normal)
        .def_readwrite("var_position", &SurfaceMappingOctreeNode::SurfaceData::var_position)
        .def_readwrite("var_normal", &SurfaceMappingOctreeNode::SurfaceData::var_normal)
        .def("__eq__", (&SurfaceMappingOctreeNode::SurfaceData::operator==), py::arg("other"))
        .def("__ne__", (&SurfaceMappingOctreeNode::SurfaceData::operator!=), py::arg("other"));
    node.def_property_readonly("surface_data", &SurfaceMappingOctreeNode::GetSurfaceData);
}
