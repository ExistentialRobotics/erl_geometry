#include "erl_common/pybind11.hpp"
#include "erl_geometry/pybind11_occupancy_quadtree.hpp"
#include "erl_geometry/surface_mapping_quadtree_node.hpp"

void
BindSurfaceMappingQuadtreeNode(const py::module& m) {
    using namespace erl::geometry;
    auto node = BindOccupancyQuadtreeNode<SurfaceMappingQuadtreeNode, OccupancyQuadtreeNode>(m, "SurfaceMappingQuadtreeNode");
    py::class_<SurfaceMappingQuadtreeNode::SurfaceData, std::shared_ptr<SurfaceMappingQuadtreeNode::SurfaceData>>(node, "SurfaceData")
        .def_readwrite("position", &SurfaceMappingQuadtreeNode::SurfaceData::position)
        .def_readwrite("normal", &SurfaceMappingQuadtreeNode::SurfaceData::normal)
        .def_readwrite("var_position", &SurfaceMappingQuadtreeNode::SurfaceData::var_position)
        .def_readwrite("var_normal", &SurfaceMappingQuadtreeNode::SurfaceData::var_normal)
        .def("__eq__", (&SurfaceMappingQuadtreeNode::SurfaceData::operator==), py::arg("other"))
        .def("__ne__", (&SurfaceMappingQuadtreeNode::SurfaceData::operator!=), py::arg("other"));
    node.def_property_readonly("surface_data", &SurfaceMappingQuadtreeNode::GetSurfaceData);
}
