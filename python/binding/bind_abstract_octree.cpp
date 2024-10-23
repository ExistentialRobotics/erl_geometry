#include "erl_common/pybind11.hpp"
#include "erl_geometry/abstract_octree.hpp"

void
BindAbstractOctree(const py::module& m) {
    using namespace erl::geometry;
    py::class_<AbstractOctree, std::shared_ptr<AbstractOctree>> tree(m, "AbstractOctree");
    tree.def("apply_setting", &AbstractOctree::ApplySetting, "apply the latest setting to the tree")
        .def(
            "write_raw",
            [](const AbstractOctree& self, const std::string& filename) -> bool { return self.Write(filename); },
            py::arg("filename"))
        .def(
            "read_raw",
            [](AbstractOctree& self, const std::string& filename) -> bool { return self.LoadData(filename); },
            py::arg("filename"))
        .def("search_node", &AbstractOctree::SearchNode, py::arg("x"), py::arg("y"), py::arg("z"), py::arg("max_depth"));
    py::class_<AbstractOctree::OctreeNodeIterator, std::shared_ptr<AbstractOctree::OctreeNodeIterator>>(tree, "OctreeNodeIterator")
        .def_property_readonly("x", &AbstractOctree::OctreeNodeIterator::GetX)
        .def_property_readonly("y", &AbstractOctree::OctreeNodeIterator::GetY)
        .def_property_readonly("z", &AbstractOctree::OctreeNodeIterator::GetZ)
        .def_property_readonly("node_size", &AbstractOctree::OctreeNodeIterator::GetNodeSize)
        .def_property_readonly("depth", &AbstractOctree::OctreeNodeIterator::GetDepth)
        .def("next", &AbstractOctree::OctreeNodeIterator::Next)
        .def_property_readonly("is_valid", &AbstractOctree::OctreeNodeIterator::IsValid)
        .def_property_readonly("node", &AbstractOctree::OctreeNodeIterator::GetNode);
    tree.def("get_leaf_iterator", &AbstractOctree::GetLeafIterator, py::arg("max_depth"))
        .def("get_leaf_in_aabb_iterator", &AbstractOctree::GetLeafInAabbIterator, py::arg("aabb"), py::arg("max_depth"))
        .def("get_tree_iterator", &AbstractOctree::GetTreeIterator, py::arg("max_depth"))
        .def("get_tree_in_aabb_iterator", &AbstractOctree::GetTreeInAabbIterator, py::arg("aabb"), py::arg("max_depth"))
        .def(
            "get_node_on_ray_iterator",
            &AbstractOctree::GetNodeOnRayIterator,
            py::arg("px"),
            py::arg("py"),
            py::arg("pz"),
            py::arg("vx"),
            py::arg("vy"),
            py::arg("vz"),
            py::arg("max_range"),
            py::arg("node_padding"),
            py::arg("bidirectional"),
            py::arg("leaf_only"),
            py::arg("min_node_depth"),
            py::arg("max_node_depth"));
}
