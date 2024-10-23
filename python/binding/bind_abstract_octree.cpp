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
    py::class_<AbstractOctree::OctreeNodeIterator>(tree, "OctreeNodeIterator")
        .def_property_readonly("x", &AbstractOctree::OctreeNodeIterator::GetX)
        .def_property_readonly("y", &AbstractOctree::OctreeNodeIterator::GetY)
        .def_property_readonly("z", &AbstractOctree::OctreeNodeIterator::GetZ)
        .def_property_readonly("node_size", &AbstractOctree::OctreeNodeIterator::GetNodeSize)
        .def_property_readonly("depth", &AbstractOctree::OctreeNodeIterator::GetDepth)
        .def("next", &AbstractOctree::OctreeNodeIterator::Next)
        .def_property_readonly("is_valid", &AbstractOctree::OctreeNodeIterator::IsValid);
}
