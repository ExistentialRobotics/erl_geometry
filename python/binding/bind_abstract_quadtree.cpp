#include "erl_common/pybind11.hpp"
#include "erl_geometry/abstract_quadtree.hpp"

void
BindAbstractQuadtree(const py::module& m) {
    using namespace erl::geometry;
    py::class_<AbstractQuadtree, std::shared_ptr<AbstractQuadtree>> tree(m, "AbstractQuadtree");
    tree.def("apply_setting", &AbstractQuadtree::ApplySetting, "apply the latest setting to the tree")
        .def(
            "write_raw",
            [](const AbstractQuadtree& self, const std::string& filename) -> bool { return self.Write(filename); },
            py::arg("filename"))
        .def(
            "read_raw",
            [](AbstractQuadtree& self, const std::string& filename) -> bool { return self.LoadData(filename); },
            py::arg("filename"))
        .def("search_node", &AbstractQuadtree::SearchNode, py::arg("x"), py::arg("y"), py::arg("max_depth"));
    py::class_<AbstractQuadtree::QuadtreeNodeIterator>(tree, "QuadtreeNodeIterator")
        .def_property_readonly("x", &AbstractQuadtree::QuadtreeNodeIterator::GetX)
        .def_property_readonly("y", &AbstractQuadtree::QuadtreeNodeIterator::GetY)
        .def_property_readonly("node_size", &AbstractQuadtree::QuadtreeNodeIterator::GetNodeSize)
        .def_property_readonly("depth", &AbstractQuadtree::QuadtreeNodeIterator::GetDepth)
        .def("next", &AbstractQuadtree::QuadtreeNodeIterator::Next)
        .def_property_readonly("is_valid", &AbstractQuadtree::QuadtreeNodeIterator::IsValid);
}
