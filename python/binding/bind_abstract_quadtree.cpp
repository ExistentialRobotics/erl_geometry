#include "erl_common/pybind11.hpp"
#include "erl_geometry/abstract_quadtree.hpp"

template<typename Dtype>
void
BindAbstractQuadtreeImpl(const py::module& m, const char* name) {
    using namespace erl::geometry;
    using T = AbstractQuadtree<Dtype>;

    py::class_<T, std::shared_ptr<T>> tree(m, name);
    tree.def("apply_setting", &T::ApplySetting, "apply the latest setting to the tree")
        .def(
            "write_raw",
            [](const T& self, const std::string& filename) -> bool { return self.Write(filename); },
            py::arg("filename"))
        .def(
            "read_raw",
            [](T& self, const std::string& filename) -> bool { return self.LoadData(filename); },
            py::arg("filename"))
        .def("search_node", &T::SearchNode, py::arg("x"), py::arg("y"), py::arg("max_depth"));
    py::class_<typename T::QuadtreeNodeIterator>(tree, "QuadtreeNodeIterator")
        .def_property_readonly("x", &T::QuadtreeNodeIterator::GetX)
        .def_property_readonly("y", &T::QuadtreeNodeIterator::GetY)
        .def_property_readonly("node_size", &T::QuadtreeNodeIterator::GetNodeSize)
        .def_property_readonly("depth", &T::QuadtreeNodeIterator::GetDepth)
        .def("next", &T::QuadtreeNodeIterator::Next)
        .def_property_readonly("is_valid", &T::QuadtreeNodeIterator::IsValid);
}

void
BindAbstractQuadtree(const py::module& m) {
    BindAbstractQuadtreeImpl<double>(m, "AbstractQuadtreeD");
    BindAbstractQuadtreeImpl<float>(m, "AbstractQuadtreeF");
}
