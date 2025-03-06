#include "erl_common/pybind11.hpp"
#include "erl_geometry/abstract_octree.hpp"

template<typename Dtype>
void
BindAbstractOctreeImpl(const py::module& m, const char* name) {
    using namespace erl::geometry;
    using T = AbstractOctree<Dtype>;

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
        .def("search_node", &T::SearchNode, py::arg("x"), py::arg("y"), py::arg("z"), py::arg("max_depth"));
    py::class_<typename T::OctreeNodeIterator>(tree, "OctreeNodeIterator")
        .def_property_readonly("x", &T::OctreeNodeIterator::GetX)
        .def_property_readonly("y", &T::OctreeNodeIterator::GetY)
        .def_property_readonly("z", &T::OctreeNodeIterator::GetZ)
        .def_property_readonly("node_size", &T::OctreeNodeIterator::GetNodeSize)
        .def_property_readonly("depth", &T::OctreeNodeIterator::GetDepth)
        .def("next", &T::OctreeNodeIterator::Next)
        .def_property_readonly("is_valid", &T::OctreeNodeIterator::IsValid);
}

void
BindAbstractOctree(const py::module& m) {
    BindAbstractOctreeImpl<double>(m, "AbstractOctreeD");
    BindAbstractOctreeImpl<float>(m, "AbstractOctreeF");
}
