#include "erl_common/pybind11.hpp"
#include "erl_geometry/quadtree_key.hpp"

void
BindQuadtreeKey(const py::module &m) {
    using namespace erl::geometry;

    py::class_<QuadtreeKey>(m, "QuadtreeKey")
        .def("__eq__", [](const QuadtreeKey &self, const QuadtreeKey &other) { return self == other; })
        .def("__ne__", [](const QuadtreeKey &self, const QuadtreeKey &other) { return self != other; })
        .def("__getitem__", [](const QuadtreeKey &self, const int idx) { return self[idx]; });
}
