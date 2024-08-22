#include "erl_common/pybind11.hpp"
#include "erl_geometry/octree_key.hpp"

void
BindOctreeKey(const py::module &m) {
    using namespace erl::geometry;

    py::class_<OctreeKey>(m, "OctreeKey")
        .def("__eq__", [](const OctreeKey &self, const OctreeKey &other) { return self == other; })
        .def("__ne__", [](const OctreeKey &self, const OctreeKey &other) { return self != other; })
        .def("__getitem__", [](const OctreeKey &self, const int idx) { return self[idx]; })
        .def("__hash__", [](const OctreeKey &self) { return OctreeKey::KeyHash()(self); });
}
