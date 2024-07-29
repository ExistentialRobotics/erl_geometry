#include "erl_common/pybind11.hpp"
#include "erl_geometry/abstract_octree.hpp"

void
BindAbstractOctree(const py::module& m) {
    using namespace erl::geometry;
    py::class_<AbstractOctree, std::shared_ptr<AbstractOctree>>(m, "AbstractOctree")
        .def("apply_setting", &AbstractOctree::ApplySetting, "apply the latest setting to the tree")
        .def(
            "write_raw",
            [](const AbstractOctree& self, const std::string& filename) -> bool { return self.Write(filename); },
            py::arg("filename"))
        .def("read_raw", [](AbstractOctree& self, const std::string& filename) -> bool { return self.LoadData(filename); }, py::arg("filename"));
}
