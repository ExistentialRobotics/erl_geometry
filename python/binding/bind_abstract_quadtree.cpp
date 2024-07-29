#include "erl_common/pybind11.hpp"
#include "erl_geometry/abstract_quadtree.hpp"

void
BindAbstractQuadtree(const py::module& m) {
    using namespace erl::geometry;
    py::class_<AbstractQuadtree, std::shared_ptr<AbstractQuadtree>>(m, "AbstractQuadtree")
        .def("apply_setting", &AbstractQuadtree::ApplySetting, "apply the latest setting to the tree")
        .def(
            "write_raw",
            [](const AbstractQuadtree& self, const std::string& filename) -> bool { return self.Write(filename); },
            py::arg("filename"))
        .def("read_raw", [](AbstractQuadtree& self, const std::string& filename) -> bool { return self.LoadData(filename); }, py::arg("filename"));
}
