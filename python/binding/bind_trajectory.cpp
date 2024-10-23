#include "erl_common/pybind11.hpp"
#include "erl_geometry/trajectory.hpp"

void
BindTrajectory(const py::module &m) {
    using namespace erl::geometry;

    py::class_<Trajectory>(m, "Trajectory")
        .def(py::init<>())
        .def_static("load_2d", &Trajectory::Load2D, py::arg("filename"), py::arg("binary"))
        .def_static("load_3d", &Trajectory::Load3D, py::arg("filename"), py::arg("binary"))
        .def_static("load_se2", &Trajectory::LoadSe2, py::arg("filename"), py::arg("binary"))
        .def_static("load_se3", &Trajectory::LoadSe3, py::arg("filename"), py::arg("binary"));
}
