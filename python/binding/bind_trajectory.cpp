#include "erl_common/pybind11.hpp"
#include "erl_geometry/trajectory.hpp"

template<typename Dtype>
void
BindTrajectoryImpl(const py::module &m, const char *name) {
    using namespace erl::geometry;
    using T = Trajectory<Dtype>;

    py::class_<T>(m, name)
        .def(py::init<>())
        .def_static("load_2d", &T::Load2D, py::arg("filename"), py::arg("binary"))
        .def_static("load_3d", &T::Load3D, py::arg("filename"), py::arg("binary"))
        .def_static("load_se2", &T::LoadSe2, py::arg("filename"), py::arg("binary"))
        .def_static("load_se3", &T::LoadSe3, py::arg("filename"), py::arg("binary"));
}

void
BindTrajectory(const py::module &m) {
    BindTrajectoryImpl<double>(m, "TrajectoryD");
    BindTrajectoryImpl<float>(m, "TrajectoryF");
}
