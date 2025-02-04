#include "erl_common/pybind11.hpp"
#include "erl_geometry/depth_camera_3d.hpp"

void
BindDepthCamera3D(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<DepthCamera3D, CameraBase3D, RangeSensor3D, std::shared_ptr<DepthCamera3D>>(m, "DepthCamera3D")
        .def("Setting", []() { return std::make_shared<DepthCamera3D::Setting>(); })
        .def(py::init<std::shared_ptr<DepthCamera3D::Setting>>(), py::arg("setting").none(false))
        .def_property_readonly("setting", &DepthCamera3D::GetSetting);
}
