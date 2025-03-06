#include "erl_common/pybind11.hpp"
#include "erl_geometry/depth_camera_3d.hpp"

template<typename Dtype>
void
BindDepthCamera3DImpl(const py::module &m, const char *name) {
    using namespace erl::common;
    using namespace erl::geometry;
    using T = DepthCamera3D<Dtype>;

    py::class_<T, CameraBase3D<Dtype>, RangeSensor3D<Dtype>, std::shared_ptr<T>>(m, name)
        .def("Setting", []() { return std::make_shared<typename T::Setting>(); })
        .def(py::init<std::shared_ptr<typename T::Setting>>(), py::arg("setting").none(false))
        .def_property_readonly("setting", &T::GetSetting);
}

void
BindDepthCamera3D(const py::module &m) {
    BindDepthCamera3DImpl<double>(m, "DepthCamera3Dd");
    BindDepthCamera3DImpl<float>(m, "DepthCamera3Df");
}
