#include "erl_common/pybind11.hpp"
#include "erl_geometry/depth_camera_3d.hpp"

void
BindDepthCamera3D(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<DepthCamera3D, CameraBase3D, std::shared_ptr<DepthCamera3D>>(m, "DepthCamera3D")
        .def("Setting", []() { return std::make_shared<DepthCamera3D::Setting>(); })
        .def(
            py::init<std::shared_ptr<DepthCamera3D::Setting>, const Eigen::Ref<const Eigen::Matrix3Xd> &, const Eigen::Ref<const Eigen::Matrix3Xi> &>(),
            py::arg("setting").none(false),
            py::arg("vertices"),
            py::arg("triangles"))
        .def(
            py::init<std::shared_ptr<DepthCamera3D::Setting>, const std::vector<Eigen::Vector3d> &, const std::vector<Eigen::Vector3i> &>(),
            py::arg("setting").none(false),
            py::arg("vertices"),
            py::arg("triangles"))
        .def_property_readonly("setting", &DepthCamera3D::GetSetting)
        .def_property_readonly(
            "ray_directions_in_frame",
            [](const DepthCamera3D &self) -> py::array_t<double> {
                const Eigen::MatrixX<Eigen::Vector3d> dirs = self.GetRayDirectionsInFrame();
                return py::cast_to_array(dirs);
            })
        .def("scan", &DepthCamera3D::Scan, py::arg("orientation"), py::arg("translation"), py::arg("add_noise") = false, py::arg("noise_stddev") = 0.03);
}
