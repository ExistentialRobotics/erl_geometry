#include "erl_common/pybind11.hpp"
#include "erl_geometry/depth_camera_3d.hpp"

void
BindDepthCamera3D(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    auto py_camera = py::class_<DepthCamera3D, std::shared_ptr<DepthCamera3D>>(m, "DepthCamera3D");
    py::class_<DepthCamera3D::Setting, YamlableBase, std::shared_ptr<DepthCamera3D::Setting>>(py_camera, "Setting")
        .def(py::init<>())
        .def_readwrite("image_height", &DepthCamera3D::Setting::image_height)
        .def_readwrite("image_width", &DepthCamera3D::Setting::image_width)
        .def_readwrite("camera_fx", &DepthCamera3D::Setting::camera_fx)
        .def_readwrite("camera_fy", &DepthCamera3D::Setting::camera_fy)
        .def_readwrite("camera_cx", &DepthCamera3D::Setting::camera_cx)
        .def_readwrite("camera_cy", &DepthCamera3D::Setting::camera_cy);

    py_camera
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
                Eigen::MatrixX<Eigen::Vector3d> rays = self.GetRayDirectionsInFrame();
                const long n_azimuths = rays.rows();
                const long n_elevations = rays.cols();
                py::array_t<double> out({n_azimuths, n_elevations, 3l});
                for (long i = 0; i < n_azimuths; ++i) {
                    for (long j = 0; j < n_elevations; ++j) {
                        const auto &dir = rays(i, j);
                        out.mutable_at(i, j, 0) = dir[0];
                        out.mutable_at(i, j, 1) = dir[1];
                        out.mutable_at(i, j, 2) = dir[2];
                    }
                }
                return out;
            })
        .def("scan", &DepthCamera3D::Scan, py::arg("orientation"), py::arg("translation"), py::arg("add_noise") = false, py::arg("noise_stddev") = 0.03);
}
