#include "erl_common/pybind11.hpp"
#include "erl_geometry/rgbd_camera_3d.hpp"

void
BindRgbdCamera3D(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<RgbdCamera3D, CameraBase3D, std::shared_ptr<RgbdCamera3D>>(m, "RgbdCamera3D")
        .def("Setting", []() { return std::make_shared<RgbdCamera3D::Setting>(); })
        .def(py::init<std::shared_ptr<RgbdCamera3D::Setting>>(), py::arg("setting"))
        .def_property_readonly("setting", &RgbdCamera3D::GetSetting)
        .def("add_mesh", &RgbdCamera3D::AddMesh, py::arg("mesh_path"))
        .def(
            "scan",
            [](const RgbdCamera3D &self, const Eigen::Matrix3d &orientation, const Eigen::Vector3d &translation) {
                const auto [rgb, depth] = self.Scan(orientation, translation);
                py::dict result;
                result["rgb"] = rgb;
                result["depth"] = depth;
                return result;
            })
        .def_property_readonly("ray_directions_in_frame", [](const RgbdCamera3D &self) { return py::cast_to_array(self.GetRayDirectionsInFrame()); });
}
