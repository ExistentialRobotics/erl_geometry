#include "erl_common/pybind11.hpp"
#include "erl_geometry/lidar_2d.hpp"

void
BindLidar2D(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    auto py_lidar = py::class_<Lidar2D, std::shared_ptr<Lidar2D>>(m, "Lidar2D");

    py::enum_<Lidar2D::Mode>(py_lidar, "Mode", py::arithmetic(), "Mode of directed distance.")
        .value("kDdf", Lidar2D::Mode::kDdf, "Compute unsigned directed distance.")
        .value("kSddfV1", Lidar2D::Mode::kSddfV1, "Compute signed directed distance, version 1.")
        .value("kSddfV2", Lidar2D::Mode::kSddfV2, "Compute signed directed distance, version 2.")
        .export_values();

    py::class_<Lidar2D::Setting, YamlableBase, std::shared_ptr<Lidar2D::Setting>>(py_lidar, "Setting")
        .def(py::init<>())
        .def_readwrite("min_angle", &Lidar2D::Setting::min_angle)
        .def_readwrite("max_angle", &Lidar2D::Setting::max_angle)
        .def_readwrite("num_lines", &Lidar2D::Setting::num_lines)
        .def_readwrite("mode", &Lidar2D::Setting::mode)
        .def_readwrite("sign_method", &Lidar2D::Setting::sign_method);

    py_lidar.def(py::init<std::shared_ptr<Lidar2D::Setting>, std::shared_ptr<Space2D>>(), py::arg("setting").none(false), py::arg("space2d").none(false))
        .def_property_readonly("setting", &Lidar2D::GetSetting)
        .def_property_readonly("angles", &Lidar2D::GetAngles)
        .def_property_readonly("ray_directions_in_frame", &Lidar2D::GetRayDirectionsInFrame)
        .def(
            "scan",
            py::overload_cast<double, const Eigen::Ref<const Eigen::Vector2d> &, bool>(&Lidar2D::Scan, py::const_),
            py::arg("rotation_angle"),
            py::arg("translation"),
            py::arg("parallel") = false)
        .def(
            "scan",
            py::overload_cast<const Eigen::Ref<const Eigen::Matrix2d> &, const Eigen::Ref<const Eigen::Vector2d> &, bool>(&Lidar2D::Scan, py::const_),
            py::arg("rotation"),
            py::arg("translation"),
            py::arg("parallel") = false)
        .def(
            "scan_multi_poses",
            py::overload_cast<const std::vector<Eigen::Matrix3d> &, bool>(&Lidar2D::ScanMultiPoses, py::const_),  // const method should use py::const_
            py::arg("poses"),
            py::arg("parallel") = false);
}
