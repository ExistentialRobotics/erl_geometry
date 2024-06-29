#include "erl_common/pybind11.hpp"
#include "erl_geometry/rgbd_frame_3d.hpp"

void
BindRgbdFrame3D(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<RgbdFrame3D, RangeSensorFrame3D, std::shared_ptr<RgbdFrame3D>> rgbd_frame(m, "RgbdFrame3D");
    py::class_<RgbdFrame3D::Setting, RangeSensorFrame3D::Setting, std::shared_ptr<RgbdFrame3D::Setting>>(rgbd_frame, "Setting")
        .def(py::init<>())
        .def_readwrite("image_height", &RgbdFrame3D::Setting::image_height)
        .def_readwrite("image_width", &RgbdFrame3D::Setting::image_width)
        .def_readwrite("camera_fx", &RgbdFrame3D::Setting::camera_fx)
        .def_readwrite("camera_fy", &RgbdFrame3D::Setting::camera_fy)
        .def_readwrite("camera_cx", &RgbdFrame3D::Setting::camera_cx)
        .def_readwrite("camera_cy", &RgbdFrame3D::Setting::camera_cy)
        .def_readwrite("depth_scale", &RgbdFrame3D::Setting::depth_scale)
        .def("resize", &RgbdFrame3D::Setting::Resize, py::arg("factor"));
    rgbd_frame.def(py::init<std::shared_ptr<RgbdFrame3D::Setting>>(), py::arg("setting").none(false))
        .def("reset", [](RgbdFrame3D &self) { self.Reset(); })
        .def(
            "update_ranges",
            py::overload_cast<const Eigen::Ref<const Eigen::Matrix3d> &, const Eigen::Ref<const Eigen::Vector3d> &, Eigen::MatrixXd, bool>(
                &RgbdFrame3D::UpdateRanges),
            py::arg("rotation"),
            py::arg("translation"),
            py::arg("depth"),
            py::arg("partition_rays"))
        .def(
            "update_ranges",
            py::overload_cast<const Eigen::Ref<const Eigen::Matrix3d> &, const Eigen::Ref<const Eigen::Vector3d> &, const std::string &, bool>(
                &RgbdFrame3D::UpdateRanges),
            py::arg("rotation"),
            py::arg("translation"),
            py::arg("depth_file"),
            py::arg("partition_rays") = false)
        .def_property_readonly("setting", &RgbdFrame3D::GetSetting)
        .def_property_readonly("image_height", &RgbdFrame3D::GetImageHeight)
        .def_property_readonly("image_width", &RgbdFrame3D::GetImageWidth)
        .def_property_readonly("camera_extrinsic_matrix", &RgbdFrame3D::GetCameraExtrinsicMatrix)
        .def_property_readonly("camera_intrinsic_matrix", &RgbdFrame3D::GetCameraIntrinsicMatrix)
        .def_property_readonly("is_partitioned", [](const RgbdFrame3D &self) { return self.IsPartitioned(); })
        .def_property_readonly("partitions", [](const RgbdFrame3D &self) { return self.GetPartitions(); });
}
