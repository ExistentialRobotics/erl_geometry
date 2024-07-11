#include "erl_common/pybind11.hpp"
#include "erl_geometry/depth_frame_3d.hpp"

void
BindDepthFrame3D(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<DepthFrame3D, RangeSensorFrame3D, std::shared_ptr<DepthFrame3D>> depth_frame(m, "DepthFrame3D");
    py::class_<DepthFrame3D::Setting, RangeSensorFrame3D::Setting, std::shared_ptr<DepthFrame3D::Setting>>(depth_frame, "Setting")
        .def(py::init<>())
        .def_readwrite("camera_to_optical", &DepthFrame3D::Setting::camera_to_optical)
        .def_readwrite("image_height", &DepthFrame3D::Setting::image_height)
        .def_readwrite("image_width", &DepthFrame3D::Setting::image_width)
        .def_readwrite("camera_fx", &DepthFrame3D::Setting::camera_fx)
        .def_readwrite("camera_fy", &DepthFrame3D::Setting::camera_fy)
        .def_readwrite("camera_cx", &DepthFrame3D::Setting::camera_cx)
        .def_readwrite("camera_cy", &DepthFrame3D::Setting::camera_cy)
        .def_readwrite("depth_scale", &DepthFrame3D::Setting::depth_scale)
        .def("resize", &DepthFrame3D::Setting::Resize, py::arg("factor"));
    depth_frame.def(py::init<std::shared_ptr<DepthFrame3D::Setting>>(), py::arg("setting").none(false))
        .def_property_readonly_static("frame_type", &DepthFrame3D::GetFrameType)
        .def("reset", [](DepthFrame3D &self) { self.Reset(); })
        .def(
            "update_ranges",
            py::overload_cast<const Eigen::Ref<const Eigen::Matrix3d> &, const Eigen::Ref<const Eigen::Vector3d> &, Eigen::MatrixXd, bool>(
                &DepthFrame3D::UpdateRanges),
            py::arg("rotation"),
            py::arg("translation"),
            py::arg("depth"),
            py::arg("partition_rays"))
        .def(
            "update_ranges",
            py::overload_cast<const Eigen::Ref<const Eigen::Matrix3d> &, const Eigen::Ref<const Eigen::Vector3d> &, const std::string &, bool>(
                &DepthFrame3D::UpdateRanges),
            py::arg("rotation"),
            py::arg("translation"),
            py::arg("depth_file"),
            py::arg("partition_rays") = false)
        .def_property_readonly("setting", &DepthFrame3D::GetSetting)
        .def_property_readonly("image_height", &DepthFrame3D::GetImageHeight)
        .def_property_readonly("image_width", &DepthFrame3D::GetImageWidth)
        .def_property_readonly("camera_extrinsic_matrix", &DepthFrame3D::GetCameraExtrinsicMatrix)
        .def_property_readonly("camera_intrinsic_matrix", &DepthFrame3D::GetCameraIntrinsicMatrix)
        .def_property_readonly("is_partitioned", [](const DepthFrame3D &self) { return self.IsPartitioned(); })
        .def_property_readonly("partitions", [](const DepthFrame3D &self) { return self.GetPartitions(); });
}
