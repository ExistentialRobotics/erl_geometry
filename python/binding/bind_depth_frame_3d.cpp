#include "erl_common/pybind11.hpp"
#include "erl_geometry/depth_frame_3d.hpp"

void
BindDepthFrame3D(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;
    using T = DepthFrame3D;

    py::class_<T, RangeSensorFrame3D, std::shared_ptr<T>> depth_frame(m, "DepthFrame3D");
    py::class_<T::Setting, RangeSensorFrame3D::Setting, std::shared_ptr<T::Setting>>(depth_frame, "Setting")
        .def(py::init<>())
        .def_readwrite("camera_to_optical", &T::Setting::camera_to_optical)
        .def_readwrite("image_height", &T::Setting::image_height)
        .def_readwrite("image_width", &T::Setting::image_width)
        .def_readwrite("camera_fx", &T::Setting::camera_fx)
        .def_readwrite("camera_fy", &T::Setting::camera_fy)
        .def_readwrite("camera_cx", &T::Setting::camera_cx)
        .def_readwrite("camera_cy", &T::Setting::camera_cy)
        .def("resize", &T::Setting::Resize, py::arg("factor"));
    depth_frame.def(py::init<std::shared_ptr<T::Setting>>(), py::arg("setting").none(false))
        .def("reset", [](T &self) { self.Reset(); })
        .def(
            "update_ranges",
            py::overload_cast<const Eigen::Ref<const Eigen::Matrix3d> &, const Eigen::Ref<const Eigen::Vector3d> &, Eigen::MatrixXd, bool>(&T::UpdateRanges),
            py::arg("rotation"),
            py::arg("translation"),
            py::arg("depth"),
            py::arg("partition_rays"))
        .def(
            "update_ranges",
            py::overload_cast<const Eigen::Ref<const Eigen::Matrix3d> &, const Eigen::Ref<const Eigen::Vector3d> &, const std::string &, double, bool>(
                &T::UpdateRanges),
            py::arg("rotation"),
            py::arg("translation"),
            py::arg("depth_file"),
            py::arg("depth_scale"),
            py::arg("partition_rays") = false)
        .def_static("depth_image_to_depth", &T::DepthImageToDepth, py::arg("depth_image"), py::arg("depth_scale"))
        .def_static("depth_to_depth_image", &T::DepthToDepthImage, py::arg("depth"), py::arg("depth_scale"))
        .def_property_readonly("setting", &T::GetSetting)
        .def_property_readonly("image_height", &T::GetImageHeight)
        .def_property_readonly("image_width", &T::GetImageWidth)
        .def_property_readonly("camera_extrinsic_matrix", &T::GetCameraExtrinsicMatrix)
        .def_property_readonly("camera_intrinsic_matrix", &T::GetCameraIntrinsicMatrix)
        .def_property_readonly("is_partitioned", [](const T &self) { return self.IsPartitioned(); })
        .def_property_readonly("partitions", [](const T &self) { return self.GetPartitions(); });
}
