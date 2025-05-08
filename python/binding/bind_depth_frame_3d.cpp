#include "erl_common/pybind11.hpp"
#include "erl_geometry/depth_frame_3d.hpp"

template<typename Dtype>
void
BindDepthFrame3DImpl(const py::module &m, const char *name) {
    using namespace erl::common;
    using namespace erl::geometry;
    using T = DepthFrame3D<Dtype>;
    using Matrix3 = Eigen::Matrix3<Dtype>;
    using MatrixX = Eigen::MatrixX<Dtype>;
    using Vector3 = Eigen::Vector3<Dtype>;

    py::class_<T, RangeSensorFrame3D<Dtype>, std::shared_ptr<T>> depth_frame(m, name);
    py::class_<
        typename T::Setting,
        typename RangeSensorFrame3D<Dtype>::Setting,
        std::shared_ptr<typename T::Setting>>(depth_frame, "Setting")
        .def(py::init<>())
        .def_readwrite("camera_intrinsic", &T::Setting::camera_intrinsic);
    depth_frame
        .def(py::init<std::shared_ptr<typename T::Setting>>(), py::arg("setting").none(false))
        .def("reset", [](T &self) { self.Reset(); })
        .def(
            "update_ranges",
            py::overload_cast<
                const Eigen::Ref<const Matrix3> &,
                const Eigen::Ref<const Vector3> &,
                MatrixX>(&T::UpdateRanges),
            py::arg("rotation"),
            py::arg("translation"),
            py::arg("depth"))
        .def(
            "update_ranges",
            py::overload_cast<
                const Eigen::Ref<const Matrix3> &,
                const Eigen::Ref<const Vector3> &,
                const std::string &,
                double>(&T::UpdateRanges),
            py::arg("rotation"),
            py::arg("translation"),
            py::arg("depth_file"),
            py::arg("depth_scale"))
        .def_static(
            "depth_image_to_depth",
            &T::DepthImageToDepth,
            py::arg("depth_image"),
            py::arg("depth_scale"))
        .def_static(
            "depth_to_depth_image",
            &T::DepthToDepthImage,
            py::arg("depth"),
            py::arg("depth_scale"))
        .def_property_readonly("setting", &T::GetSetting)
        .def_property_readonly("image_height", &T::GetImageHeight)
        .def_property_readonly("image_width", &T::GetImageWidth);
}

void
BindDepthFrame3D(const py::module &m) {
    BindDepthFrame3DImpl<double>(m, "DepthFrame3Dd");
    BindDepthFrame3DImpl<float>(m, "DepthFrame3Df");
}
