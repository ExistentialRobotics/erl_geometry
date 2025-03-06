#include "erl_common/pybind11.hpp"
#include "erl_geometry/camera_intrinsic.hpp"

template<typename Dtype>
void
BindCameraIntrinsicImpl(const py::module &m, const char *name) {
    using namespace erl::common;
    using namespace erl::geometry;
    using T = CameraIntrinsic<Dtype>;

    py::class_<T, YamlableBase, std::shared_ptr<T>>(m, name)
        .def(py::init<>())
        .def_readwrite("image_height", &T::image_height)
        .def_readwrite("image_width", &T::image_width)
        .def_readwrite("camera_fx", &T::camera_fx)
        .def_readwrite("camera_fy", &T::camera_fy)
        .def_readwrite("camera_cx", &T::camera_cx)
        .def_readwrite("camera_cy", &T::camera_cy)
        .def_property_readonly("matrix", &T::GetIntrinsicMatrix)
        .def("resize", &T::Resize, py::arg("factor"))
        .def(
            "compute_frame_direction",
            [](const T &self, const long u, const long v) {
                Dtype dir_x, dir_y, dir_z;
                self.ComputeFrameDirection(u, v, dir_x, dir_y, dir_z);
                return std::make_tuple(dir_x, dir_y, dir_z);
            },
            py::arg("u"),
            py::arg("v"))
        .def(
            "compute_frame_directions",
            [](const T &self) {
                Eigen::MatrixX<Eigen::Vector3<Dtype>> dirs;
                self.ComputeFrameDirections(dirs);
                return py::cast_to_array(dirs);
            })
        .def(
            "convert_depth_to_distance",
            [](const T &self, const Eigen::MatrixX<Dtype> &depth, const cv::Mat &rgb, const std::optional<Eigen::Matrix4<Dtype>> &optical_pose) {
                std::vector<Eigen::Vector3<Dtype>> points, colors;
                self.ConvertRgbdToPointCloud(depth, rgb, optical_pose, points, colors);
                py::dict out;
                out["points"] = points;
                out["colors"] = colors;
                return out;
            },
            py::arg("depth"),
            py::arg("rgb"),
            py::arg("optical_pose") = std::nullopt);
}

void
BindCameraIntrinsic(const py::module &m) {
    BindCameraIntrinsicImpl<double>(m, "CameraIntrinsicD");
    BindCameraIntrinsicImpl<float>(m, "CameraIntrinsicF");
}
