#include "erl_common/pybind11.hpp"
#include "erl_geometry/camera_intrinsic.hpp"

void
BindCameraIntrinsic(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<CameraIntrinsic, YamlableBase, std::shared_ptr<CameraIntrinsic>>(m, "CameraIntrinsic")
        .def(py::init<>())
        .def_readwrite("image_height", &CameraIntrinsic::image_height)
        .def_readwrite("image_width", &CameraIntrinsic::image_width)
        .def_readwrite("camera_fx", &CameraIntrinsic::camera_fx)
        .def_readwrite("camera_fy", &CameraIntrinsic::camera_fy)
        .def_readwrite("camera_cx", &CameraIntrinsic::camera_cx)
        .def_readwrite("camera_cy", &CameraIntrinsic::camera_cy)
        .def_property_readonly("matrix", &CameraIntrinsic::GetIntrinsicMatrix)
        .def("resize", &CameraIntrinsic::Resize, py::arg("factor"))
        .def(
            "compute_frame_direction",
            [](const CameraIntrinsic &self, const long u, const long v) {
                double dir_x, dir_y, dir_z;
                self.ComputeFrameDirection(u, v, dir_x, dir_y, dir_z);
                return std::make_tuple(dir_x, dir_y, dir_z);
            },
            py::arg("u"),
            py::arg("v"))
        .def(
            "compute_frame_directions",
            [](const CameraIntrinsic &self) {
                Eigen::MatrixX<Eigen::Vector3d> dirs;
                self.ComputeFrameDirections(dirs);
                return py::cast_to_array(dirs);
            })
        .def(
            "convert_depth_to_distance",
            [](const CameraIntrinsic &self, const Eigen::MatrixXd &depth, const cv::Mat &rgb, const std::optional<Eigen::Matrix4d> &optical_pose) {
                std::vector<Eigen::Vector3d> points, colors;
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
