#include "erl_common/pybind11.hpp"
#include "erl_geometry/camera_base_3d.hpp"

void
BindCameraBase3D(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<CameraBase3D, std::shared_ptr<CameraBase3D>>(m, "CameraBase3D")
        .def_readonly_static("cTo", &CameraBase3D::cTo)
        .def_readonly_static("oTc", &CameraBase3D::oTc)
        .def_static("compute_optical_pose", &CameraBase3D::ComputeOpticalPose, py::arg("orientation"), py::arg("translation"))
        .def_static("compute_camera_pose", &CameraBase3D::ComputeCameraPose, py::arg("orientation"), py::arg("translation"))
        .def_static("compute_extrinsic", &CameraBase3D::ComputeExtrinsic, py::arg("camera_orientation"), py::arg("camera_translation"));
}
