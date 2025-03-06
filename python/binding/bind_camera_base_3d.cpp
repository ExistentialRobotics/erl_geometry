#include "erl_common/pybind11.hpp"
#include "erl_geometry/camera_base_3d.hpp"

template<typename Dtype>
void
BindCameraBase3DImpl(const py::module &m, const char *name) {
    using namespace erl::common;
    using namespace erl::geometry;
    using T = CameraBase3D<Dtype>;

    py::class_<T, std::shared_ptr<T>>(m, name)
        .def_readonly_static("cTo", &T::cTo)
        .def_readonly_static("oTc", &T::oTc)
        .def_static("compute_optical_pose", &T::ComputeOpticalPose, py::arg("orientation"), py::arg("translation"))
        .def_static("compute_camera_pose", &T::ComputeCameraPose, py::arg("orientation"), py::arg("translation"))
        .def_static("compute_extrinsic", &T::ComputeExtrinsic, py::arg("camera_orientation"), py::arg("camera_translation"));
}

void
BindCameraBase3D(const py::module &m) {
    BindCameraBase3DImpl<double>(m, "CameraBase3Dd");
    BindCameraBase3DImpl<float>(m, "CameraBase3Df");
}
