#include "erl_common/pybind11.hpp"
#include "erl_geometry/rgbd_camera_3d.hpp"

template<typename Dtype>
void
BindRgbdCamera3DImpl(const py::module &m, const char *name) {
    using namespace erl::common;
    using namespace erl::geometry;
    using T = RgbdCamera3D<Dtype>;

    py::class_<T, CameraBase3D<Dtype>, std::shared_ptr<T>>(m, name)
        .def_static("Setting", []() { return std::make_shared<typename T::Setting>(); })
        .def(py::init<std::shared_ptr<typename T::Setting>>(), py::arg("setting"))
        .def_property_readonly("setting", &T::GetSetting)
        .def("add_mesh", &T::AddMesh, py::arg("mesh_path"))
        .def(
            "scan",
            [](const T &self, const Eigen::Matrix3<Dtype> &orientation, const Eigen::Vector3<Dtype> &translation) {
                const auto [rgb, depth] = self.Scan(orientation, translation);
                py::dict result;
                result["rgb"] = rgb;
                result["depth"] = depth;
                return result;
            })
        .def_property_readonly("ray_directions_in_frame", [](const T &self) { return py::cast_to_array(self.GetRayDirectionsInFrame()); });
}

void
BindRgbdCamera3D(const py::module &m) {
    BindRgbdCamera3DImpl<double>(m, "RgbdCamera3Dd");
    BindRgbdCamera3DImpl<float>(m, "RgbdCamera3Df");
}
