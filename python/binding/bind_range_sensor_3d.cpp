#include "erl_common/pybind11.hpp"
#include "erl_geometry/range_sensor_3d.hpp"

template<typename Dtype>
void
BindRangeSensor3DImpl(const py::module &m, const char *name) {
    using namespace erl::geometry;
    using T = RangeSensor3D<Dtype>;

    py::class_<T, std::shared_ptr<T>>(m, name)
        .def_property_readonly(
            "ray_directions_in_frame",
            [](const T &self) -> py::array_t<double> { return py::cast_to_array(self.GetRayDirectionsInFrame()); })
        .def("add_mesh", py::overload_cast<const std::string &>(&T::AddMesh, py::const_), py::arg("mesh_path"))
        .def(
            "add_mesh",
            py::overload_cast<const Eigen::Ref<const Eigen::Matrix3X<Dtype>> &, const Eigen::Ref<const Eigen::Matrix3Xi> &>(&T::AddMesh, py::const_),
            py::arg("vertices"),
            py::arg("triangles"))
        .def(
            "add_mesh",
            py::overload_cast<const std::vector<Eigen::Vector3<Dtype>> &, const std::vector<Eigen::Vector3i> &>(&T::AddMesh, py::const_),
            py::arg("vertices"),
            py::arg("triangles"))
        .def(
            "scan",
            &T::Scan,
            py::arg("orientation"),
            py::arg("translation"),
            py::arg("add_noise") = false,
            py::arg("noise_stddev") = 0.03,
            py::arg("cache_normals") = false)
        .def_property_readonly("optical_pose", &T::GetOpticalPose)
        .def_property_readonly("cached_normals", [](const T &self) -> py::array_t<double> { return py::cast_to_array(self.GetCachedNormals()); });
}

void
BindRangeSensor3D(const py::module &m) {
    BindRangeSensor3DImpl<double>(m, "RangeSensor3Dd");
    BindRangeSensor3DImpl<float>(m, "RangeSensor3Df");
}
