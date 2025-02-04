#include "erl_common/pybind11.hpp"
#include "erl_geometry/range_sensor_3d.hpp"

void
BindRangeSensor3D(const py::module &m) {
    using namespace erl::geometry;

    py::class_<RangeSensor3D, std::shared_ptr<RangeSensor3D>>(m, "RangeSensor3D")
        .def_property_readonly(
            "ray_directions_in_frame",
            [](const RangeSensor3D &self) -> py::array_t<double> { return py::cast_to_array(self.GetRayDirectionsInFrame()); })
        .def("add_mesh", py::overload_cast<const std::string &>(&RangeSensor3D::AddMesh, py::const_), py::arg("mesh_path"))
        .def(
            "add_mesh",
            py::overload_cast<const Eigen::Ref<const Eigen::Matrix3Xd> &, const Eigen::Ref<const Eigen::Matrix3Xi> &>(&RangeSensor3D::AddMesh, py::const_),
            py::arg("vertices"),
            py::arg("triangles"))
        .def(
            "add_mesh",
            py::overload_cast<const std::vector<Eigen::Vector3d> &, const std::vector<Eigen::Vector3i> &>(&RangeSensor3D::AddMesh, py::const_),
            py::arg("vertices"),
            py::arg("triangles"))
        .def(
            "scan",
            &RangeSensor3D::Scan,
            py::arg("orientation"),
            py::arg("translation"),
            py::arg("add_noise") = false,
            py::arg("noise_stddev") = 0.03,
            py::arg("cache_normals") = false)
        .def_property_readonly("optical_pose", &RangeSensor3D::GetOpticalPose)
        .def_property_readonly("cached_normals", [](const RangeSensor3D &self) -> py::array_t<double> { return py::cast_to_array(self.GetCachedNormals()); });
    ;
}
