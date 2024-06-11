#include "erl_common/pybind11.hpp"
#include "erl_geometry/lidar_3d.hpp"

void
BindLidar3D(const py::module &m) {
    using namespace erl::geometry;

    auto py_lidar = py::class_<Lidar3D, std::shared_ptr<Lidar3D>>(m, "Lidar3D");
    py::class_<Lidar3D::Setting, erl::common::YamlableBase, std::shared_ptr<Lidar3D::Setting>>(py_lidar, "Setting")
        .def(py::init<>())
        .def_readwrite("azimuth_min", &Lidar3D::Setting::azimuth_min)
        .def_readwrite("azimuth_max", &Lidar3D::Setting::azimuth_max)
        .def_readwrite("elevation_min", &Lidar3D::Setting::elevation_min)
        .def_readwrite("elevation_max", &Lidar3D::Setting::elevation_max)
        .def_readwrite("num_azimuth_lines", &Lidar3D::Setting::num_azimuth_lines)
        .def_readwrite("num_elevation_lines", &Lidar3D::Setting::num_elevation_lines);

    py_lidar
        .def(
            py::init<std::shared_ptr<Lidar3D::Setting>, const Eigen::Ref<const Eigen::Matrix3Xd> &, const Eigen::Ref<const Eigen::Matrix3Xi> &>(),
            py::arg("setting").none(false),
            py::arg("vertices"),
            py::arg("triangles"))
        .def(
            py::init<std::shared_ptr<Lidar3D::Setting>, const std::vector<Eigen::Vector3d> &, const std::vector<Eigen::Vector3i> &>(),
            py::arg("setting").none(false),
            py::arg("vertices"),
            py::arg("triangles"))
        .def_property_readonly("setting", &Lidar3D::GetSetting)
        .def_property_readonly("azimuth_angles", &Lidar3D::GetAzimuthAngles)
        .def_property_readonly("elevation_angles", &Lidar3D::GetElevationAngles)
        .def_property_readonly(
            "ray_directions_in_frame",
            [](const Lidar3D &self) -> py::array_t<double> {
                Eigen::MatrixX<Eigen::Vector3d> rays = self.GetRayDirectionsInFrame();
                const long n_azimuths = rays.rows();
                const long n_elevations = rays.cols();
                py::array_t<double> out({n_azimuths, n_elevations, 3l});
                for (long i = 0; i < n_azimuths; ++i) {
                    for (long j = 0; j < n_elevations; ++j) {
                        const auto &dir = rays(i, j);
                        out.mutable_at(i, j, 0) = dir[0];
                        out.mutable_at(i, j, 1) = dir[1];
                        out.mutable_at(i, j, 2) = dir[2];
                    }
                }
                return out;
            })
        .def("scan", &Lidar3D::Scan, py::arg("orientation"), py::arg("translation"), py::arg("add_noise") = false, py::arg("noise_stddev") = 0.03);
}
