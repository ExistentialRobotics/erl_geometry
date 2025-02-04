#include "erl_common/pybind11.hpp"
#include "erl_geometry/lidar_3d.hpp"

void
BindLidar3D(const py::module &m) {
    using namespace erl::geometry;

    auto py_lidar = py::class_<Lidar3D, RangeSensor3D, std::shared_ptr<Lidar3D>>(m, "Lidar3D");
    py::class_<Lidar3D::Setting, erl::common::YamlableBase, std::shared_ptr<Lidar3D::Setting>>(py_lidar, "Setting")
        .def(py::init<>())
        .def_readwrite("azimuth_min", &Lidar3D::Setting::azimuth_min)
        .def_readwrite("azimuth_max", &Lidar3D::Setting::azimuth_max)
        .def_readwrite("elevation_min", &Lidar3D::Setting::elevation_min)
        .def_readwrite("elevation_max", &Lidar3D::Setting::elevation_max)
        .def_readwrite("num_azimuth_lines", &Lidar3D::Setting::num_azimuth_lines)
        .def_readwrite("num_elevation_lines", &Lidar3D::Setting::num_elevation_lines);

    py_lidar.def(py::init<std::shared_ptr<Lidar3D::Setting>>(), py::arg("setting").none(false))
        .def_property_readonly("setting", &Lidar3D::GetSetting)
        .def_property_readonly("azimuth_angles", &Lidar3D::GetAzimuthAngles)
        .def_property_readonly("elevation_angles", &Lidar3D::GetElevationAngles);
}
