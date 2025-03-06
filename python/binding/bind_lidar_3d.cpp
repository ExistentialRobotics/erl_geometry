#include "erl_common/pybind11.hpp"
#include "erl_geometry/lidar_3d.hpp"

template<typename Dtype>
void
BindLidar3DImpl(const py::module &m, const char *name) {
    using namespace erl::geometry;
    using T = Lidar3D<Dtype>;

    auto py_lidar = py::class_<T, RangeSensor3D<Dtype>, std::shared_ptr<T>>(m, name);
    py::class_<typename T::Setting, erl::common::YamlableBase, std::shared_ptr<typename T::Setting>>(py_lidar, "Setting")
        .def(py::init<>())
        .def_readwrite("azimuth_min", &T::Setting::azimuth_min)
        .def_readwrite("azimuth_max", &T::Setting::azimuth_max)
        .def_readwrite("elevation_min", &T::Setting::elevation_min)
        .def_readwrite("elevation_max", &T::Setting::elevation_max)
        .def_readwrite("num_azimuth_lines", &T::Setting::num_azimuth_lines)
        .def_readwrite("num_elevation_lines", &T::Setting::num_elevation_lines);

    py_lidar.def(py::init<std::shared_ptr<typename T::Setting>>(), py::arg("setting").none(false))
        .def_property_readonly("setting", &T::GetSetting)
        .def_property_readonly("azimuth_angles", &T::GetAzimuthAngles)
        .def_property_readonly("elevation_angles", &T::GetElevationAngles);
}

void
BindLidar3D(const py::module &m) {
    BindLidar3DImpl<double>(m, "Lidar3Dd");
    BindLidar3DImpl<float>(m, "Lidar3Df");
}
