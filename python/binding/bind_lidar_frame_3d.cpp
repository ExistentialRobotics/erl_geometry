#include "erl_common/pybind11.hpp"
#include "erl_geometry/lidar_frame_3d.hpp"

void
BindLidarFrame3D(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;
    using T = LidarFrame3D;

    py::class_<T, RangeSensorFrame3D, std::shared_ptr<T>> lidar_frame(m, "LidarFrame3D");
    py::class_<T::Setting, RangeSensorFrame3D::Setting, std::shared_ptr<T::Setting>>(lidar_frame, "Setting")
        .def(py::init<>())
        .def_readwrite("azimuth_min", &T::Setting::azimuth_min)
        .def_readwrite("azimuth_max", &T::Setting::azimuth_max)
        .def_readwrite("elevation_min", &T::Setting::elevation_min)
        .def_readwrite("elevation_max", &T::Setting::elevation_max)
        .def_readwrite("num_azimuth_lines", &T::Setting::num_azimuth_lines)
        .def_readwrite("num_elevation_lines", &T::Setting::num_elevation_lines);
    lidar_frame.def(py::init<std::shared_ptr<T::Setting>>(), py::arg("setting").none(false))
        .def("reset", &T::Reset)
        .def("update_ranges", &T::UpdateRanges, py::arg("rotation"), py::arg("translation"), py::arg("ranges"), py::arg("partition_rays"))
        .def_property_readonly("setting", &T::GetSetting)
        .def_property_readonly("num_azimuth_lines", &T::GetNumAzimuthLines)
        .def_property_readonly("num_elevation_lines", &T::GetNumElevationLines)
        .def_property_readonly("is_partitioned", &T::IsPartitioned)
        .def_property_readonly("partitions", &T::GetPartitions);
}
