#include "erl_common/pybind11.hpp"
#include "erl_geometry/lidar_frame_3d.hpp"

void
BindLidarFrame3D(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<LidarFrame3D, RangeSensorFrame3D, std::shared_ptr<LidarFrame3D>> lidar_frame(m, "LidarFrame3D");
    py::class_<LidarFrame3D::Setting, YamlableBase, std::shared_ptr<LidarFrame3D::Setting>>(lidar_frame, "Setting")
        .def(py::init<>())
        .def_readwrite("azimuth_min", &LidarFrame3D::Setting::azimuth_min)
        .def_readwrite("azimuth_max", &LidarFrame3D::Setting::azimuth_max)
        .def_readwrite("elevation_min", &LidarFrame3D::Setting::elevation_min)
        .def_readwrite("elevation_max", &LidarFrame3D::Setting::elevation_max)
        .def_readwrite("num_azimuth_lines", &LidarFrame3D::Setting::num_azimuth_lines)
        .def_readwrite("num_elevation_lines", &LidarFrame3D::Setting::num_elevation_lines);
    lidar_frame.def(py::init<std::shared_ptr<LidarFrame3D::Setting>>(), py::arg("setting").none(false))
        .def("reset", &LidarFrame3D::Reset)
        .def("update_ranges", &LidarFrame3D::UpdateRanges, py::arg("rotation"), py::arg("translation"), py::arg("ranges"), py::arg("partition_rays"))
        .def_property_readonly("setting", &LidarFrame3D::GetSetting)
        .def_property_readonly("num_azimuth_lines", &LidarFrame3D::GetNumAzimuthLines)
        .def_property_readonly("num_elevation_lines", &LidarFrame3D::GetNumElevationLines)
        .def_property_readonly("is_partitioned", &LidarFrame3D::IsPartitioned)
        .def_property_readonly("partitions", &LidarFrame3D::GetPartitions);
}
