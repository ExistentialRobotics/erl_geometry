#include "erl_common/pybind11.hpp"
#include "erl_geometry/lidar_frame_3d.hpp"

template<typename Dtype>
void
BindLidarFrame3DImpl(const py::module &m, const char *name) {
    using namespace erl::common;
    using namespace erl::geometry;
    using T = LidarFrame3D<Dtype>;

    py::class_<T, RangeSensorFrame3D<Dtype>, std::shared_ptr<T>> lidar_frame(m, name);
    py::class_<
        typename T::Setting,
        typename RangeSensorFrame3D<Dtype>::Setting,
        std::shared_ptr<typename T::Setting>>(lidar_frame, "Setting")
        .def(py::init<>())
        .def_readwrite("azimuth_min", &T::Setting::azimuth_min)
        .def_readwrite("azimuth_max", &T::Setting::azimuth_max)
        .def_readwrite("elevation_min", &T::Setting::elevation_min)
        .def_readwrite("elevation_max", &T::Setting::elevation_max)
        .def_readwrite("num_azimuth_lines", &T::Setting::num_azimuth_lines)
        .def_readwrite("num_elevation_lines", &T::Setting::num_elevation_lines);
    lidar_frame
        .def(py::init<std::shared_ptr<typename T::Setting>>(), py::arg("setting").none(false))
        .def("reset", &T::Reset)
        .def(
            "update_ranges",
            &T::UpdateRanges,
            py::arg("rotation"),
            py::arg("translation"),
            py::arg("ranges"))
        .def_property_readonly("setting", &T::GetSetting)
        .def_property_readonly("num_azimuth_lines", &T::GetNumAzimuthLines)
        .def_property_readonly("num_elevation_lines", &T::GetNumElevationLines);
}

void
BindLidarFrame3D(const py::module &m) {
    BindLidarFrame3DImpl<double>(m, "LidarFrame3Dd");
    BindLidarFrame3DImpl<float>(m, "LidarFrame3Df");
}
