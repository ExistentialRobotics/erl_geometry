#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_nd_tree_setting.hpp"

void
BindOccupancyNdTreeSetting(const py::module& m) {
    using namespace erl::geometry;
    py::class_<OccupancyNdTreeSetting, NdTreeSetting, std::shared_ptr<OccupancyNdTreeSetting>>(m, "OccupancyNdTreeSetting")
        .def(py::init<>())
        .def_readwrite("log_odd_min", &OccupancyNdTreeSetting::log_odd_min)
        .def_readwrite("log_odd_max", &OccupancyNdTreeSetting::log_odd_max)
        .def_readwrite("log_odd_hit", &OccupancyNdTreeSetting::log_odd_hit)
        .def_readwrite("log_odd_miss", &OccupancyNdTreeSetting::log_odd_miss)
        .def_readwrite("log_odd_occ_threshold", &OccupancyNdTreeSetting::log_odd_occ_threshold)
        .def_property("probability_hit", &OccupancyNdTreeSetting::GetProbabilityHit, &OccupancyNdTreeSetting::SetProbabilityHit)
        .def_property("probability_miss", &OccupancyNdTreeSetting::GetProbabilityMiss, &OccupancyNdTreeSetting::SetProbabilityMiss)
        .def_property(
            "probability_occupied_threshold",
            &OccupancyNdTreeSetting::GetProbabilityOccupiedThreshold,
            &OccupancyNdTreeSetting::SetProbabilityOccupiedThreshold);
}
