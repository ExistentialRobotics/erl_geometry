#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_octree_base.hpp"

void
BindOccupancyOctreeBaseSetting(const py::module& m) {
    using namespace erl::geometry;
    py::class_<OccupancyOctreeBaseSetting, OccupancyNdTreeSetting, std::shared_ptr<OccupancyOctreeBaseSetting>>(m, "OccupancyOctreeBaseSetting")
        .def(py::init<>())
        .def_readwrite("use_change_detection", &OccupancyOctreeBaseSetting::use_change_detection)
        .def_readwrite("use_aabb_limit", &OccupancyOctreeBaseSetting::use_aabb_limit)
        .def_readwrite("aabb", &OccupancyOctreeBaseSetting::aabb);
}
