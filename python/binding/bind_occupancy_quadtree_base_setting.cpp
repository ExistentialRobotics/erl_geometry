#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_quadtree_base.hpp"

void
BindOccupancyQuadtreeBaseSetting(const py::module& m) {
    using namespace erl::geometry;
    py::class_<OccupancyQuadtreeBaseSetting, OccupancyNdTreeSetting, std::shared_ptr<OccupancyQuadtreeBaseSetting>>(m, "OccupancyQuadtreeBaseSetting")
        .def(py::init<>())
        .def_readwrite("use_change_detection", &OccupancyQuadtreeBaseSetting::use_change_detection)
        .def_readwrite("use_aabb_limit", &OccupancyQuadtreeBaseSetting::use_aabb_limit)
        .def_readwrite("aabb", &OccupancyQuadtreeBaseSetting::aabb);
}
