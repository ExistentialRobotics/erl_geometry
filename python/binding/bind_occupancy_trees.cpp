#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_nd_tree_setting.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/pybind11_occupancy_octree.hpp"
#include "erl_geometry/pybind11_occupancy_octree_drawer.hpp"
#include "erl_geometry/pybind11_occupancy_quadtree.hpp"
#include "erl_geometry/pybind11_occupancy_quadtree_drawer.hpp"

void
BindOccupancyTrees(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    // common tree settings
    py::class_<OccupancyNdTreeSetting, YamlableBase, std::shared_ptr<OccupancyNdTreeSetting>>(m, "OccupancyNdTreeSetting")
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
    py::class_<OccupancyQuadtreeBaseSetting, YamlableBase, std::shared_ptr<OccupancyQuadtreeBaseSetting>>(m, "OccupancyQuadtreeBaseSetting")
        .def(py::init<>())
        .def_readwrite("use_change_detection", &OccupancyQuadtreeBaseSetting::use_change_detection)
        .def_readwrite("use_aabb_limit", &OccupancyQuadtreeBaseSetting::use_aabb_limit)
        .def_readwrite("aabb", &OccupancyQuadtreeBaseSetting::aabb);
    py::class_<OccupancyOctreeBaseSetting, YamlableBase, std::shared_ptr<OccupancyOctreeBaseSetting>>(m, "OccupancyOctreeBaseSetting")
        .def(py::init<>())
        .def_readwrite("use_change_detection", &OccupancyOctreeBaseSetting::use_change_detection)
        .def_readwrite("use_aabb_limit", &OccupancyOctreeBaseSetting::use_aabb_limit)
        .def_readwrite("aabb", &OccupancyOctreeBaseSetting::aabb);

    // quadtree
    BindOccupancyQuadtree<OccupancyQuadtree, OccupancyQuadtreeNode>(m, "OccupancyQuadtree", "OccupancyQuadtreeNode");
    BindOccupancyQuadtreeDrawer<OccupancyQuadtreeDrawer<OccupancyQuadtree>, OccupancyQuadtree>(m, "OccupancyQuadtreeDrawer");

    BindOccupancyOctree<OccupancyOctree, OccupancyOctreeNode>(m, "OccupancyOctree", "OccupancyOctreeNode");
    BindOccupancyOctreeDrawer<OccupancyOctreeDrawer<OccupancyOctree>, OccupancyOctree>(m, "OccupancyOctreeDrawer");
}
