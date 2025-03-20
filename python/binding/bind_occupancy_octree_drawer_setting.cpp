#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_octree_drawer.hpp"

void
BindOccupancyOctreeDrawerSetting(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;
    using T = OccupancyOctreeDrawerSetting;
    py::class_<T, YamlableBase, std::shared_ptr<T>>(m, "OccupancyOctreeDrawerSetting")
        .def(py::init<>([]() { return std::make_shared<T>(); }))
        .def_readwrite("area_min", &T::area_min)
        .def_readwrite("area_max", &T::area_max)
        .def_readwrite("border_color", &T::border_color)
        .def_readwrite("occupied_only", &T::occupied_only)
        .def_readwrite("occupied_color", &T::occupied_color)
        .def_readwrite("draw_node_boxes", &T::draw_node_boxes)
        .def_readwrite("draw_node_borders", &T::draw_node_borders);
}
