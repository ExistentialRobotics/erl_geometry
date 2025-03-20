#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"

template<typename Dtype>
void
BindOccupancyQuadtreeDrawerSettingImpl(const py::module &m, const char *name) {
    using namespace erl::common;
    using namespace erl::geometry;
    using T = OccupancyQuadtreeDrawerSetting<Dtype>;
    py::class_<T, YamlableBase, std::shared_ptr<T>>(m, name)
        .def(py::init<>([]() { return std::make_shared<T>(); }))
        .def_readwrite("padding", &T::padding)
        .def_readwrite("bg_color", &T::bg_color)
        .def_readwrite("fg_color", &T::fg_color)
        .def_readwrite("border_color", &T::border_color)
        .def_readwrite("border_thickness", &T::border_thickness)
        .def_readwrite("area_min", &T::area_min)
        .def_readwrite("area_max", &T::area_max)
        .def_readwrite("resolution", &T::resolution)
        .def_readwrite("scaling", &T::scaling)
        .def_readwrite("occupied_color", &T::occupied_color)
        .def_readwrite("free_color", &T::free_color);
}

void
BindOccupancyQuadtreeDrawerSetting(const py::module &m) {
    BindOccupancyQuadtreeDrawerSettingImpl<double>(m, "OccupancyQuadtreeDrawerSettingD");
    BindOccupancyQuadtreeDrawerSettingImpl<float>(m, "OccupancyQuadtreeDrawerSettingF");
}
