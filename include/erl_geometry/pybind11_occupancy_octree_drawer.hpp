#pragma once

#include "erl_common/pybind11.hpp"
#include "erl_common/yaml.hpp"

template<class Drawer, class Tree>
void
BindOccupancyOctreeDrawer(const py::handle &m, const char *name) {
    using namespace erl::common;

    py::class_<Drawer> drawer(m, name);
    py::class_<typename Drawer::Setting, YamlableBase, std::shared_ptr<typename Drawer::Setting>>(drawer, "Setting")
        .def(py::init<>([]() { return std::make_shared<typename Drawer::Setting>(); }))
        .def_readwrite("area_min", &Drawer::Setting::area_min)
        .def_readwrite("area_max", &Drawer::Setting::area_max)
        .def_readwrite("border_color", &Drawer::Setting::border_color)
        .def_readwrite("occupied_color", &Drawer::Setting::occupied_color);
    drawer.def(py::init<std::shared_ptr<typename Drawer::Setting>, std::shared_ptr<const Tree>>(), py::arg("setting"), py::arg("octree") = nullptr)
        .def_property_readonly("setting", &Drawer::GetSetting)
        .def("set_draw_tree_callback", &Drawer::SetDrawTreeCallback, py::arg("callback"))
        .def("set_draw_leaf_callback", &Drawer::SetDrawLeafCallback, py::arg("callback"))
        .def(
            "draw_tree",
            [](const Drawer &self) {
                std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries;
                self.DrawTree(geometries);
                return geometries;
            })
        .def("draw_leaves", [](const Drawer &self) {
            std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries;
            self.DrawLeaves(geometries);
            return geometries;
        });
}
