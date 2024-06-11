#pragma once

#include "erl_common/pybind11.hpp"
#include "erl_common/yaml.hpp"

template<class Drawer, class Tree>
void
BindOccupancyQuadtreeDrawer(const py::handle &m, const char *name) {
    using namespace erl::common;

    py::class_<Drawer> drawer(m, name);
    py::class_<typename Drawer::Setting, YamlableBase, std::shared_ptr<typename Drawer::Setting>>(drawer, "Setting")
        .def(py::init<>([]() { return std::make_shared<typename Drawer::Setting>(); }))
        .def_readwrite("area_min", &Drawer::Setting::area_min)
        .def_readwrite("area_max", &Drawer::Setting::area_max)
        .def_readwrite("resolution", &Drawer::Setting::resolution)
        .def_readwrite("padding", &Drawer::Setting::padding)
        .def_readwrite("bg_color", &Drawer::Setting::bg_color)
        .def_readwrite("fg_color", &Drawer::Setting::fg_color)
        .def_readwrite("border_color", &Drawer::Setting::border_color)
        .def_readwrite("border_thickness", &Drawer::Setting::border_thickness)
        .def_readwrite("occupied_color", &Drawer::Setting::occupied_color)
        .def_readwrite("free_color", &Drawer::Setting::free_color);
    drawer.def(py::init<std::shared_ptr<typename Drawer::Setting>, std::shared_ptr<const Tree>>(), py::arg("setting"), py::arg("quadtree") = nullptr)
        .def_property_readonly("setting", &Drawer::GetSetting)
        .def_property_readonly("grid_map_info", &Drawer::GetGridMapInfo)
        .def("set_draw_tree_callback", &Drawer::SetDrawTreeCallback, py::arg("callback"))
        .def("set_draw_leaf_callback", &Drawer::SetDrawLeafCallback, py::arg("callback"))
        .def(
            "draw_tree",
            [](const Drawer &self) {
                cv::Mat mat;
                self.DrawTree(mat);
                return mat;
            })
        .def("draw_leaves", [](const Drawer &self) {
            cv::Mat mat;
            self.DrawLeaves(mat);
            return mat;
        });
}
