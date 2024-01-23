#pragma once

#include "erl_common/pybind11.hpp"
#include "erl_common/pybind11_opencv.hpp"
#include "erl_common/yaml.hpp"

template<class Drawer, class Tree>
void
BindOccupancyOctreeDrawer(py::module &m, const char *name) {
    using namespace erl::common;

    py::class_<Drawer> drawer(m, name);
    py::class_<typename Drawer::Setting, YamlableBase, std::shared_ptr<typename Drawer::Setting>>(drawer, "Setting")
        .def(py::init<>([]() { return std::make_shared<typename Drawer::Setting>(); }))
        .def_readwrite("area_min", &Drawer::Setting::area_min)
        .def_readwrite("area_max", &Drawer::Setting::area_max)
        .def_readwrite("bg_color", &Drawer::Setting::bg_color)
        .def_readwrite("border_color", &Drawer::Setting::border_color)
        .def_readwrite("occupied_color", &Drawer::Setting::occupied_color)
        .def_readwrite("free_color", &Drawer::Setting::free_color);
    drawer.def(py::init<std::shared_ptr<typename Drawer::Setting>, std::shared_ptr<const Tree>>(), py::arg("setting"), py::arg("octree") = nullptr);
}
