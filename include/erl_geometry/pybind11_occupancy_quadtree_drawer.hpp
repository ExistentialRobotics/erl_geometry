#pragma once

#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"

template<class Tree>
void
BindOccupancyQuadtreeDrawer(const py::handle &m, const char *name) {
    using namespace erl::common;
    using namespace erl::geometry;
    using Drawer = OccupancyQuadtreeDrawer<Tree>;

    py::class_<Drawer>(m, name)
        .def_static("Setting", []() { return std::make_shared<typename Drawer::Setting>(); })
        .def(
            py::init<std::shared_ptr<typename Drawer::Setting>, std::shared_ptr<const Tree>>(),
            py::arg("setting"),
            py::arg("quadtree") = nullptr)
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
        .def(
            "draw_leaves",
            [](const Drawer &self) {
                cv::Mat mat;
                self.DrawLeaves(mat);
                return mat;
            })
        .def(
            "draw_tree",
            py::overload_cast<const std::string &>(&Drawer::DrawTree, py::const_),
            py::arg("filename"))
        .def(
            "draw_leaves",
            py::overload_cast<const std::string &>(&Drawer::DrawLeaves, py::const_),
            py::arg("filename"));
    ;
}
