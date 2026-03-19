#pragma once

#include "erl_common/pybind11.hpp"
#include "erl_geometry/semi_sparse_octree_drawer.hpp"

template<class Tree>
void
BindSemiSparseOctreeDrawer(const py::handle &m, const char *name) {
    using namespace erl::common;
    using namespace erl::geometry;
    using Drawer = SemiSparseOctreeDrawer<Tree>;

    py::class_<Drawer>(m, name)
        .def_static("Setting", []() { return std::make_shared<typename Drawer::Setting>(); })
        .def(
            py::init<std::shared_ptr<typename Drawer::Setting>, std::shared_ptr<const Tree>>(),
            py::arg("setting"),
            py::arg("octree") = nullptr)
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
        .def(
            "draw_leaves",
            [](const Drawer &self) {
                std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries;
                self.DrawLeaves(geometries);
                return geometries;
            })
        .def(
            "draw_tree",
            py::overload_cast<const std::string &>(&AbstractOctreeDrawer::DrawTree, py::const_),
            py::arg("filename"))
        .def(
            "draw_leaves",
            py::overload_cast<const std::string &>(&AbstractOctreeDrawer::DrawLeaves, py::const_),
            py::arg("filename"));
}
