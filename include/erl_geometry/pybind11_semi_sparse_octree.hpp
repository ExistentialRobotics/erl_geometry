#pragma once

#include "open3d_visualizer_wrapper.hpp"
#include "pybind11_octree_impl.hpp"
#include "pybind11_semi_sparse_octree_drawer.hpp"
#include "semi_sparse_nd_tree_setting.hpp"
#include "semi_sparse_octree_base.hpp"

template<class Octree, class Node>
auto
BindSemiSparseOctree(
    const py::module &m,
    const char *tree_name,
    std::function<void(py::class_<
                       Octree,
                       erl::geometry::AbstractOctree<typename Octree::DataType>,
                       std::shared_ptr<Octree>> &)> additional_bindings = nullptr) {

    using namespace erl::common;
    using namespace erl::geometry;
    using Dtype = typename Octree::DataType;
    using Matrix3X = Eigen::Matrix3X<Dtype>;

    py::class_<Octree, AbstractOctree<Dtype>, std::shared_ptr<Octree>> tree(m, tree_name);

    if (additional_bindings) { additional_bindings(tree); }
    if (std::is_same_v<typename Octree::Setting, SemiSparseNdTreeSetting> &&
        !py::hasattr(tree, "Setting")) {
        ERL_DEBUG("Bind default Setting type to {}", tree_name);
        tree.def("Setting", []() { return std::make_shared<typename Octree::Setting>(); });
    }

    // AbstractOctree methods are defined in bind_abstract_octree.cpp

    // SemiSparseOctree
    tree.def(py::init<>())
        .def(
            py::init<>([](const std::shared_ptr<typename Octree::Setting> &setting) {
                return std::make_shared<Octree>(setting);
            }),
            py::arg("setting"))
        .def(
            py::init<>(
                [](const std::string &filename) { return std::make_shared<Octree>(filename); }),
            py::arg("filename"))
        .def_property_readonly("setting", &Octree::template GetSetting<typename Octree::Setting>)
        .def_property_readonly("parents", &Octree::GetParents)
        .def_property_readonly("children", &Octree::GetChildren)
        .def_property_readonly("voxels", &Octree::GetVoxels)
        .def_property_readonly("vertices", &Octree::GetVertices)
        .def_property_readonly("num_vertices", &Octree::GetVertexCount)
        .def_property_readonly("vertex_keys", &Octree::GetVertexKeys)
        .def(
            "insert_points",
            py::overload_cast<const Matrix3X &>(&Octree::InsertPoints),
            py::arg("points"))
        .def("insert_point", &Octree::InsertPoint, py::arg("key"), py::arg("max_depth"))
        .def(
            "find_voxel_indices",
            py::overload_cast<const Matrix3X &, bool>(&Octree::FindVoxelIndices, py::const_),
            py::arg("points"),
            py::arg("parallel"))
        .def("find_voxel_index", &Octree::FindVoxelIndex, py::arg("key"));

    BindOctreeImpl<decltype(tree), Dtype, Octree, Node>(tree);
    BindSemiSparseOctreeDrawer<Octree>(tree, "Drawer");

    tree.def(
        "visualize",
        [](std::shared_ptr<Octree> &self,
           const bool leaf_only,
           float scaling,
           const Eigen::Vector3d &area_min,
           const Eigen::Vector3d &area_max,
           const Eigen::Vector3d &border_color,
           const int window_width,
           const int window_height,
           const int window_left,
           const int window_top) {
            using Drawer = SemiSparseOctreeDrawer<Octree>;
            auto drawer_setting = std::make_shared<typename Drawer::Setting>();
            drawer_setting->scaling = scaling;
            drawer_setting->area_min = area_min;
            drawer_setting->area_max = area_max;
            drawer_setting->border_color = border_color;

            auto drawer = std::make_shared<Drawer>(drawer_setting, self);
            auto visualizer_setting = std::make_shared<Open3dVisualizerWrapper::Setting>();
            visualizer_setting->window_width = window_width;
            visualizer_setting->window_height = window_height;
            visualizer_setting->window_left = window_left;
            visualizer_setting->window_top = window_top;
            const auto visualizer = std::make_shared<Open3dVisualizerWrapper>(visualizer_setting);
            std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries;
            if (leaf_only) {
                drawer->DrawLeaves(geometries);
            } else {
                drawer->DrawTree(geometries);
            }
            visualizer->AddGeometries(geometries);
            visualizer->Show();
        },
        py::arg("leaf_only") = false,
        py::arg("scaling") = 1.0f,
        py::arg("area_min") = Eigen::Vector3f(-1.0, -1.0, -1.0),
        py::arg("area_max") = Eigen::Vector3f(1.0, 1.0, 1.0),
        py::arg("border_color") = Eigen::Vector3f(0.0, 0.0, 0.0),
        py::arg("window_width") = 1920,
        py::arg("window_height") = 1080,
        py::arg("window_left") = 50,
        py::arg("window_top") = 50);

    return tree;
}
