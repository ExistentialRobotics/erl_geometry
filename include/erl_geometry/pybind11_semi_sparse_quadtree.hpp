#pragma once

#include "pybind11_quadtree_impl.hpp"
#include "pybind11_semi_sparse_quadtree_drawer.hpp"
#include "semi_sparse_nd_tree_setting.hpp"
#include "semi_sparse_quadtree_base.hpp"

template<class Quadtree, class Node>
auto
BindSemiSparseQuadtree(
    const py::module &m,
    const char *tree_name,
    std::function<void(py::class_<
                       Quadtree,
                       erl::geometry::AbstractQuadtree<typename Quadtree::DataType>,
                       std::shared_ptr<Quadtree>> &)> additional_bindings = nullptr) {

    using namespace erl::common;
    using namespace erl::geometry;
    using Dtype = typename Quadtree::DataType;
    using Matrix2X = Eigen::Matrix2X<Dtype>;

    py::class_<Quadtree, AbstractQuadtree<Dtype>, std::shared_ptr<Quadtree>> tree(m, tree_name);

    if (additional_bindings) { additional_bindings(tree); }
    if (std::is_same_v<typename Quadtree::Setting, SemiSparseNdTreeSetting> &&
        !py::hasattr(tree, "Setting")) {
        ERL_DEBUG("Bind default Setting type to {}", tree_name);
        tree.def("Setting", []() { return std::make_shared<typename Quadtree::Setting>(); });
    }

    // AbstractQuadtree methods are defined in bind_abstract_quadtree.cpp

    // SemiSparseQuadtree
    tree.def(py::init<>())
        .def(
            py::init<>([](const std::shared_ptr<typename Quadtree::Setting> &setting) {
                return std::make_shared<Quadtree>(setting);
            }),
            py::arg("setting"))
        .def(
            py::init<>(
                [](const std::string &filename) { return std::make_shared<Quadtree>(filename); }),
            py::arg("filename"))
        .def_property_readonly(
            "setting",
            &Quadtree::template GetSetting<typename Quadtree::Setting>)
        .def_property_readonly("parents", &Quadtree::GetParents)
        .def_property_readonly("children", &Quadtree::GetChildren)
        .def_property_readonly("voxels", &Quadtree::GetVoxels)
        .def_property_readonly("vertices", &Quadtree::GetVertices)
        .def_property_readonly("num_vertices", &Quadtree::GetVertexCount)
        .def_property_readonly("vertex_keys", &Quadtree::GetVertexKeys)
        .def(
            "insert_points",
            py::overload_cast<const Matrix2X &>(&Quadtree::InsertPoints),
            py::arg("points"))
        .def("insert_point", &Quadtree::InsertPoint, py::arg("key"), py::arg("max_depth"))
        .def(
            "find_voxel_indices",
            py::overload_cast<const Matrix2X &, bool>(&Quadtree::FindVoxelIndices, py::const_),
            py::arg("points"),
            py::arg("parallel"))
        .def("find_voxel_index", &Quadtree::FindVoxelIndex, py::arg("key"));

    BindQuadtreeImpl<decltype(tree), Dtype, Quadtree, Node>(tree);
    BindSemiSparseQuadtreeDrawer<Quadtree>(tree, "Drawer");

    tree.def(
        "visualize",
        [](std::shared_ptr<Quadtree> &self,
           const bool leaf_only,
           std::optional<Eigen::Vector2f> area_min,
           std::optional<Eigen::Vector2f> area_max,
           const Dtype resolution,
           const int padding,
           Eigen::Vector4i bg_color,
           Eigen::Vector4i fg_color,
           Eigen::Vector4i border_color,
           const int border_thickness) {
            using Drawer = SemiSparseQuadtreeDrawer<Quadtree>;
            auto drawer_setting = std::make_shared<typename Drawer::Setting>();
            if (area_min.has_value()) {
                drawer_setting->area_min = area_min.value();
            } else {
                Dtype min_x, min_y;
                self->GetMetricMin(min_x, min_y);
                drawer_setting->area_min[0] = min_x;
                drawer_setting->area_min[1] = min_y;
            }
            if (area_max.has_value()) {
                drawer_setting->area_max = area_max.value();
            } else {
                Dtype max_x, max_y;
                self->GetMetricMax(max_x, max_y);
                drawer_setting->area_max[0] = max_x;
                drawer_setting->area_max[1] = max_y;
            }
            drawer_setting->resolution = resolution;
            drawer_setting->padding = padding;
            for (int i = 0; i < 4; ++i) {
                drawer_setting->bg_color[0] = bg_color[0];
                drawer_setting->bg_color[1] = bg_color[1];
                drawer_setting->bg_color[2] = bg_color[2];
                drawer_setting->bg_color[3] = bg_color[3];

                drawer_setting->fg_color[0] = fg_color[0];
                drawer_setting->fg_color[1] = fg_color[1];
                drawer_setting->fg_color[2] = fg_color[2];
                drawer_setting->fg_color[3] = fg_color[3];

                drawer_setting->border_color[0] = border_color[0];
                drawer_setting->border_color[1] = border_color[1];
                drawer_setting->border_color[2] = border_color[2];
                drawer_setting->border_color[3] = border_color[3];
            }
            drawer_setting->border_thickness = border_thickness;

            auto drawer = std::make_shared<Drawer>(drawer_setting, self);
            cv::Mat mat;
            if (leaf_only) {
                drawer->DrawLeaves(mat);
            } else {
                drawer->DrawTree(mat);
            }
            cv::cvtColor(mat, mat, cv::COLOR_BGRA2RGBA);
            Eigen::MatrixX8U image;
            cv::cv2eigen(mat, image);
            return image;
        },
        py::arg("leaf_only") = false,
        py::arg("area_min") = py::none(),
        py::arg("area_max") = py::none(),
        py::arg("resolution") = 0.1,
        py::arg("padding") = 1,
        py::arg("bg_color") = Eigen::Vector4i(128, 128, 128, 255),
        py::arg("fg_color") = Eigen::Vector4i(255, 255, 255, 255),
        py::arg("border_color") = Eigen::Vector4i(0, 0, 0, 255),
        py::arg("border_thickness") = 1);

    return tree;
}
