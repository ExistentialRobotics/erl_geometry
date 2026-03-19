#pragma once

#ifdef ERL_USE_OPENCV
    #include "pybind11_occupancy_quadtree_drawer.hpp"
#endif

#include "occupancy_quadtree_base.hpp"
#include "pybind11_quadtree_impl.hpp"

template<class Node, class NodeParent = void>
std::enable_if_t<std::is_same_v<NodeParent, void>, py::class_<Node, py::RawPtrWrapper<Node>>>
BindOccupancyQuadtreeNode(const py::module &m, const char *node_name) {
    py::class_<Node, py::RawPtrWrapper<Node>> node(m, node_name);
    node.def(
        "get_child",
        py::overload_cast<uint32_t>(&Node::template GetChild<Node>),
        py::arg("child_idx"));
    return node;
}

template<class Node, class NodeParent>
std::enable_if_t<!std::is_void_v<NodeParent>, py::class_<Node, NodeParent, py::RawPtrWrapper<Node>>>
BindOccupancyQuadtreeNode(const py::module &m, const char *node_name) {
    py::class_<Node, NodeParent, py::RawPtrWrapper<Node>> node(m, node_name);
    node.def(
            "get_child",
            py::overload_cast<uint32_t>(&Node::template GetChild<Node>),
            py::arg("child_idx"))
        .def_property_readonly("occupancy", &Node::GetOccupancy)
        .def_property_readonly("log_odds", &Node::GetLogOdds)
        .def_property_readonly("mean_child_log_odds", &Node::GetMeanChildLogOdds)
        .def_property_readonly("max_child_log_odds", &Node::GetMaxChildLogOdds)
        .def("allow_update_log_odds", &Node::AllowUpdateLogOdds, py::arg("delta"))
        .def("add_log_odds", &Node::AddLogOdds, py::arg("log_odds"));
    return node;
}

template<class Quadtree, class Node>
auto
BindOccupancyQuadtree(
    const py::module &m,
    const char *tree_name,
    std::function<void(py::class_<
                       Quadtree,
                       erl::geometry::AbstractOccupancyQuadtree<typename Quadtree::DataType>,
                       std::shared_ptr<Quadtree>> &)> additional_bindings = nullptr) {

    using namespace erl::common;
    using namespace erl::geometry;
    using Dtype = typename Quadtree::DataType;
    using Vector2 = Eigen::Vector2<Dtype>;
    using VectorX = Eigen::VectorX<Dtype>;
    using Matrix2 = Eigen::Matrix2<Dtype>;
    using Matrix2X = Eigen::Matrix2X<Dtype>;

    py::class_<
        Quadtree,
        AbstractOccupancyQuadtree<typename Quadtree::DataType>,
        std::shared_ptr<Quadtree>>
        tree(m, tree_name);

    if (additional_bindings) { additional_bindings(tree); }
    if (std::is_same_v<typename Quadtree::Setting, OccupancyQuadtreeBaseSetting> &&
        !py::hasattr(tree, "Setting")) {
        ERL_DEBUG("Bind default Setting type to {}", tree_name);
        tree.def("Setting", []() { return std::make_shared<typename Quadtree::Setting>(); });
    }

    // AbstractQuadtree methods are defined in bind_abstract_quadtree.cpp
    // AbstractOccupancyQuadtree methods are defined in bind_abstract_occupancy_quadtree.cpp

    using BatchRayCaster = OccupancyNdTreeBatchRayCaster<Quadtree, 2>;
    py::class_<BatchRayCaster>(tree, "BatchRayCaster")
        .def_property_readonly("num_rays", &BatchRayCaster::GetNumRays)
        .def_property_readonly("ray_origins", &BatchRayCaster::GetRayOrigins)
        .def_property_readonly("ray_directions", &BatchRayCaster::GetRayDirections)
        .def_property_readonly("hit_flags", &BatchRayCaster::GetHitFlags)
        .def_property_readonly("ever_hit_flags", &BatchRayCaster::GetEverHitFlags)
        .def_property_readonly("hit_distances", &BatchRayCaster::GetHitDistances)
        .def_property_readonly("hit_nodes", &BatchRayCaster::GetHitNodes)
        .def_property_readonly("hit_positions", &BatchRayCaster::GetHitPositions)
        .def_property_readonly("frontier_nodes", &BatchRayCaster::GetFrontierNodes)
        .def_property_readonly("frontier_keys", &BatchRayCaster::GetFrontierKeys)
        .def_property_readonly("frontier_ray_indices", &BatchRayCaster::GetFrontierRayIndices)
        .def("step", &BatchRayCaster::Step, py::arg("mask") = py::array_t<bool>());

    // OccupancyQuadtreeBase methods, except iterators
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
        .def(
            "insert_point_cloud",
            &Quadtree::InsertPointCloud,
            py::arg("points"),
            py::arg("sensor_origin"),
            py::arg("min_range"),
            py::arg("max_range"),
            py::arg("with_count"),
            py::arg("parallel"),
            py::arg("lazy_eval"),
            py::arg("discrete"),
            py::call_guard<py::gil_scoped_release>())
        .def(
            "insert_point_cloud_rays",
            &Quadtree::InsertPointCloudRays,
            py::arg("points"),
            py::arg("sensor_origin"),
            py::arg("min_range"),
            py::arg("max_range"),
            py::arg("parallel"),
            py::arg("lazy_eval"),
            py::call_guard<py::gil_scoped_release>())
        .def(
            "insert_ray",
            &Quadtree::InsertRay,
            py::arg("sx"),
            py::arg("sy"),
            py::arg("ex"),
            py::arg("ey"),
            py::arg("min_range"),
            py::arg("max_range"),
            py::arg("lazy_eval"))
        .def(
            "sample_positions",
            [](const Quadtree &self, std::size_t num_positions) {
                std::vector<Vector2> positions;
                self.SamplePositions(num_positions, positions);
                return positions;
            },
            py::arg("num_positions"))
        .def(
            "cast_rays",
            [](const Quadtree &self,
               const Eigen::Ref<const Vector2> &position,
               const Eigen::Ref<const Matrix2> &rotation,
               const Eigen::Ref<const VectorX> &angles,
               bool ignore_unknown,
               Dtype max_range,
               bool prune_rays,
               bool parallel) -> py::dict {
                std::vector<long> hit_ray_indices;
                std::vector<Vector2> hit_positions;
                std::vector<const Node *> hit_nodes;
                {
                    py::gil_scoped_release release;
                    self.CastRays(
                        position,
                        rotation,
                        angles,
                        ignore_unknown,
                        max_range,
                        prune_rays,
                        parallel,
                        hit_ray_indices,
                        hit_positions,
                        hit_nodes);
                }

                py::dict result;
                result["hit_ray_indices"] = hit_ray_indices;
                result["hit_positions"] = hit_positions;
                result["hit_nodes"] = hit_nodes;
                return result;
            },
            py::arg("position"),
            py::arg("rotation"),
            py::arg("angles"),
            py::arg("ignore_unknown"),
            py::arg("max_range"),
            py::arg("prune_rays"),
            py::arg("parallel"))
        .def(
            "cast_rays",
            [](const Quadtree &self,
               const Eigen::Ref<const Matrix2X> &positions,
               const Eigen::Ref<const Matrix2X> &directions,
               bool ignore_unknown,
               Dtype max_range,
               bool prune_rays,
               bool parallel) -> py::dict {
                std::vector<long> hit_ray_indices;
                std::vector<Vector2> hit_positions;
                std::vector<const Node *> hit_nodes;
                {
                    py::gil_scoped_release release;
                    self.CastRays(
                        positions,
                        directions,
                        ignore_unknown,
                        max_range,
                        prune_rays,
                        parallel,
                        hit_ray_indices,
                        hit_positions,
                        hit_nodes);
                }
                py::dict result;
                result["hit_ray_indices"] = hit_ray_indices;
                result["hit_positions"] = hit_positions;
                result["hit_nodes"] = hit_nodes;
                return result;
            },
            py::arg("positions"),
            py::arg("directions"),
            py::arg("ignore_unknown"),
            py::arg("max_range"),
            py::arg("prune_rays"),
            py::arg("parallel"))
        .def(
            "get_batch_ray_caster",
            &Quadtree::GetBatchRayCaster,
            py::arg("origins"),
            py::arg("directions"),
            py::arg("max_ranges") = py::array_t<Dtype>(),
            py::arg("node_paddings") = py::array_t<Dtype>(),
            py::arg("bidirectional_flags") = py::array_t<bool>(),
            py::arg("leaf_only_flags") = py::array_t<bool>(),
            py::arg("min_node_depths") = py::array_t<int>(),
            py::arg("max_node_depths") = py::array_t<int>())
        .def(
            "cast_ray",
            [](const Quadtree &self,
               Dtype px,
               Dtype py,
               Dtype vx,
               Dtype vy,
               bool ignore_unknown,
               Dtype max_range) {
                Dtype ex, ey;
                const Node *hit_node =
                    self.CastRay(px, py, vx, vy, ignore_unknown, max_range, ex, ey);
                py::dict result;
                result["hit_node"] = hit_node;
                result["ex"] = ex;
                result["ey"] = ey;
                return result;
            },
            py::arg("px"),
            py::arg("py"),
            py::arg("vx"),
            py::arg("vy"),
            py::arg("ignore_unknown"),
            py::arg("max_range"))
        .def(
            "update_node",
            py::overload_cast<Dtype, Dtype, bool, bool>(&Quadtree::UpdateNode),
            py::arg("x"),
            py::arg("y"),
            py::arg("occupied"),
            py::arg("lazy_eval"))
        .def(
            "update_node",
            py::overload_cast<const QuadtreeKey &, bool, bool>(&Quadtree::UpdateNode),
            py::arg("node_key"),
            py::arg("occupied"),
            py::arg("lazy_eval"))
        .def(
            "update_node",
            py::overload_cast<Dtype, Dtype, float, bool>(&Quadtree::UpdateNode),
            py::arg("x"),
            py::arg("y"),
            py::arg("log_odds_delta"),
            py::arg("lazy_eval"))
        .def(
            "update_node",
            py::overload_cast<const QuadtreeKey &, float, bool>(&Quadtree::UpdateNode),
            py::arg("node_key"),
            py::arg("log_odds_delta"),
            py::arg("lazy_eval"))
        .def("update_inner_occupancy", &Quadtree::UpdateInnerOccupancy)
        .def("to_max_likelihood", &Quadtree::ToMaxLikelihood);

    BindQuadtreeImpl<decltype(tree), Dtype, Quadtree, Node>(tree);

#ifdef ERL_USE_OPENCV
    BindOccupancyQuadtreeDrawer<Quadtree>(tree, "Drawer");

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
           Eigen::Vector4i occupied_color,
           Eigen::Vector4i free_color,
           Eigen::Vector4i border_color,
           const int border_thickness) {
            using Drawer = OccupancyQuadtreeDrawer<Quadtree>;
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

                drawer_setting->occupied_color[0] = occupied_color[0];
                drawer_setting->occupied_color[1] = occupied_color[1];
                drawer_setting->occupied_color[2] = occupied_color[2];
                drawer_setting->occupied_color[3] = occupied_color[3];

                drawer_setting->free_color[0] = free_color[0];
                drawer_setting->free_color[1] = free_color[1];
                drawer_setting->free_color[2] = free_color[2];
                drawer_setting->free_color[3] = free_color[3];

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
        py::arg("occupied_color") = Eigen::Vector4i(0, 0, 0, 255),
        py::arg("free_color") = Eigen::Vector4i(255, 255, 255, 255),
        py::arg("border_color") = Eigen::Vector4i(0, 0, 0, 255),
        py::arg("border_thickness") = 1);
#endif

    return tree;
}
