#pragma once

#ifdef ERL_USE_OPEN3D
    #include "open3d_visualizer_wrapper.hpp"
    #include "pybind11_occupancy_octree_drawer.hpp"
#endif

#include "occupancy_octree_base.hpp"
#include "pybind11_octree_impl.hpp"

template<class Node, class NodeParent = void>
std::enable_if_t<std::is_same_v<NodeParent, void>, py::class_<Node, py::RawPtrWrapper<Node>>>
BindOccupancyOctreeNode(const py::module &m, const char *node_name) {
    py::class_<Node, py::RawPtrWrapper<Node>> node(m, node_name);
    node.def(
        "get_child",
        py::overload_cast<uint32_t>(&Node::template GetChild<Node>),
        py::arg("child_idx"));
    return node;
}

template<class Node, class NodeParent>
std::enable_if_t<
    !std::is_same_v<NodeParent, void>,
    py::class_<Node, NodeParent, py::RawPtrWrapper<Node>>>
BindOccupancyOctreeNode(const py::module &m, const char *node_name) {
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

template<class Octree, class Node>
auto
BindOccupancyOctree(
    const py::module &m,
    const char *tree_name,
    std::function<void(py::class_<
                       Octree,
                       erl::geometry::AbstractOccupancyOctree<typename Octree::DataType>,
                       std::shared_ptr<Octree>> &)> additional_bindings = nullptr) {

    using namespace erl::common;
    using namespace erl::geometry;
    using Dtype = typename Octree::DataType;
    using Vector3 = Eigen::Vector3<Dtype>;
    using VectorX = Eigen::VectorX<Dtype>;
    using Matrix3X = Eigen::Matrix3X<Dtype>;
    using Matrix3 = Eigen::Matrix3<Dtype>;

    py::class_<Octree, AbstractOccupancyOctree<Dtype>, std::shared_ptr<Octree>> tree(m, tree_name);

    if (additional_bindings) { additional_bindings(tree); }
    if (std::is_same_v<typename Octree::Setting, OccupancyOctreeBaseSetting> &&
        !py::hasattr(tree, "Setting")) {
        ERL_DEBUG("Bind default Setting type to {}", tree_name);
        tree.def("Setting", []() { return std::make_shared<typename Octree::Setting>(); });
    }

    // AbstractOctree methods are defined in bind_abstract_octree.cpp
    // AbstractOccupancyOctree methods are defined in bind_abstract_occupancy_octree.cpp

    using BatchRayCaster = OccupancyNdTreeBatchRayCaster<Octree, 3>;
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
        .def("step", &BatchRayCaster::Step, py::arg("max_depth") = 0);

    // OccupancyOctreeBase methods, except iterators
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
        .def(
            "insert_point_cloud",
            &Octree::InsertPointCloud,
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
            &Octree::InsertPointCloudRays,
            py::arg("points"),
            py::arg("sensor_origin"),
            py::arg("min_range"),
            py::arg("max_range"),
            py::arg("parallel"),
            py::arg("lazy_eval"),
            py::call_guard<py::gil_scoped_release>())
        .def(
            "insert_ray",
            &Octree::InsertRay,
            py::arg("sx"),
            py::arg("sy"),
            py::arg("sz"),
            py::arg("ex"),
            py::arg("ey"),
            py::arg("ez"),
            py::arg("min_range"),
            py::arg("max_range"),
            py::arg("lazy_eval"))
        .def(
            "sample_positions",
            [](const Octree &self, std::size_t num_positions) {
                std::vector<Vector3> positions;
                self.SamplePositions(num_positions, positions);
                return positions;
            },
            py::arg("num_positions"))
        .def(
            "cast_rays",
            [](const Octree &self,
               const Eigen::Ref<const Vector3> &position,
               const Eigen::Ref<const Matrix3> &rotation,
               const Eigen::Ref<const VectorX> &azimuth_angles,
               const Eigen::Ref<const VectorX> &elevation_angles,
               bool ignore_unknown,
               Dtype max_range,
               bool prune_rays,
               bool parallel) -> py::dict {
                std::vector<std::pair<long, long>> hit_ray_indices;
                std::vector<Vector3> hit_positions;
                std::vector<const Node *> hit_nodes;
                {
                    py::gil_scoped_release release;
                    self.CastRays(
                        position,
                        rotation,
                        azimuth_angles,
                        elevation_angles,
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
            py::arg("azimuth_angles"),
            py::arg("elevation_angles"),
            py::arg("ignore_unknown"),
            py::arg("max_range"),
            py::arg("prune_rays"),
            py::arg("parallel"))
        .def(
            "cast_rays",
            [](const Octree &self,
               const Eigen::Ref<const Matrix3X> &positions,
               const Eigen::Ref<const Matrix3X> &directions,
               bool ignore_unknown,
               Dtype max_range,
               bool prune_rays,
               bool parallel) -> py::dict {
                std::vector<long> hit_ray_indices;
                std::vector<Vector3> hit_positions;
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
            &Octree::GetBatchRayCaster,
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
            [](const Octree &self,
               Dtype px,
               Dtype py,
               Dtype pz,
               Dtype vx,
               Dtype vy,
               Dtype vz,
               bool ignore_unknown,
               Dtype max_range) {
                Dtype ex, ey, ez;
                const Node *hit_node =
                    self.CastRay(px, py, pz, vx, vy, vz, ignore_unknown, max_range, ex, ey, ez);
                py::dict result;
                result["hit_node"] = hit_node;
                result["ex"] = ex;
                result["ey"] = ey;
                result["ez"] = ez;
                return result;
            },
            py::arg("px"),
            py::arg("py"),
            py::arg("pz"),
            py::arg("vx"),
            py::arg("vy"),
            py::arg("vz"),
            py::arg("ignore_unknown"),
            py::arg("max_range"))
        .def(
            "update_node",
            py::overload_cast<Dtype, Dtype, Dtype, bool, bool>(&Octree::UpdateNode),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("occupied"),
            py::arg("lazy_eval"))
        .def(
            "update_node",
            py::overload_cast<const OctreeKey &, bool, bool>(&Octree::UpdateNode),
            py::arg("node_key"),
            py::arg("occupied"),
            py::arg("lazy_eval"))
        .def(
            "update_node",
            py::overload_cast<Dtype, Dtype, Dtype, float, bool>(&Octree::UpdateNode),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("log_odds_delta"),
            py::arg("lazy_eval"))
        .def(
            "update_node",
            py::overload_cast<const OctreeKey &, float, bool>(&Octree::UpdateNode),
            py::arg("node_key"),
            py::arg("log_odds_delta"),
            py::arg("lazy_eval"))
        .def("update_inner_occupancy", &Octree::UpdateInnerOccupancy)
        .def("to_max_likelihood", &Octree::ToMaxLikelihood);

    BindOctreeImpl<decltype(tree), Dtype, Octree, Node>(tree);

#ifdef ERL_USE_OPEN3D
    BindOccupancyOctreeDrawer<Octree>(tree, "Drawer");

    tree.def(
        "visualize",
        [](std::shared_ptr<Octree> &self,
           const bool leaf_only,
           float scaling,
           const Eigen::Vector3d &area_min,
           const Eigen::Vector3d &area_max,
           const Eigen::Vector3d &border_color,
           const Eigen::Vector3d &occupied_color,
           const bool occupied_only,
           const bool draw_node_boxes,
           const bool draw_node_borders,
           const int window_width,
           const int window_height,
           const int window_left,
           const int window_top) {
            using Drawer = OccupancyOctreeDrawer<Octree>;
            auto drawer_setting = std::make_shared<typename Drawer::Setting>();
            drawer_setting->scaling = scaling;
            drawer_setting->area_min = area_min;
            drawer_setting->area_max = area_max;
            drawer_setting->border_color = border_color;
            drawer_setting->occupied_color = occupied_color;
            drawer_setting->occupied_only = occupied_only;
            drawer_setting->draw_node_boxes = draw_node_boxes;
            drawer_setting->draw_node_borders = draw_node_borders;

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
        py::arg("area_min") = Vector3(-1.0, -1.0, -1.0),
        py::arg("area_max") = Vector3(1.0, 1.0, 1.0),
        py::arg("border_color") = Vector3(0.0, 0.0, 0.0),
        py::arg("occupied_color") = Vector3(0.5, 0.5, 0.5),
        py::arg("occupied_only") = false,
        py::arg("draw_node_boxes") = true,
        py::arg("draw_node_borders") = true,
        py::arg("window_width") = 1920,
        py::arg("window_height") = 1080,
        py::arg("window_left") = 50,
        py::arg("window_top") = 50);
#endif

    return tree;
}
