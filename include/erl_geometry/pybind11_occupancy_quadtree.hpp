#pragma once

#include "occupancy_quadtree_base.hpp"
#include "occupancy_quadtree_drawer.hpp"
#include "pybind11_occupancy_quadtree_drawer.hpp"

template<class Node, class NodeParent = void>
std::enable_if_t<std::is_same_v<NodeParent, void>, py::class_<Node, py::RawPtrWrapper<Node>>>
BindOccupancyQuadtreeNode(const py::module& m, const char* node_name) {
    py::class_<Node, py::RawPtrWrapper<Node>> node(m, node_name);
    node.def("get_child", py::overload_cast<uint32_t>(&Node::template GetChild<Node>), py::arg("child_idx"));
    return node;
}

template<class Node, class NodeParent>
std::enable_if_t<!std::is_void_v<NodeParent>, py::class_<Node, NodeParent, py::RawPtrWrapper<Node>>>
BindOccupancyQuadtreeNode(const py::module& m, const char* node_name) {
    py::class_<Node, NodeParent, py::RawPtrWrapper<Node>> node(m, node_name);
    node.def("get_child", py::overload_cast<uint32_t>(&Node::template GetChild<Node>), py::arg("child_idx"))
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
    const py::module& m,
    const char* tree_name,
    std::function<void(py::class_<Quadtree, erl::geometry::AbstractOccupancyQuadtree, std::shared_ptr<Quadtree>>&)> additional_bindings = nullptr) {

    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<Quadtree, AbstractOccupancyQuadtree, std::shared_ptr<Quadtree>> tree(m, tree_name);

    if (additional_bindings) { additional_bindings(tree); }
    if (std::is_same_v<typename Quadtree::Setting, OccupancyQuadtreeBaseSetting> && !py::hasattr(tree, "Setting")) {
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
        .def(py::init<>([](const std::shared_ptr<typename Quadtree::Setting>& setting) { return std::make_shared<Quadtree>(setting); }), py::arg("setting"))
        .def(
            py::init<>([](const std::string& filename, const bool use_derived_constructor) {
                if (use_derived_constructor) { return std::make_shared<Quadtree>(filename); }
                if (std::shared_ptr<Quadtree> quadtree = Quadtree::template ReadAs<Quadtree>(filename)) { return quadtree; }
                throw std::runtime_error("Failed to read Quadtree from " + filename);
            }),
            py::arg("filename"),
            py::arg("use_derived_constructor"))
        .def_property_readonly("tree_type", &Quadtree::GetTreeType)
        .def_property_readonly("setting", &Quadtree::template GetSetting<typename Quadtree::Setting>)
        .def(
            "insert_point_cloud",
            &Quadtree::InsertPointCloud,
            py::arg("points"),
            py::arg("sensor_origin"),
            py::arg("max_range"),
            py::arg("parallel"),
            py::arg("lazy_eval"),
            py::arg("discretize"))
        .def(
            "insert_point_cloud_rays",
            &Quadtree::InsertPointCloudRays,
            py::arg("points"),
            py::arg("sensor_origin"),
            py::arg("max_range"),
            py::arg("parallel"),
            py::arg("lazy_eval"))
        .def("insert_ray", &Quadtree::InsertRay, py::arg("sx"), py::arg("sy"), py::arg("ex"), py::arg("ey"), py::arg("max_range"), py::arg("lazy_eval"))
        .def(
            "sample_positions",
            [](const Quadtree& self, std::size_t num_positions) {
                std::vector<Eigen::Vector2d> positions;
                self.SamplePositions(num_positions, positions);
                return positions;
            },
            py::arg("num_positions"))
        .def(
            "cast_rays",
            [](const Quadtree& self,
               const Eigen::Ref<const Eigen::Vector2d>& position,
               const Eigen::Ref<const Eigen::Matrix2d>& rotation,
               const Eigen::Ref<const Eigen::VectorXd>& angles,
               bool ignore_unknown,
               double max_range,
               bool prune_rays,
               bool parallel) -> py::dict {
                std::vector<long> hit_ray_indices;
                std::vector<Eigen::Vector2d> hit_positions;
                std::vector<const Node*> hit_nodes;
                self.CastRays(position, rotation, angles, ignore_unknown, max_range, prune_rays, parallel, hit_ray_indices, hit_positions, hit_nodes);

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
            [](const Quadtree& self,
               const Eigen::Ref<const Eigen::Matrix2Xd>& positions,
               const Eigen::Ref<const Eigen::Matrix2Xd>& directions,
               bool ignore_unknown,
               double max_range,
               bool prune_rays,
               bool parallel) -> py::dict {
                std::vector<long> hit_ray_indices;
                std::vector<Eigen::Vector2d> hit_positions;
                std::vector<const Node*> hit_nodes;
                self.CastRays(positions, directions, ignore_unknown, max_range, prune_rays, parallel, hit_ray_indices, hit_positions, hit_nodes);
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
            py::arg("max_ranges") = py::array_t<double>(),
            py::arg("node_paddings") = py::array_t<double>(),
            py::arg("bidirectional_flags") = py::array_t<bool>(),
            py::arg("leaf_only_flags") = py::array_t<bool>(),
            py::arg("min_node_depths") = py::array_t<int>(),
            py::arg("max_node_depths") = py::array_t<int>())
        .def(
            "cast_ray",
            [](const Quadtree& self, double px, double py, double vx, double vy, bool ignore_unknown, double max_range) {
                double ex, ey;
                const Node* hit_node = self.CastRay(px, py, vx, vy, ignore_unknown, max_range, ex, ey);
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
            py::overload_cast<double, double, bool, bool>(&Quadtree::UpdateNode),
            py::arg("x"),
            py::arg("y"),
            py::arg("occupied"),
            py::arg("lazy_eval"))
        .def(
            "update_node",
            py::overload_cast<const QuadtreeKey&, bool, bool>(&Quadtree::UpdateNode),
            py::arg("node_key"),
            py::arg("occupied"),
            py::arg("lazy_eval"))
        .def(
            "update_node",
            py::overload_cast<double, double, float, bool>(&Quadtree::UpdateNode),
            py::arg("x"),
            py::arg("y"),
            py::arg("log_odds_delta"),
            py::arg("lazy_eval"))
        .def(
            "update_node",
            py::overload_cast<const QuadtreeKey&, float, bool>(&Quadtree::UpdateNode),
            py::arg("node_key"),
            py::arg("log_odds_delta"),
            py::arg("lazy_eval"))
        .def("update_inner_occupancy", &Quadtree::UpdateInnerOccupancy)
        .def("to_max_likelihood", &Quadtree::ToMaxLikelihood);

    // QuadtreeImpl methods
    tree.def_property_readonly("number_of_nodes", &Quadtree::GetSize)
        .def_property_readonly("resolution", &Quadtree::GetResolution)
        .def_property_readonly("tree_depth", &Quadtree::GetTreeDepth)
        .def_property_readonly("tree_center", &Quadtree::GetTreeCenter)
        .def_property_readonly("tree_center_key", &Quadtree::GetTreeCenterKey)
        .def_property_readonly("tree_max_half_size", &Quadtree::GetTreeMaxHalfSize)
        .def_property_readonly("metric_min", [](Quadtree& self) { return self.GetMetricMin(); })
        .def_property_readonly("metric_max", [](Quadtree& self) { return self.GetMetricMax(); })
        .def_property_readonly("metric_min_max", [](Quadtree& self) { return self.GetMetricMinMax(); })
        .def_property_readonly("metric_aabb", [](Quadtree& self) { return self.GetMetricAabb(); })
        .def_property_readonly("metric_size", [](Quadtree& self) { return self.GetMetricSize(); })
        .def("get_node_size", &Quadtree::GetNodeSize, py::arg("depth"))
        .def_property_readonly("number_of_leaf_nodes", &Quadtree::ComputeNumberOfLeafNodes)
        .def_property_readonly("memory_usage", &Quadtree::GetMemoryUsage)
        .def_property_readonly("memory_usage_per_node", &Quadtree::GetMemoryUsagePerNode)
        .def("coord_to_key", py::overload_cast<double>(&Quadtree::CoordToKey, py::const_), py::arg("coordinate"))
        .def("coord_to_key", py::overload_cast<double, uint32_t>(&Quadtree::CoordToKey, py::const_), py::arg("coordinate"), py::arg("depth"))
        .def("coord_to_key", py::overload_cast<double, double>(&Quadtree::CoordToKey, py::const_), py::arg("x"), py::arg("y"))
        .def("coord_to_key", py::overload_cast<double, double, uint32_t>(&Quadtree::CoordToKey, py::const_), py::arg("x"), py::arg("y"), py::arg("depth"))
        .def(
            "coord_to_key_checked",
            [](const Quadtree& self, double coordinate) {
                if (QuadtreeKey::KeyType key; self.CoordToKeyChecked(coordinate, key)) { return std::optional<QuadtreeKey::KeyType>(key); }
                return std::optional<QuadtreeKey::KeyType>();
            },
            py::arg("coordinate"))
        .def(
            "coord_to_key_checked",
            [](const Quadtree& self, double coordinate, uint32_t depth) {
                if (QuadtreeKey::KeyType key; self.CoordToKeyChecked(coordinate, depth, key)) { return std::optional<QuadtreeKey::KeyType>(key); }
                return std::optional<QuadtreeKey::KeyType>();
            },
            py::arg("coordinate"),
            py::arg("depth"))
        .def(
            "coord_to_key_checked",
            [](const Quadtree& self, double x, double y) {
                if (QuadtreeKey key; self.CoordToKeyChecked(x, y, key)) { return std::optional<QuadtreeKey>(key); }
                return std::optional<QuadtreeKey>();
            },
            py::arg("x"),
            py::arg("y"))
        .def(
            "coord_to_key_checked",
            [](const Quadtree& self, double x, double y, uint32_t depth) {
                if (QuadtreeKey key; self.CoordToKeyChecked(x, y, depth, key)) { return std::optional<QuadtreeKey>(key); }
                return std::optional<QuadtreeKey>();
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("depth"))
        .def(
            "adjust_key_to_depth",
            py::overload_cast<QuadtreeKey::KeyType, uint32_t>(&Quadtree::AdjustKeyToDepth, py::const_),
            py::arg("key"),
            py::arg("depth"))
        .def("adjust_key_to_depth", py::overload_cast<const QuadtreeKey&, uint32_t>(&Quadtree::AdjustKeyToDepth, py::const_), py::arg("key"), py::arg("depth"))
        .def(
            "compute_common_ancestor_key",
            [](const Quadtree& self, const QuadtreeKey& key1, const QuadtreeKey& key2) {
                QuadtreeKey key;
                uint32_t ancestor_depth;
                self.ComputeCommonAncestorKey(key1, key2, key, ancestor_depth);
                return std::make_tuple(key, ancestor_depth);
            })
        .def(
            "compute_west_neighbor_key",
            [](const Quadtree& self, const QuadtreeKey& key, uint32_t depth) {
                if (QuadtreeKey neighbor_key; self.ComputeWestNeighborKey(key, depth, neighbor_key)) { return std::optional<QuadtreeKey>(neighbor_key); }
                return std::optional<QuadtreeKey>();
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_east_neighbor_key",
            [](const Quadtree& self, const QuadtreeKey& key, uint32_t depth) {
                if (QuadtreeKey neighbor_key; self.ComputeEastNeighborKey(key, depth, neighbor_key)) { return std::optional<QuadtreeKey>(neighbor_key); }
                return std::optional<QuadtreeKey>();
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_north_neighbor_key",
            [](const Quadtree& self, const QuadtreeKey& key, uint32_t depth) {
                if (QuadtreeKey neighbor_key; self.ComputeNorthNeighborKey(key, depth, neighbor_key)) { return std::optional<QuadtreeKey>(neighbor_key); }
                return std::optional<QuadtreeKey>();
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_south_neighbor_key",
            [](const Quadtree& self, const QuadtreeKey& key, uint32_t depth) {
                if (QuadtreeKey neighbor_key; self.ComputeSouthNeighborKey(key, depth, neighbor_key)) { return std::optional<QuadtreeKey>(neighbor_key); }
                return std::optional<QuadtreeKey>();
            },
            py::arg("key"),
            py::arg("depth"))
        .def("key_to_coord", py::overload_cast<QuadtreeKey::KeyType>(&Quadtree::KeyToCoord, py::const_), py::arg("key"))
        .def("key_to_coord", py::overload_cast<QuadtreeKey::KeyType, uint32_t>(&Quadtree::KeyToCoord, py::const_), py::arg("key"), py::arg("depth"))
        .def(
            "key_to_coord",
            [](const Quadtree& self, const QuadtreeKey& key) {
                double x, y;
                self.KeyToCoord(key, x, y);
                return std::make_tuple(x, y);
            },
            py::arg("key"))
        .def(
            "key_to_coord",
            [](const Quadtree& self, const QuadtreeKey& key, uint32_t depth) {
                double x, y;
                self.KeyToCoord(key, depth, x, y);
                return std::make_tuple(x, y);
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_ray_keys",
            [](const Quadtree& self, double sx, double sy, double ex, double ey) {
                if (QuadtreeKeyRay ray; self.ComputeRayKeys(sx, sy, ex, ey, ray)) { return std::optional<QuadtreeKeyRay>(ray); }
                return std::optional<QuadtreeKeyRay>();
            },
            py::arg("sx"),
            py::arg("sy"),
            py::arg("ex"),
            py::arg("ey"))
        .def(
            "compute_ray_coords",
            [](const Quadtree& self, double sx, double sy, double ex, double ey) {
                if (std::vector<Eigen::Vector2d> ray; self.ComputeRayCoords(sx, sy, ex, ey, ray)) { return std::optional<std::vector<Eigen::Vector2d>>(ray); }
                return std::optional<std::vector<Eigen::Vector2d>>();
            },
            py::arg("sx"),
            py::arg("sy"),
            py::arg("ex"),
            py::arg("ey"))
        .def("create_node_child", &Quadtree::CreateNodeChild, py::arg("node"), py::arg("child_idx"))
        .def("delete_node_child", &Quadtree::DeleteNodeChild, py::arg("node"), py::arg("child_idx"), py::arg("key"))
        .def("get_node_child", py::overload_cast<Node*, uint32_t>(&Quadtree::GetNodeChild), py::arg("node"), py::arg("child_idx"))
        .def("is_node_collapsible", &Quadtree::IsNodeCollapsible, py::arg("node"))
        .def("expand_node", &Quadtree::ExpandNode, py::arg("node"))
        .def("prune_node", &Quadtree::PruneNode, py::arg("node"))
        .def("delete_node", py::overload_cast<double, double, uint32_t>(&Quadtree::DeleteNode), py::arg("x"), py::arg("y"), py::arg("depth"))
        .def("delete_node", py::overload_cast<const QuadtreeKey&, uint32_t>(&Quadtree::DeleteNode), py::arg("key"), py::arg("depth"))
        .def("clear", &Quadtree::Clear)
        .def("prune", &Quadtree::Prune)
        .def("expand", &Quadtree::Expand)
        .def_property_readonly("root", &Quadtree::GetRoot)
        .def("search", py::overload_cast<double, double, uint32_t>(&Quadtree::Search, py::const_), py::arg("x"), py::arg("y"), py::arg("max_depth") = 0)
        .def("search", py::overload_cast<const QuadtreeKey&, uint32_t>(&Quadtree::Search, py::const_), py::arg("key"), py::arg("max_depth") = 0)
        .def("insert_node", py::overload_cast<double, double, uint32_t>(&Quadtree::InsertNode), py::arg("x"), py::arg("y"), py::arg("depth"))
        .def("insert_node", py::overload_cast<const QuadtreeKey&, uint32_t>(&Quadtree::InsertNode), py::arg("key"), py::arg("depth"))
        .def(
            "visualize",
            [](std::shared_ptr<Quadtree>& self,
               const bool leaf_only,
               std::optional<Eigen::Vector2d> area_min,
               std::optional<Eigen::Vector2d> area_max,
               const double resolution,
               const int padding,
               Eigen::Vector4i bg_color,
               Eigen::Vector4i fg_color,
               Eigen::Vector4i occupied_color,
               Eigen::Vector4i free_color,
               Eigen::Vector4i border_color,
               const int border_thickness) {
                auto drawer_setting = std::make_shared<typename Quadtree::Drawer::Setting>();
                if (area_min.has_value()) {
                    drawer_setting->area_min = area_min.value();
                } else {
                    self->GetMetricMin(drawer_setting->area_min[0], drawer_setting->area_min[1]);
                }
                if (area_max.has_value()) {
                    drawer_setting->area_max = area_max.value();
                } else {
                    self->GetMetricMax(drawer_setting->area_max[0], drawer_setting->area_max[1]);
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

                auto drawer = std::make_shared<typename Quadtree::Drawer>(drawer_setting, self);
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

    // Iterators defined in QuadtreeImpl
    py::class_<typename Quadtree::IteratorBase, AbstractQuadtree::QuadtreeNodeIterator>(tree, "IteratorBase")
        .def("__eq__", [](const typename Quadtree::IteratorBase& self, const typename Quadtree::IteratorBase& other) { return self == other; })
        .def("__ne__", [](const typename Quadtree::IteratorBase& self, const typename Quadtree::IteratorBase& other) { return self != other; })
        .def_property_readonly("node_aabb", &Quadtree::IteratorBase::GetNodeAabb)
        .def_property_readonly("node", py::overload_cast<>(&Quadtree::IteratorBase::GetNode))
        .def_property_readonly("key", &Quadtree::IteratorBase::GetKey)
        .def_property_readonly("index_key", &Quadtree::IteratorBase::GetIndexKey);

    (void) py::class_<typename Quadtree::TreeIterator, typename Quadtree::IteratorBase>(tree, "TreeIterator");
    (void) py::class_<typename Quadtree::TreeInAabbIterator, typename Quadtree::IteratorBase>(tree, "TreeInAabbIterator");
    (void) py::class_<typename Quadtree::LeafIterator, typename Quadtree::IteratorBase>(tree, "LeafIterator");
    (void) py::class_<typename Quadtree::LeafOfNodeIterator, typename Quadtree::IteratorBase>(tree, "LeafOfNodeIterator");
    (void) py::class_<typename Quadtree::LeafInAabbIterator, typename Quadtree::IteratorBase>(tree, "LeafInAabbIterator");
    (void) py::class_<typename Quadtree::WestLeafNeighborIterator, typename Quadtree::IteratorBase>(tree, "WestLeafNeighborIterator");
    (void) py::class_<typename Quadtree::EastLeafNeighborIterator, typename Quadtree::IteratorBase>(tree, "EastLeafNeighborIterator");
    (void) py::class_<typename Quadtree::NorthLeafNeighborIterator, typename Quadtree::IteratorBase>(tree, "NorthLeafNeighborIterator");
    (void) py::class_<typename Quadtree::SouthLeafNeighborIterator, typename Quadtree::IteratorBase>(tree, "SouthLeafNeighborIterator");
    py::class_<typename Quadtree::NodeOnRayIterator, typename Quadtree::IteratorBase>(tree, "NodeOnRayIterator")
        .def_property_readonly("distance", &Quadtree::NodeOnRayIterator::GetDistance);

    tree.def(
            "iter_leaf",
            [](Quadtree& self, const uint32_t max_depth) { return py::wrap_iterator(self.BeginLeaf(max_depth), self.EndLeaf()); },
            py::arg("max_depth") = 0)
        .def(
            "iter_leaf_of_node",
            [](Quadtree& self, const QuadtreeKey& node_key, const uint32_t node_depth, const uint32_t max_depth) {
                return py::wrap_iterator(self.BeginLeafOfNode(node_key, node_depth, max_depth), self.EndLeafOfNode());
            },
            py::arg("node_key"),
            py::arg("node_depth"),
            py::arg("max_depth") = 0)
        .def(
            "iter_leaf_in_aabb",
            [](Quadtree& self, const double aabb_min_x, const double aabb_min_y, const double aabb_max_x, const double aabb_max_y, const uint32_t max_depth) {
                return py::wrap_iterator(self.BeginLeafInAabb(aabb_min_x, aabb_min_y, aabb_max_x, aabb_max_y, max_depth), self.EndLeafInAabb());
            },
            py::arg("aabb_min_x"),
            py::arg("aabb_min_y"),
            py::arg("aabb_max_x"),
            py::arg("aabb_max_y"),
            py::arg("max_depth") = 0)
        .def(
            "iter_leaf_in_aabb",
            [](Quadtree& self, const QuadtreeKey& aabb_min_key, const QuadtreeKey& aabb_max_key, const uint32_t max_depth) {
                return py::wrap_iterator(self.BeginLeafInAabb(aabb_min_key, aabb_max_key, max_depth), self.EndLeafInAabb());
            },
            py::arg("aabb_min_key"),
            py::arg("aabb_max_key"),
            py::arg("max_depth") = 0)
        .def(
            "iter_node",
            [](Quadtree& self, const uint32_t max_depth) { return py::wrap_iterator(self.BeginTree(max_depth), self.EndTree()); },
            py::arg("max_depth") = 0)
        .def(
            "iter_node_in_aabb",
            [](Quadtree& self, const double aabb_min_x, const double aabb_min_y, const double aabb_max_x, const double aabb_max_y, const uint32_t max_depth) {
                return py::wrap_iterator(self.BeginTreeInAabb(aabb_min_x, aabb_min_y, aabb_max_x, aabb_max_y, max_depth), self.EndTreeInAabb());
            },
            py::arg("aabb_min_x"),
            py::arg("aabb_min_y"),
            py::arg("aabb_max_x"),
            py::arg("aabb_max_y"),
            py::arg("max_depth") = 0)
        .def(
            "iter_node_in_aabb",
            [](Quadtree& self, const QuadtreeKey& aabb_min_key, const QuadtreeKey& aabb_max_key, const uint32_t max_depth) {
                return py::wrap_iterator(self.BeginTreeInAabb(aabb_min_key, aabb_max_key, max_depth), self.EndTreeInAabb());
            },
            py::arg("aabb_min_key"),
            py::arg("aabb_max_key"),
            py::arg("max_depth") = 0)
        .def(
            "iter_west_leaf_neighbor",
            [](Quadtree& self, const double x, const double y, const uint32_t max_leaf_depth) {
                return py::wrap_iterator(self.BeginWestLeafNeighbor(x, y, max_leaf_depth), self.EndWestLeafNeighbor());
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_west_leaf_neighbor",
            [](Quadtree& self, const QuadtreeKey& key, const uint32_t key_depth, const uint32_t max_leaf_depth) {
                return py::wrap_iterator(self.BeginWestLeafNeighbor(key, key_depth, max_leaf_depth), self.EndWestLeafNeighbor());
            },
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_east_leaf_neighbor",
            [](Quadtree& self, const double x, const double y, const uint32_t max_leaf_depth) {
                return py::wrap_iterator(self.BeginEastLeafNeighbor(x, y, max_leaf_depth), self.EndEastLeafNeighbor());
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_east_leaf_neighbor",
            [](Quadtree& self, const QuadtreeKey& key, const uint32_t key_depth, const uint32_t max_leaf_depth) {
                return py::wrap_iterator(self.BeginEastLeafNeighbor(key, key_depth, max_leaf_depth), self.EndEastLeafNeighbor());
            },
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_north_leaf_neighbor",
            [](Quadtree& self, const double x, const double y, const uint32_t max_leaf_depth) {
                return py::wrap_iterator(self.BeginNorthLeafNeighbor(x, y, max_leaf_depth), self.EndNorthLeafNeighbor());
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_north_leaf_neighbor",
            [](Quadtree& self, const QuadtreeKey& key, const uint32_t key_depth, const uint32_t max_leaf_depth) {
                return py::wrap_iterator(self.BeginNorthLeafNeighbor(key, key_depth, max_leaf_depth), self.EndNorthLeafNeighbor());
            },
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_south_leaf_neighbor",
            [](Quadtree& self, const double x, const double y, const uint32_t max_leaf_depth) {
                return py::wrap_iterator(self.BeginSouthLeafNeighbor(x, y, max_leaf_depth), self.EndSouthLeafNeighbor());
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_south_leaf_neighbor",
            [](Quadtree& self, const QuadtreeKey& key, const uint32_t key_depth, const uint32_t max_leaf_depth) {
                return py::wrap_iterator(self.BeginSouthLeafNeighbor(key, key_depth, max_leaf_depth), self.EndSouthLeafNeighbor());
            },
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_node_on_ray",
            [](Quadtree& self,
               const double px,
               const double py,
               const double vx,
               const double vy,
               const double max_range,
               const double node_padding,
               const bool bidirectional,
               const bool leaf_only,
               const uint32_t min_node_depth,
               const uint32_t max_node_depth) {
                return py::wrap_iterator(
                    self.BeginNodeOnRay(px, py, vx, vy, max_range, node_padding, bidirectional, leaf_only, min_node_depth, max_node_depth),
                    self.EndNodeOnRay());
            },
            py::arg("px"),
            py::arg("py"),
            py::arg("vx"),
            py::arg("vy"),
            py::arg("max_range") = -1,
            py::arg("node_padding") = 0,
            py::arg("bidirectional") = false,
            py::arg("leaf_only") = false,
            py::arg("min_node_depth") = 0,
            py::arg("max_node_depth") = 0);
    BindOccupancyQuadtreeDrawer<OccupancyQuadtreeDrawer<Quadtree>, Quadtree>(tree, "Drawer");

    return tree;
}
