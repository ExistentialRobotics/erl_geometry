#pragma once

#include "occupancy_octree_base.hpp"
#include "occupancy_octree_drawer.hpp"
#include "open3d_visualizer_wrapper.hpp"
#include "pybind11_occupancy_octree_drawer.hpp"

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
        .def_property_readonly("tree_type", &Octree::GetTreeType)
        .def_property_readonly("setting", &Octree::template GetSetting<typename Octree::Setting>)
        .def(
            "insert_point_cloud",
            &Octree::InsertPointCloud,
            py::arg("points"),
            py::arg("sensor_origin"),
            py::arg("max_range"),
            py::arg("parallel"),
            py::arg("lazy_eval"),
            py::arg("discretize"))
        .def(
            "insert_point_cloud_rays",
            &Octree::InsertPointCloudRays,
            py::arg("points"),
            py::arg("sensor_origin"),
            py::arg("max_range"),
            py::arg("parallel"),
            py::arg("lazy_eval"))
        .def(
            "insert_ray",
            &Octree::InsertRay,
            py::arg("sx"),
            py::arg("sy"),
            py::arg("sz"),
            py::arg("ex"),
            py::arg("ey"),
            py::arg("ez"),
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

    // OctreeImpl methods
    tree.def_property_readonly("number_of_nodes", &Octree::GetSize)
        .def_property_readonly("resolution", &Octree::GetResolution)
        .def_property_readonly("tree_depth", &Octree::GetTreeDepth)
        .def_property_readonly("tree_center", &Octree::GetTreeCenter)
        .def_property_readonly("tree_center_key", &Octree::GetTreeCenterKey)
        .def_property_readonly("tree_max_half_size", &Octree::GetTreeMaxHalfSize)
        .def_property_readonly("metric_min", [](Octree &self) { return self.GetMetricMin(); })
        .def_property_readonly("metric_max", [](Octree &self) { return self.GetMetricMax(); })
        .def_property_readonly(
            "metric_min_max",
            [](Octree &self) { return self.GetMetricMinMax(); })
        .def_property_readonly("metric_aabb", [](Octree &self) { return self.GetMetricAabb(); })
        .def_property_readonly("metric_size", [](Octree &self) { return self.GetMetricSize(); })
        .def("get_node_size", &Octree::GetNodeSize, py::arg("depth"))
        .def_property_readonly("number_of_leaf_nodes", &Octree::ComputeNumberOfLeafNodes)
        .def_property_readonly("memory_usage", &Octree::GetMemoryUsage)
        .def_property_readonly("memory_usage_per_node", &Octree::GetMemoryUsagePerNode)
        .def(
            "coord_to_key",
            py::overload_cast<Dtype>(&Octree::CoordToKey, py::const_),
            py::arg("coordinate"))
        .def(
            "coord_to_key",
            py::overload_cast<Dtype, uint32_t>(&Octree::CoordToKey, py::const_),
            py::arg("coordinate"),
            py::arg("depth"))
        .def(
            "coord_to_key",
            py::overload_cast<Dtype, Dtype, Dtype>(&Octree::CoordToKey, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"))
        .def(
            "coord_to_key",
            py::overload_cast<Dtype, Dtype, Dtype, uint32_t>(&Octree::CoordToKey, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("depth"))
        .def(
            "coord_to_key_checked",
            [](const Octree &self, Dtype coordinate) {
                if (OctreeKey::KeyType key; self.CoordToKeyChecked(coordinate, key)) {
                    return std::optional<OctreeKey::KeyType>(key);
                }
                return std::optional<OctreeKey::KeyType>();
            },
            py::arg("coordinate"))
        .def(
            "coord_to_key_checked",
            [](const Octree &self, Dtype coordinate, uint32_t depth) {
                if (OctreeKey::KeyType key; self.CoordToKeyChecked(coordinate, depth, key)) {
                    return std::optional<OctreeKey::KeyType>(key);
                }
                return std::optional<OctreeKey::KeyType>();
            },
            py::arg("coordinate"),
            py::arg("depth"))
        .def(
            "coord_to_key_checked",
            [](const Octree &self, Dtype x, Dtype y, Dtype z) {
                if (OctreeKey key; self.CoordToKeyChecked(x, y, z, key)) {
                    return std::optional<OctreeKey>(key);
                }
                return std::optional<OctreeKey>();
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("z"))
        .def(
            "coord_to_key_checked",
            [](const Octree &self, Dtype x, Dtype y, Dtype z, uint32_t depth) {
                if (OctreeKey key; self.CoordToKeyChecked(x, y, z, depth, key)) {
                    return std::optional<OctreeKey>(key);
                }
                return std::optional<OctreeKey>();
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("depth"))
        .def(
            "adjust_key_to_depth",
            py::overload_cast<OctreeKey::KeyType, uint32_t>(&Octree::AdjustKeyToDepth, py::const_),
            py::arg("key"),
            py::arg("depth"))
        .def(
            "adjust_key_to_depth",
            py::overload_cast<const OctreeKey &, uint32_t>(&Octree::AdjustKeyToDepth, py::const_),
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_common_ancestor_key",
            [](const Octree &self, const OctreeKey &key1, const OctreeKey &key2) {
                OctreeKey key;
                uint32_t ancestor_depth;
                self.ComputeCommonAncestorKey(key1, key2, key, ancestor_depth);
                return std::make_tuple(key, ancestor_depth);
            })
        .def(
            "compute_west_neighbor_key",
            [](const Octree &self, const OctreeKey &key, uint32_t depth) {
                if (OctreeKey neighbor_key; self.ComputeWestNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<OctreeKey>(neighbor_key);
                }
                return std::optional<OctreeKey>();
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_east_neighbor_key",
            [](const Octree &self, const OctreeKey &key, uint32_t depth) {
                if (OctreeKey neighbor_key; self.ComputeEastNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<OctreeKey>(neighbor_key);
                }
                return std::optional<OctreeKey>();
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_north_neighbor_key",
            [](const Octree &self, const OctreeKey &key, uint32_t depth) {
                if (OctreeKey neighbor_key;
                    self.ComputeNorthNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<OctreeKey>(neighbor_key);
                }
                return std::optional<OctreeKey>();
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_south_neighbor_key",
            [](const Octree &self, const OctreeKey &key, uint32_t depth) {
                if (OctreeKey neighbor_key;
                    self.ComputeSouthNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<OctreeKey>(neighbor_key);
                }
                return std::optional<OctreeKey>();
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_top_neighbor_key",
            [](const Octree &self, const OctreeKey &key, uint32_t depth) {
                if (OctreeKey neighbor_key; self.ComputeTopNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<OctreeKey>(neighbor_key);
                }
                return std::optional<OctreeKey>();
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_bottom_neighbor_key",
            [](const Octree &self, const OctreeKey &key, uint32_t depth) {
                if (OctreeKey neighbor_key;
                    self.ComputeBottomNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<OctreeKey>(neighbor_key);
                }
                return std::optional<OctreeKey>();
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "key_to_coord",
            py::overload_cast<OctreeKey::KeyType>(&Octree::KeyToCoord, py::const_),
            py::arg("key"))
        .def(
            "key_to_coord",
            py::overload_cast<OctreeKey::KeyType, uint32_t>(&Octree::KeyToCoord, py::const_),
            py::arg("key"),
            py::arg("depth"))
        .def(
            "key_to_coord",
            [](const Octree &self, const OctreeKey &key) {
                Dtype x, y, z;
                self.KeyToCoord(key, x, y, z);
                return std::make_tuple(x, y, z);
            },
            py::arg("key"))
        .def(
            "key_to_coord",
            [](const Octree &self, const OctreeKey &key, uint32_t depth) {
                Dtype x, y, z;
                self.KeyToCoord(key, depth, x, y, z);
                return std::make_tuple(x, y, z);
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_ray_keys",
            [](const Octree &self, Dtype sx, Dtype sy, Dtype sz, Dtype ex, Dtype ey, Dtype ez) {
                if (OctreeKeyRay ray; self.ComputeRayKeys(sx, sy, sz, ex, ey, ez, ray)) {
                    return std::optional<OctreeKeyRay>(ray);
                }
                return std::optional<OctreeKeyRay>();
            },
            py::arg("sx"),
            py::arg("sy"),
            py::arg("sz"),
            py::arg("ex"),
            py::arg("ey"),
            py::arg("ez"))
        .def(
            "compute_ray_coords",
            [](const Octree &self, Dtype sx, Dtype sy, Dtype sz, Dtype ex, Dtype ey, Dtype ez) {
                if (std::vector<Vector3> ray; self.ComputeRayCoords(sx, sy, sz, ex, ey, ez, ray)) {
                    return std::optional<std::vector<Vector3>>(ray);
                }
                return std::optional<std::vector<Vector3>>();
            },
            py::arg("sx"),
            py::arg("sy"),
            py::arg("sz"),
            py::arg("ex"),
            py::arg("ey"),
            py::arg("ez"))
        .def("create_node_child", &Octree::CreateNodeChild, py::arg("node"), py::arg("child_idx"))
        .def(
            "delete_node_child",
            &Octree::DeleteNodeChild,
            py::arg("node"),
            py::arg("child_idx"),
            py::arg("key"))
        .def(
            "get_node_child",
            py::overload_cast<Node *, uint32_t>(&Octree::GetNodeChild),
            py::arg("node"),
            py::arg("child_idx"))
        .def("is_node_collapsible", &Octree::IsNodeCollapsible, py::arg("node"))
        .def("expand_node", &Octree::ExpandNode, py::arg("node"))
        .def("prune_node", &Octree::PruneNode, py::arg("node"))
        .def(
            "delete_node",
            py::overload_cast<Dtype, Dtype, Dtype, uint32_t>(&Octree::DeleteNode),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("depth"))
        .def(
            "delete_node",
            py::overload_cast<const OctreeKey &, uint32_t>(&Octree::DeleteNode),
            py::arg("key"),
            py::arg("depth"))
        .def("clear", &Octree::Clear)
        .def("prune", &Octree::Prune)
        .def("expand", &Octree::Expand)
        .def_property_readonly("root", &Octree::GetRoot)
        .def(
            "search",
            py::overload_cast<Dtype, Dtype, Dtype, uint32_t>(&Octree::Search, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("max_depth") = 0)
        .def(
            "search",
            py::overload_cast<const OctreeKey &, uint32_t>(&Octree::Search, py::const_),
            py::arg("key"),
            py::arg("max_depth") = 0)
        .def(
            "insert_node",
            py::overload_cast<Dtype, Dtype, Dtype, uint32_t>(&Octree::InsertNode),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("depth"))
        .def(
            "insert_node",
            py::overload_cast<const OctreeKey &, uint32_t>(&Octree::InsertNode),
            py::arg("key"),
            py::arg("depth"))
        .def(
            "visualize",
            [](std::shared_ptr<Octree> &self,
               const bool leaf_only,
               Vector3 occupied_color,
               Vector3 border_color,
               const int window_width,
               const int window_height,
               const int window_left,
               const int window_top) {
                using Drawer = OccupancyOctreeDrawer<Octree>;
                auto drawer_setting = std::make_shared<typename Drawer::Setting>();
                for (int i = 0; i < 3; ++i) {
                    drawer_setting->occupied_color[0] = occupied_color[0];
                    drawer_setting->occupied_color[1] = occupied_color[1];
                    drawer_setting->occupied_color[2] = occupied_color[2];

                    drawer_setting->border_color[0] = border_color[0];
                    drawer_setting->border_color[1] = border_color[1];
                    drawer_setting->border_color[2] = border_color[2];
                }

                auto drawer = std::make_shared<Drawer>(drawer_setting, self);
                auto visualizer_setting = std::make_shared<Open3dVisualizerWrapper::Setting>();
                visualizer_setting->window_width = window_width;
                visualizer_setting->window_height = window_height;
                visualizer_setting->window_left = window_left;
                visualizer_setting->window_top = window_top;
                const auto visualizer =
                    std::make_shared<Open3dVisualizerWrapper>(visualizer_setting);
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
            py::arg("occupied_color") = Vector3(0.5, 0.5, 0.5),
            py::arg("border_color") = Vector3(0.0, 0.0, 0.0),
            py::arg("window_width") = 1920,
            py::arg("window_height") = 1080,
            py::arg("window_left") = 50,
            py::arg("window_top") = 50);

    // Iterators defined in OctreeImpl
    py::class_<typename Octree::IteratorBase, typename AbstractOctree<Dtype>::OctreeNodeIterator>(
        tree,
        "IteratorBase")
        .def(
            "__eq__",
            [](const typename Octree::IteratorBase &self,
               const typename Octree::IteratorBase &other) { return self == other; })
        .def(
            "__ne__",
            [](const typename Octree::IteratorBase &self,
               const typename Octree::IteratorBase &other) { return self != other; })
        .def_property_readonly("node", py::overload_cast<>(&Octree::IteratorBase::GetNode))
        .def_property_readonly("node_aabb", &Octree::IteratorBase::GetNodeAabb)
        .def_property_readonly("key", &Octree::IteratorBase::GetKey)
        .def_property_readonly("index_key", &Octree::IteratorBase::GetIndexKey);

    (void) py::class_<typename Octree::TreeIterator, typename Octree::IteratorBase>(
        tree,
        "TreeIterator");
    (void) py::class_<typename Octree::TreeInAabbIterator, typename Octree::IteratorBase>(
        tree,
        "TreeInAabbIterator");
    (void) py::class_<typename Octree::LeafIterator, typename Octree::IteratorBase>(
        tree,
        "LeafIterator");
    (void) py::class_<typename Octree::LeafOfNodeIterator, typename Octree::IteratorBase>(
        tree,
        "LeafOfNodeIterator");
    (void) py::class_<typename Octree::LeafInAabbIterator, typename Octree::IteratorBase>(
        tree,
        "LeafInAabbIterator");
    (void) py::class_<typename Octree::WestLeafNeighborIterator, typename Octree::IteratorBase>(
        tree,
        "WestLeafNeighborIterator");
    (void) py::class_<typename Octree::EastLeafNeighborIterator, typename Octree::IteratorBase>(
        tree,
        "EastLeafNeighborIterator");
    (void) py::class_<typename Octree::NorthLeafNeighborIterator, typename Octree::IteratorBase>(
        tree,
        "NorthLeafNeighborIterator");
    (void) py::class_<typename Octree::SouthLeafNeighborIterator, typename Octree::IteratorBase>(
        tree,
        "SouthLeafNeighborIterator");
    (void) py::class_<typename Octree::TopLeafNeighborIterator, typename Octree::IteratorBase>(
        tree,
        "TopLeafNeighborIterator");
    (void) py::class_<typename Octree::BottomLeafNeighborIterator, typename Octree::IteratorBase>(
        tree,
        "BottomLeafNeighborIterator");
    py::class_<typename Octree::NodeOnRayIterator, typename Octree::IteratorBase>(
        tree,
        "NodeOnRayIterator")
        .def_property_readonly("distance", &Octree::NodeOnRayIterator::GetDistance);

    tree.def(
            "iter_leaf",
            [](Octree &self, const uint32_t max_depth) {
                return py::wrap_iterator(self.BeginLeaf(max_depth), self.EndLeaf());
            },
            py::arg("max_depth") = 0)
        .def(
            "iter_leaf_of_node",
            [](Octree &self,
               const OctreeKey &node_key,
               const uint32_t node_depth,
               const uint32_t max_depth) {
                return py::wrap_iterator(
                    self.BeginLeafOfNode(node_key, node_depth, max_depth),
                    self.EndLeafOfNode());
            },
            py::arg("node_key"),
            py::arg("node_depth"),
            py::arg("max_depth") = 0)
        .def(
            "iter_leaf_in_aabb",
            [](Octree &self,
               const Dtype aabb_min_x,
               const Dtype aabb_min_y,
               const Dtype aabb_min_z,
               const Dtype aabb_max_x,
               const Dtype aabb_max_y,
               const Dtype aabb_max_z,
               const uint32_t max_depth) {
                return py::wrap_iterator(
                    self.BeginLeafInAabb(
                        aabb_min_x,
                        aabb_min_y,
                        aabb_min_z,
                        aabb_max_x,
                        aabb_max_y,
                        aabb_max_z,
                        max_depth),
                    self.EndLeafInAabb());
            },
            py::arg("aabb_min_x"),
            py::arg("aabb_min_y"),
            py::arg("aabb_min_z"),
            py::arg("aabb_max_x"),
            py::arg("aabb_max_y"),
            py::arg("aabb_max_z"),
            py::arg("max_depth") = 0)
        .def(
            "iter_leaf_in_aabb",
            [](Octree &self,
               const OctreeKey &aabb_min_key,
               const OctreeKey &aabb_max_key,
               const uint32_t max_depth) {
                return py::wrap_iterator(
                    self.BeginLeafInAabb(aabb_min_key, aabb_max_key, max_depth),
                    self.EndLeafInAabb());
            },
            py::arg("aabb_min_key"),
            py::arg("aabb_max_key"),
            py::arg("max_depth") = 0)
        .def(
            "iter_node",
            [](Octree &self, const uint32_t max_depth) {
                return py::wrap_iterator(self.BeginTree(max_depth), self.EndTree());
            },
            py::arg("max_depth") = 0)
        .def(
            "iter_node_in_aabb",
            [](Octree &self,
               const Dtype aabb_min_x,
               const Dtype aabb_min_y,
               const Dtype aabb_min_z,
               const Dtype aabb_max_x,
               const Dtype aabb_max_y,
               const Dtype aabb_max_z,
               const uint32_t max_depth) {
                return py::wrap_iterator(
                    self.BeginTreeInAabb(
                        aabb_min_x,
                        aabb_min_y,
                        aabb_min_z,
                        aabb_max_x,
                        aabb_max_y,
                        aabb_max_z,
                        max_depth),
                    self.EndTreeInAabb());
            },
            py::arg("aabb_min_x"),
            py::arg("aabb_min_y"),
            py::arg("aabb_min_z"),
            py::arg("aabb_max_x"),
            py::arg("aabb_max_y"),
            py::arg("aabb_max_z"),
            py::arg("max_depth") = 0)
        .def(
            "iter_node_in_aabb",
            [](Octree &self,
               const OctreeKey &aabb_min_key,
               const OctreeKey &aabb_max_key,
               const uint32_t max_depth) {
                return py::wrap_iterator(
                    self.BeginTreeInAabb(aabb_min_key, aabb_max_key, max_depth),
                    self.EndTreeInAabb());
            },
            py::arg("aabb_min_key"),
            py::arg("aabb_max_key"),
            py::arg("max_depth") = 0)
        .def(
            "iter_west_leaf_neighbor",
            [](Octree &self,
               const Dtype x,
               const Dtype y,
               const Dtype z,
               const uint32_t max_leaf_depth) {
                return py::wrap_iterator(
                    self.BeginWestLeafNeighbor(x, y, z, max_leaf_depth),
                    self.EndWestLeafNeighbor());
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_west_leaf_neighbor",
            [](Octree &self,
               const OctreeKey &key,
               const uint32_t key_depth,
               const uint32_t max_leaf_depth) {
                return py::wrap_iterator(
                    self.BeginWestLeafNeighbor(key, key_depth, max_leaf_depth),
                    self.EndWestLeafNeighbor());
            },
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_east_leaf_neighbor",
            [](Octree &self,
               const Dtype x,
               const Dtype y,
               const Dtype z,
               const uint32_t max_leaf_depth) {
                return py::wrap_iterator(
                    self.BeginEastLeafNeighbor(x, y, z, max_leaf_depth),
                    self.EndEastLeafNeighbor());
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_east_leaf_neighbor",
            [](Octree &self,
               const OctreeKey &key,
               const uint32_t key_depth,
               const uint32_t max_leaf_depth) {
                return py::wrap_iterator(
                    self.BeginEastLeafNeighbor(key, key_depth, max_leaf_depth),
                    self.EndEastLeafNeighbor());
            },
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_north_leaf_neighbor",
            [](Octree &self,
               const Dtype x,
               const Dtype y,
               const Dtype z,
               const uint32_t max_leaf_depth) {
                return py::wrap_iterator(
                    self.BeginNorthLeafNeighbor(x, y, z, max_leaf_depth),
                    self.EndNorthLeafNeighbor());
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_north_leaf_neighbor",
            [](Octree &self,
               const OctreeKey &key,
               const uint32_t key_depth,
               const uint32_t max_leaf_depth) {
                return py::wrap_iterator(
                    self.BeginNorthLeafNeighbor(key, key_depth, max_leaf_depth),
                    self.EndNorthLeafNeighbor());
            },
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_south_leaf_neighbor",
            [](Octree &self,
               const Dtype x,
               const Dtype y,
               const Dtype z,
               const uint32_t max_leaf_depth) {
                return py::wrap_iterator(
                    self.BeginSouthLeafNeighbor(x, y, z, max_leaf_depth),
                    self.EndSouthLeafNeighbor());
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_south_leaf_neighbor",
            [](Octree &self,
               const OctreeKey &key,
               const uint32_t key_depth,
               const uint32_t max_leaf_depth) {
                return py::wrap_iterator(
                    self.BeginSouthLeafNeighbor(key, key_depth, max_leaf_depth),
                    self.EndSouthLeafNeighbor());
            },
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_top_leaf_neighbor",
            [](Octree &self,
               const Dtype x,
               const Dtype y,
               const Dtype z,
               const uint32_t max_leaf_depth) {
                return py::wrap_iterator(
                    self.BeginTopLeafNeighbor(x, y, z, max_leaf_depth),
                    self.EndTopLeafNeighbor());
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_top_leaf_neighbor",
            [](Octree &self,
               const OctreeKey &key,
               const uint32_t key_depth,
               const uint32_t max_leaf_depth) {
                return py::wrap_iterator(
                    self.BeginTopLeafNeighbor(key, key_depth, max_leaf_depth),
                    self.EndTopLeafNeighbor());
            },
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_bottom_leaf_neighbor",
            [](Octree &self,
               const Dtype x,
               const Dtype y,
               const Dtype z,
               const uint32_t max_leaf_depth) {
                return py::wrap_iterator(
                    self.BeginBottomLeafNeighbor(x, y, z, max_leaf_depth),
                    self.EndBottomLeafNeighbor());
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_bottom_leaf_neighbor",
            [](Octree &self,
               const OctreeKey &key,
               const uint32_t key_depth,
               const uint32_t max_leaf_depth) {
                return py::wrap_iterator(
                    self.BeginBottomLeafNeighbor(key, key_depth, max_leaf_depth),
                    self.EndBottomLeafNeighbor());
            },
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_node_on_ray",
            [](Octree &self,
               const Dtype px,
               const Dtype py,
               const Dtype pz,
               const Dtype vx,
               const Dtype vy,
               const Dtype vz,
               const Dtype max_range,
               const Dtype node_padding,
               const bool bidirectional,
               const bool leaf_only,
               const uint32_t min_node_depth,
               const uint32_t max_node_depth) {
                return py::wrap_iterator(
                    self.BeginNodeOnRay(
                        px,
                        py,
                        pz,
                        vx,
                        vy,
                        vz,
                        max_range,
                        node_padding,
                        bidirectional,
                        leaf_only,
                        min_node_depth,
                        max_node_depth),
                    self.EndNodeOnRay());
            },
            py::arg("px"),
            py::arg("py"),
            py::arg("pz"),
            py::arg("vx"),
            py::arg("vy"),
            py::arg("vz"),
            py::arg("max_range") = -1,
            py::arg("node_padding") = 0,
            py::arg("bidirectional") = false,
            py::arg("leaf_only") = true,
            py::arg("min_node_depth") = 0,
            py::arg("max_node_depth") = 0);
    BindOccupancyOctreeDrawer<Octree>(tree, "Drawer");

    return tree;
}
