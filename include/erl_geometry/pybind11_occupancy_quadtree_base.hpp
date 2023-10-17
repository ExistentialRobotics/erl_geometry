#pragma once

#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_quadtree_base.hpp"

template<class Quadtree, class Node>
void
BindOccupancyQuadtree(py::module &m, const char *name) {
    py::class_<Quadtree, std::shared_ptr<Quadtree>> tree(m, name);
    using namespace erl::geometry;

    // OccupancyQuadtreeBase methods, except iterators
    tree.def(py::init<>([](double resolution) { return std::make_shared<Quadtree>(resolution); }), py::arg("resolution"))
        .def_property_readonly("tree_type", &Quadtree::GetTreeType)
        .def("is_node_collapsible", &Quadtree::IsNodeCollapsible, py::arg("node"))
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
            py::arg("lazy_eval"))
        .def("insert_ray", &Quadtree::InsertRay, py::arg("sx"), py::arg("sy"), py::arg("ex"), py::arg("ey"), py::arg("max_range"), py::arg("lazy_eval"))
        .def(
            "cast_ray",
            [](const Quadtree &self, double px, double py, double vx, double vy, bool ignore_unknown, double max_range) {
                double ex, ey;
                bool hit = self.CastRay(px, py, vx, vy, ignore_unknown, max_range, ex, ey);
                if (hit) {
                    return std::make_tuple(true, ex, ey);
                } else {
                    return std::make_tuple(false, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
                }
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
            py::overload_cast<const QuadtreeKey &, bool, bool>(&Quadtree::UpdateNode),
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
            py::overload_cast<const QuadtreeKey &, float, bool>(&Quadtree::UpdateNode),
            py::arg("node_key"),
            py::arg("log_odds_delta"),
            py::arg("lazy_eval"))
        .def("update_inner_occupancy", &Quadtree::UpdateInnerOccupancy)
        .def("to_max_likelihood", &Quadtree::ToMaxLikelihood);

    // QuadtreeImpl methods
    tree.def_property_readonly("number_of_nodes", &Quadtree::GetSize)
        .def_property("resolution", &Quadtree::GetResolution, &Quadtree::SetResolution)
        .def_property_readonly("max_tree_depth", &Quadtree::GetTreeDepth)
        .def_property_readonly("tree_key_offset", &Quadtree::GetTreeKeyOffset)
        .def_property_readonly(
            "metric_min",
            [](Quadtree &self) {
                double x, y;
                self.GetMetricMin(x, y);
                return std::make_tuple(x, y);
            })
        .def_property_readonly(
            "metric_max",
            [](Quadtree &self) {
                double x, y;
                self.GetMetricMax(x, y);
                return std::make_tuple(x, y);
            })
        .def_property_readonly(
            "metric_min_max",
            [](Quadtree &self) {
                double min_x, min_y, max_x, max_y;
                self.GetMetricMinMax(min_x, min_y, max_x, max_y);
                return std::make_tuple(std::make_tuple(min_x, min_y), std::make_tuple(max_x, max_y));
            })
        .def_property_readonly(
            "metric_size",
            [](Quadtree &self) {
                double x, y;
                self.GetMetricSize(x, y);
                return std::make_tuple(x, y);
            })
        .def("get_node_size", &Quadtree::GetNodeSize, py::arg("depth"))
        .def_property_readonly("number_of_leaf_nodes", &Quadtree::ComputeNumberOfLeafNodes)
        .def_property_readonly("memory_usage", &Quadtree::GetMemoryUsage)
        .def("coord_to_key", py::overload_cast<double>(&Quadtree::CoordToKey, py::const_), py::arg("coordinate"))
        .def("coord_to_key", py::overload_cast<double, unsigned int>(&Quadtree::CoordToKey, py::const_), py::arg("coordinate"), py::arg("depth"))
        .def("coord_to_key", py::overload_cast<double, double>(&Quadtree::CoordToKey, py::const_), py::arg("x"), py::arg("y"))
        .def("coord_to_key", py::overload_cast<double, double, unsigned int>(&Quadtree::CoordToKey, py::const_), py::arg("x"), py::arg("y"), py::arg("depth"))
        .def(
            "coord_to_key_checked",
            [](const Quadtree &self, double coordinate) {
                QuadtreeKey::KeyType key;
                if (self.CoordToKeyChecked(coordinate, key)) {
                    return std::optional<QuadtreeKey::KeyType>(key);
                } else {
                    return std::optional<QuadtreeKey::KeyType>();
                }
            },
            py::arg("coordinate"))
        .def(
            "coord_to_key_checked",
            [](const Quadtree &self, double coordinate, unsigned int depth) {
                QuadtreeKey key;
                if (self.CoordToKeyChecked(coordinate, depth, key)) {
                    return std::optional<QuadtreeKey>(key);
                } else {
                    return std::optional<QuadtreeKey>();
                }
            },
            py::arg("coordinate"),
            py::arg("depth"))
        .def(
            "coord_to_key_checked",
            [](const Quadtree &self, double x, double y) {
                QuadtreeKey key;
                if (self.CoordToKeyChecked(x, y, key)) {
                    return std::optional<QuadtreeKey>(key);
                } else {
                    return std::optional<QuadtreeKey>();
                }
            },
            py::arg("x"),
            py::arg("y"))
        .def(
            "coord_to_key_checked",
            [](const Quadtree &self, double x, double y, unsigned int depth) {
                QuadtreeKey key;
                if (self.CoordToKeyChecked(x, y, depth, key)) {
                    return std::optional<QuadtreeKey>(key);
                } else {
                    return std::optional<QuadtreeKey>();
                }
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("depth"))
        .def(
            "adjust_key_to_depth",
            py::overload_cast<QuadtreeKey::KeyType, unsigned int>(&Quadtree::AdjustKeyToDepth, py::const_),
            py::arg("key"),
            py::arg("depth"))
        .def(
            "adjust_key_to_depth",
            py::overload_cast<const QuadtreeKey &, unsigned int>(&Quadtree::AdjustKeyToDepth, py::const_),
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_common_ancestor_key",
            [](const Quadtree &self, const QuadtreeKey &key1, const QuadtreeKey &key2) {
                QuadtreeKey key;
                unsigned int ancestor_depth;
                self.ComputeCommonAncestorKey(key1, key2, key, ancestor_depth);
                return std::make_tuple(key, ancestor_depth);
            })
        .def(
            "compute_west_neighbor_key",
            [](const Quadtree &self, const QuadtreeKey &key, unsigned int depth) {
                QuadtreeKey neighbor_key;
                if (self.ComputeWestNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<QuadtreeKey>(neighbor_key);
                } else {
                    return std::optional<QuadtreeKey>();
                }
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_east_neighbor_key",
            [](const Quadtree &self, const QuadtreeKey &key, unsigned int depth) {
                QuadtreeKey neighbor_key;
                if (self.ComputeEastNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<QuadtreeKey>(neighbor_key);
                } else {
                    return std::optional<QuadtreeKey>();
                }
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_north_neighbor_key",
            [](const Quadtree &self, const QuadtreeKey &key, unsigned int depth) {
                QuadtreeKey neighbor_key;
                if (self.ComputeNorthNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<QuadtreeKey>(neighbor_key);
                } else {
                    return std::optional<QuadtreeKey>();
                }
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_south_neighbor_key",
            [](const Quadtree &self, const QuadtreeKey &key, unsigned int depth) {
                QuadtreeKey neighbor_key;
                if (self.ComputeSouthNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<QuadtreeKey>(neighbor_key);
                } else {
                    return std::optional<QuadtreeKey>();
                }
            },
            py::arg("key"),
            py::arg("depth"))
        .def("key_to_coord", py::overload_cast<QuadtreeKey::KeyType>(&Quadtree::KeyToCoord, py::const_), py::arg("key"))
        .def("key_to_coord", py::overload_cast<QuadtreeKey::KeyType, unsigned int>(&Quadtree::KeyToCoord, py::const_), py::arg("key"), py::arg("depth"))
        .def(
            "key_to_coord",
            [](const Quadtree &self, const QuadtreeKey &key) {
                double x, y;
                self.KeyToCoord(key, x, y);
                return std::make_tuple(x, y);
            },
            py::arg("key"))
        .def(
            "key_to_coord",
            [](const Quadtree &self, const QuadtreeKey &key, unsigned int depth) {
                double x, y;
                self.KeyToCoord(key, depth, x, y);
                return std::make_tuple(x, y);
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_ray_keys",
            [](const Quadtree &self, double sx, double sy, double ex, double ey) {
                QuadtreeKeyRay ray;
                if (self.ComputeRayKeys(sx, sy, ex, ey, ray)) {
                    return std::optional<QuadtreeKeyRay>(ray);
                } else {
                    return std::optional<QuadtreeKeyRay>();
                }
            },
            py::arg("sx"),
            py::arg("sy"),
            py::arg("ex"),
            py::arg("ey"))
        .def(
            "compute_ray_coords",
            [](const Quadtree &self, double sx, double sy, double ex, double ey) {
                std::vector<std::array<double, 2>> ray;
                if (self.ComputeRayCoords(sx, sy, ex, ey, ray)) {
                    return std::optional<std::vector<std::array<double, 2>>>(ray);
                } else {
                    return std::optional<std::vector<std::array<double, 2>>>();
                }
            },
            py::arg("sx"),
            py::arg("sy"),
            py::arg("ex"),
            py::arg("ey"))
        .def("create_node_child", &Quadtree::CreateNodeChild, py::arg("node"), py::arg("child_idx"))
        .def("delete_node_child", &Quadtree::DeleteNodeChild, py::arg("node"), py::arg("child_idx"))
        .def("get_node_child", py::overload_cast<std::shared_ptr<Node> &, unsigned int>(&Quadtree::GetNodeChild), py::arg("node"), py::arg("child_idx"))
        .def("expand_node", &Quadtree::ExpandNode, py::arg("node"))
        .def("prune_node", &Quadtree::PruneNode, py::arg("node"))
        .def("delete_node", py::overload_cast<double, double, unsigned int>(&Quadtree::DeleteNode), py::arg("x"), py::arg("y"), py::arg("depth"))
        .def("delete_node", py::overload_cast<const QuadtreeKey &, unsigned int>(&Quadtree::DeleteNode), py::arg("key"), py::arg("depth"))
        .def("clear", &Quadtree::Clear)
        .def("prune", &Quadtree::Prune)
        .def("expand", &Quadtree::Expand)
        .def_property_readonly("root", &Quadtree::GetRoot)
        .def(
            "search",
            [](Quadtree &self, double x, double y) {
                unsigned int depth;
                std::shared_ptr<Node> node = self.Search(x, y, depth);
                return std::make_tuple(node, depth);
            },
            py::arg("x"),
            py::arg("y"))
        .def(
            "search",
            [](Quadtree &self, const QuadtreeKey &key) {
                unsigned int depth;
                std::shared_ptr<Node> node = self.Search(key, depth);
                return std::make_tuple(node, depth);
            },
            py::arg("key"))
        .def("insert_node", py::overload_cast<double, double, unsigned int>(&Quadtree::InsertNode), py::arg("x"), py::arg("y"), py::arg("depth"))
        .def("insert_node", py::overload_cast<const QuadtreeKey &, unsigned int>(&Quadtree::InsertNode), py::arg("key"), py::arg("depth"));

    // Iterators
    py::class_<typename Quadtree::IteratorBase>(tree, "IteratorBase")
        .def("__eq__", [](const typename Quadtree::IteratorBase &self, const typename Quadtree::IteratorBase &other) { return self == other; })
        .def("__ne__", [](const typename Quadtree::IteratorBase &self, const typename Quadtree::IteratorBase &other) { return self != other; })
        .def("get", [](const typename Quadtree::IteratorBase &self) { return *self; })
        .def_property_readonly("x", &Quadtree::IteratorBase::GetX)
        .def_property_readonly("y", &Quadtree::IteratorBase::GetY)
        .def_property_readonly("node_size", &Quadtree::IteratorBase::GetNodeSize)
        .def_property_readonly("depth", &Quadtree::IteratorBase::GetDepth)
        .def_property_readonly("key", &Quadtree::IteratorBase::GetKey)
        .def_property_readonly("index_key", &Quadtree::IteratorBase::GetIndexKey);

    py::class_<typename Quadtree::TreeIterator, typename Quadtree::IteratorBase>(tree, "TreeIterator")
        .def("next", [](typename Quadtree::TreeIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::TreeIterator &self) { return self == typename Quadtree::TreeIterator(); });

    py::class_<typename Quadtree::TreeInAabbIterator, typename Quadtree::IteratorBase>(tree, "TreeInAabbIterator")
        .def("next", [](typename Quadtree::TreeInAabbIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::TreeInAabbIterator &self) { return self == typename Quadtree::TreeInAabbIterator(); });

    py::class_<typename Quadtree::LeafIterator, typename Quadtree::IteratorBase>(tree, "LeafIterator")
        .def("next", [](typename Quadtree::LeafIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::LeafIterator &self) { return self == typename Quadtree::LeafIterator(); });

    py::class_<typename Quadtree::LeafOfNodeIterator, typename Quadtree::IteratorBase>(tree, "LeafOfNodeIterator")
        .def("next", [](typename Quadtree::LeafOfNodeIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::LeafOfNodeIterator &self) { return self == typename Quadtree::LeafOfNodeIterator(); });

    py::class_<typename Quadtree::LeafInAabbIterator, typename Quadtree::IteratorBase>(tree, "LeafInAabbIterator")
        .def("next", [](typename Quadtree::LeafInAabbIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::LeafInAabbIterator &self) { return self == typename Quadtree::LeafInAabbIterator(); });

    py::class_<typename Quadtree::LeafNeighborOnWestIterator, typename Quadtree::IteratorBase>(tree, "LeafNeighborOnWestIterator")
        .def("next", [](typename Quadtree::LeafNeighborOnWestIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::LeafNeighborOnWestIterator &self) {
            return self == typename Quadtree::LeafNeighborOnWestIterator();
        });

    py::class_<typename Quadtree::LeafNeighborOnEastIterator, typename Quadtree::IteratorBase>(tree, "LeafNeighborOnEastIterator")
        .def("next", [](typename Quadtree::LeafNeighborOnEastIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::LeafNeighborOnEastIterator &self) {
            return self == typename Quadtree::LeafNeighborOnEastIterator();
        });

    py::class_<typename Quadtree::LeafNeighborOnNorthIterator, typename Quadtree::IteratorBase>(tree, "LeafNeighborOnNorthIterator")
        .def("next", [](typename Quadtree::LeafNeighborOnNorthIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::LeafNeighborOnNorthIterator &self) {
            return self == typename Quadtree::LeafNeighborOnNorthIterator();
        });

    py::class_<typename Quadtree::LeafNeighborOnSouthIterator, typename Quadtree::IteratorBase>(tree, "LeafNeighborOnSouthIterator")
        .def("next", [](typename Quadtree::LeafNeighborOnSouthIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::LeafNeighborOnSouthIterator &self) {
            return self == typename Quadtree::LeafNeighborOnSouthIterator();
        });

    py::class_<typename Quadtree::LeafOnRayIterator, typename Quadtree::IteratorBase>(tree, "LeafOnRayIterator")
        .def_property_readonly("distance", &Quadtree::LeafOnRayIterator::GetDistance)
        .def("next", [](typename Quadtree::LeafOnRayIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::LeafOnRayIterator &self) { return self == typename Quadtree::LeafOnRayIterator(); });

    py::class_<typename Quadtree::OccupiedLeafOnRayIterator, typename Quadtree::IteratorBase>(tree, "OccupiedLeafOnRayIterator")
        .def("next", [](typename Quadtree::OccupiedLeafOnRayIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::OccupiedLeafOnRayIterator &self) {
            return self == typename Quadtree::OccupiedLeafOnRayIterator();
        });

    tree.def("iter_leaf", &Quadtree::BeginLeaf, py::arg("max_depth") = 0)
        .def("iter_leaf_of_node", &Quadtree::BeginLeafOfNode, py::arg("node_key"), py::arg("node_depth"), py::arg("max_depth") = 0)
        .def(
            "iter_leaf_in_aabb",
            py::overload_cast<double, double, double, double, unsigned int>(&Quadtree::BeginLeafInAabb, py::const_),
            py::arg("aabb_min_x"),
            py::arg("aabb_min_y"),
            py::arg("aabb_max_x"),
            py::arg("aabb_max_y"),
            py::arg("max_depth") = 0)
        .def(
            "iter_leaf_in_aabb",
            py::overload_cast<const QuadtreeKey &, const QuadtreeKey &, unsigned int>(&Quadtree::BeginLeafInAabb, py::const_),
            py::arg("aabb_min_key"),
            py::arg("aabb_max_key"),
            py::arg("max_depth") = 0)
        .def("iter_node", &Quadtree::BeginTree, py::arg("max_depth") = 0)
        .def(
            "iter_node_in_aabb",
            py::overload_cast<double, double, double, double, unsigned int>(&Quadtree::BeginTreeInAabb, py::const_),
            py::arg("aabb_min_x"),
            py::arg("aabb_min_y"),
            py::arg("aabb_max_x"),
            py::arg("aabb_max_y"),
            py::arg("max_depth") = 0)
        .def(
            "iter_node_in_aabb",
            py::overload_cast<const QuadtreeKey &, const QuadtreeKey &, unsigned int>(&Quadtree::BeginTreeInAabb, py::const_),
            py::arg("aabb_min_key"),
            py::arg("aabb_max_key"),
            py::arg("max_depth") = 0)
        .def(
            "iter_leaf_neighbor_on_west",
            py::overload_cast<double, double, unsigned int>(&Quadtree::BeginLeafNeighborOnWest, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_leaf_neighbor_on_west",
            py::overload_cast<const QuadtreeKey &, unsigned int, unsigned int>(&Quadtree::BeginLeafNeighborOnWest, py::const_),
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_leaf_neighbor_on_east",
            py::overload_cast<double, double, unsigned int>(&Quadtree::BeginLeafNeighborOnEast, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_leaf_neighbor_on_east",
            py::overload_cast<const QuadtreeKey &, unsigned int, unsigned int>(&Quadtree::BeginLeafNeighborOnEast, py::const_),
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_leaf_neighbor_on_north",
            py::overload_cast<double, double, unsigned int>(&Quadtree::BeginLeafNeighborOnNorth, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("max_leaf_depth") = 0

            )
        .def(
            "iter_leaf_neighbor_on_north",
            py::overload_cast<const QuadtreeKey &, unsigned int, unsigned int>(&Quadtree::BeginLeafNeighborOnNorth, py::const_),
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_leaf_neighbor_on_south",
            py::overload_cast<double, double, unsigned int>(&Quadtree::BeginLeafNeighborOnSouth, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_leaf_neighbor_on_south",
            py::overload_cast<const QuadtreeKey &, unsigned int, unsigned int>(&Quadtree::BeginLeafNeighborOnSouth, py::const_),
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_leaf_on_ray",
            &Quadtree::BeginLeafOnRay,
            py::arg("px"),
            py::arg("py"),
            py::arg("vx"),
            py::arg("vy"),
            py::arg("max_range") = -1,
            py::arg("bidirectional") = false,
            py::arg("max_leaf_depth") = 0);
}
