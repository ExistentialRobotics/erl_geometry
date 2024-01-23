#pragma once

#include "erl_common/pybind11.hpp"
#include "erl_common/yaml.hpp"
#include "occupancy_octree_base.hpp"
#include "open3d_visualizer_wrapper.hpp"

template<class Octree, class Node>
void
BindOccupancyOctree(py::module &m, const char *name) {
    py::class_<Octree, std::shared_ptr<Octree>> tree(m, name);
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<typename Octree::Setting, YamlableBase, std::shared_ptr<typename Octree::Setting>>(tree, "Setting")
        .def_readwrite("log_odd_min", &Octree::Setting::log_odd_min)
        .def_readwrite("log_odd_max", &Octree::Setting::log_odd_max)
        .def_readwrite("probability_hit", &Octree::Setting::probability_hit)
        .def_readwrite("probability_miss", &Octree::Setting::probability_miss)
        .def_readwrite("probability_occupied", &Octree::Setting::probability_occupied)
        .def_readwrite("resolution", &Octree::Setting::resolution)
        .def_readwrite("use_change_detection", &Octree::Setting::use_change_detection)
        .def_readwrite("use_aabb_limit", &Octree::Setting::use_aabb_limit)
        .def_readwrite("aabb", &Octree::Setting::aabb);

    // OccupancyOctreeBase methods, except iterators
    tree.def(py::init<>([](double resolution) { return std::make_shared<Octree>(resolution); }), py::arg("resolution"))
        .def(py::init<>([](const std::shared_ptr<typename Octree::Setting> &setting) { return std::make_shared<Octree>(setting); }), py::arg("setting"))
        .def_property_readonly("tree_type", &Octree::GetTreeType)
        .def_property("setting", &Octree::GetSetting, &Octree::SetSetting)
        .def("is_node_collapsible", &Octree::IsNodeCollapsible, py::arg("node"))
        .def(
            "insert_point_cloud",
            &Octree::InsertPointCloud,
            py::arg("points"),
            py::arg("sensor_origin"),
            py::arg("max_range"),
            py::arg("parallel"),
            py::arg("lazy_eval"),
            py::arg("discretize"))
        .def("insert_point_cloud_rays", &Octree::InsertPointCloudRays, py::arg("points"), py::arg("sensor_origin"), py::arg("max_range"), py::arg("lazy_eval"))
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
            "cast_ray",
            [](const Octree &self, double px, double py, double pz, double vx, double vy, double vz, bool ignore_unknown, double max_range) {
                double ex, ey, ez;
                bool hit = self.CastRay(px, py, pz, vx, vy, vz, ignore_unknown, max_range, ex, ey, ez);
                if (hit) {
                    return std::make_tuple(true, ex, ey, ez);
                } else {
                    return std::make_tuple(
                        false,
                        std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::infinity());
                }
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
            py::overload_cast<double, double, double, bool, bool>(&Octree::UpdateNode),
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
            py::overload_cast<double, double, double, float, bool>(&Octree::UpdateNode),
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
        .def_property("resolution", &Octree::GetResolution, &Octree::SetResolution)
        .def_property_readonly("max_tree_depth", &Octree::GetTreeDepth)
        .def_property_readonly("tree_key_offset", &Octree::GetTreeKeyOffset)
        .def_property_readonly(
            "metric_min",
            [](Octree &self) {
                double x, y, z;
                self.GetMetricMin(x, y, z);
                return std::make_tuple(x, y, z);
            })
        .def_property_readonly(
            "metric_max",
            [](Octree &self) {
                double x, y, z;
                self.GetMetricMax(x, y, z);
                return std::make_tuple(x, y, z);
            })
        .def_property_readonly(
            "metric_min_max",
            [](Octree &self) {
                double min_x, min_y, min_z, max_x, max_y, max_z;
                self.GetMetricMinMax(min_x, min_y, min_z, max_x, max_y, max_z);
                return std::make_tuple(std::make_tuple(min_x, min_y, min_z), std::make_tuple(max_x, max_y, max_z));
            })
        .def_property_readonly(
            "metric_size",
            [](Octree &self) {
                double x, y, z;
                self.GetMetricSize(x, y, z);
                return std::make_tuple(x, y, z);
            })
        .def("get_node_size", &Octree::GetNodeSize, py::arg("depth"))
        .def_property_readonly("number_of_leaf_nodes", &Octree::ComputeNumberOfLeafNodes)
        .def_property_readonly("memory_usage", &Octree::GetMemoryUsage)
        .def("coord_to_key", py::overload_cast<double>(&Octree::CoordToKey, py::const_), py::arg("coordinate"))
        .def("coord_to_key", py::overload_cast<double, unsigned int>(&Octree::CoordToKey, py::const_), py::arg("coordinate"), py::arg("depth"))
        .def("coord_to_key", py::overload_cast<double, double, double>(&Octree::CoordToKey, py::const_), py::arg("x"), py::arg("y"), py::arg("z"))
        .def(
            "coord_to_key",
            py::overload_cast<double, double, double, unsigned int>(&Octree::CoordToKey, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("depth"))
        .def(
            "coord_to_key_checked",
            [](const Octree &self, double coordinate) {
                OctreeKey::KeyType key;
                if (self.CoordToKeyChecked(coordinate, key)) {
                    return std::optional<OctreeKey::KeyType>(key);
                } else {
                    return std::optional<OctreeKey::KeyType>();
                }
            },
            py::arg("coordinate"))
        .def(
            "coord_to_key_checked",
            [](const Octree &self, double coordinate, unsigned int depth) {
                OctreeKey::KeyType key;
                if (self.CoordToKeyChecked(coordinate, depth, key)) {
                    return std::optional<OctreeKey::KeyType>(key);
                } else {
                    return std::optional<OctreeKey::KeyType>();
                }
            },
            py::arg("coordinate"),
            py::arg("depth"))
        .def(
            "coord_to_key_checked",
            [](const Octree &self, double x, double y, double z) {
                OctreeKey key;
                if (self.CoordToKeyChecked(x, y, z, key)) {
                    return std::optional<OctreeKey>(key);
                } else {
                    return std::optional<OctreeKey>();
                }
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("z"))
        .def(
            "coord_to_key_checked",
            [](const Octree &self, double x, double y, double z, unsigned int depth) {
                OctreeKey key;
                if (self.CoordToKeyChecked(x, y, z, depth, key)) {
                    return std::optional<OctreeKey>(key);
                } else {
                    return std::optional<OctreeKey>();
                }
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("depth"))
        .def(
            "adjust_key_to_depth",
            py::overload_cast<OctreeKey::KeyType, unsigned int>(&Octree::AdjustKeyToDepth, py::const_),
            py::arg("key"),
            py::arg("depth"))
        .def("adjust_key_to_depth", py::overload_cast<const OctreeKey &, unsigned int>(&Octree::AdjustKeyToDepth, py::const_), py::arg("key"), py::arg("depth"))
        .def(
            "compute_common_ancestor_key",
            [](const Octree &self, const OctreeKey &key1, const OctreeKey &key2) {
                OctreeKey key;
                unsigned int ancestor_depth;
                self.ComputeCommonAncestorKey(key1, key2, key, ancestor_depth);
                return std::make_tuple(key, ancestor_depth);
            })
        .def(
            "compute_west_neighbor_key",
            [](const Octree &self, const OctreeKey &key, unsigned int depth) {
                OctreeKey neighbor_key;
                if (self.ComputeWestNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<OctreeKey>(neighbor_key);
                } else {
                    return std::optional<OctreeKey>();
                }
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_east_neighbor_key",
            [](const Octree &self, const OctreeKey &key, unsigned int depth) {
                OctreeKey neighbor_key;
                if (self.ComputeEastNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<OctreeKey>(neighbor_key);
                } else {
                    return std::optional<OctreeKey>();
                }
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_north_neighbor_key",
            [](const Octree &self, const OctreeKey &key, unsigned int depth) {
                OctreeKey neighbor_key;
                if (self.ComputeNorthNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<OctreeKey>(neighbor_key);
                } else {
                    return std::optional<OctreeKey>();
                }
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_south_neighbor_key",
            [](const Octree &self, const OctreeKey &key, unsigned int depth) {
                OctreeKey neighbor_key;
                if (self.ComputeSouthNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<OctreeKey>(neighbor_key);
                } else {
                    return std::optional<OctreeKey>();
                }
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_top_neighbor_key",
            [](const Octree &self, const OctreeKey &key, unsigned int depth) {
                OctreeKey neighbor_key;
                if (self.ComputeTopNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<OctreeKey>(neighbor_key);
                } else {
                    return std::optional<OctreeKey>();
                }
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_bottom_neighbor_key",
            [](const Octree &self, const OctreeKey &key, unsigned int depth) {
                OctreeKey neighbor_key;
                if (self.ComputeBottomNeighborKey(key, depth, neighbor_key)) {
                    return std::optional<OctreeKey>(neighbor_key);
                } else {
                    return std::optional<OctreeKey>();
                }
            },
            py::arg("key"),
            py::arg("depth"))
        .def("key_to_coord", py::overload_cast<OctreeKey::KeyType>(&Octree::KeyToCoord, py::const_), py::arg("key"))
        .def("key_to_coord", py::overload_cast<OctreeKey::KeyType, unsigned int>(&Octree::KeyToCoord, py::const_), py::arg("key"), py::arg("depth"))
        .def(
            "key_to_coord",
            [](const Octree &self, const OctreeKey &key) {
                double x, y, z;
                self.KeyToCoord(key, x, y, z);
                return std::make_tuple(x, y, z);
            },
            py::arg("key"))
        .def(
            "key_to_coord",
            [](const Octree &self, const OctreeKey &key, unsigned int depth) {
                double x, y, z;
                self.KeyToCoord(key, depth, x, y, z);
                return std::make_tuple(x, y, z);
            },
            py::arg("key"),
            py::arg("depth"))
        .def(
            "compute_ray_keys",
            [](const Octree &self, double sx, double sy, double sz, double ex, double ey, double ez) {
                OctreeKeyRay ray;
                if (self.ComputeRayKeys(sx, sy, sz, ex, ey, ez, ray)) {
                    return std::optional<OctreeKeyRay>(ray);
                } else {
                    return std::optional<OctreeKeyRay>();
                }
            },
            py::arg("sx"),
            py::arg("sy"),
            py::arg("sz"),
            py::arg("ex"),
            py::arg("ey"),
            py::arg("ez"))
        .def(
            "compute_ray_coords",
            [](const Octree &self, double sx, double sy, double sz, double ex, double ey, double ez) {
                std::vector<std::array<double, 3>> ray;
                if (self.ComputeRayCoords(sx, sy, sz, ex, ey, ez, ray)) {
                    return std::optional<std::vector<std::array<double, 3>>>(ray);
                } else {
                    return std::optional<std::vector<std::array<double, 3>>>();
                }
            },
            py::arg("sx"),
            py::arg("sy"),
            py::arg("sz"),
            py::arg("ex"),
            py::arg("ey"),
            py::arg("ez"))
        .def("create_node_child", &Octree::CreateNodeChild, py::arg("node"), py::arg("child_idx"))
        .def("delete_node_child", &Octree::DeleteNodeChild, py::arg("node"), py::arg("child_idx"))
        .def("get_node_child", py::overload_cast<std::shared_ptr<Node> &, unsigned int>(&Octree::GetNodeChild), py::arg("node"), py::arg("child_idx"))
        .def("expand_node", &Octree::ExpandNode, py::arg("node"))
        .def("prune_node", &Octree::PruneNode, py::arg("node"))
        .def(
            "delete_node",
            py::overload_cast<double, double, double, unsigned int>(&Octree::DeleteNode),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("depth"))
        .def("delete_node", py::overload_cast<const OctreeKey &, unsigned int>(&Octree::DeleteNode), py::arg("key"), py::arg("depth"))
        .def("clear", &Octree::Clear)
        .def("prune", &Octree::Prune)
        .def("expand", &Octree::Expand)
        .def_property_readonly("root", &Octree::GetRoot)
        .def(
            "search",
            [](Octree &self, double x, double y, double z) {
                unsigned int depth;
                std::shared_ptr<Node> node = self.Search(x, y, z, depth);
                return std::make_tuple(node, depth);
            },
            py::arg("x"),
            py::arg("y"),
            py::arg("z"))
        .def(
            "search",
            [](Octree &self, const OctreeKey &key) {
                unsigned int depth;
                std::shared_ptr<Node> node = self.Search(key, depth);
                return std::make_tuple(node, depth);
            },
            py::arg("key"))
        .def(
            "insert_node",
            py::overload_cast<double, double, double, unsigned int>(&Octree::InsertNode),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("depth"))
        .def("insert_node", py::overload_cast<const OctreeKey &, unsigned int>(&Octree::InsertNode), py::arg("key"), py::arg("depth"))
        .def(
            "visualize",
            [](std::shared_ptr<Octree> &self,
               bool leaf_only,
               Eigen::Vector3d bg_color,
               Eigen::Vector3d occupied_color,
               Eigen::Vector3d free_color,
               Eigen::Vector3d border_color,
               int window_width,
               int window_height,
               int window_left,
               int window_top) {
                auto drawer_setting = std::make_shared<typename Octree::Drawer::Setting>();
                for (int i = 0; i < 3; ++i) {
                    drawer_setting->bg_color[0] = bg_color[0];
                    drawer_setting->bg_color[1] = bg_color[1];
                    drawer_setting->bg_color[2] = bg_color[2];

                    drawer_setting->occupied_color[0] = occupied_color[0];
                    drawer_setting->occupied_color[1] = occupied_color[1];
                    drawer_setting->occupied_color[2] = occupied_color[2];

                    drawer_setting->free_color[0] = free_color[0];
                    drawer_setting->free_color[1] = free_color[1];
                    drawer_setting->free_color[2] = free_color[2];

                    drawer_setting->border_color[0] = border_color[0];
                    drawer_setting->border_color[1] = border_color[1];
                    drawer_setting->border_color[2] = border_color[2];
                }

                auto drawer = std::make_shared<typename Octree::Drawer>(drawer_setting, self);
                auto visualizer_setting = std::make_shared<Open3dVisualizerWrapper::Setting>();
                visualizer_setting->window_width = window_width;
                visualizer_setting->window_height = window_height;
                visualizer_setting->window_left = window_left;
                visualizer_setting->window_top = window_top;
                auto visualizer = std::make_shared<Open3dVisualizerWrapper>(visualizer_setting);
                if (leaf_only) {
                    drawer->DrawLeaves(visualizer->GetVisualizer().get());
                } else {
                    drawer->DrawTree(visualizer->GetVisualizer().get());
                }
                visualizer->Show();
            },
            py::arg("leaf_only") = false,
            py::arg("bg_color") = Eigen::Vector3d(1.0, 1.0, 1.0),
            py::arg("occupied_color") = Eigen::Vector3d(0.5, 0.5, 0.5),
            py::arg("free_color") = Eigen::Vector3d(1.0, 1.0, 1.0),
            py::arg("border_color") = Eigen::Vector3d(0.0, 0.0, 0.0),
            py::arg("window_width") = 1920,
            py::arg("window_height") = 1080,
            py::arg("window_left") = 50,
            py::arg("window_top") = 50);

    // Iterators defined in OctreeImpl
    py::class_<typename Octree::IteratorBase>(tree, "IteratorBase")
        .def("__eq__", [](const typename Octree::IteratorBase &self, const typename Octree::IteratorBase &other) { return self == other; })
        .def("__ne__", [](const typename Octree::IteratorBase &self, const typename Octree::IteratorBase &other) { return self != other; })
        .def("get", [](const typename Octree::IteratorBase &self) { return *self; })
        .def_property_readonly("x", &Octree::IteratorBase::GetX)
        .def_property_readonly("y", &Octree::IteratorBase::GetY)
        .def_property_readonly("z", &Octree::IteratorBase::GetZ)
        .def_property_readonly("node_size", &Octree::IteratorBase::GetNodeSize)
        .def_property_readonly("depth", &Octree::IteratorBase::GetDepth)
        .def_property_readonly("key", &Octree::IteratorBase::GetKey)
        .def_property_readonly("index_key", &Octree::IteratorBase::GetIndexKey);

    py::class_<typename Octree::TreeIterator, typename Octree::IteratorBase>(tree, "TreeIterator")
        .def("next", [](typename Octree::TreeIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Octree::TreeIterator &self) { return self == typename Octree::TreeIterator(); });

    py::class_<typename Octree::TreeInAabbIterator, typename Octree::IteratorBase>(tree, "TreeInAabbIterator")
        .def("next", [](typename Octree::TreeInAabbIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Octree::TreeInAabbIterator &self) { return self == typename Octree::TreeInAabbIterator(); });

    py::class_<typename Octree::LeafIterator, typename Octree::IteratorBase>(tree, "LeafIterator")
        .def("next", [](typename Octree::LeafIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Octree::LeafIterator &self) { return self == typename Octree::LeafIterator(); });

    py::class_<typename Octree::LeafOfNodeIterator, typename Octree::IteratorBase>(tree, "LeafOfNodeIterator")
        .def("next", [](typename Octree::LeafOfNodeIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Octree::LeafOfNodeIterator &self) { return self == typename Octree::LeafOfNodeIterator(); });

    py::class_<typename Octree::LeafInAabbIterator, typename Octree::IteratorBase>(tree, "LeafInAabbIterator")
        .def("next", [](typename Octree::LeafInAabbIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Octree::LeafInAabbIterator &self) { return self == typename Octree::LeafInAabbIterator(); });

    py::class_<typename Octree::WestLeafNeighborIterator, typename Octree::IteratorBase>(tree, "WestLeafNeighborIterator")
        .def("next", [](typename Octree::WestLeafNeighborIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Octree::WestLeafNeighborIterator &self) {
            return self == typename Octree::WestLeafNeighborIterator();
        });

    py::class_<typename Octree::EastLeafNeighborIterator, typename Octree::IteratorBase>(tree, "EastLeafNeighborIterator")
        .def("next", [](typename Octree::EastLeafNeighborIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Octree::EastLeafNeighborIterator &self) {
            return self == typename Octree::EastLeafNeighborIterator();
        });

    py::class_<typename Octree::NorthLeafNeighborIterator, typename Octree::IteratorBase>(tree, "NorthLeafNeighborIterator")
        .def("next", [](typename Octree::NorthLeafNeighborIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Octree::NorthLeafNeighborIterator &self) {
            return self == typename Octree::NorthLeafNeighborIterator();
        });

    py::class_<typename Octree::SouthLeafNeighborIterator, typename Octree::IteratorBase>(tree, "SouthLeafNeighborIterator")
        .def("next", [](typename Octree::SouthLeafNeighborIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Octree::SouthLeafNeighborIterator &self) {
            return self == typename Octree::SouthLeafNeighborIterator();
        });

    py::class_<typename Octree::TopLeafNeighborIterator, typename Octree::IteratorBase>(tree, "TopLeafNeighborIterator")
        .def("next", [](typename Octree::TopLeafNeighborIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Octree::TopLeafNeighborIterator &self) {
            return self == typename Octree::TopLeafNeighborIterator();
        });

    py::class_<typename Octree::BottomLeafNeighborIterator, typename Octree::IteratorBase>(tree, "BottomLeafNeighborIterator")
        .def("next", [](typename Octree::BottomLeafNeighborIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Octree::BottomLeafNeighborIterator &self) {
            return self == typename Octree::BottomLeafNeighborIterator();
        });

    py::class_<typename Octree::LeafOnRayIterator, typename Octree::IteratorBase>(tree, "LeafOnRayIterator")
        .def_property_readonly("distance", &Octree::LeafOnRayIterator::GetDistance)
        .def("next", [](typename Octree::LeafOnRayIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Octree::LeafOnRayIterator &self) { return self == typename Octree::LeafOnRayIterator(); });

    // Iterators defined in OccupancyOctreeBase
    py::class_<typename Octree::OccupiedLeafOnRayIterator, typename Octree::IteratorBase>(tree, "OccupiedLeafOnRayIterator")
        .def("next", [](typename Octree::OccupiedLeafOnRayIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Octree::OccupiedLeafOnRayIterator &self) {
            return self == typename Octree::OccupiedLeafOnRayIterator();
        });

    tree.def("iter_leaf", &Octree::BeginLeaf, py::arg("max_depth") = 0)
        .def("iter_leaf_of_node", &Octree::BeginLeafOfNode, py::arg("node_key"), py::arg("node_depth"), py::arg("max_depth") = 0)
        .def(
            "iter_leaf_in_aabb",
            py::overload_cast<double, double, double, double, double, double, unsigned int>(&Octree::BeginLeafInAabb, py::const_),
            py::arg("aabb_min_x"),
            py::arg("aabb_min_y"),
            py::arg("aabb_min_z"),
            py::arg("aabb_max_x"),
            py::arg("aabb_max_y"),
            py::arg("aabb_max_z"),
            py::arg("max_depth") = 0)
        .def(
            "iter_leaf_in_aabb",
            py::overload_cast<const OctreeKey &, const OctreeKey &, unsigned int>(&Octree::BeginLeafInAabb, py::const_),
            py::arg("aabb_min_key"),
            py::arg("aabb_max_key"),
            py::arg("max_depth") = 0)
        .def("iter_node", &Octree::BeginTree, py::arg("max_depth") = 0)
        .def(
            "iter_node_in_aabb",
            py::overload_cast<double, double, double, double, double, double, unsigned int>(&Octree::BeginTreeInAabb, py::const_),
            py::arg("aabb_min_x"),
            py::arg("aabb_min_y"),
            py::arg("aabb_min_z"),
            py::arg("aabb_max_x"),
            py::arg("aabb_max_y"),
            py::arg("aabb_max_z"),
            py::arg("max_depth") = 0)
        .def(
            "iter_node_in_aabb",
            py::overload_cast<const OctreeKey &, const OctreeKey &, unsigned int>(&Octree::BeginTreeInAabb, py::const_),
            py::arg("aabb_min_key"),
            py::arg("aabb_max_key"),
            py::arg("max_depth") = 0)
        .def(
            "iter_west_leaf_neighbor",
            py::overload_cast<double, double, double, unsigned int>(&Octree::BeginWestLeafNeighbor, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_west_leaf_neighbor",
            py::overload_cast<const OctreeKey &, unsigned int, unsigned int>(&Octree::BeginWestLeafNeighbor, py::const_),
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_east_leaf_neighbor",
            py::overload_cast<double, double, double, unsigned int>(&Octree::BeginEastLeafNeighbor, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_east_leaf_neighbor",
            py::overload_cast<const OctreeKey &, unsigned int, unsigned int>(&Octree::BeginEastLeafNeighbor, py::const_),
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_north_leaf_neighbor",
            py::overload_cast<double, double, double, unsigned int>(&Octree::BeginNorthLeafNeighbor, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_north_leaf_neighbor",
            py::overload_cast<const OctreeKey &, unsigned int, unsigned int>(&Octree::BeginNorthLeafNeighbor, py::const_),
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_south_leaf_neighbor",
            py::overload_cast<double, double, double, unsigned int>(&Octree::BeginSouthLeafNeighbor, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_south_leaf_neighbor",
            py::overload_cast<const OctreeKey &, unsigned int, unsigned int>(&Octree::BeginSouthLeafNeighbor, py::const_),
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_top_leaf_neighbor",
            py::overload_cast<double, double, double, unsigned int>(&Octree::BeginTopLeafNeighbor, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_top_leaf_neighbor",
            py::overload_cast<const OctreeKey &, unsigned int, unsigned int>(&Octree::BeginTopLeafNeighbor, py::const_),
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_bottom_leaf_neighbor",
            py::overload_cast<double, double, double, unsigned int>(&Octree::BeginBottomLeafNeighbor, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("z"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_bottom_leaf_neighbor",
            py::overload_cast<const OctreeKey &, unsigned int, unsigned int>(&Octree::BeginBottomLeafNeighbor, py::const_),
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_leaf_on_ray",
            &Octree::BeginLeafOnRay,
            py::arg("px"),
            py::arg("py"),
            py::arg("pz"),
            py::arg("vx"),
            py::arg("vy"),
            py::arg("vz"),
            py::arg("max_range") = -1,
            py::arg("bidirectional") = false,
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_occupied_leaf_on_ray",
            &Octree::BeginOccupiedLeafOnRay,
            py::arg("px"),
            py::arg("py"),
            py::arg("pz"),
            py::arg("vx"),
            py::arg("vy"),
            py::arg("vz"),
            py::arg("max_range") = -1,
            py::arg("bidirectional") = false,
            py::arg("max_leaf_depth") = 0);
}
