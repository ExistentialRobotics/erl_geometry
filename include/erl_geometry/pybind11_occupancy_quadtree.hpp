#pragma once

#include "erl_common/pybind11.hpp"
#include "erl_common/yaml.hpp"
#include "erl_geometry/occupancy_quadtree_base.hpp"

template<class Quadtree, class Node>
auto
BindOccupancyQuadtree(py::module &m, const char *name) {
    py::class_<Quadtree, std::shared_ptr<Quadtree>> tree(m, name);
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<typename Quadtree::Setting, YamlableBase, std::shared_ptr<typename Quadtree::Setting>> setting(tree, "Setting", py::module_local());
    setting.def(py::init<>())
        .def_readwrite("log_odd_min", &Quadtree::Setting::log_odd_min)
        .def_readwrite("log_odd_max", &Quadtree::Setting::log_odd_max)
        .def_readwrite("probability_hit", &Quadtree::Setting::probability_hit)
        .def_readwrite("probability_miss", &Quadtree::Setting::probability_miss)
        .def_readwrite("probability_occupied_threshold", &Quadtree::Setting::probability_occupied_threshold)
        .def_readwrite("resolution", &Quadtree::Setting::resolution)
        .def_readwrite("use_change_detection", &Quadtree::Setting::use_change_detection)
        .def_readwrite("use_aabb_limit", &Quadtree::Setting::use_aabb_limit)
        .def_readwrite("aabb", &Quadtree::Setting::aabb);

    // AbstractQuadtree methods
    tree.def("read_raw", [](Quadtree &self, const std::string &filename) -> bool { return self.LoadData(filename); }, py::arg("filename"));

    // AbstractOccupancyQuadtree methods
    tree.def(
            "write_binary",
            [](Quadtree &self, const std::string &filename, bool prune_at_first) -> bool {
                if (prune_at_first) {
                    return self.WriteBinary(filename);
                } else {
                    return const_cast<const Quadtree &>(self).WriteBinary(filename);
                }
            },
            py::arg("filename"),
            py::arg("prune_at_first"))
        .def("read_binary", [](Quadtree &self, const std::string &filename) -> bool { return self.ReadBinary(filename); }, py::arg("filename"));

    // OccupancyQuadtreeBase methods, except iterators
    tree.def(py::init<>([](double resolution) { return std::make_shared<Quadtree>(resolution); }), py::arg("resolution"))
        .def(py::init<>([](const std::shared_ptr<typename Quadtree::Setting> &setting) { return std::make_shared<Quadtree>(setting); }), py::arg("setting"))
        .def(py::init<>([](const std::string &filename, bool is_binary) {
            if (is_binary) {
                return std::make_shared<Quadtree>(filename);
            } else {
                std::shared_ptr<Quadtree> tree = Quadtree::template ReadAs<Quadtree>(filename);
                if (tree) {
                    return tree;
                } else {
                    throw std::runtime_error("Failed to read Quadtree from " + filename);
                }
            }
        }))
        .def_property_readonly("tree_type", &Quadtree::GetTreeType)
        .def_property_readonly("setting", &Quadtree::GetSetting)
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
            [](const Quadtree &self, std::size_t num_positions) {
                std::vector<Eigen::Vector2d> positions;
                self.SamplePositions(num_positions, positions);
                return positions;
            },
            py::arg("num_positions"))
        .def(
            "cast_rays",
            [](const Quadtree &self,
               const Eigen::Ref<const Eigen::Vector2d> &position,
               const Eigen::Ref<const Eigen::Matrix2d> &rotation,
               const Eigen::Ref<const Eigen::VectorXd> &angles,
               bool ignore_unknown,
               double max_range,
               bool prune_rays,
               bool parallel) -> py::dict {
                std::vector<long> hit_ray_indices;
                std::vector<Eigen::Vector2d> hit_positions;
                std::vector<const Node *> hit_nodes;
                std::vector<uint32_t> node_depths;
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
                    hit_nodes,
                    node_depths);

                py::dict result;
                result["hit_ray_indices"] = hit_ray_indices;
                result["hit_positions"] = hit_positions;
                result["hit_nodes"] = hit_nodes;
                result["node_depths"] = node_depths;
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
               const Eigen::Ref<const Eigen::Matrix2Xd> &positions,
               const Eigen::Ref<const Eigen::Matrix2Xd> &directions,
               bool ignore_unknown,
               double max_range,
               bool prune_rays,
               bool parallel) -> py::dict {
                std::vector<long> hit_ray_indices;
                std::vector<Eigen::Vector2d> hit_positions;
                std::vector<const Node *> hit_nodes;
                std::vector<uint32_t> node_depths;
                self.CastRays(positions, directions, ignore_unknown, max_range, prune_rays, parallel, hit_ray_indices, hit_positions, hit_nodes, node_depths);
                py::dict result;
                result["hit_ray_indices"] = hit_ray_indices;
                result["hit_positions"] = hit_positions;
                result["hit_nodes"] = hit_nodes;
                result["node_depths"] = node_depths;
                return result;
            },
            py::arg("positions"),
            py::arg("directions"),
            py::arg("ignore_unknown"),
            py::arg("max_range"),
            py::arg("prune_rays"),
            py::arg("parallel"))
        .def(
            "cast_ray",
            [](const Quadtree &self, double px, double py, double vx, double vy, bool ignore_unknown, double max_range) {
                double ex, ey;
                uint32_t depth = 0;
                const Node *hit_node = self.CastRay(px, py, vx, vy, ignore_unknown, max_range, ex, ey, depth);
                return std::make_tuple(hit_node, ex, ey, depth);
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
        .def_property_readonly("memory_usage_per_node", &Quadtree::GetMemoryUsagePerNode)
        .def("coord_to_key", py::overload_cast<double>(&Quadtree::CoordToKey, py::const_), py::arg("coordinate"))
        .def("coord_to_key", py::overload_cast<double, uint32_t>(&Quadtree::CoordToKey, py::const_), py::arg("coordinate"), py::arg("depth"))
        .def("coord_to_key", py::overload_cast<double, double>(&Quadtree::CoordToKey, py::const_), py::arg("x"), py::arg("y"))
        .def("coord_to_key", py::overload_cast<double, double, uint32_t>(&Quadtree::CoordToKey, py::const_), py::arg("x"), py::arg("y"), py::arg("depth"))
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
            [](const Quadtree &self, double coordinate, uint32_t depth) {
                QuadtreeKey::KeyType key;
                if (self.CoordToKeyChecked(coordinate, depth, key)) {
                    return std::optional<QuadtreeKey::KeyType>(key);
                } else {
                    return std::optional<QuadtreeKey::KeyType>();
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
            [](const Quadtree &self, double x, double y, uint32_t depth) {
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
            py::overload_cast<QuadtreeKey::KeyType, uint32_t>(&Quadtree::AdjustKeyToDepth, py::const_),
            py::arg("key"),
            py::arg("depth"))
        .def("adjust_key_to_depth", py::overload_cast<const QuadtreeKey &, uint32_t>(&Quadtree::AdjustKeyToDepth, py::const_), py::arg("key"), py::arg("depth"))
        .def(
            "compute_common_ancestor_key",
            [](const Quadtree &self, const QuadtreeKey &key1, const QuadtreeKey &key2) {
                QuadtreeKey key;
                uint32_t ancestor_depth;
                self.ComputeCommonAncestorKey(key1, key2, key, ancestor_depth);
                return std::make_tuple(key, ancestor_depth);
            })
        .def(
            "compute_west_neighbor_key",
            [](const Quadtree &self, const QuadtreeKey &key, uint32_t depth) {
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
            [](const Quadtree &self, const QuadtreeKey &key, uint32_t depth) {
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
            [](const Quadtree &self, const QuadtreeKey &key, uint32_t depth) {
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
            [](const Quadtree &self, const QuadtreeKey &key, uint32_t depth) {
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
        .def("key_to_coord", py::overload_cast<QuadtreeKey::KeyType, uint32_t>(&Quadtree::KeyToCoord, py::const_), py::arg("key"), py::arg("depth"))
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
            [](const Quadtree &self, const QuadtreeKey &key, uint32_t depth) {
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
        .def("is_node_collapsible", &Quadtree::IsNodeCollapsible, py::arg("node"))
        .def("get_node_child", py::overload_cast<Node *, uint32_t>(&Quadtree::GetNodeChild), py::arg("node"), py::arg("child_idx"))
        .def("expand_node", &Quadtree::ExpandNode, py::arg("node"))
        .def("prune_node", &Quadtree::PruneNode, py::arg("node"))
        .def("delete_node", py::overload_cast<double, double, uint32_t>(&Quadtree::DeleteNode), py::arg("x"), py::arg("y"), py::arg("depth"))
        .def("delete_node", py::overload_cast<const QuadtreeKey &, uint32_t>(&Quadtree::DeleteNode), py::arg("key"), py::arg("depth"))
        .def("clear", &Quadtree::Clear)
        .def("prune", &Quadtree::Prune)
        .def("expand", &Quadtree::Expand)
        .def_property_readonly("root", &Quadtree::GetRoot)
        .def(
            "search",
            [](Quadtree &self, double x, double y) {
                uint32_t depth = 0;
                Node *node = self.Search(x, y, depth);
                return std::make_tuple(node, depth);
            },
            py::arg("x"),
            py::arg("y"))
        .def(
            "search",
            [](Quadtree &self, const QuadtreeKey &key) {
                uint32_t depth = 0;
                Node *node = self.Search(key, depth);
                return std::make_tuple(node, depth);
            },
            py::arg("key"))
        .def("insert_node", py::overload_cast<double, double, uint32_t>(&Quadtree::InsertNode), py::arg("x"), py::arg("y"), py::arg("depth"))
        .def("insert_node", py::overload_cast<const QuadtreeKey &, uint32_t>(&Quadtree::InsertNode), py::arg("key"), py::arg("depth"))
        .def(
            "visualize",
            [](std::shared_ptr<Quadtree> &self,
               bool leaf_only,
               std::optional<Eigen::Vector2d> area_min,
               std::optional<Eigen::Vector2d> area_max,
               double resolution,
               int padding,
               Eigen::Vector4i bg_color,
               Eigen::Vector4i fg_color,
               Eigen::Vector4i occupied_color,
               Eigen::Vector4i free_color,
               Eigen::Vector4i border_color,
               int border_thickness) {
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

    py::class_<typename Quadtree::WestLeafNeighborIterator, typename Quadtree::IteratorBase>(tree, "WestLeafNeighborIterator")
        .def("next", [](typename Quadtree::WestLeafNeighborIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::WestLeafNeighborIterator &self) {
            return self == typename Quadtree::WestLeafNeighborIterator();
        });

    py::class_<typename Quadtree::EastLeafNeighborIterator, typename Quadtree::IteratorBase>(tree, "EastLeafNeighborIterator")
        .def("next", [](typename Quadtree::EastLeafNeighborIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::EastLeafNeighborIterator &self) {
            return self == typename Quadtree::EastLeafNeighborIterator();
        });

    py::class_<typename Quadtree::NorthLeafNeighborIterator, typename Quadtree::IteratorBase>(tree, "NorthLeafNeighborIterator")
        .def("next", [](typename Quadtree::NorthLeafNeighborIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::NorthLeafNeighborIterator &self) {
            return self == typename Quadtree::NorthLeafNeighborIterator();
        });

    py::class_<typename Quadtree::SouthLeafNeighborIterator, typename Quadtree::IteratorBase>(tree, "SouthLeafNeighborIterator")
        .def("next", [](typename Quadtree::SouthLeafNeighborIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::SouthLeafNeighborIterator &self) {
            return self == typename Quadtree::SouthLeafNeighborIterator();
        });

    py::class_<typename Quadtree::LeafOnRayIterator, typename Quadtree::IteratorBase>(tree, "LeafOnRayIterator")
        .def_property_readonly("distance", &Quadtree::LeafOnRayIterator::GetDistance)
        .def("next", [](typename Quadtree::LeafOnRayIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::LeafOnRayIterator &self) { return self == typename Quadtree::LeafOnRayIterator(); });

    // Iterators defined in OccupancyQuadtreeBase
    py::class_<typename Quadtree::OccupiedLeafOnRayIterator, typename Quadtree::IteratorBase>(tree, "OccupiedLeafOnRayIterator")
        .def("next", [](typename Quadtree::OccupiedLeafOnRayIterator &self) { return ++self; })
        .def_property_readonly("is_end", [](const typename Quadtree::OccupiedLeafOnRayIterator &self) {
            return self == typename Quadtree::OccupiedLeafOnRayIterator();
        });

    tree.def("iter_leaf", &Quadtree::BeginLeaf, py::arg("max_depth") = 0)
        .def("iter_leaf_of_node", &Quadtree::BeginLeafOfNode, py::arg("node_key"), py::arg("node_depth"), py::arg("max_depth") = 0)
        .def(
            "iter_leaf_in_aabb",
            py::overload_cast<double, double, double, double, uint32_t>(&Quadtree::BeginLeafInAabb, py::const_),
            py::arg("aabb_min_x"),
            py::arg("aabb_min_y"),
            py::arg("aabb_max_x"),
            py::arg("aabb_max_y"),
            py::arg("max_depth") = 0)
        .def(
            "iter_leaf_in_aabb",
            py::overload_cast<const QuadtreeKey &, const QuadtreeKey &, uint32_t>(&Quadtree::BeginLeafInAabb, py::const_),
            py::arg("aabb_min_key"),
            py::arg("aabb_max_key"),
            py::arg("max_depth") = 0)
        .def("iter_node", &Quadtree::BeginTree, py::arg("max_depth") = 0)
        .def(
            "iter_node_in_aabb",
            py::overload_cast<double, double, double, double, uint32_t>(&Quadtree::BeginTreeInAabb, py::const_),
            py::arg("aabb_min_x"),
            py::arg("aabb_min_y"),
            py::arg("aabb_max_x"),
            py::arg("aabb_max_y"),
            py::arg("max_depth") = 0)
        .def(
            "iter_node_in_aabb",
            py::overload_cast<const QuadtreeKey &, const QuadtreeKey &, uint32_t>(&Quadtree::BeginTreeInAabb, py::const_),
            py::arg("aabb_min_key"),
            py::arg("aabb_max_key"),
            py::arg("max_depth") = 0)
        .def(
            "iter_west_leaf_neighbor",
            py::overload_cast<double, double, uint32_t>(&Quadtree::BeginWestLeafNeighbor, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_west_leaf_neighbor",
            py::overload_cast<const QuadtreeKey &, uint32_t, uint32_t>(&Quadtree::BeginWestLeafNeighbor, py::const_),
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_east_leaf_neighbor",
            py::overload_cast<double, double, uint32_t>(&Quadtree::BeginEastLeafNeighbor, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_east_leaf_neighbor",
            py::overload_cast<const QuadtreeKey &, uint32_t, uint32_t>(&Quadtree::BeginEastLeafNeighbor, py::const_),
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_north_leaf_neighbor",
            py::overload_cast<double, double, uint32_t>(&Quadtree::BeginNorthLeafNeighbor, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("max_leaf_depth") = 0

            )
        .def(
            "iter_north_leaf_neighbor",
            py::overload_cast<const QuadtreeKey &, uint32_t, uint32_t>(&Quadtree::BeginNorthLeafNeighbor, py::const_),
            py::arg("key"),
            py::arg("key_depth"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_south_leaf_neighbor",
            py::overload_cast<double, double, uint32_t>(&Quadtree::BeginSouthLeafNeighbor, py::const_),
            py::arg("x"),
            py::arg("y"),
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_south_leaf_neighbor",
            py::overload_cast<const QuadtreeKey &, uint32_t, uint32_t>(&Quadtree::BeginSouthLeafNeighbor, py::const_),
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
            py::arg("max_leaf_depth") = 0)
        .def(
            "iter_occupied_leaf_on_ray",
            &Quadtree::BeginOccupiedLeafOnRay,
            py::arg("px"),
            py::arg("py"),
            py::arg("vx"),
            py::arg("vy"),
            py::arg("max_range") = -1,
            py::arg("bidirectional") = false,
            py::arg("max_leaf_depth") = 0);
    return std::make_pair(tree, setting);
}
