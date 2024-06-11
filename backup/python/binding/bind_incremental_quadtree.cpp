#include "erl_common/pybind11.hpp"
#include "erl_geometry/incremental_quadtree.hpp"
#include "erl_geometry/node.hpp"
#include "erl_geometry/node_container.hpp"
#include "erl_geometry/node_container_multi_types.hpp"

using namespace erl::common;
using namespace erl::geometry;

static void
BindNode(const py::module &m) {
    py::class_<NodeData, std::shared_ptr<NodeData>>(m, "NodeData").def("__str__", [](const NodeData &node_data) -> std::string {
        std::stringstream ss;
        node_data.Print(ss);
        return ss.str();
    });

    auto py_node = py::class_<Node, std::shared_ptr<Node>>(m, "Node")
                       .def(py::init<int, Eigen::VectorXd, std::shared_ptr<NodeData>>(), py::arg("type"), py::arg("position"), py::arg("node_data") = nullptr)
                       .def_readwrite("position", &Node::position);
    ERL_PYBIND_WRAP_PROPERTY_AS_READONLY(py_node, Node, node_data);
    ERL_PYBIND_WRAP_PROPERTY_AS_READONLY(py_node, Node, type);
}

static void
BindNodeContainer(const py::module &m) {
    py::class_<NodeContainer, std::shared_ptr<NodeContainer>>(m, "NodeContainer")
        .def_property_readonly("node_types", &NodeContainer::GetNodeTypes)
        .def("node_type_name", &NodeContainer::GetNodeTypeName, py::arg("type"))
        .def("size", py::overload_cast<>(&NodeContainer::Size, py::const_))
        .def("size_of_type", py::overload_cast<int>(&NodeContainer::Size, py::const_), py::arg("type"))
        .def("empty", py::overload_cast<>(&NodeContainer::Empty, py::const_))
        .def("empty_of_type", py::overload_cast<int>(&NodeContainer::Empty, py::const_))
        .def("clear", &NodeContainer::Clear)
        .def("collect_nodes", py::overload_cast<>(&NodeContainer::CollectNodes, py::const_))
        .def("collect_nodes_of_type", py::overload_cast<int>(&NodeContainer::CollectNodesOfType, py::const_), py::arg("type"))
        .def(
            "collect_nodes_of_type_in_aabb_2d",
            py::overload_cast<int, const Aabb2D &>(&NodeContainer::CollectNodesOfTypeInAabb2D, py::const_),
            py::arg("type"),
            py::arg("aabb_2d"))
        .def(
            "collect_nodes_of_type_in_aabb_3d",
            py::overload_cast<int, const Aabb3D &>(&NodeContainer::CollectNodesOfTypeInAabb3D, py::const_),
            py::arg("type"),
            py::arg("aabb_3d"))
        .def(
            "insert",
            [](NodeContainer &node_container, const std::shared_ptr<Node> &node) -> bool {
                bool too_close = false;
                node_container.Insert(node, too_close);
                return too_close;
            },
            py::arg("node"))
        .def("remove", &NodeContainer::Remove, py::arg("node"));
}

static void
BindNodeContainerMultiTypes(const py::module &m) {
    auto py_node_container = py::class_<NodeContainerMultiTypes, NodeContainer, std::shared_ptr<NodeContainerMultiTypes>>(m, "NodeContainerMultiTypes");
    py::class_<NodeContainerMultiTypes::Setting, YamlableBase, std::shared_ptr<NodeContainerMultiTypes::Setting>>(py_node_container, "Setting")
        .def(py::init<>())
        .def(py::init<int, int, double>(), py::arg("num_node_types"), py::arg("capacity_per_type") = 1, py::arg("min_squared_distance") = 0.04)
        .def_readwrite("num_node_types", &NodeContainerMultiTypes::Setting::num_node_types)
        .def_readwrite("node_type_capacity", &NodeContainerMultiTypes::Setting::node_type_capacity)
        .def_readwrite("node_type_min_squared_distance", &NodeContainerMultiTypes::Setting::node_type_min_squared_distance);
    py_node_container.def(py::init<>(&NodeContainerMultiTypes::Create), py::arg("setting"));
}

void
BindIncrementalQuadTree(const py::module &m) {
    auto py_quadtree = py::class_<IncrementalQuadtree, std::shared_ptr<IncrementalQuadtree>>(m, "IncrementalQuadtree");

    // IncrementalQuadtree::Children
    auto py_quadtree_children = py::class_<IncrementalQuadtree::Children>(py_quadtree, "Children");

    // IncrementalQuadtree::Children::Type
    py::enum_<IncrementalQuadtree::Children::Type>(py_quadtree_children, "Type", py::arithmetic(), "Type of QuadTree child.")
        .value(IncrementalQuadtree::Children::GetTypeName(IncrementalQuadtree::Children::Type::kNorthWest), IncrementalQuadtree::Children::Type::kNorthWest)
        .value(IncrementalQuadtree::Children::GetTypeName(IncrementalQuadtree::Children::Type::kNorthEast), IncrementalQuadtree::Children::Type::kNorthEast)
        .value(IncrementalQuadtree::Children::GetTypeName(IncrementalQuadtree::Children::Type::kSouthWest), IncrementalQuadtree::Children::Type::kSouthWest)
        .value(IncrementalQuadtree::Children::GetTypeName(IncrementalQuadtree::Children::Type::kSouthEast), IncrementalQuadtree::Children::Type::kSouthEast)
        .value(IncrementalQuadtree::Children::GetTypeName(IncrementalQuadtree::Children::Type::kRoot), IncrementalQuadtree::Children::Type::kRoot)
        .export_values();

    py_quadtree_children.def("__getitem__", &IncrementalQuadtree::Children::operator[], py::arg("child_type").none(false))
        .def("reset", &IncrementalQuadtree::Children::Reset)
        .def_property_readonly("north_west", py::overload_cast<>(&IncrementalQuadtree::Children::NorthWest, py::const_))
        .def_property_readonly("north_east", py::overload_cast<>(&IncrementalQuadtree::Children::NorthEast, py::const_))
        .def_property_readonly("south_west", py::overload_cast<>(&IncrementalQuadtree::Children::SouthWest, py::const_))
        .def_property_readonly("south_east", py::overload_cast<>(&IncrementalQuadtree::Children::SouthEast, py::const_));

    // IncrementalQuadtree::Setting
    py::class_<IncrementalQuadtree::Setting, YamlableBase, std::shared_ptr<IncrementalQuadtree::Setting>>(py_quadtree, "Setting")
        .def(py::init<>())
        .def_readwrite("cluster_half_area_size", &IncrementalQuadtree::Setting::cluster_half_area_size)
        .def_readwrite("max_half_area_size", &IncrementalQuadtree::Setting::max_half_area_size)
        .def_readwrite("min_half_area_size", &IncrementalQuadtree::Setting::min_half_area_size);

    // bindings of QuadTree
    py_quadtree
        .def(
            py::init([](std::shared_ptr<IncrementalQuadtree::Setting> setting,
                        const Aabb2D &area,
                        const std::function<std::shared_ptr<NodeContainer>()> &node_container_constructor) {
                return IncrementalQuadtree::Create(std::move(setting), area, node_container_constructor);
            }),
            py::arg("setting"),
            py::arg("area"),
            py::arg("node_container_constructor"))
        .def_property_readonly("setting", &IncrementalQuadtree::GetSetting)
        .def_property_readonly("root", &IncrementalQuadtree::GetRoot)
        .def_property_readonly("cluster", &IncrementalQuadtree::GetCluster)
        .def_property_readonly("parent", &IncrementalQuadtree::GetParent)
        .def_property_readonly("area", &IncrementalQuadtree::GetArea)
        .def_property_readonly("child_type", &IncrementalQuadtree::GetChildType)
        .def_property_readonly("children", [](const IncrementalQuadtree &quadtree) -> const IncrementalQuadtree::Children & { return quadtree.m_children_; })
        .def_property_readonly("is_root", &IncrementalQuadtree::IsRoot)
        .def_property_readonly("is_leaf", &IncrementalQuadtree::IsLeaf)
        .def("is_empty", py::overload_cast<>(&IncrementalQuadtree::IsEmpty, py::const_))
        .def("is_empty_of_type", py::overload_cast<int>(&IncrementalQuadtree::IsEmpty, py::const_), py::arg("type"))
        .def_property_readonly("is_in_cluster", &IncrementalQuadtree::IsInCluster)
        .def_property_readonly("is_expandable", &IncrementalQuadtree::IsExpandable)
        .def_property_readonly("is_subdividable", &IncrementalQuadtree::IsSubdividable)
        .def(
            "insert",
            [](IncrementalQuadtree &quadtree, const std::shared_ptr<Node> &node) {
                std::shared_ptr<IncrementalQuadtree> new_root = nullptr;
                auto inserted_node = quadtree.Insert(node, new_root);
                return py::make_tuple(new_root, inserted_node);
            },
            py::arg("node"))
        .def("remove", &IncrementalQuadtree::Remove, py::arg("node"))
        .def(
            "collect_trees",
            [](const IncrementalQuadtree &quadtree, const std::function<bool(const std::shared_ptr<const IncrementalQuadtree> &)> &qualify)
                -> std::vector<std::shared_ptr<const IncrementalQuadtree>> {
                std::vector<std::shared_ptr<const IncrementalQuadtree>> out;
                quadtree.CollectTrees(qualify, out);
                return out;
            },
            py::arg("qualify"))
        .def(
            "collect_non_empty_clusters",
            [](IncrementalQuadtree &quadtree, const Aabb2D &area) {
                std::vector<std::shared_ptr<IncrementalQuadtree>> clusters;
                std::vector<double> square_distances;
                quadtree.CollectNonEmptyClusters(area, clusters, square_distances);
                return py::make_tuple(clusters, square_distances);
            },
            py::arg("area"))
        .def(
            "collect_nodes",
            [](const IncrementalQuadtree &quadtree) -> std::vector<std::shared_ptr<Node>> {
                std::vector<std::shared_ptr<Node>> out;
                quadtree.CollectNodes(out);
                return out;
            })
        .def(
            "collect_nodes_of_type",
            [](const IncrementalQuadtree &quadtree, const int type) -> std::vector<std::shared_ptr<Node>> {
                std::vector<std::shared_ptr<Node>> out;
                quadtree.CollectNodesOfType(type, out);
                return out;
            },
            py::arg("type"))
        .def(
            "collect_nodes_of_type_in_area",
            [](const IncrementalQuadtree &quadtree, const int type, const Aabb2D &area) -> std::vector<std::shared_ptr<Node>> {
                std::vector<std::shared_ptr<Node>> out;
                quadtree.CollectNodesOfTypeInArea(type, area, out);
                return out;
            },
            py::arg("type"),
            py::arg("area"))
        .def_property_readonly("node_types", &IncrementalQuadtree::GetNodeTypes)
        .def(
            "ray_tracing",
            [](const IncrementalQuadtree &self,
               const Eigen::Ref<const Eigen::Vector2d> &ray_origin,
               const Eigen::Ref<const Eigen::Vector2d> &ray_direction,
               const double hit_distance_threshold) -> std::pair<double, std::shared_ptr<Node>> {
                double ray_travel_distance = 0.0;
                std::shared_ptr<Node> hit_node = nullptr;
                self.RayTracing(ray_origin, ray_direction, hit_distance_threshold, ray_travel_distance, hit_node);
                return std::make_pair(ray_travel_distance, hit_node);
            },
            py::arg("ray_origin"),
            py::arg("ray_direction"),
            py::arg("hit_distance_threshold"))
        .def(
            "ray_tracing",
            [](const IncrementalQuadtree &self,
               const Eigen::Ref<const Eigen::Matrix2Xd> &ray_origins,
               const Eigen::Ref<const Eigen::Matrix2Xd> &ray_directions,
               const double hit_distance_threshold) -> std::pair<std::vector<double>, std::vector<std::shared_ptr<Node>>> {
                std::vector<double> ray_travel_distances;
                std::vector<std::shared_ptr<Node>> hit_nodes;
                self.RayTracing(ray_origins, ray_directions, hit_distance_threshold, 0, ray_travel_distances, hit_nodes);
                return std::make_pair(ray_travel_distances, hit_nodes);
            },
            py::arg("ray_origins"),
            py::arg("ray_directions"),
            py::arg("hit_distance_threshold"))
        .def(
            "ray_tracing",
            [](const IncrementalQuadtree &self,
               const int node_type,
               const Eigen::Ref<const Eigen::Vector2d> &ray_origin,
               const Eigen::Ref<const Eigen::Vector2d> &ray_direction,
               const double hit_distance_threshold) -> std::pair<double, std::shared_ptr<Node>> {
                double ray_travel_distance = 0.0;
                std::shared_ptr<Node> hit_node = nullptr;
                self.RayTracing(node_type, ray_origin, ray_direction, hit_distance_threshold, ray_travel_distance, hit_node);
                return std::make_pair(ray_travel_distance, hit_node);
            },
            py::arg("node_type"),
            py::arg("ray_origin"),
            py::arg("ray_direction"),
            py::arg("hit_distance_threshold"))
        .def(
            "ray_tracing",
            [](const IncrementalQuadtree &self,
               const int node_type,
               const Eigen::Ref<const Eigen::Matrix2Xd> &ray_origins,
               const Eigen::Ref<const Eigen::Matrix2Xd> &ray_directions,
               const double hit_distance_threshold,
               const int num_threads) -> std::pair<std::vector<double>, std::vector<std::shared_ptr<Node>>> {
                std::vector<double> ray_travel_distances;
                std::vector<std::shared_ptr<Node>> hit_nodes;
                self.RayTracing(node_type, ray_origins, ray_directions, hit_distance_threshold, num_threads, ray_travel_distances, hit_nodes);
                return std::make_pair(ray_travel_distances, hit_nodes);
            },
            py::arg("node_type"),
            py::arg("ray_origins"),
            py::arg("ray_directions"),
            py::arg("hit_distance_threshold"),
            py::arg("num_threads"))
        .def(
            "plot",
            &IncrementalQuadtree::Plot,
            py::arg("grid_map_info").none(false),
            py::arg("node_types"),
            py::arg("node_type_colors"),
            py::arg("node_type_radius"),
            py::arg("bg_color") = cv::Scalar{255, 255, 255},
            py::arg("area_rect_color") = cv::Scalar{0, 0, 0},
            py::arg("area_rect_thickness") = 2,
            py::arg("tree_data_color") = cv::Scalar{255, 0, 0},
            py::arg("tree_data_radius") = 2,
            py::arg("plot_node_data") = py::none())
        .def("__str__", [](const IncrementalQuadtree &quadtree) -> std::string {
            std::stringstream ss;
            quadtree.Print(ss);
            return ss.str();
        });
}
