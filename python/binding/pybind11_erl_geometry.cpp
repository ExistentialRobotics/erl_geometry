#include "erl_common/grid_map.hpp"
#include "erl_common/pybind11.hpp"
#include "erl_common/string_utils.hpp"
#include "erl_common/yaml.hpp"
#include "erl_geometry/aabb.hpp"
#include "erl_geometry/bresenham_2d.hpp"
#include "erl_geometry/grid_collision_checker_se2.hpp"
#include "erl_geometry/grid_collision_checker_3d.hpp"
#include "erl_geometry/incremental_quadtree.hpp"
#include "erl_geometry/lidar_2d.hpp"
#include "erl_geometry/lidar_frame_2d.hpp"
#include "erl_geometry/log_odd_map_2d.hpp"
#include "erl_geometry/marching_square.hpp"
#include "erl_geometry/node.hpp"
#include "erl_geometry/node_container.hpp"
#include "erl_geometry/node_container_multi_types.hpp"
#include "erl_geometry/point_collision_checker.hpp"
#include "erl_geometry/space_2d.hpp"
#include "erl_geometry/surface_2d.hpp"
#include "erl_geometry/winding_number.hpp"
#include "erl_geometry/utils.hpp"
#include "erl_geometry/house_expo.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"
#include "erl_geometry/pybind11_occupancy_quadtree_base.hpp"
#include "erl_geometry/pybind11_occupancy_quadtree_drawer.hpp"

using namespace erl::common;
using namespace erl::geometry;

template<typename Scalar, int Dim>
static typename std::enable_if<Dim <= 3>::type
BindAabb(py::module &m, const char *py_class_name) {

    typedef AABB<Scalar, Dim> Cls;

    auto py_aabb = py::class_<Cls>(m, py_class_name)
                       .def(py::init<typename Cls::Point, typename Cls::Scalar>(), py::arg("center"), py::arg("half_size"))
                       .def(py::init<typename Cls::Point, typename Cls::Point>(), py::arg("min"), py::arg("max"));
    if (Dim == 1) {
        py::enum_<typename Cls::CornerType>(py_aabb, "CornerType", py::arithmetic(), "Type of corners.")
            .value("kMin", Cls::CornerType::Min)
            .value("kMax", Cls::CornerType::Max)
            .export_values();
    } else if (Dim == 2) {
        py::enum_<typename Cls::CornerType>(py_aabb, "CornerType", py::arithmetic(), "Type of corners.")
            .value("kBottomLeft", Cls::CornerType::BottomLeft)
            .value("kBottomRight", Cls::CornerType::BottomRight)
            .value("kTopLeft", Cls::CornerType::TopLeft)
            .value("kTopRight", Cls::CornerType::TopRight)
            .export_values();
    } else {
        py::enum_<typename Cls::CornerType>(py_aabb, "CornerType", py::arithmetic(), "Type of corners.")
            .value("kBottomLeftFloor", Cls::CornerType::BottomLeftFloor)
            .value("kBottomRightFloor", Cls::CornerType::BottomRightFloor)
            .value("kTopLeftFloor", Cls::CornerType::TopLeftFloor)
            .value("kTopRightFloor", Cls::CornerType::TopRightFloor)
            .value("kBottomLeftCeil", Cls::CornerType::BottomLeftCeil)
            .value("kBottomRightCeil", Cls::CornerType::BottomRightCeil)
            .value("kTopLeftCeil", Cls::CornerType::TopLeftCeil)
            .value("kTopRightCeil", Cls::CornerType::TopRightCeil)
            .export_values();
    }

    ERL_PYBIND_WRAP_PROPERTY_AS_READONLY(py_aabb, Cls, center);
    ERL_PYBIND_WRAP_PROPERTY_AS_READONLY(py_aabb, Cls, half_sizes);

    py_aabb
        .def(
            "__contains__",
            [](const Cls &aabb, const Eigen::Vector<Scalar, Dim> &point) { return aabb.contains(point); },
            py::arg("point"))
        .def(
            "__contains__",
            [](const Cls &aabb_1, const Cls &aabb_2) { return aabb_1.contains(aabb_2); },
            py::arg("another_aabb"))
        .def("corner", &Cls::corner, py::arg("corner_type"))
        .def("intersects", &Cls::intersects, py::arg("another_aabb"));
}

static void
BindNode(py::module &m) {
    py::class_<NodeData, std::shared_ptr<NodeData>>(m, ERL_AS_STRING(NodeData)).def("__str__", [](const NodeData &node_data) -> std::string {
        std::stringstream ss;
        node_data.Print(ss);
        return ss.str();
    });

    auto py_node = py::class_<Node, std::shared_ptr<Node>>(m, ERL_AS_STRING(Node))
                       .def(py::init<int, Eigen::VectorXd, std::shared_ptr<NodeData>>(), py::arg("type"), py::arg("position"), py::arg("node_data") = nullptr)
                       .def_readwrite("position", &Node::position);
    ERL_PYBIND_WRAP_PROPERTY_AS_READONLY(py_node, Node, node_data);
    ERL_PYBIND_WRAP_PROPERTY_AS_READONLY(py_node, Node, type);
}

static void
BindNodeContainer(py::module &m) {
    py::class_<NodeContainer, std::shared_ptr<NodeContainer>>(m, ERL_AS_STRING(NodeContainer))
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
BindNodeContainerMultiTypes(py::module &m) {
    auto py_node_container =
        py::class_<NodeContainerMultiTypes, NodeContainer, std::shared_ptr<NodeContainerMultiTypes>>(m, ERL_AS_STRING(NodeContainerMultiTypes));
    py::class_<NodeContainerMultiTypes::Setting, YamlableBase, std::shared_ptr<NodeContainerMultiTypes::Setting>>(py_node_container, "Setting")
        .def(py::init<>())
        .def(py::init<int, int, double>(), py::arg("num_node_types"), py::arg("capacity_per_type") = 1, py::arg("min_squared_distance") = 0.04)
        .def_readwrite("num_node_types", &NodeContainerMultiTypes::Setting::num_node_types)
        .def_readwrite("node_type_capacity", &NodeContainerMultiTypes::Setting::node_type_capacity)
        .def_readwrite("node_type_min_squared_distance", &NodeContainerMultiTypes::Setting::node_type_min_squared_distance);
    py_node_container.def(py::init<>(&NodeContainerMultiTypes::Create), py::arg("setting"));
}

static void
BindIncrementalQuadTree(py::module &m) {
    auto py_quadtree = py::class_<IncrementalQuadtree, std::shared_ptr<IncrementalQuadtree>>(m, ERL_AS_STRING(IncrementalQuadtree));

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
            py::init(py::overload_cast<std::shared_ptr<IncrementalQuadtree::Setting>, const Aabb2D &, const std::function<std::shared_ptr<NodeContainer>()> &>(
                &IncrementalQuadtree::Create)),
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
            [](const IncrementalQuadtree &quadtree, int type) -> std::vector<std::shared_ptr<Node>> {
                std::vector<std::shared_ptr<Node>> out;
                quadtree.CollectNodesOfType(type, out);
                return out;
            },
            py::arg("type"))
        .def(
            "collect_nodes_of_type_in_area",
            [](const IncrementalQuadtree &quadtree, int type, const Aabb2D &area) -> std::vector<std::shared_ptr<Node>> {
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
               double hit_distance_threshold) -> std::pair<double, std::shared_ptr<Node>> {
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
               double hit_distance_threshold) -> std::pair<std::vector<double>, std::vector<std::shared_ptr<Node>>> {
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
               int node_type,
               const Eigen::Ref<const Eigen::Vector2d> &ray_origin,
               const Eigen::Ref<const Eigen::Vector2d> &ray_direction,
               double hit_distance_threshold) -> std::pair<double, std::shared_ptr<Node>> {
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
               int node_type,
               const Eigen::Ref<const Eigen::Matrix2Xd> &ray_origins,
               const Eigen::Ref<const Eigen::Matrix2Xd> &ray_directions,
               double hit_distance_threshold,
               int num_threads) -> std::pair<std::vector<double>, std::vector<std::shared_ptr<Node>>> {
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

static void
BindOccupancyQuadtree(py::module &m) {
    py::class_<QuadtreeKey>(m, "QuadtreeKey")
        .def("__eq__", [](const QuadtreeKey &self, const QuadtreeKey &other) { return self == other; })
        .def("__ne__", [](const QuadtreeKey &self, const QuadtreeKey &other) { return self != other; })
        .def("__getitem__", [](const QuadtreeKey &self, int idx) { return self[idx]; });

    py::class_<QuadtreeKeyRay>(m, "QuadtreeKeyRay")
        .def("__len__", &QuadtreeKeyRay::size)
        .def("__getitem__", &QuadtreeKeyRay::operator[], py::arg("idx"));

    py::class_<OccupancyQuadtreeNode, std::shared_ptr<OccupancyQuadtreeNode>>(m, "OccupancyQuadtreeNode")
        .def_property_readonly("occupancy", &OccupancyQuadtreeNode::GetOccupancy)
        .def_property_readonly("log_odds", &OccupancyQuadtreeNode::GetLogOdds)
        .def_property_readonly("mean_child_log_odds", &OccupancyQuadtreeNode::GetMeanChildLogOdds)
        .def_property_readonly("max_child_log_odds", &OccupancyQuadtreeNode::GetMaxChildLogOdds)
        .def("allow_update_log_odds", &OccupancyQuadtreeNode::AllowUpdateLogOdds, py::arg("delta"))
        .def("add_log_odds", &OccupancyQuadtreeNode::AddLogOdds, py::arg("log_odds"));
    BindOccupancyQuadtree<OccupancyQuadtree, OccupancyQuadtreeNode>(m, "OccupancyQuadtree");
    BindOccupancyQuadtreeDrawer<OccupancyQuadtreeDrawer<OccupancyQuadtree>, OccupancyQuadtree>(m, "OccupancyQuadtreeDrawer");
}

static void
BindSurface2D(py::module &m) {

    py::class_<Surface2D, std::shared_ptr<Surface2D>>(m, ERL_AS_STRING(Surface2D))
        .def(
            py::init<Eigen::Matrix2Xd, Eigen::Matrix2Xd, Eigen::Matrix2Xi, Eigen::Matrix2Xi, Eigen::VectorXb>(),
            py::arg("vertices"),
            py::arg("normals"),
            py::arg("lines2vertices"),
            py::arg("objects2lines"),
            py::arg("outside_flags"))
        .def(py::init<const Surface2D &>(), py::arg("surface"))
        .def_property_readonly("num_vertices", &Surface2D::GetNumVertices)
        .def_property_readonly("num_lines", &Surface2D::GetNumLines)
        .def_property_readonly("num_objects", &Surface2D::GetNumObjects)
        .def_readwrite("vertices", &Surface2D::vertices)
        .def_readwrite("normals", &Surface2D::normals)
        .def_readwrite("lines_to_vertices", &Surface2D::lines_to_vertices)
        .def_readwrite("objects_to_lines", &Surface2D::objects_to_lines)
        .def_readwrite("vertices_to_objects", &Surface2D::vertices_to_objects)
        .def_readwrite("outside_flags", &Surface2D::outside_flags)
        .def_property_readonly("normals_available", &Surface2D::NormalsAvailable)
        .def_property_readonly("outside_flags_available", &Surface2D::OutsideFlagsAvailable)
        .def(
            "get_object_vertices",
            [](Surface2D &surface, int idx_object) {
                return Eigen::Matrix2Xd(surface.GetObjectVertices(idx_object));
            },  // cast Eigen::Ref -> Eigen::Matrix2Xfp -> numpy array
            py::arg("index_object"))
        .def(
            "get_object_normals",
            [](Surface2D &surface, int idx_object) {
                return Eigen::Matrix2Xd(surface.GetObjectNormals(idx_object));
            },  // cast Eigen::Ref -> Eigen::Matrix2Xfp -> numpy array
            py::arg("index_object"))
        .def("get_vertex_neighbors", &Surface2D::GetVertexNeighbors, py::arg("index_vertex"));
}

static void
BindSpace2D(py::module &m) {
    auto py_space = py::class_<Space2D, std::shared_ptr<Space2D>>(m, ERL_AS_STRING(Space2D));

    py::enum_<Space2D::SignMethod>(py_space, "SignMethod", py::arithmetic(), "Algorithm to determine SDF sign.")
        .value(
            Space2D::GetSignMethodName(Space2D::SignMethod::kPointNormal),
            Space2D::SignMethod::kPointNormal,
            "Use the normal of the nearest vertex to determine the sign.")
        .value(
            Space2D::GetSignMethodName(Space2D::SignMethod::kLineNormal),
            Space2D::SignMethod::kLineNormal,
            "Use the normal of the nearest line segment to determine the sign.")
        .value(
            Space2D::GetSignMethodName(Space2D::SignMethod::kPolygon),
            Space2D::SignMethod::kPolygon,
            "Use the nearest object polygon and the winding number algorithm to determine the sign.")
        .export_values();

    py_space
        .def(
            py::init<const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &, const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &>(),
            py::arg("ordered_object_vertices"),
            py::arg("ordered_object_normals"))
        .def(
            py::init<const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &, const Eigen::Ref<const Eigen::VectorXb> &, double, bool>(),
            py::arg("ordered_object_vertices"),
            py::arg("outside_flags"),
            py::arg("delta") = 0.01,
            py::arg("parallel") = false)
        .def(
            py::init<const Eigen::Ref<const Eigen::MatrixXd> &, const GridMapInfo2D &, double, double, bool>(),
            py::arg("map_image"),
            py::arg("grid_map_info"),
            py::arg("free_threshold"),
            py::arg("delta") = 0.01,
            py::arg("parallel") = false)
        .def(py::init<const Space2D &>(), py::arg("space2d"))
        .def_static("get_sign_method_name", &Space2D::GetSignMethodName, py::arg("sign_method"))
        .def_static("get_sign_method_from_name", &Space2D::GetSignMethodFromName, py::arg("sign_method_name"))
        .def_property_readonly("surface", &Space2D::GetSurface)
        .def("generate_map_image", &Space2D::GenerateMapImage, py::arg("grid_map_info"), py::arg("anti_aliased") = false)
        .def(
            "compute_sdf_image",
            &Space2D::ComputeSdfImage,
            py::arg("grid_map_info"),
            py::arg("sign_method") = Space2D::SignMethod::kLineNormal,
            py::arg("use_kdtree") = false,
            py::arg("parallel") = false)
        .def(
            "compute_sdf",
            &Space2D::ComputeSdf,
            py::arg("query_points"),
            py::arg("sign_method") = Space2D::SignMethod::kLineNormal,
            py::arg("use_kd_tree") = false,
            py::arg("parallel") = false)
        .def("compute_sdf_with_kdtree", &Space2D::ComputeSdfWithKdtree, py::arg("q"), py::arg("sign_method"))
        .def("compute_sdf_greedily", &Space2D::ComputeSdfGreedily, py::arg("q"), py::arg("sign_method"))
        .def(
            "compute_ddf",
            [](const Space2D &space, const GridMapInfo2D &grid_map_info, const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions, bool parallel) {
                auto result = space.ComputeDdf(grid_map_info, query_directions, parallel);
                long n_rows = result.rows();
                long n_cols = result.cols();
                long n_dirs = query_directions.cols();
                py::array_t<double> out({n_rows, n_cols, n_dirs});
                for (long i = 0; i < n_rows; i++) {
                    for (long j = 0; j < n_cols; j++) {
                        for (long k = 0; k < n_dirs; k++) { out.mutable_at(i, j, k) = result(i, j)[k]; }
                    }
                }
                return out;
            },
            py::arg("grid_map_info"),
            py::arg("query_directions"),
            py::arg("parallel") = false)
        .def(
            "compute_ddf",
            py::overload_cast<const Eigen::Ref<const Eigen::Matrix2Xd> &, const Eigen::Ref<const Eigen::Matrix2Xd> &, bool>(&Space2D::ComputeDdf, py::const_),
            py::arg("query_points"),
            py::arg("query_directions"),
            py::arg("parallel") = false)
        .def(
            "compute_sddf_v1",
            [](const Space2D &space, const GridMapInfo2D &grid_map_info, const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions, bool parallel) {
                auto result = space.ComputeSddfV1(grid_map_info, query_directions, parallel);
                long n_rows = result.rows();
                long n_cols = result.cols();
                long n_dirs = query_directions.cols();
                py::array_t<double> out({n_rows, n_cols, n_dirs});
                for (long i = 0; i < n_rows; i++) {
                    for (long j = 0; j < n_cols; j++) {
                        for (long k = 0; k < n_dirs; k++) { out.mutable_at(i, j, k) = result(i, j)[k]; }
                    }
                }
                return out;
            },
            py::arg("grid_map_info"),
            py::arg("query_directions"),
            py::arg("parallel") = false)
        .def(
            "compute_sddf_v1",
            py::overload_cast<const Eigen::Ref<const Eigen::Matrix2Xd> &, const Eigen::Ref<const Eigen::Matrix2Xd> &, bool>(
                &Space2D::ComputeSddfV1,
                py::const_),
            py::arg("query_points"),
            py::arg("query_directions"),
            py::arg("parallel") = false)
        .def(
            "compute_sddf_v2",
            [](const Space2D &space,
               const GridMapInfo2D &grid_map_info,
               const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions,
               Space2D::SignMethod sign_method,
               bool parallel) {
                auto result = space.ComputeSddfV2(grid_map_info, query_directions, sign_method, parallel);
                long n_rows = result.rows();
                long n_cols = result.cols();
                long n_dirs = query_directions.cols();
                py::array_t<double> out({n_rows, n_cols, n_dirs});
                for (long i = 0; i < n_rows; i++) {
                    for (long j = 0; j < n_cols; j++) {
                        for (long k = 0; k < n_dirs; k++) { out.mutable_at(i, j, k) = result(i, j)[k]; }
                    }
                }
                return out;
            },
            py::arg("map_image_info"),
            py::arg("query_directions"),
            py::arg("sign_method") = Space2D::SignMethod::kLineNormal,
            py::arg("parallel") = false)
        .def(
            "compute_sddf_v2",
            py::overload_cast<const Eigen::Ref<const Eigen::Matrix2Xd> &, const Eigen::Ref<const Eigen::Matrix2Xd> &, Space2D::SignMethod, bool>(
                &Space2D::ComputeSddfV2,
                py::const_),
            py::arg("query_points"),
            py::arg("query_directions"),
            py::arg("sign_method") = Space2D::SignMethod::kLineNormal,
            py::arg("parallel") = false);
}

static void
BindLidar2D(py::module &m) {
    auto py_lidar = py::class_<Lidar2D, std::shared_ptr<Lidar2D>>(m, ERL_AS_STRING(Lidar2D));

    py::enum_<Lidar2D::Mode>(py_lidar, "Mode", py::arithmetic(), "Mode of directed distance.")
        .value(Lidar2D::GetModeName(Lidar2D::Mode::kDdf), Lidar2D::Mode::kDdf, "Compute unsigned directed distance.")
        .value(Lidar2D::GetModeName(Lidar2D::Mode::kSddfV1), Lidar2D::Mode::kSddfV1, "Compute signed directed distance, version 1.")
        .value(Lidar2D::GetModeName(Lidar2D::Mode::kSddfV2), Lidar2D::Mode::kSddfV2, "Compute signed directed distance, version 2.")
        .export_values();

    py_lidar.def(py::init<std::shared_ptr<Space2D>>(), py::arg("space2d").none(false))
        .def_property("pose", &Lidar2D::GetPose, &Lidar2D::SetPose)
        .def_property("translation", &Lidar2D::GetTranslation, &Lidar2D::SetTranslation)
        .def_property_readonly("rotation", &Lidar2D::GetRotation)
        .def("set_rotation", py::overload_cast<const Eigen::Ref<const Eigen::Matrix2d> &>(&Lidar2D::SetRotation), py::arg("rotation_matrix"))
        .def("set_rotation", py::overload_cast<double>(&Lidar2D::SetRotation), py::arg("angle"))
        .def_property("min_angle", &Lidar2D::GetMinAngle, &Lidar2D::SetMinAngle)
        .def_property("max_angle", &Lidar2D::GetMaxAngle, &Lidar2D::SetMaxAngle)
        .def_property("num_lines", &Lidar2D::GetNumLines, &Lidar2D::SetNumLines)
        .def_property_readonly("angles", &Lidar2D::GetAngles)
        .def_property_readonly("ray_directions", &Lidar2D::GetRayDirections)
        .def_property_readonly("oriented_ray_directions", &Lidar2D::GetOrientedRayDirections)
        .def_property("mode", &Lidar2D::GetMode, &Lidar2D::SetMode)
        .def_property("sign_method", &Lidar2D::GetSignMethod, &Lidar2D::SetSignMethod)
        .def("scan", &Lidar2D::Scan, py::arg("parallel") = false)
        .def(
            "scan_multi_poses",
            py::overload_cast<const std::vector<Eigen::Matrix3d> &, bool>(&Lidar2D::ScanMultiPoses, py::const_),  // const method should use py::const_
            py::arg("poses"),
            py::arg("parallel") = false)
        .def(
            "scan_multi_poses",
            py::overload_cast<
                const Eigen::Ref<const Eigen::VectorXd> &,
                const Eigen::Ref<const Eigen::VectorXd> &,
                const Eigen::Ref<const Eigen::VectorXd> &,
                bool>(&Lidar2D::ScanMultiPoses, py::const_),  // const method should use py::const_
            py::arg("xs"),
            py::arg("ys"),
            py::arg("thetas"),
            py::arg("parallel") = false)
        .def("get_rays", &Lidar2D::GetRays, py::arg("parallel") = false)
        .def(
            "get_rays_of_multi_poses",
            py::overload_cast<const std::vector<Eigen::Matrix3d> &, bool>(&Lidar2D::GetRaysOfMultiPoses, py::const_),
            py::arg("poses"),
            py::arg("parallel") = false)
        .def(
            "get_rays_of_multi_poses",
            py::overload_cast<
                const Eigen::Ref<const Eigen::VectorXd> &,
                const Eigen::Ref<const Eigen::VectorXd> &,
                const Eigen::Ref<const Eigen::VectorXd> &,
                bool>(&Lidar2D::GetRaysOfMultiPoses, py::const_),  // const method should use py::const_
            py::arg("xs"),
            py::arg("ys"),
            py::arg("thetas"),
            py::arg("parallel") = false);
}

static void
BindLidarFrame2D(py::module &m) {
    py::class_<LidarFramePartition2D>(m, "LidarFramePartition2D")
        .def_property_readonly("index_begin", &LidarFramePartition2D::GetIndexBegin)
        .def_property_readonly("index_end", &LidarFramePartition2D::GetIndexEnd)
        .def("angle_in_partition", &LidarFramePartition2D::AngleInPartition, py::arg("angle_world"));

    py::class_<LidarFrame2D, std::shared_ptr<LidarFrame2D>> lidar_frame_2d(m, "LidarFrame2D");

    py::class_<LidarFrame2D::Setting, YamlableBase, std::shared_ptr<LidarFrame2D::Setting>>(lidar_frame_2d, "Setting")
        .def(py::init<>())
        .def_readwrite("valid_range_min", &LidarFrame2D::Setting::valid_range_min)
        .def_readwrite("valid_range_max", &LidarFrame2D::Setting::valid_range_max)
        .def_readwrite("valid_angle_min", &LidarFrame2D::Setting::valid_angle_min)
        .def_readwrite("valid_angle_max", &LidarFrame2D::Setting::valid_angle_max)
        .def_readwrite("discontinuity_factor", &LidarFrame2D::Setting::discontinuity_factor)
        .def_readwrite("min_partition_size", &LidarFrame2D::Setting::min_partition_size);

    lidar_frame_2d.def(py::init<std::shared_ptr<LidarFrame2D::Setting>>(), py::arg("setting"))
        .def("update", &LidarFrame2D::Update, py::arg("rotation"), py::arg("translation"), py::arg("angles"), py::arg("ranges"))
        .def_property_readonly("setting", &LidarFrame2D::GetSetting)
        .def_property_readonly("num_rays", &LidarFrame2D::GetNumRays)
        .def_property_readonly("rotation_matrix", &LidarFrame2D::GetRotationMatrix)
        .def_property_readonly("rotation_angle", &LidarFrame2D::GetRotationAngle)
        .def_property_readonly("translation_vector", &LidarFrame2D::GetTranslationVector)
        .def_property_readonly("pose_matrix", &LidarFrame2D::GetPoseMatrix)
        .def_property_readonly("angles_in_frame", &LidarFrame2D::GetAnglesInFrame)
        .def_property_readonly("angles_in_world", &LidarFrame2D::GetAnglesInWorld)
        .def_property_readonly("ranges", &LidarFrame2D::GetRanges)
        .def_property_readonly("ray_directions_in_frame", &LidarFrame2D::GetRayDirectionsInFrame)
        .def_property_readonly("ray_directions_in_world", &LidarFrame2D::GetRayDirectionsInWorld)
        .def_property_readonly("end_points_in_frame", &LidarFrame2D::GetEndPointsInFrame)
        .def_property_readonly("end_points_in_world", &LidarFrame2D::GetEndPointsInWorld)
        .def_property_readonly("max_valid_range", &LidarFrame2D::GetMaxValidRange)
        .def_property_readonly("partitions", &LidarFrame2D::GetPartitions)
        .def_property_readonly("is_valid", &LidarFrame2D::IsValid)
        .def(
            "compute_closest_end_point",
            [](const LidarFrame2D &self, const Eigen::Ref<const Eigen::Vector2d> &position) {
                long end_point_index = -1;
                double distance = 0.0;
                self.ComputeClosestEndPoint(position, end_point_index, distance);
                py::dict out;
                out["end_point_index"] = end_point_index;
                out["distance"] = distance;
                return out;
            },
            py::arg("position"))
        .def(
            "sample_along_rays",
            [](const LidarFrame2D &self, int num_samples_per_ray, double max_in_obstacle_dist) {
                Eigen::Matrix2Xd positions;
                Eigen::Matrix2Xd directions;
                Eigen::VectorXd distances;
                self.SampleAlongRays(num_samples_per_ray, max_in_obstacle_dist, positions, directions, distances);
                py::dict out;
                out["positions"] = positions;
                out["directions"] = directions;
                out["distances"] = distances;
                return out;
            },
            py::arg("num_samples_per_ray"),
            py::arg("max_in_obstacle_dist"))
        .def(
            "sample_along_rays",
            [](const LidarFrame2D &self, double range_step, double max_in_obstacle_dist) {
                Eigen::Matrix2Xd positions;
                Eigen::Matrix2Xd directions;
                Eigen::VectorXd distances;
                self.SampleAlongRays(range_step, max_in_obstacle_dist, positions, directions, distances);
                py::dict out;
                out["positions"] = positions;
                out["directions"] = directions;
                out["distances"] = distances;
                return out;
            },
            py::arg("range_step"),
            py::arg("max_in_obstacle_dist"))
        .def(
            "sample_near_surface",
            [](const LidarFrame2D &self, int num_samples_per_ray, double max_offset) {
                Eigen::Matrix2Xd positions;
                Eigen::Matrix2Xd directions;
                Eigen::VectorXd distances;
                self.SampleNearSurface(num_samples_per_ray, max_offset, positions, directions, distances);
                py::dict out;
                out["positions"] = positions;
                out["directions"] = directions;
                out["distances"] = distances;
                return out;
            },
            py::arg("num_samples_per_ray"),
            py::arg("max_offset"))
        .def(
            "sample_in_region",
            [](const LidarFrame2D &self, int num_samples) {
                Eigen::Matrix2Xd positions;
                Eigen::Matrix2Xd directions;
                Eigen::VectorXd distances;
                self.SampleInRegion(num_samples, positions, directions, distances);
                py::dict out;
                out["positions"] = positions;
                out["directions"] = directions;
                out["distances"] = distances;
                return out;
            },
            py::arg("num_samples"))
        .def(
            "compute_rays_at",
            [](const LidarFrame2D &self, const Eigen::Ref<const Eigen::Vector2d> &position) {
                Eigen::Matrix2Xd directions;
                Eigen::VectorXd distances;
                self.ComputeRaysAt(position, directions, distances);
                py::dict out;
                out["directions"] = directions;
                out["distances"] = distances;
                return out;
            },
            py::arg("position"));
}

static void
BindLogOddMap2D(py::module &m) {
    auto py_log_odd_map = py::class_<LogOddMap2D, std::shared_ptr<LogOddMap2D>>(m, ERL_AS_STRING(LogOddMap2D));

    py::enum_<LogOddMap2D::CellType>(py_log_odd_map, "CellType", py::arithmetic(), "Type of grid cell.")
        .value(LogOddMap2D::GetCellTypeName(LogOddMap2D::CellType::kOccupied), LogOddMap2D::CellType::kOccupied)
        .value(LogOddMap2D::GetCellTypeName(LogOddMap2D::CellType::kUnexplored), LogOddMap2D::CellType::kUnexplored)
        .value(LogOddMap2D::GetCellTypeName(LogOddMap2D::CellType::kFree), LogOddMap2D::CellType::kFree)
        .export_values();

    py::class_<LogOddMap2D::Setting, YamlableBase, std::shared_ptr<LogOddMap2D::Setting>>(py_log_odd_map, "Setting")
        .def(py::init<>())
        .def_readwrite("sensor_min_range", &LogOddMap2D::Setting::sensor_min_range)
        .def_readwrite("sensor_max_range", &LogOddMap2D::Setting::sensor_max_range)
        .def_readwrite("measurement_certainty", &LogOddMap2D::Setting::measurement_certainty)
        .def_readwrite("max_log_odd", &LogOddMap2D::Setting::max_log_odd)
        .def_readwrite("min_log_odd", &LogOddMap2D::Setting::min_log_odd)
        .def_readwrite("threshold_occupied", &LogOddMap2D::Setting::threshold_occupied)
        .def_readwrite("threshold_free", &LogOddMap2D::Setting::threshold_free)
        .def_readwrite("use_cross_kernel", &LogOddMap2D::Setting::use_cross_kernel)
        .def_readwrite("num_iters_for_cleaned_mask", &LogOddMap2D::Setting::num_iters_for_cleaned_mask)
        .def_readwrite("filter_obstacles_in_cleaned_mask", &LogOddMap2D::Setting::filter_obstacles_in_cleaned_mask);

    py_log_odd_map
        .def(py::init<std::shared_ptr<LogOddMap2D::Setting>, std::shared_ptr<GridMapInfo2D>>(), py::arg("setting").none(false), py::arg("grid_map_info"))
        .def(
            py::init<std::shared_ptr<LogOddMap2D::Setting>, std::shared_ptr<GridMapInfo2D>, const Eigen::Ref<const Eigen::Matrix2Xd> &>(),
            py::arg("setting").none(false),
            py::arg("grid_map_info"),
            py::arg("shape_vertices"))
        .def_static("get_cell_type_name", &LogOddMap2D::GetCellTypeName, py::arg("cell_type"))
        .def_static("get_cell_type_from_name", &LogOddMap2D::GetCellTypeFromName, py::arg("cell_type_name"))
        .def("update", &LogOddMap2D::Update, py::arg("position"), py::arg("theta"), py::arg("angles_body"), py::arg("ranges"))
        .def("load_external_possibility_map", &LogOddMap2D::LoadExternalPossibilityMap, py::arg("position"), py::arg("theta"), py::arg("possibility_map"))
        .def(
            "compute_statistics_of_lidar_frame",
            [](const LogOddMap2D &self,
               const Eigen::Ref<const Eigen::Vector2d> &position,
               double theta,
               const Eigen::Ref<const Eigen::VectorXd> &angles_body,
               const Eigen::Ref<const Eigen::VectorXd> &ranges,
               bool clip_ranges) {
                std::shared_ptr<LogOddMap2D::LidarFrameMask> mask = nullptr;
                int num_occupied_cells;
                int num_free_cells;
                int num_unexplored_cells;
                int num_out_of_map_cells;
                self.ComputeStatisticsOfLidarFrame(
                    position,
                    theta,
                    angles_body,
                    ranges,
                    clip_ranges,
                    mask,
                    num_occupied_cells,
                    num_free_cells,
                    num_unexplored_cells,
                    num_out_of_map_cells);
                return std::make_tuple(num_occupied_cells, num_free_cells, num_unexplored_cells, num_out_of_map_cells);
            },
            py::arg("position"),
            py::arg("theta"),
            py::arg("angles_body"),
            py::arg("ranges"),
            py::arg("clip_ranges"))
        .def_property_readonly("setting", &LogOddMap2D::GetSetting)
        .def_property_readonly("log_map", &LogOddMap2D::GetLogMap)
        .def_property_readonly("possibility_map", &LogOddMap2D::GetPossibilityMap)
        .def_property_readonly("occupancy_map", &LogOddMap2D::GetOccupancyMap)
        .def_property_readonly("unexplored_mask", &LogOddMap2D::GetUnexploredMask)
        .def_property_readonly("occupied_mask", &LogOddMap2D::GetOccupiedMask)
        .def_property_readonly("free_mask", &LogOddMap2D::GetFreeMask)
        .def_property_readonly("num_unexplored_cells", &LogOddMap2D::GetNumUnexploredCells)
        .def_property_readonly("num_occupied_cells", &LogOddMap2D::GetNumOccupiedCells)
        .def_property_readonly("num_free_cells", &LogOddMap2D::GetNumFreeCells)
        .def_property_readonly("cleaned_free_mask", &LogOddMap2D::GetCleanedFreeMask)
        .def_property_readonly("cleaned_occupied_mask", &LogOddMap2D::GetCleanedOccupiedMask)
        .def_property_readonly("cleaned_unexplored_mask", &LogOddMap2D::GetCleanedUnexploredMask)
        .def("get_frontiers", &LogOddMap2D::GetFrontiers, py::arg("clean_at_first") = true, py::arg("approx_iters") = 4);
}

static void
BindCollisionCheckers(py::module &m) {
    py::class_<CollisionCheckerBase>(m, ERL_AS_STRING(CollisionCheckerBase)).def("is_collided", &CollisionCheckerBase::IsCollided, py::arg("grid_coords"));

    py::class_<PointCollisionChecker2D, CollisionCheckerBase>(m, ERL_AS_STRING(PointCollisionChecker2D))
        .def(py::init<std::shared_ptr<GridMap<uint8_t, 2>>>(), py::arg("grid_map"));

    py::class_<PointCollisionChecker3D, CollisionCheckerBase>(m, ERL_AS_STRING(PointCollisionChecker3D))
        .def(py::init<std::shared_ptr<GridMap<uint8_t, 3>>>(), py::arg("grid_map"));

    py::class_<GridCollisionCheckerSe2, CollisionCheckerBase>(m, ERL_AS_STRING(GridCollisionCheckerSe2))
        .def(
            py::init<std::shared_ptr<GridMap<uint8_t, 2>>, const std::shared_ptr<GridMapInfo3D> &, Eigen::Matrix2Xd>(),
            py::arg("grid_map"),
            py::arg("se2_grid_map_info"),
            py::arg("metric_shape"))
        .def("is_collided", py::overload_cast<const Eigen::Ref<const Eigen::Matrix3d> &>(&GridCollisionCheckerSe2::IsCollided, py::const_), py::arg("pose"));

    py::class_<GridCollisionChecker3D, CollisionCheckerBase>(m, ERL_AS_STRING(GridCollisionChecker3D))
        .def(py::init<std::shared_ptr<GridMap<uint8_t, 3>>, Eigen::Matrix3Xd>(), py::arg("grid_map"), py::arg("metric_voxels"))
        .def("is_collided", py::overload_cast<const Eigen::Ref<const Eigen::Matrix4d> &>(&GridCollisionChecker3D::IsCollided, py::const_), py::arg("pose"));
}

static void
BindHouseExpo(py::module &m) {
    py::class_<HouseExpoMap>(m, ERL_AS_STRING(HouseExpoMap))
        .def(py::init<const char *>(), py::arg("file"))
        .def(py::init<const char *, double>(), py::arg("file"), py::arg("wall_thickness"))
        .def_property_readonly("file", &HouseExpoMap::GetFile)
        .def_property_readonly("room_id", &HouseExpoMap::GetRoomId)
        .def_property_readonly("meter_space", &HouseExpoMap::GetMeterSpace);
}

PYBIND11_MODULE(PYBIND_MODULE_NAME, m) {
    m.doc() = "Python 3 Interface of erl_geometry";
    m.def(
         "bresenham_2d",
         [](const Eigen::Ref<const Eigen::Vector2i> &start,
            const Eigen::Ref<const Eigen::Vector2i> &end,
            const std::optional<std::function<bool(long, long)>> &stop) {
             return stop.has_value() ? Bresenham2D(start, end, stop.value()) : Bresenham2D(start, end);
         },
         py::arg("start"),
         py::arg("end"),
         py::arg("stop") = py::none())
        .def("compute_pixels_of_polygon_contour", &ComputePixelsOfPolygonContour, py::arg("polygon_vertices"))
        .def(
            "marching_square",
            [](const Eigen::Ref<const Eigen::MatrixXd> &img, double iso_value) {
                Eigen::Matrix2Xd vertices;
                Eigen::Matrix2Xi lines_to_vertices;
                Eigen::Matrix2Xi objects_to_lines;
                MarchingSquare(img, iso_value, vertices, lines_to_vertices, objects_to_lines);

                return py::make_tuple(vertices, lines_to_vertices, objects_to_lines);
            },
            py::arg("img"),
            py::arg("iso_value"))
        .def("winding_number", &WindingNumber, py::arg("p"), py::arg("vertices"))
        .def(
            "compute_nearest_distance_from_point_to_line_segment_2d",
            &ComputeNearestDistanceFromPointToLineSegment2D,
            py::arg("point_x"),
            py::arg("point_y"),
            py::arg("line_segment_x1"),
            py::arg("line_segment_y1"),
            py::arg("line_segment_x2"),
            py::arg("line_segment_y2"))
        .def(
            "compute_intersection_between_ray_and_segment_2d",
            [](const Eigen::Ref<const Eigen::Vector2d> &ray_start_point,
               const Eigen::Ref<const Eigen::Vector2d> &ray_direction,
               const Eigen::Ref<const Eigen::Vector2d> &segment_point1,
               const Eigen::Ref<const Eigen::Vector2d> &segment_point2) {
                double lambda = 0;
                double distance = 0;
                ComputeIntersectionBetweenRayAndSegment2D(ray_start_point, ray_direction, segment_point1, segment_point2, lambda, distance);
                return py::make_tuple(lambda, distance);
            },
            py::arg("ray_start_point"),
            py::arg("ray_direction"),
            py::arg("segment_point1"),
            py::arg("segment_point2"))
        .def(
            "compute_intersection_between_ray_and_aabb_2d",
            [](const Eigen::Ref<const Eigen::Vector2d> &p,
               const Eigen::Ref<const Eigen::Vector2d> &r,
               const Eigen::Ref<const Eigen::Vector2d> &box_min,
               const Eigen::Ref<const Eigen::Vector2d> &box_max) {
                double d = 0;
                bool intersected = false;
                ComputeIntersectionBetweenRayAndAabb2D(p, r.cwiseInverse(), box_min, box_max, d, intersected);
                return py::make_tuple(d, intersected);
            },
            py::arg("ray_start_point"),
            py::arg("ray_direction"),
            py::arg("aabb_min"),
            py::arg("aabb_max"));

    BindAabb<double, 2>(m, "Aabb2D");
    BindAabb<double, 3>(m, "Aabb3D");
    BindNode(m);
    BindNodeContainer(m);
    BindNodeContainerMultiTypes(m);
    BindIncrementalQuadTree(m);
    BindOccupancyQuadtree(m);
    BindSurface2D(m);
    BindSpace2D(m);
    BindLidar2D(m);
    BindLidarFrame2D(m);
    BindLogOddMap2D(m);
    BindCollisionCheckers(m);
    BindHouseExpo(m);
}
