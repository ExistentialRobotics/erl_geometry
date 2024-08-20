#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_octree_node.hpp"
#include "erl_geometry/pybind11_occupancy_octree.hpp"

void
BindOccupancyOctreeNode(const py::module& m) {
    using namespace erl::geometry;
    BindOccupancyOctreeNode<OccupancyOctreeNode, AbstractOctreeNode>(m, "OccupancyOctreeNode")
        .def_property_readonly("occupancy", &OccupancyOctreeNode::GetOccupancy)
        .def_property_readonly("log_odds", &OccupancyOctreeNode::GetLogOdds)
        .def_property_readonly("mean_child_log_odds", &OccupancyOctreeNode::GetMeanChildLogOdds)
        .def_property_readonly("max_child_log_odds", &OccupancyOctreeNode::GetMaxChildLogOdds)
        .def("allow_update_log_odds", &OccupancyOctreeNode::AllowUpdateLogOdds, py::arg("delta"))
        .def("add_log_odds", &OccupancyOctreeNode::AddLogOdds, py::arg("log_odds"))
        .def("get_child", py::overload_cast<uint32_t>(&OccupancyOctreeNode::GetChild<OccupancyOctreeNode>, py::const_), py::arg("child_idx"));
}
