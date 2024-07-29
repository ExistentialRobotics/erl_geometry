#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_quadtree_node.hpp"

void
BindOccupancyQuadtreeNode(const py::module &m) {
    using namespace erl::geometry;
    py::class_<OccupancyQuadtreeNode, AbstractQuadtreeNode, py::RawPtrWrapper<OccupancyQuadtreeNode>>(m, "OccupancyQuadtreeNode")
        .def_property_readonly("occupancy", &OccupancyQuadtreeNode::GetOccupancy)
        .def_property_readonly("log_odds", &OccupancyQuadtreeNode::GetLogOdds)
        .def_property_readonly("mean_child_log_odds", &OccupancyQuadtreeNode::GetMeanChildLogOdds)
        .def_property_readonly("max_child_log_odds", &OccupancyQuadtreeNode::GetMaxChildLogOdds)
        .def("allow_update_log_odds", &OccupancyQuadtreeNode::AllowUpdateLogOdds, py::arg("delta"))
        .def("add_log_odds", &OccupancyQuadtreeNode::AddLogOdds, py::arg("log_odds"))
        .def("get_child", py::overload_cast<uint32_t>(&OccupancyQuadtreeNode::GetChild<OccupancyQuadtreeNode>, py::const_), py::arg("child_idx"));
}
