#include "erl_common/pybind11.hpp"
#include "erl_geometry/abstract_occupancy_quadtree.hpp"

void
BindAbstractOccupancyQuadtree(const py::module& m) {
    using namespace erl::geometry;
    py::class_<AbstractOccupancyQuadtree, AbstractQuadtree, std::shared_ptr<AbstractOccupancyQuadtree>>(m, "AbstractOccupancyQuadtree")
        .def(
            "write_binary",
            [](AbstractOccupancyQuadtree& self, const std::string& filename, const bool prune_at_first) -> bool {
                if (prune_at_first) { return self.WriteBinary(filename); }
                return const_cast<const AbstractOccupancyQuadtree&>(self).WriteBinary(filename);
            },
            py::arg("filename"),
            py::arg("prune_at_first"))
        .def(
            "read_binary",
            [](AbstractOccupancyQuadtree& self, const std::string& filename) -> bool { return self.ReadBinary(filename); },
            py::arg("filename"))
        .def("is_node_occupied", &AbstractOccupancyQuadtree::IsNodeOccupied, py::arg("node"))
        .def("is_node_at_threshold", &AbstractOccupancyQuadtree::IsNodeAtThreshold, py::arg("node"))
        .def(
            "get_hit_occupied_node",
            [](const AbstractOccupancyQuadtree& self,
               const double px,
               const double py,
               const double vx,
               const double vy,
               const bool ignore_unknown,
               const double max_range) {
                double ex, ey;
                auto* node = self.GetHitOccupiedNode(px, py, vx, vy, ignore_unknown, max_range, ex, ey);
                py::dict result;
                if (node == nullptr) { return result; }
                result["node"] = node;
                result["ex"] = ex;
                result["ey"] = ey;
                return result;
            },
            py::arg("px"),
            py::arg("py"),
            py::arg("vx"),
            py::arg("vy"),
            py::arg("ignore_unknown"),
            py::arg("max_range"));
}
