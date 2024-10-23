#include "erl_common/pybind11.hpp"
#include "erl_geometry/abstract_occupancy_octree.hpp"

void
BindAbstractOccupancyOctree(const py::module &m) {
    using namespace erl::geometry;
    py::class_<AbstractOccupancyOctree, AbstractOctree, std::shared_ptr<AbstractOccupancyOctree>>(m, "AbstractOccupancyOctree")
        .def(
            "write_binary",
            [](AbstractOccupancyOctree &self, const std::string &filename, const bool prune_at_first) -> bool {
                if (prune_at_first) { return self.WriteBinary(filename); }
                return const_cast<const AbstractOccupancyOctree &>(self).WriteBinary(filename);
            },
            py::arg("filename"),
            py::arg("prune_at_first"))
        .def(
            "read_binary",
            [](AbstractOccupancyOctree &self, const std::string &filename) -> bool { return self.ReadBinary(filename); },
            py::arg("filename"))
        .def("is_node_occupied", &AbstractOccupancyOctree::IsNodeOccupied, py::arg("node"))
        .def("is_node_at_threshold", &AbstractOccupancyOctree::IsNodeAtThreshold, py::arg("node"))
        .def(
            "get_hit_occupied_node",
            [](const AbstractOccupancyOctree &self,
               const double px,
               const double py,
               const double pz,
               const double vx,
               const double vy,
               const double vz,
               const bool ignore_unknown,
               const double max_range) {
                double ex, ey, ez;
                auto *node = self.GetHitOccupiedNode(px, py, pz, vx, vy, vz, ignore_unknown, max_range, ex, ey, ez);
                py::dict result;
                if (node == nullptr) { return result; }
                result["node"] = node;
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
            py::arg("max_range"));
}
