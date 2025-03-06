#include "erl_common/pybind11.hpp"
#include "erl_geometry/abstract_occupancy_quadtree.hpp"

template<typename Dtype>
void
BindAbstractOccupancyQuadtreeImpl(const py::module& m, const char* name) {
    using namespace erl::geometry;
    using T = AbstractOccupancyQuadtree<Dtype>;

    py::class_<T, AbstractQuadtree<Dtype>, std::shared_ptr<T>>(m, name)
        .def(
            "write_binary",
            [](T& self, const std::string& filename, const bool prune_at_first) -> bool {
                if (prune_at_first) { return self.WriteBinary(filename); }
                return const_cast<const T&>(self).WriteBinary(filename);
            },
            py::arg("filename"),
            py::arg("prune_at_first"))
        .def(
            "read_binary",
            [](T& self, const std::string& filename) -> bool { return self.ReadBinary(filename); },
            py::arg("filename"))
        .def("is_node_occupied", &T::IsNodeOccupied, py::arg("node"))
        .def("is_node_at_threshold", &T::IsNodeAtThreshold, py::arg("node"))
        .def(
            "get_hit_occupied_node",
            [](const T& self, const Dtype px, const Dtype py, const Dtype vx, const Dtype vy, const bool ignore_unknown, const Dtype max_range) {
                Dtype ex, ey;
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

void
BindAbstractOccupancyQuadtree(const py::module& m) {
    BindAbstractOccupancyQuadtreeImpl<double>(m, "AbstractOccupancyQuadtreeD");
    BindAbstractOccupancyQuadtreeImpl<float>(m, "AbstractOccupancyQuadtreeF");
}
