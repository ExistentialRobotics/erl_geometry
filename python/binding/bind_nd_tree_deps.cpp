#include "erl_common/pybind11.hpp"

#include "erl_geometry/quadtree_key.hpp"
#include "erl_geometry/octree_key.hpp"
#include "erl_geometry/nd_tree_setting.hpp"
#include "erl_geometry/abstract_octree_node.hpp"

void
BindNdTreeDeps(py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<NdTreeSetting, YamlableBase, std::shared_ptr<NdTreeSetting>>(m, "NdTreeSetting")
        .def(py::init<>())
        .def_readwrite("resolution", &NdTreeSetting::resolution)
        .def_readwrite("tree_depth", &NdTreeSetting::tree_depth);

    py::class_<QuadtreeKey>(m, "QuadtreeKey")
        .def("__eq__", [](const QuadtreeKey &self, const QuadtreeKey &other) { return self == other; })
        .def("__ne__", [](const QuadtreeKey &self, const QuadtreeKey &other) { return self != other; })
        .def("__getitem__", [](const QuadtreeKey &self, int idx) { return self[idx]; });
    // QuadtreeKeyRay is std::vector<QuadtreeKey>

    py::class_<OctreeKey>(m, "OctreeKey")
        .def("__eq__", [](const OctreeKey &self, const OctreeKey &other) { return self == other; })
        .def("__ne__", [](const OctreeKey &self, const OctreeKey &other) { return self != other; })
        .def("__getitem__", [](const OctreeKey &self, int idx) { return self[idx]; });
    // OctreeKeyRay is std::vector<OctreeKey>

    py::class_<AbstractOctreeNode, py::raw_ptr_wrapper<AbstractOctreeNode>>(m, "AbstractOctreeNode")
        .def_property_readonly("depth", &AbstractOctreeNode::GetDepth)
        .def_property_readonly("child_index", &AbstractOctreeNode::GetChildIndex)
        .def_property_readonly("num_children", &AbstractOctreeNode::GetNumChildren);
}
