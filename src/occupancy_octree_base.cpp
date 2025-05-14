#include "erl_geometry/occupancy_octree_base.hpp"

namespace erl::geometry {
    bool
    OccupancyOctreeBaseSetting::operator==(const NdTreeSetting &other) const {
        if (OccupancyNdTreeSetting::operator==(other)) {
            const auto that = reinterpret_cast<const OccupancyOctreeBaseSetting &>(other);
            return use_change_detection == that.use_change_detection &&  //
                   use_aabb_limit == that.use_aabb_limit &&              //
                   aabb == that.aabb;
        }
        return false;
    }
}  // namespace erl::geometry

YAML::Node
YAML::convert<erl::geometry::OccupancyOctreeBaseSetting>::encode(
    const erl::geometry::OccupancyOctreeBaseSetting &rhs) {
    Node node = convert<erl::geometry::OccupancyNdTreeSetting>::encode(rhs);
    ERL_YAML_SAVE_ATTR(node, rhs, use_change_detection);
    ERL_YAML_SAVE_ATTR(node, rhs, use_aabb_limit);
    ERL_YAML_SAVE_ATTR(node, rhs, aabb);
    return node;
}

bool
YAML::convert<erl::geometry::OccupancyOctreeBaseSetting>::decode(
    const Node &node,
    erl::geometry::OccupancyOctreeBaseSetting &rhs) {
    if (!node.IsMap()) { return false; }
    if (!convert<erl::geometry::OccupancyNdTreeSetting>::decode(node, rhs)) { return false; }
    ERL_YAML_LOAD_ATTR(node, rhs, use_change_detection);
    ERL_YAML_LOAD_ATTR(node, rhs, use_aabb_limit);
    ERL_YAML_LOAD_ATTR(node, rhs, aabb);
    return true;
}
