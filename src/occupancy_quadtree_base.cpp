#include "erl_geometry/occupancy_quadtree_base.hpp"

namespace erl::geometry {
    bool
    OccupancyQuadtreeBaseSetting::operator==(const NdTreeSetting &rhs) const {
        if (OccupancyNdTreeSetting::operator==(rhs)) {
            const auto that = reinterpret_cast<const OccupancyQuadtreeBaseSetting &>(rhs);
            return use_change_detection == that.use_change_detection &&  //
                   use_aabb_limit == that.use_aabb_limit &&              //
                   aabb == that.aabb;
        }
        return false;
    }
}  // namespace erl::geometry

YAML::Node
YAML::convert<erl::geometry::OccupancyQuadtreeBaseSetting>::encode(
    const erl::geometry::OccupancyQuadtreeBaseSetting &setting) {
    Node node = convert<erl::geometry::OccupancyNdTreeSetting>::encode(setting);
    ERL_YAML_SAVE_ATTR(node, setting, use_change_detection);
    ERL_YAML_SAVE_ATTR(node, setting, use_aabb_limit);
    ERL_YAML_SAVE_ATTR(node, setting, aabb);
    return node;
}

bool
YAML::convert<erl::geometry::OccupancyQuadtreeBaseSetting>::decode(
    const Node &node,
    erl::geometry::OccupancyQuadtreeBaseSetting &setting) {
    if (!node.IsMap()) { return false; }
    if (!convert<erl::geometry::OccupancyNdTreeSetting>::decode(node, setting)) { return false; }
    ERL_YAML_LOAD_ATTR(node, setting, use_change_detection);
    ERL_YAML_LOAD_ATTR(node, setting, use_aabb_limit);
    ERL_YAML_LOAD_ATTR(node, setting, aabb);
    return true;
}
