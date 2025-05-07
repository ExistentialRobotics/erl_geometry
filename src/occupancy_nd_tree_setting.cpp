#include "erl_geometry/occupancy_nd_tree_setting.hpp"

bool
erl::geometry::OccupancyNdTreeSetting::operator==(const NdTreeSetting &other) const {
    if (NdTreeSetting::operator==(other)) {
        const auto that = reinterpret_cast<const OccupancyNdTreeSetting &>(other);
        return log_odd_min == that.log_odd_min && log_odd_max == that.log_odd_max &&    //
               log_odd_hit == that.log_odd_hit && log_odd_miss == that.log_odd_miss &&  //
               log_odd_occ_threshold == that.log_odd_occ_threshold;
    }
    return false;
}

YAML::Node
YAML::convert<erl::geometry::OccupancyNdTreeSetting>::encode(
    const erl::geometry::OccupancyNdTreeSetting &setting) {
    Node node = convert<erl::geometry::NdTreeSetting>::encode(setting);
    ERL_YAML_SAVE_ATTR(node, setting, log_odd_min);
    ERL_YAML_SAVE_ATTR(node, setting, log_odd_max);
    ERL_YAML_SAVE_ATTR(node, setting, log_odd_hit);
    ERL_YAML_SAVE_ATTR(node, setting, log_odd_miss);
    ERL_YAML_SAVE_ATTR(node, setting, log_odd_occ_threshold);
    return node;
}

bool
YAML::convert<erl::geometry::OccupancyNdTreeSetting>::decode(
    const Node &node,
    erl::geometry::OccupancyNdTreeSetting &rhs) {
    if (!node.IsMap()) { return false; }
    if (!convert<erl::geometry::NdTreeSetting>::decode(node, rhs)) { return false; }
    ERL_YAML_LOAD_ATTR_TYPE(node, rhs, log_odd_min, float);
    ERL_YAML_LOAD_ATTR_TYPE(node, rhs, log_odd_max, float);
    ERL_YAML_LOAD_ATTR_TYPE(node, rhs, log_odd_hit, float);
    ERL_YAML_LOAD_ATTR_TYPE(node, rhs, log_odd_miss, float);
    ERL_YAML_LOAD_ATTR_TYPE(node, rhs, log_odd_occ_threshold, float);
    return true;
}
