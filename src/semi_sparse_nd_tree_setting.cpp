#include "erl_geometry/semi_sparse_nd_tree_setting.hpp"

bool
erl::geometry::SemiSparseNdTreeSetting::operator==(const NdTreeSetting &other) const {
    if (NdTreeSetting::operator==(other)) {
        const auto that = reinterpret_cast<const SemiSparseNdTreeSetting &>(other);
        return full_depth == that.full_depth && init_voxel_num == that.init_voxel_num;
    }
    return false;
}

YAML::Node
YAML::convert<erl::geometry::SemiSparseNdTreeSetting>::encode(
    const erl::geometry::SemiSparseNdTreeSetting &setting) {
    Node node = convert<erl::geometry::NdTreeSetting>::encode(setting);
    ERL_YAML_SAVE_ATTR(node, setting, full_depth);
    ERL_YAML_SAVE_ATTR(node, setting, init_voxel_num);
    return node;
}

bool
YAML::convert<erl::geometry::SemiSparseNdTreeSetting>::decode(
    const Node &node,
    erl::geometry::SemiSparseNdTreeSetting &setting) {
    if (!node.IsMap()) { return false; }
    if (!convert<erl::geometry::NdTreeSetting>::decode(node, setting)) { return false; }
    ERL_YAML_LOAD_ATTR(node, setting, full_depth);
    ERL_YAML_LOAD_ATTR(node, setting, init_voxel_num);
    return true;
}
