#pragma once

#include "nd_tree_setting.hpp"

#include "erl_common/yaml.hpp"

namespace erl::geometry {

    struct SemiSparseNdTreeSetting : common::Yamlable<SemiSparseNdTreeSetting, NdTreeSetting> {

        uint32_t full_depth = 2;  // depth up to which all child nodes are always allocated
        std::size_t init_voxel_num = 200000;  // initial number of voxels to allocate memory for

        bool
        operator==(const NdTreeSetting& other) const override;
    };
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::SemiSparseNdTreeSetting> {
    static Node
    encode(const erl::geometry::SemiSparseNdTreeSetting& setting);

    static bool
    decode(const Node& node, erl::geometry::SemiSparseNdTreeSetting& setting);
};  // namespace YAML
