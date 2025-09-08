#pragma once

#include "erl_common/yaml.hpp"

#include <typeinfo>

namespace erl::geometry {

    /**
     * NDTreeSetting is a base class for all n-d tree settings.
     */
    class NdTreeSetting : public common::Yamlable<NdTreeSetting> {
    public:
        float resolution = 0.1;
        uint32_t tree_depth = 16;

        virtual bool
        operator==(const NdTreeSetting& other) const;

        bool
        operator!=(const NdTreeSetting& rhs) const {
            return !(*this == rhs);
        }
    };

    struct SemiSparseNdTreeSetting : common::Yamlable<SemiSparseNdTreeSetting, NdTreeSetting> {

        uint32_t full_depth = 2;  // depth up to which all child nodes are always allocated
        std::size_t init_voxel_num = 200000;  // initial number of voxels to allocate memory for

        bool
        operator==(const NdTreeSetting& other) const override;
    };
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::NdTreeSetting> {
    static Node
    encode(const erl::geometry::NdTreeSetting& setting);

    static bool
    decode(const Node& node, erl::geometry::NdTreeSetting& setting);
};  // namespace YAML

template<>
struct YAML::convert<erl::geometry::SemiSparseNdTreeSetting> {
    static Node
    encode(const erl::geometry::SemiSparseNdTreeSetting& setting);

    static bool
    decode(const Node& node, erl::geometry::SemiSparseNdTreeSetting& setting);
};  // namespace YAML
