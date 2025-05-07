#pragma once

#include "erl_common/yaml.hpp"

#include <cstdint>
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
        operator==(const NdTreeSetting& other) const {
            if (typeid(*this) != typeid(other)) { return false; }
            return resolution == other.resolution && tree_depth == other.tree_depth;
        }

        bool
        operator!=(const NdTreeSetting& rhs) const {
            return !(*this == rhs);
        }
    };
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::NdTreeSetting> {
    static Node
    encode(const erl::geometry::NdTreeSetting& setting);

    static bool
    decode(const Node& node, erl::geometry::NdTreeSetting& setting);
};  // namespace YAML
