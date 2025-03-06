#pragma once

#include "occupancy_octree_base.hpp"
#include "occupancy_octree_drawer.hpp"
#include "occupancy_octree_node.hpp"

namespace erl::geometry {

    template<typename Dtype>
    class OccupancyOctree : public OccupancyOctreeBase<Dtype, OccupancyOctreeNode, OccupancyOctreeBaseSetting> {
    public:
        using Setting = OccupancyOctreeBaseSetting;
        using Super = OccupancyOctreeBase<Dtype, OccupancyOctreeNode, OccupancyOctreeBaseSetting>;
        using Drawer = OccupancyOctreeDrawer<OccupancyOctree>;

        explicit OccupancyOctree(const std::shared_ptr<OccupancyOctreeBaseSetting> &setting)
            : Super(setting) {}

        OccupancyOctree()
            : OccupancyOctree(std::make_shared<OccupancyOctreeBaseSetting>()) {}

        explicit OccupancyOctree(const std::string &filename)
            : OccupancyOctree() {  // resolution will be set by LoadData
            ERL_ASSERTM(this->LoadData(filename), "Failed to read OccupancyOctree from file: {}", filename);
        }

        OccupancyOctree(const OccupancyOctree &other) = default;
        OccupancyOctree &
        operator=(const OccupancyOctree &other) = default;
        OccupancyOctree(OccupancyOctree &&other) = default;
        OccupancyOctree &
        operator=(OccupancyOctree &&other) = default;

    protected:
        [[nodiscard]] std::shared_ptr<AbstractOctree<Dtype>>
        Create(const std::shared_ptr<NdTreeSetting> &setting) const override {
            auto tree_setting = std::dynamic_pointer_cast<Setting>(setting);
            if (tree_setting == nullptr) {
                ERL_DEBUG_ASSERT(setting == nullptr, "setting is not the type for OccupancyOctree.");
                tree_setting = std::make_shared<Setting>();
            }
            return std::make_shared<OccupancyOctree>(tree_setting);
        }
    };

    using OccupancyOctreeD = OccupancyOctree<double>;
    using OccupancyOctreeF = OccupancyOctree<float>;
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::OccupancyOctreeD::Drawer::Setting> : erl::geometry::OccupancyOctreeD::Drawer::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::geometry::OccupancyOctreeF::Drawer::Setting> : erl::geometry::OccupancyOctreeF::Drawer::Setting::YamlConvertImpl {};
