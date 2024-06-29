#pragma once

#include "occupancy_octree_base.hpp"
#include "occupancy_octree_drawer.hpp"
#include "occupancy_octree_node.hpp"

namespace erl::geometry {

    class OccupancyOctree : public OccupancyOctreeBase<OccupancyOctreeNode, OccupancyOctreeBaseSetting> {
    public:
        using Setting = OccupancyOctreeBaseSetting;
        using Drawer = OccupancyOctreeDrawer<OccupancyOctree>;

        explicit OccupancyOctree(const std::shared_ptr<OccupancyOctreeBaseSetting> &setting)
            : OccupancyOctreeBase(setting) {}

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
        [[nodiscard]] std::shared_ptr<AbstractOctree>
        Create(const std::shared_ptr<NdTreeSetting> &setting) const override {
            auto tree_setting = std::dynamic_pointer_cast<Setting>(setting);
            if (tree_setting == nullptr) { tree_setting = std::make_shared<Setting>(); }
            return std::make_shared<OccupancyOctree>(tree_setting);
        }
    };

    ERL_REGISTER_OCTREE(OccupancyOctree);
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::OccupancyOctree::Drawer::Setting>
    : public ConvertOccupancyOctreeDrawerSetting<erl::geometry::OccupancyOctree::Drawer::Setting> {};  // namespace YAML
