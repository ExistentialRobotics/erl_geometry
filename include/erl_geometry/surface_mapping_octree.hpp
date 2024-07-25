#pragma once

#include "occupancy_octree_base.hpp"
#include "occupancy_octree_drawer.hpp"
#include "surface_mapping_octree_node.hpp"

namespace erl::geometry {

    class SurfaceMappingOctree : public OccupancyOctreeBase<SurfaceMappingOctreeNode, OccupancyOctreeBaseSetting> {

    public:
        using Setting = OccupancyOctreeBaseSetting;
        using Drawer = OccupancyOctreeDrawer<SurfaceMappingOctree>;

        explicit SurfaceMappingOctree(const std::shared_ptr<Setting> &setting)
            : OccupancyOctreeBase(setting) {}

        SurfaceMappingOctree()
            : SurfaceMappingOctree(std::make_shared<Setting>()) {}

        explicit SurfaceMappingOctree(const std::string &filename)
            : SurfaceMappingOctree() {  // resolution will be set by LoadData
            ERL_ASSERTM(this->LoadData(filename), "Failed to read SurfaceMappingOctree from file: {}", filename);
        }

        SurfaceMappingOctree(const SurfaceMappingOctree &) = default;
        SurfaceMappingOctree &
        operator=(const SurfaceMappingOctree &) = default;
        SurfaceMappingOctree(SurfaceMappingOctree &&) = default;
        SurfaceMappingOctree &
        operator=(SurfaceMappingOctree &&) = default;

    protected:
        [[nodiscard]] std::shared_ptr<AbstractOctree>
        Create(const std::shared_ptr<NdTreeSetting> &setting) const override {
            auto tree_setting = std::dynamic_pointer_cast<Setting>(setting);
            if (tree_setting == nullptr) {
                ERL_DEBUG_ASSERT(setting == nullptr, "setting is not the type for SurfaceMappingOctree.");
                tree_setting = std::make_shared<Setting>();
            }
            return std::make_shared<SurfaceMappingOctree>(tree_setting);
        }
    };

    ERL_REGISTER_OCTREE(SurfaceMappingOctree);

}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::SurfaceMappingOctree::Drawer::Setting>
    : public ConvertOccupancyOctreeDrawerSetting<erl::geometry::SurfaceMappingOctree::Drawer::Setting> {};  // namespace YAML
