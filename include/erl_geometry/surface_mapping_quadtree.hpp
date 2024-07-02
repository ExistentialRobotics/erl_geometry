#pragma once

#include "occupancy_quadtree_base.hpp"
#include "occupancy_quadtree_drawer.hpp"
#include "surface_mapping_quadtree_node.hpp"

namespace erl::geometry {

    class SurfaceMappingQuadtree : public geometry::OccupancyQuadtreeBase<SurfaceMappingQuadtreeNode, geometry::OccupancyQuadtreeBaseSetting> {

    public:
        using Setting = geometry::OccupancyQuadtreeBaseSetting;
        using Drawer = geometry::OccupancyQuadtreeDrawer<SurfaceMappingQuadtree>;

        explicit SurfaceMappingQuadtree(const std::shared_ptr<Setting> &setting)
            : OccupancyQuadtreeBase(setting) {}

        SurfaceMappingQuadtree()
            : SurfaceMappingQuadtree(std::make_shared<Setting>()) {}

        explicit SurfaceMappingQuadtree(const std::string &filename)
            : SurfaceMappingQuadtree() {  // resolution will be set by LoadData
            ERL_ASSERTM(this->LoadData(filename), "Failed to read SurfaceMappingQuadtree from file: {}", filename);
        }

        SurfaceMappingQuadtree(const SurfaceMappingQuadtree &) = delete;  // no copy constructor

    protected:
        [[nodiscard]] std::shared_ptr<AbstractQuadtree>
        Create(const std::shared_ptr<geometry::NdTreeSetting> &setting) const override {
            auto tree_setting = std::dynamic_pointer_cast<Setting>(setting);
            if (tree_setting == nullptr) { tree_setting = std::make_shared<Setting>(); }
            return std::make_shared<SurfaceMappingQuadtree>(tree_setting);
        }
    };

    ERL_REGISTER_QUADTREE(SurfaceMappingQuadtree);

}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::SurfaceMappingQuadtree::Drawer::Setting>
    : public ConvertOccupancyQuadtreeDrawerSetting<erl::geometry::SurfaceMappingQuadtree::Drawer::Setting> {};  // namespace YAML
