#pragma once

#include "abstract_octree_drawer.hpp"

#include "erl_common/yaml.hpp"

#include <open3d/geometry/VoxelGrid.h>

#include <functional>

namespace erl::geometry {

    struct OccupancyOctreeDrawerSetting : common::Yamlable<OccupancyOctreeDrawerSetting, AbstractOctreeDrawer::Setting> {
        bool occupied_only = false;
        Eigen::Vector3d occupied_color = {0.67, 0.33, 0.0};  // brown
        bool draw_node_boxes = true;
        bool draw_node_borders = true;

        struct YamlConvertImpl {
            static YAML::Node
            encode(const OccupancyOctreeDrawerSetting &setting);

            static bool
            decode(const YAML::Node &node, OccupancyOctreeDrawerSetting &setting);
        };
    };

    template<typename OccupancyOctreeType>
    class OccupancyOctreeDrawer : public AbstractOctreeDrawer {
    public:
        using Setting = OccupancyOctreeDrawerSetting;
        using DrawTreeCallback = std::function<void(
            const OccupancyOctreeDrawer *,                               // this
            std::vector<std::shared_ptr<open3d::geometry::Geometry>> &,  // geometries
            typename OccupancyOctreeType::TreeInAabbIterator &)>;
        using DrawLeafCallback = std::function<void(
            const OccupancyOctreeDrawer *,                               // this
            std::vector<std::shared_ptr<open3d::geometry::Geometry>> &,  // geometries
            typename OccupancyOctreeType::LeafInAabbIterator &)>;

    private:
        std::shared_ptr<Setting> m_setting_ = {};
        std::shared_ptr<const OccupancyOctreeType> m_octree_ = nullptr;
        DrawTreeCallback m_draw_tree_ = {};
        DrawLeafCallback m_draw_leaf_ = {};

    public:
        explicit OccupancyOctreeDrawer(std::shared_ptr<Setting> setting, std::shared_ptr<const OccupancyOctreeType> octree = nullptr);

        using AbstractOctreeDrawer::DrawLeaves;
        using AbstractOctreeDrawer::DrawTree;

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const;

        void
        SetOctree(std::shared_ptr<const OccupancyOctreeType> octree);

        void
        SetDrawTreeCallback(DrawTreeCallback draw_tree);

        void
        SetDrawLeafCallback(DrawLeafCallback draw_leaf);

        void
        DrawTree(std::vector<std::shared_ptr<open3d::geometry::Geometry>> &geometries) const override;

        void
        DrawLeaves(std::vector<std::shared_ptr<open3d::geometry::Geometry>> &geometries) const override;
    };
}  // namespace erl::geometry

#include "occupancy_octree_drawer.tpp"

template<>
struct YAML::convert<erl::geometry::OccupancyOctreeDrawerSetting> : erl::geometry::OccupancyOctreeDrawerSetting::YamlConvertImpl {};
