#pragma once

#include "open3d_visualizer_wrapper.hpp"

#include "erl_common/yaml.hpp"

#include <open3d/geometry/VoxelGrid.h>

namespace erl::geometry {

    class AbstractOctreeDrawer {
    public:
        struct Setting : public common::Yamlable<Setting> {
            float scaling = 1.0f;
            Eigen::Vector3d area_min = {-1.0, -1.0, -1.0};
            Eigen::Vector3d area_max = {1.0, 1.0, 1.0};
            Eigen::Vector3d border_color = {0.0, 0.0, 0.0};

            ERL_REFLECT_SCHEMA(
                Setting,
                ERL_REFLECT_MEMBER(Setting, scaling),
                ERL_REFLECT_MEMBER(Setting, area_min),
                ERL_REFLECT_MEMBER(Setting, area_max),
                ERL_REFLECT_MEMBER(Setting, border_color));
        };

    private:
        std::shared_ptr<Setting> m_setting_ = {};

    public:
        explicit AbstractOctreeDrawer(std::shared_ptr<Setting> setting);

        virtual ~AbstractOctreeDrawer() = default;

        /**
         * Create blank geometries for drawing.
         * @return
         */
        static std::vector<std::shared_ptr<open3d::geometry::Geometry>>
        GetBlankGeometries();

        void
        DrawTree(const std::string &filename) const;

        virtual void
        DrawTree(std::vector<std::shared_ptr<open3d::geometry::Geometry>> &geometries) const = 0;

        void
        DrawLeaves(const std::string &filename) const;

        virtual void
        DrawLeaves(std::vector<std::shared_ptr<open3d::geometry::Geometry>> &geometries) const = 0;
    };
}  // namespace erl::geometry
