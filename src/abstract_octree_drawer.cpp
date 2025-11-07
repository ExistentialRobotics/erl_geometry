#include "erl_geometry/abstract_octree_drawer.hpp"

#include <open3d/geometry/LineSet.h>

#include <utility>

namespace erl::geometry {
    AbstractOctreeDrawer::AbstractOctreeDrawer(std::shared_ptr<Setting> setting)
        : m_setting_(std::move(setting)) {
        ERL_ASSERTM(m_setting_, "setting is nullptr.");
    }

    std::vector<std::shared_ptr<open3d::geometry::Geometry>>
    AbstractOctreeDrawer::GetBlankGeometries() {
        auto boxes = std::make_shared<open3d::geometry::VoxelGrid>();
        auto node_border = std::make_shared<open3d::geometry::LineSet>();
        return {boxes, node_border};
    }

    void
    AbstractOctreeDrawer::DrawTree(const std::string &filename) const {
        std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries = GetBlankGeometries();
        DrawTree(geometries);

        const auto visualizer_setting = std::make_shared<Open3dVisualizerWrapper::Setting>();
        visualizer_setting->window_name = "Press Ctrl+S to save the view as an image";
        visualizer_setting->screenshot_filename = filename;
        Open3dVisualizerWrapper visualizer(visualizer_setting);
        visualizer.AddGeometries(geometries);
        visualizer.Show();
    }

    void
    AbstractOctreeDrawer::DrawLeaves(const std::string &filename) const {
        std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries = GetBlankGeometries();
        DrawLeaves(geometries);

        const auto visualizer_setting = std::make_shared<Open3dVisualizerWrapper::Setting>();
        visualizer_setting->window_name = "Press Ctrl+S to save the view as an image";
        visualizer_setting->screenshot_filename = filename;
        Open3dVisualizerWrapper visualizer(visualizer_setting);
        visualizer.AddGeometries(geometries);
        visualizer.Show();
    }
}  // namespace erl::geometry
