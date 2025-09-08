#include "erl_geometry/abstract_quadtree_drawer.hpp"

namespace erl::geometry {

    AbstractQuadtreeDrawer::AbstractQuadtreeDrawer(std::shared_ptr<Setting> setting)
        : m_setting_(std::move(setting)) {
        ERL_ASSERTM(m_setting_, "setting is nullptr.");
        m_grid_map_info_ = std::make_shared<common::GridMapInfo2Df>(
            m_setting_->area_min,
            m_setting_->area_max,
            Eigen::Vector2f(m_setting_->resolution, m_setting_->resolution),
            Eigen::Vector2i(m_setting_->padding, m_setting_->padding));
    }

    std::shared_ptr<const common::GridMapInfo2Df>
    AbstractQuadtreeDrawer::GetGridMapInfo() const {
        return m_grid_map_info_;
    }

    Eigen::Matrix2Xi
    AbstractQuadtreeDrawer::GetPixelCoordsForPositions(
        const Eigen::Matrix2Xf &positions,
        const bool scaled_position) const {
        if (scaled_position) {
            return m_grid_map_info_->MeterToPixelForPoints(positions.array() / m_setting_->scaling);
        }
        return m_grid_map_info_->MeterToPixelForPoints(positions);
    }

    Eigen::Matrix2Xi
    AbstractQuadtreeDrawer::GetPixelCoordsForVectors(const Eigen::Matrix2Xf &vectors) const {
        return m_grid_map_info_->MeterToPixelForVectors(vectors);
    }

    void
    AbstractQuadtreeDrawer::DrawTree(const std::string &filename) const {
        cv::Mat img;
        DrawTree(img);
        cv::imwrite(filename, img);
    }

    void
    AbstractQuadtreeDrawer::DrawLeaves(const std::string &filename) const {
        cv::Mat img;
        DrawLeaves(img);
        cv::imwrite(filename, img);
    }
}  // namespace erl::geometry

YAML::Node
YAML::convert<erl::geometry::AbstractQuadtreeDrawer::Setting>::encode(
    const erl::geometry::AbstractQuadtreeDrawer::Setting &setting) {
    Node node;
    ERL_YAML_SAVE_ATTR(node, setting, area_min);
    ERL_YAML_SAVE_ATTR(node, setting, area_max);
    ERL_YAML_SAVE_ATTR(node, setting, resolution);
    ERL_YAML_SAVE_ATTR(node, setting, scaling);
    ERL_YAML_SAVE_ATTR(node, setting, padding);
    ERL_YAML_SAVE_ATTR(node, setting, bg_color);
    ERL_YAML_SAVE_ATTR(node, setting, fg_color);
    ERL_YAML_SAVE_ATTR(node, setting, border_color);
    ERL_YAML_SAVE_ATTR(node, setting, border_thickness);
    return node;
}

bool
YAML::convert<erl::geometry::AbstractQuadtreeDrawer::Setting>::decode(
    const Node &node,
    erl::geometry::AbstractQuadtreeDrawer::Setting &setting) {
    if (!node.IsMap()) { return false; }
    ERL_YAML_LOAD_ATTR(node, setting, area_min);
    ERL_YAML_LOAD_ATTR(node, setting, area_max);
    ERL_YAML_LOAD_ATTR(node, setting, resolution);
    ERL_YAML_LOAD_ATTR(node, setting, scaling);
    ERL_YAML_LOAD_ATTR(node, setting, padding);
    ERL_YAML_LOAD_ATTR(node, setting, bg_color);
    ERL_YAML_LOAD_ATTR(node, setting, fg_color);
    ERL_YAML_LOAD_ATTR(node, setting, border_color);
    ERL_YAML_LOAD_ATTR(node, setting, border_thickness);
    return true;
}
