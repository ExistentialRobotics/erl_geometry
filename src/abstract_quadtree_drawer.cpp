#include "erl_geometry/abstract_quadtree_drawer.hpp"

namespace erl::geometry {

    AbstractQuadtreeDrawer::AbstractQuadtreeDrawer(std::shared_ptr<Setting> setting)
        : m_setting_(std::move(setting)) {
        ERL_ASSERTM(m_setting_, "setting is nullptr.");
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
    ERL_YAML_LOAD_ATTR(node, setting, padding);
    ERL_YAML_LOAD_ATTR(node, setting, bg_color);
    ERL_YAML_LOAD_ATTR(node, setting, fg_color);
    ERL_YAML_LOAD_ATTR(node, setting, border_color);
    ERL_YAML_LOAD_ATTR(node, setting, border_thickness);
    return true;
}
