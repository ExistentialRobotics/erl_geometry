#include "erl_geometry/occupancy_octree_drawer.hpp"

namespace erl::geometry {

    YAML::Node
    OccupancyOctreeDrawerSetting::YamlConvertImpl::encode(
        const OccupancyOctreeDrawerSetting &setting) {
        YAML::Node node = YAML::convert<AbstractOctreeDrawer::Setting>::encode(setting);
        node["occupied_only"] = setting.occupied_only;
        node["occupied_color"] = setting.occupied_color;
        node["draw_node_boxes"] = setting.draw_node_boxes;
        node["draw_node_borders"] = setting.draw_node_borders;
        return node;
    }

    bool
    OccupancyOctreeDrawerSetting::YamlConvertImpl::decode(
        const YAML::Node &node,
        OccupancyOctreeDrawerSetting &setting) {
        if (!node.IsMap()) { return false; }
        if (!YAML::convert<AbstractOctreeDrawer::Setting>::decode(node, setting)) { return false; }
        setting.occupied_only = node["occupied_only"].as<bool>();
        setting.occupied_color = node["occupied_color"].as<Eigen::Vector3d>();
        setting.draw_node_boxes = node["draw_node_boxes"].as<bool>();
        setting.draw_node_borders = node["draw_node_borders"].as<bool>();
        return true;
    }

}  // namespace erl::geometry
