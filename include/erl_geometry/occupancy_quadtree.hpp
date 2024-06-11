#pragma once

#include "occupancy_quadtree_base.hpp"
#include "occupancy_quadtree_drawer.hpp"
#include "occupancy_quadtree_node.hpp"

namespace erl::geometry {

    class OccupancyQuadtree : public OccupancyQuadtreeBase<OccupancyQuadtreeNode, OccupancyQuadtreeBaseSetting> {
    public:
        using Setting = OccupancyQuadtreeBaseSetting;
        using Drawer = OccupancyQuadtreeDrawer<OccupancyQuadtree>;

        explicit OccupancyQuadtree(const std::shared_ptr<OccupancyQuadtreeBaseSetting> &setting)
            : OccupancyQuadtreeBase(setting) {}

        OccupancyQuadtree()
            : OccupancyQuadtree(std::make_shared<Setting>()) {}

        explicit OccupancyQuadtree(const std::string &filename)
            : OccupancyQuadtree() {  // resolution will be set by LoadData
            ERL_ASSERTM(this->LoadData(filename), "Failed to read OccupancyQuadtree from file: {}", filename);
        }

        /**
         *
         * @param map_info instance of GridMapInfo2D to provide min, max and resolution of the map
         * @param image_map 1-channel cv::Mat image of the map, where rows is x and cols is y
         * @param occupied_threshold a pixel is considered occupied if its value is greater than this threshold
         * @param padding padding around the map with obstacles
         */
        OccupancyQuadtree(
            const std::shared_ptr<common::GridMapInfo2D> &map_info,
            const cv::Mat &image_map,
            const double occupied_threshold,
            const int padding = 0)
            : OccupancyQuadtreeBase<OccupancyQuadtreeNode, OccupancyQuadtreeBaseSetting>(map_info, image_map, occupied_threshold, padding) {}

        OccupancyQuadtree(const OccupancyQuadtree &) = default;
        OccupancyQuadtree &
        operator=(const OccupancyQuadtree &) = default;
        OccupancyQuadtree(OccupancyQuadtree &&) = default;
        OccupancyQuadtree &
        operator=(OccupancyQuadtree &&) = default;

    protected:
        [[nodiscard]] std::shared_ptr<AbstractQuadtree>
        Create() const override {
            return std::make_shared<OccupancyQuadtree>();
        }
    };

    ERL_REGISTER_QUADTREE(OccupancyQuadtree);
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::OccupancyQuadtree::Drawer::Setting>
    : public ConvertOccupancyQuadtreeDrawerSetting<erl::geometry::OccupancyQuadtree::Drawer::Setting> {};  // namespace YAML
