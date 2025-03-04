#pragma once

#include "occupancy_quadtree_base.hpp"
// #include "occupancy_quadtree_drawer.hpp"
#include "occupancy_quadtree_node.hpp"

namespace erl::geometry {

    template <typename Dtype>
    class OccupancyQuadtree : public OccupancyQuadtreeBase<OccupancyQuadtreeNode, OccupancyQuadtreeBaseSetting, Dtype> {
    public:
        using Setting = OccupancyQuadtreeBaseSetting;
        using Super = OccupancyQuadtreeBase<OccupancyQuadtreeNode, OccupancyQuadtreeBaseSetting, Dtype>;
        // using Drawer = OccupancyQuadtreeDrawer<OccupancyQuadtree>;

        explicit OccupancyQuadtree(const std::shared_ptr<OccupancyQuadtreeBaseSetting> &setting)
            : Super(setting) {}

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
            const std::shared_ptr<common::GridMapInfo2D<Dtype>> &map_info,
            const cv::Mat &image_map,
            const double occupied_threshold,
            const int padding = 0)
            : Super(std::make_shared<Setting>(), map_info, image_map, occupied_threshold, padding) {}

        OccupancyQuadtree(const OccupancyQuadtree &) = default;
        OccupancyQuadtree &
        operator=(const OccupancyQuadtree &) = default;
        OccupancyQuadtree(OccupancyQuadtree &&) = default;
        OccupancyQuadtree &
        operator=(OccupancyQuadtree &&) = default;

    protected:
        [[nodiscard]] std::shared_ptr<AbstractQuadtree<Dtype>>
        Create(const std::shared_ptr<NdTreeSetting> &setting) const override {
            auto tree_setting = std::dynamic_pointer_cast<Setting>(setting);
            if (tree_setting == nullptr) {
                ERL_DEBUG_ASSERT(setting == nullptr, "setting is not the type for OccupancyQuadtree.");
                tree_setting = std::make_shared<Setting>();
            }
            return std::make_shared<OccupancyQuadtree>(std::move(tree_setting));
        }
    };

    using OccupancyQuadtree_d = OccupancyQuadtree<double>;
    using OccupancyQuadtree_f = OccupancyQuadtree<float>;

    ERL_REGISTER_QUADTREE(OccupancyQuadtree_d);
    ERL_REGISTER_QUADTREE(OccupancyQuadtree_f);
}  // namespace erl::geometry

// template<>
// struct YAML::convert<erl::geometry::OccupancyQuadtree::Drawer::Setting>
//     : public ConvertOccupancyQuadtreeDrawerSetting<erl::geometry::OccupancyQuadtree::Drawer::Setting> {};  // namespace YAML
