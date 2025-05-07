#pragma once

#include "abstract_quadtree_drawer.hpp"

#include "erl_common/yaml.hpp"

#include <functional>

namespace erl::geometry {
    template<typename Dtype>
    struct OccupancyQuadtreeDrawerSetting
        : common::Yamlable<OccupancyQuadtreeDrawerSetting<Dtype>, AbstractQuadtreeDrawer::Setting> {
        Eigen::Vector2<Dtype> area_min = {0.0, 0.0};
        Eigen::Vector2<Dtype> area_max = {1.0, 1.0};
        Dtype resolution = 0.1;
        Dtype scaling = 1.0;
        cv::Scalar occupied_color = {0, 0, 0, 255};    // black
        cv::Scalar free_color = {255, 255, 255, 255};  // white

        struct YamlConvertImpl {
            static YAML::Node
            encode(const OccupancyQuadtreeDrawerSetting& setting);

            static bool
            decode(const YAML::Node& node, OccupancyQuadtreeDrawerSetting& setting);
        };
    };

    using OccupancyQuadtreeDrawerSettingD = OccupancyQuadtreeDrawerSetting<double>;
    using OccupancyQuadtreeDrawerSettingF = OccupancyQuadtreeDrawerSetting<float>;

    template<typename OccupancyQuadtreeType>
    class OccupancyQuadtreeDrawer : public AbstractQuadtreeDrawer {
    public:
        using Tree = OccupancyQuadtreeType;
        using Dtype = typename Tree::DataType;
        using Setting = OccupancyQuadtreeDrawerSetting<Dtype>;
        using DrawTreeCallback = std::function<
            void(const OccupancyQuadtreeDrawer*, cv::Mat&, typename Tree::TreeIterator&)>;
        using DrawLeafCallback = std::function<
            void(const OccupancyQuadtreeDrawer*, cv::Mat&, typename Tree::LeafIterator&)>;

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<common::GridMapInfo2D<Dtype>> m_grid_map_info_ = nullptr;
        std::shared_ptr<const Tree> m_quadtree_ = nullptr;
        DrawTreeCallback m_draw_tree_ = {};
        DrawLeafCallback m_draw_leaf_ = {};

    public:
        explicit OccupancyQuadtreeDrawer(
            std::shared_ptr<Setting> setting,
            std::shared_ptr<const Tree> quadtree = nullptr);

        [[nodiscard]] std::shared_ptr<const Setting>
        GetSetting() const;

        [[nodiscard]] std::shared_ptr<const Tree>
        GetQuadtree() const;

        void
        SetQuadtree(std::shared_ptr<const Tree> quadtree);

        [[nodiscard]] std::shared_ptr<const common::GridMapInfo2D<Dtype>>
        GetGridMapInfo() const;

        /**
         * Compute the pixel coordinates for the given positions.
         * @param positions matrix of positions (2 x N)
         * @param scaled_position if true, the positions are already scaled by the scaling factor.
         * e.g., positions = positions_org * scaling.
         * @return matrix of pixel coordinates (2 x N)
         */
        [[nodiscard]] Eigen::Matrix2Xi
        GetPixelCoordsForPositions(
            const Eigen::Matrix2X<Dtype>& positions,
            const bool scaled_position) const {
            if (scaled_position) {
                return m_grid_map_info_->MeterToPixelForPoints(
                    positions.array() / m_setting_->scaling);
            }
            return m_grid_map_info_->MeterToPixelForPoints(positions);
        }

        [[nodiscard]] Eigen::Matrix2Xi
        GetPixelCoordsForVectors(const Eigen::Matrix2X<Dtype>& vectors) const {
            return m_grid_map_info_->MeterToPixelForVectors(vectors);
        }

        /**
         * Compute the meter coordinates for the given pixel coordinates.
         * @param pixel_coords matrix of pixel coordinates (2 x N).
         * @param scaled_position if true, scale the returned meter coordinates with the scaling
         * factor. i.e., meter_coords = meter_coords_org * scaling.
         * @return matrix of meter coordinates (2 x N)
         */
        [[nodiscard]] Eigen::Matrix2X<Dtype>
        GetMeterCoordsForPositions(const Eigen::Matrix2Xi& pixel_coords, const bool scaled_position)
            const {
            if (scaled_position) {
                return m_grid_map_info_->PixelToMeterForPoints(pixel_coords).array() *
                       m_setting_->scaling;
            }
            return m_grid_map_info_->PixelToMeterForPoints(pixel_coords);
        }

        [[nodiscard]] Eigen::Matrix2X<Dtype>
        GetMeterCoordsForVectors(const Eigen::Matrix2Xi& pixel_coords) const {
            return m_grid_map_info_->PixelToMeterForVectors(pixel_coords);
        }

        void
        SetDrawTreeCallback(
            std::function<
                void(const OccupancyQuadtreeDrawer*, cv::Mat&, typename Tree::TreeIterator&)>
                draw_tree);

        void
        SetDrawLeafCallback(
            std::function<
                void(const OccupancyQuadtreeDrawer*, cv::Mat&, typename Tree::LeafIterator&)>
                draw_leaf);

        using AbstractQuadtreeDrawer::DrawLeaves;
        using AbstractQuadtreeDrawer::DrawTree;

        void
        DrawTree(cv::Mat& mat) const override;

        void
        DrawLeaves(cv::Mat& mat) const override;
    };
}  // namespace erl::geometry

#include "occupancy_quadtree_drawer.tpp"

template<>
struct YAML::convert<erl::geometry::OccupancyQuadtreeDrawerSettingD>
    : erl::geometry::OccupancyQuadtreeDrawerSettingD::YamlConvertImpl {};

template<>
struct YAML::convert<erl::geometry::OccupancyQuadtreeDrawerSettingF>
    : erl::geometry::OccupancyQuadtreeDrawerSettingF::YamlConvertImpl {};
