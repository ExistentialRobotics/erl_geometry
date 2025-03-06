#pragma once

#include "abstract_quadtree_drawer.hpp"

#include "erl_common/yaml.hpp"

#include <functional>

namespace erl::geometry {

    template<typename OccupancyQuadtreeType>
    class OccupancyQuadtreeDrawer : public AbstractQuadtreeDrawer {
    public:
        using Dtype = typename OccupancyQuadtreeType::DataType;

        struct Setting : common::Yamlable<Setting, AbstractQuadtreeDrawer::Setting> {
            Eigen::Vector2<Dtype> area_min = {0.0, 0.0};
            Eigen::Vector2<Dtype> area_max = {1.0, 1.0};
            Dtype resolution = 0.1;
            cv::Scalar occupied_color = {0, 0, 0, 255};    // black
            cv::Scalar free_color = {255, 255, 255, 255};  // white

            struct YamlConvertImpl {
                static YAML::Node
                encode(const Setting &setting);

                static bool
                decode(const YAML::Node &node, Setting &setting);
            };
        };

        using DrawTreeCallback = std::function<void(const OccupancyQuadtreeDrawer *, cv::Mat &, typename OccupancyQuadtreeType::TreeIterator &)>;
        using DrawLeafCallback = std::function<void(const OccupancyQuadtreeDrawer *, cv::Mat &, typename OccupancyQuadtreeType::LeafIterator &)>;

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<const OccupancyQuadtreeType> m_quadtree_ = nullptr;
        DrawTreeCallback m_draw_tree_ = {};
        DrawLeafCallback m_draw_leaf_ = {};

    public:
        explicit OccupancyQuadtreeDrawer(std::shared_ptr<Setting> setting, std::shared_ptr<const OccupancyQuadtreeType> quadtree = nullptr);

        using AbstractQuadtreeDrawer::DrawLeaves;
        using AbstractQuadtreeDrawer::DrawTree;

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const;

        [[nodiscard]] std::shared_ptr<common::GridMapInfo2D<Dtype>>
        GetGridMapInfo() const {
            return std::make_shared<common::GridMapInfo2D<Dtype>>(
                m_setting_->area_min,
                m_setting_->area_max,
                Eigen::Vector2<Dtype>(m_setting_->resolution, m_setting_->resolution),
                Eigen::Vector2i(m_setting_->padding, m_setting_->padding));
        }

        void
        SetDrawTreeCallback(std::function<void(const OccupancyQuadtreeDrawer *, cv::Mat &, typename OccupancyQuadtreeType::TreeIterator &)> draw_tree);

        void
        SetDrawLeafCallback(std::function<void(const OccupancyQuadtreeDrawer *, cv::Mat &, typename OccupancyQuadtreeType::LeafIterator &)> draw_leaf);

        void
        DrawTree(cv::Mat &mat) const override;

        void
        DrawLeaves(cv::Mat &mat) const override;
    };

}  // namespace erl::geometry

#include "occupancy_quadtree_drawer.tpp"
