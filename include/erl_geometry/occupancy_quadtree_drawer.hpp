#pragma once

#include "abstract_quadtree_drawer.hpp"

#include "erl_common/yaml.hpp"

#include <functional>

namespace erl::geometry {
    template<typename Dtype>
    struct OccupancyQuadtreeDrawerSetting
        : common::Yamlable<OccupancyQuadtreeDrawerSetting<Dtype>, AbstractQuadtreeDrawer::Setting> {

        cv::Scalar occupied_color = {0, 0, 0, 255};    // black
        cv::Scalar free_color = {255, 255, 255, 255};  // white

        ERL_REFLECT_SCHEMA(
            OccupancyQuadtreeDrawerSetting,
            ERL_REFLECT_MEMBER(OccupancyQuadtreeDrawerSetting, occupied_color),
            ERL_REFLECT_MEMBER(OccupancyQuadtreeDrawerSetting, free_color));
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
            void(const OccupancyQuadtreeDrawer *, cv::Mat &, typename Tree::TreeIterator &)>;
        using DrawLeafCallback = std::function<
            void(const OccupancyQuadtreeDrawer *, cv::Mat &, typename Tree::LeafIterator &)>;

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
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

        void
        SetDrawTreeCallback(DrawTreeCallback draw_tree);

        void
        SetDrawLeafCallback(DrawLeafCallback draw_leaf);

        using AbstractQuadtreeDrawer::DrawLeaves;
        using AbstractQuadtreeDrawer::DrawTree;

        void
        DrawTree(cv::Mat &mat) const override;

        void
        DrawLeaves(cv::Mat &mat) const override;
    };
}  // namespace erl::geometry

#include "occupancy_quadtree_drawer.tpp"
