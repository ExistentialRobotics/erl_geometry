#pragma once

#include "occupancy_quadtree_drawer.hpp"

namespace erl::geometry {

    template<typename Dtype>
    YAML::Node
    OccupancyQuadtreeDrawerSetting<Dtype>::YamlConvertImpl::encode(
        const OccupancyQuadtreeDrawerSetting &setting) {
        YAML::Node node = YAML::convert<AbstractQuadtreeDrawer::Setting>::encode(setting);
        ERL_YAML_SAVE_ATTR(node, setting, occupied_color);
        ERL_YAML_SAVE_ATTR(node, setting, free_color);
        return node;
    }

    template<typename Dtype>
    bool
    OccupancyQuadtreeDrawerSetting<Dtype>::YamlConvertImpl::decode(
        const YAML::Node &node,
        OccupancyQuadtreeDrawerSetting &setting) {
        if (!node.IsMap()) { return false; }
        if (!YAML::convert<AbstractQuadtreeDrawer::Setting>::decode(node, setting)) {
            return false;
        }
        ERL_YAML_LOAD_ATTR(node, setting, occupied_color);
        ERL_YAML_LOAD_ATTR(node, setting, free_color);
        return true;
    }

    template<typename OccupancyQuadtreeType>
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::OccupancyQuadtreeDrawer(
        std::shared_ptr<Setting> setting,
        std::shared_ptr<const OccupancyQuadtreeType> quadtree)
        : AbstractQuadtreeDrawer(
              std::static_pointer_cast<AbstractQuadtreeDrawer::Setting>(setting)),
          m_setting_(std::move(setting)),
          m_quadtree_(std::move(quadtree)) {
        ERL_ASSERTM(m_setting_, "setting is nullptr.");
    }

    template<typename OccupancyQuadtreeType>
    std::shared_ptr<const typename OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::Setting>
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::GetSetting() const {
        return m_setting_;
    }

    template<typename OccupancyQuadtreeType>
    std::shared_ptr<const OccupancyQuadtreeType>
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::GetQuadtree() const {
        return m_quadtree_;
    }

    template<typename OccupancyQuadtreeType>
    void
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::SetQuadtree(
        std::shared_ptr<const OccupancyQuadtreeType> quadtree) {
        m_quadtree_ = std::move(quadtree);
    }

    template<typename OccupancyQuadtreeType>
    void
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::SetDrawTreeCallback(
        std::function<void(
            const OccupancyQuadtreeDrawer *,
            cv::Mat &,
            typename OccupancyQuadtreeType::TreeIterator &)> draw_tree) {
        m_draw_tree_ = std::move(draw_tree);
    }

    template<typename OccupancyQuadtreeType>
    void
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::SetDrawLeafCallback(
        std::function<void(
            const OccupancyQuadtreeDrawer *,
            cv::Mat &,
            typename OccupancyQuadtreeType::LeafIterator &)> draw_leaf) {
        m_draw_leaf_ = std::move(draw_leaf);
    }

    template<typename OccupancyQuadtreeType>
    void
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::DrawTree(cv::Mat &mat) const {
        if (!mat.total()) {
            mat = cv::Mat(
                std::vector<int>{m_grid_map_info_->Height(), m_grid_map_info_->Width()},
                CV_8UC4,
                m_setting_->bg_color);
        }
        if (m_quadtree_ == nullptr) { return; }

        const bool draw_border = (m_setting_->border_thickness > 0) &&
                                 (m_setting_->border_color != m_setting_->occupied_color);
        auto it = m_quadtree_->BeginTree();
        auto end = m_quadtree_->EndTree();
        Eigen::Matrix2f area;
        for (; it != end; ++it) {
            const auto node_size = static_cast<float>(it.GetNodeSize());
            const float half_size = node_size * 0.5f;
            const auto x = static_cast<float>(it.GetX());
            const auto y = static_cast<float>(it.GetY());

            area << x - half_size, x + half_size, y - half_size, y + half_size;
            Eigen::Matrix2i area_px = GetPixelCoordsForPositions(area, true);

            if (!it->HasAnyChild()) {  // leaf node
                cv::rectangle(
                    mat,
                    cv::Point(area_px(0, 0), area_px(1, 0)),  // min
                    cv::Point(area_px(0, 1), area_px(1, 1)),  // max
                    m_quadtree_->IsNodeOccupied(*it) ? m_setting_->occupied_color
                                                     : m_setting_->free_color,
                    -1);
            }

            if (draw_border) {
                cv::rectangle(
                    mat,
                    cv::Point(area_px(0, 0), area_px(1, 0)),  // min
                    cv::Point(area_px(0, 1), area_px(1, 1)),  // max
                    m_setting_->border_color,
                    m_setting_->border_thickness);
            }

            if (m_draw_tree_) { m_draw_tree_(this, mat, it); }
        }
    }

    template<typename OccupancyQuadtreeType>
    void
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::DrawLeaves(cv::Mat &mat) const {
        if (!mat.total()) {
            mat = cv::Mat(
                std::vector<int>{m_grid_map_info_->Height(), m_grid_map_info_->Width()},
                CV_8UC4,
                m_setting_->bg_color);
        }
        if (m_quadtree_ == nullptr) { return; }

        const bool draw_border = (m_setting_->border_thickness > 0) &&
                                 (m_setting_->border_color != m_setting_->occupied_color);
        auto it = m_quadtree_->BeginLeaf();
        auto end = m_quadtree_->EndLeaf();
        Eigen::Matrix2f area;
        for (; it != end; ++it) {

            ERL_DEBUG_ASSERT(!it->HasAnyChild(), "the iterator visits an inner node!");

            const auto node_size = static_cast<float>(it.GetNodeSize());
            const float half_size = node_size * 0.5f;
            const auto x = static_cast<float>(it.GetX());
            const auto y = static_cast<float>(it.GetY());

            area << x - half_size, x + half_size, y - half_size, y + half_size;
            Eigen::Matrix2i area_px = GetPixelCoordsForPositions(area, true);

            cv::rectangle(
                mat,
                cv::Point(area_px(0, 0), area_px(1, 0)),  // min
                cv::Point(area_px(0, 1), area_px(1, 1)),  // max
                m_quadtree_->IsNodeOccupied(*it) ? m_setting_->occupied_color
                                                 : m_setting_->free_color,
                -1);

            if (draw_border) {
                cv::rectangle(
                    mat,
                    cv::Point(area_px(0, 0), area_px(1, 0)),  // min
                    cv::Point(area_px(0, 1), area_px(1, 1)),  // max
                    m_setting_->border_color,
                    m_setting_->border_thickness);
            }

            if (m_draw_leaf_) { m_draw_leaf_(this, mat, it); }
        }
    }
}  // namespace erl::geometry
