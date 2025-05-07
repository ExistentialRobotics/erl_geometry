#pragma once

namespace erl::geometry {

    template<typename Dtype>
    YAML::Node
    OccupancyQuadtreeDrawerSetting<Dtype>::YamlConvertImpl::encode(
        const OccupancyQuadtreeDrawerSetting &setting) {
        YAML::Node node = YAML::convert<AbstractQuadtreeDrawer::Setting>::encode(setting);
        ERL_YAML_SAVE_ATTR(node, setting, area_min);
        ERL_YAML_SAVE_ATTR(node, setting, area_max);
        ERL_YAML_SAVE_ATTR(node, setting, resolution);
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
        ERL_YAML_LOAD_ATTR_TYPE(node, setting, area_min, Eigen::Vector2<Dtype>);
        ERL_YAML_LOAD_ATTR_TYPE(node, setting, area_max, Eigen::Vector2<Dtype>);
        ERL_YAML_LOAD_ATTR_TYPE(node, setting, resolution, Dtype);
        ERL_YAML_LOAD_ATTR_TYPE(node, setting, occupied_color, cv::Scalar);
        ERL_YAML_LOAD_ATTR_TYPE(node, setting, free_color, cv::Scalar);
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
        m_grid_map_info_ = std::make_shared<common::GridMapInfo2D<Dtype>>(
            m_setting_->area_min,
            m_setting_->area_max,
            Eigen::Vector2<Dtype>(m_setting_->resolution, m_setting_->resolution),
            Eigen::Vector2i(m_setting_->padding, m_setting_->padding));
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
    std::shared_ptr<
        const common::GridMapInfo2D<typename OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::Dtype>>
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::GetGridMapInfo() const {
        return m_grid_map_info_;
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
        std::shared_ptr<const OccupancyQuadtreeType> quadtree =
            std::dynamic_pointer_cast<const OccupancyQuadtreeType>(m_quadtree_);
        if (quadtree == nullptr) {
            ERL_WARN("quadtree is not an occupancy quadtree.");
            return;
        }

        const bool draw_border = (m_setting_->border_thickness > 0) &&
                                 (m_setting_->border_color != m_setting_->occupied_color);
        auto it = quadtree->BeginTree();
        auto end = quadtree->EndTree();
        Eigen::Matrix2<Dtype> area;
        for (; it != end; ++it) {
            const Dtype node_size = it.GetNodeSize();
            const Dtype half_size = node_size * 0.5f;
            const Dtype x = it.GetX();
            const Dtype y = it.GetY();

            area << x - half_size, x + half_size, y - half_size, y + half_size;
            Eigen::Matrix2i area_px = GetPixelCoordsForPositions(area, true);

            if (!it->HasAnyChild()) {  // leaf node
                cv::rectangle(
                    mat,
                    cv::Point(area_px(0, 0), area_px(1, 0)),  // min
                    cv::Point(area_px(0, 1), area_px(1, 1)),  // max
                    quadtree->IsNodeOccupied(*it) ? m_setting_->occupied_color
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
        std::shared_ptr<const OccupancyQuadtreeType> quadtree =
            std::dynamic_pointer_cast<const OccupancyQuadtreeType>(m_quadtree_);
        if (quadtree == nullptr) {
            ERL_WARN("quadtree is not an occupancy quadtree.");
            return;
        }

        const bool draw_border = (m_setting_->border_thickness > 0) &&
                                 (m_setting_->border_color != m_setting_->occupied_color);
        auto it = quadtree->BeginLeaf();
        auto end = quadtree->EndLeaf();
        Eigen::Matrix2<Dtype> area;
        for (; it != end; ++it) {

            ERL_DEBUG_ASSERT(!it->HasAnyChild(), "the iterator visits an inner node!");

            const Dtype node_size = it.GetNodeSize();
            const Dtype half_size = node_size * 0.5f;
            const Dtype x = it.GetX();
            const Dtype y = it.GetY();

            area << x - half_size, x + half_size, y - half_size, y + half_size;
            Eigen::Matrix2i area_px = GetPixelCoordsForPositions(area, true);

            cv::rectangle(
                mat,
                cv::Point(area_px(0, 0), area_px(1, 0)),  // min
                cv::Point(area_px(0, 1), area_px(1, 1)),  // max
                quadtree->IsNodeOccupied(*it) ? m_setting_->occupied_color : m_setting_->free_color,
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
