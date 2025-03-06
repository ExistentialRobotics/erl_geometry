#pragma once

namespace erl::geometry {

    template<typename OccupancyQuadtreeType>
    YAML::Node
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::Setting::YamlConvertImpl::encode(const Setting &setting) {
        YAML::Node node = YAML::convert<AbstractQuadtreeDrawer::Setting>::encode(setting);
        node["area_min"] = setting.area_min;
        node["area_max"] = setting.area_max;
        node["resolution"] = setting.resolution;
        node["occupied_color"] = setting.occupied_color;
        node["free_color"] = setting.free_color;
        return node;
    }

    template<typename OccupancyQuadtreeType>
    bool
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::Setting::YamlConvertImpl::decode(const YAML::Node &node, Setting &setting) {
        if (!node.IsMap()) { return false; }
        if (!YAML::convert<AbstractQuadtreeDrawer::Setting>::decode(node, setting)) { return false; }
        setting.area_min = node["area_min"].as<Eigen::Vector2<Dtype>>();
        setting.area_max = node["area_max"].as<Eigen::Vector2<Dtype>>();
        setting.resolution = node["resolution"].as<double>();
        setting.occupied_color = node["occupied_color"].as<cv::Scalar>();
        setting.free_color = node["free_color"].as<cv::Scalar>();
        return true;
    }

    template<typename OccupancyQuadtreeType>
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::OccupancyQuadtreeDrawer(
        std::shared_ptr<Setting> setting,
        std::shared_ptr<const OccupancyQuadtreeType> quadtree)
        : AbstractQuadtreeDrawer(std::static_pointer_cast<AbstractQuadtreeDrawer::Setting>(setting)),
          m_setting_(std::move(setting)),
          m_quadtree_(std::move(quadtree)) {
        ERL_ASSERTM(m_setting_, "setting is nullptr.");
    }

    template<typename OccupancyQuadtreeType>
    std::shared_ptr<typename OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::Setting>
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::GetSetting() const {
        return m_setting_;
    }

    template<typename OccupancyQuadtreeType>
    void
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::SetDrawTreeCallback(
        std::function<void(const OccupancyQuadtreeDrawer *, cv::Mat &, typename OccupancyQuadtreeType::TreeIterator &)> draw_tree) {
        m_draw_tree_ = std::move(draw_tree);
    }

    template<typename OccupancyQuadtreeType>
    void
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::SetDrawLeafCallback(
        std::function<void(const OccupancyQuadtreeDrawer *, cv::Mat &, typename OccupancyQuadtreeType::LeafIterator &)> draw_leaf) {
        m_draw_leaf_ = std::move(draw_leaf);
    }

    template<typename OccupancyQuadtreeType>
    void
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::DrawTree(cv::Mat &mat) const {
        const auto grid_map_info = GetGridMapInfo();
        if (!mat.total()) { mat = cv::Mat(std::vector<int>{grid_map_info->Height(), grid_map_info->Width()}, CV_8UC4, m_setting_->bg_color); }
        if (m_quadtree_ == nullptr) { return; }
        std::shared_ptr<const OccupancyQuadtreeType> quadtree = std::dynamic_pointer_cast<const OccupancyQuadtreeType>(m_quadtree_);
        if (quadtree == nullptr) {
            ERL_WARN("quadtree is not an occupancy quadtree.");
            return;
        }

        const bool draw_border = (m_setting_->border_thickness > 0) && (m_setting_->border_color != m_setting_->occupied_color);

        auto it = quadtree->BeginTree();
        auto end = quadtree->EndTree();
        for (; it != end; ++it) {
            const double node_size = it.GetNodeSize();
            const double half_size = node_size / 2.0;
            const double x = it.GetX();
            const double y = it.GetY();

            Eigen::Vector2i aabb_min = grid_map_info->MeterToPixelForPoints(Eigen::Vector2<Dtype>(x - half_size, y - half_size));
            Eigen::Vector2i aabb_max = grid_map_info->MeterToPixelForPoints(Eigen::Vector2<Dtype>(x + half_size, y + half_size));

            if (!it->HasAnyChild()) {  // leaf node
                cv::rectangle(
                    mat,
                    cv::Point(aabb_min.x(), aabb_min.y()),
                    cv::Point(aabb_max.x(), aabb_max.y()),
                    quadtree->IsNodeOccupied(*it) ? m_setting_->occupied_color : m_setting_->free_color,
                    -1);
            }

            if (draw_border) {
                cv::rectangle(
                    mat,
                    cv::Point(aabb_min.x(), aabb_min.y()),
                    cv::Point(aabb_max.x(), aabb_max.y()),
                    m_setting_->border_color,
                    m_setting_->border_thickness);
            }

            if (m_draw_tree_) { m_draw_tree_(this, mat, it); }
        }
    }

    template<typename OccupancyQuadtreeType>
    void
    OccupancyQuadtreeDrawer<OccupancyQuadtreeType>::DrawLeaves(cv::Mat &mat) const {
        const auto grid_map_info = GetGridMapInfo();
        if (!mat.total()) { mat = cv::Mat(std::vector<int>{grid_map_info->Height(), grid_map_info->Width()}, CV_8UC4, m_setting_->bg_color); }
        if (m_quadtree_ == nullptr) { return; }
        std::shared_ptr<const OccupancyQuadtreeType> quadtree = std::dynamic_pointer_cast<const OccupancyQuadtreeType>(m_quadtree_);
        if (quadtree == nullptr) {
            ERL_WARN("quadtree is not an occupancy quadtree.");
            return;
        }

        const bool draw_border = (m_setting_->border_thickness > 0) && (m_setting_->border_color != m_setting_->occupied_color);

        auto it = quadtree->BeginLeaf();
        auto end = quadtree->EndLeaf();
        for (; it != end; ++it) {

            ERL_DEBUG_ASSERT(!it->HasAnyChild(), "the iterator visits an inner node!");

            const double node_size = it.GetNodeSize();
            const double half_size = node_size / 2.0;
            const double x = it.GetX();
            const double y = it.GetY();

            Eigen::Vector2i aabb_min = grid_map_info->MeterToPixelForPoints(Eigen::Vector2<Dtype>(x - half_size, y - half_size));
            Eigen::Vector2i aabb_max = grid_map_info->MeterToPixelForPoints(Eigen::Vector2<Dtype>(x + half_size, y + half_size));
            cv::rectangle(
                mat,
                cv::Point(aabb_min.x(), aabb_min.y()),
                cv::Point(aabb_max.x(), aabb_max.y()),
                quadtree->IsNodeOccupied(*it) ? m_setting_->occupied_color : m_setting_->free_color,
                -1);

            if (draw_border) {
                cv::rectangle(
                    mat,
                    cv::Point(aabb_min.x(), aabb_min.y()),
                    cv::Point(aabb_max.x(), aabb_max.y()),
                    m_setting_->border_color,
                    m_setting_->border_thickness);
            }

            if (m_draw_leaf_) { m_draw_leaf_(this, mat, it); }
        }
    }
}  // namespace erl::geometry
