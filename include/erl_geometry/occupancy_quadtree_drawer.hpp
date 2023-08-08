#pragma once

#include <functional>
#include "abstract_quadtree_drawer.hpp"

namespace erl::geometry {

    template<typename OccupancyQuadtreeType>
    class OccupancyQuadtreeDrawer : public AbstractQuadtreeDrawer {
    public:
        struct Setting : public AbstractQuadtreeDrawer::Setting {
            cv::Scalar occupied_color = {0, 0, 0};    // black
            cv::Scalar free_color = {255, 255, 255};  // white
        };

    private:
        std::shared_ptr<Setting> m_setting_ = {};
        std::function<void(const OccupancyQuadtreeDrawer *, cv::Mat &, typename OccupancyQuadtreeType::TreeIterator &)> m_draw_tree_ = {};
        std::function<void(const OccupancyQuadtreeDrawer *, cv::Mat &, typename OccupancyQuadtreeType::LeafIterator &)> m_draw_leaf_ = {};

    public:
        explicit OccupancyQuadtreeDrawer(std::shared_ptr<Setting> setting, std::shared_ptr<const OccupancyQuadtreeType> quadtree = nullptr)
            : AbstractQuadtreeDrawer(std::static_pointer_cast<AbstractQuadtreeDrawer::Setting>(setting), std::move(quadtree)),
              m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_, "setting is nullptr.");
        }

        using AbstractQuadtreeDrawer::DrawLeaves;
        using AbstractQuadtreeDrawer::DrawTree;

        void
        SetDrawTreeCallback(std::function<void(const OccupancyQuadtreeDrawer *, cv::Mat &, typename OccupancyQuadtreeType::TreeIterator &)> draw_tree) {
            m_draw_tree_ = std::move(draw_tree);
        }

        void
        SetDrawLeafCallback(std::function<void(const OccupancyQuadtreeDrawer *, cv::Mat &, typename OccupancyQuadtreeType::LeafIterator &)> draw_leaf) {
            m_draw_leaf_ = std::move(draw_leaf);
        }

        void
        DrawTree(cv::Mat &mat) const override {
            auto grid_map_info = GetGridMapInfo();
            if (!mat.total()) { mat = cv::Mat(std::vector<int>{grid_map_info->Height(), grid_map_info->Width()}, CV_8UC4, m_setting_->bg_color); }
            if (m_quadtree_ == nullptr) { return; }
            std::shared_ptr<const OccupancyQuadtreeType> quadtree = std::dynamic_pointer_cast<const OccupancyQuadtreeType>(m_quadtree_);
            if (quadtree == nullptr) {
                ERL_WARN("quadtree is not an occupancy quadtree.");
                return;
            }

            bool draw_border = (m_setting_->border_thickness > 0) && (m_setting_->border_color != m_setting_->occupied_color);

            auto it = quadtree->BeginTree();
            auto end = quadtree->EndTree();
            for (; it != end; ++it) {
                double node_size = it.GetNodeSize();
                double half_size = node_size / 2.0;
                double x = it.GetX();
                double y = it.GetY();

                Eigen::Vector2i aabb_min = grid_map_info->MeterToPixelForPoints(Eigen::Vector2d(x - half_size, y - half_size));
                Eigen::Vector2i aabb_max = grid_map_info->MeterToPixelForPoints(Eigen::Vector2d(x + half_size, y + half_size));

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

        void
        DrawLeaves(cv::Mat &mat) const override {
            auto grid_map_info = GetGridMapInfo();
            if (!mat.total()) { mat = cv::Mat(std::vector<int>{grid_map_info->Height(), grid_map_info->Width()}, CV_8UC4, m_setting_->bg_color); }
            if (m_quadtree_ == nullptr) { return; }
            std::shared_ptr<const OccupancyQuadtreeType> quadtree = std::dynamic_pointer_cast<const OccupancyQuadtreeType>(m_quadtree_);
            if (quadtree == nullptr) {
                ERL_WARN("quadtree is not an occupancy quadtree.");
                return;
            }

            bool draw_border = (m_setting_->border_thickness > 0) && (m_setting_->border_color != m_setting_->occupied_color);

            auto it = quadtree->BeginLeaf();
            auto end = quadtree->EndLeaf();
            for (; it != end; ++it) {

                ERL_DEBUG_ASSERT(!it->HasAnyChild(), "the iterator visits an inner node!");

                double node_size = it.GetNodeSize();
                double half_size = node_size / 2.0;
                double x = it.GetX();
                double y = it.GetY();

                Eigen::Vector2i aabb_min = grid_map_info->MeterToPixelForPoints(Eigen::Vector2d(x - half_size, y - half_size));
                Eigen::Vector2i aabb_max = grid_map_info->MeterToPixelForPoints(Eigen::Vector2d(x + half_size, y + half_size));
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
    };
}  // namespace erl::geometry

namespace YAML {
    template<typename Node>
    struct convert {
        inline static Node
        encode(const typename erl::geometry::OccupancyQuadtreeDrawer<Node>::Setting &rhs) {
            Node node = convert<erl::geometry::AbstractQuadtreeDrawer::Setting>::encode(rhs);
            node["occupied_color"] = rhs.occupied_color;
            node["free_color"] = rhs.free_color;
            return node;
        }

        inline static bool
        decode(const Node &node, typename erl::geometry::OccupancyQuadtreeDrawer<Node>::Setting &rhs) {
            if (!node.IsMap()) { return false; }
            if (!convert<erl::geometry::AbstractQuadtreeDrawer::Setting>::decode(node, rhs)) { return false; }
            rhs.occupied_color = node["occupied_color"].template as<cv::Scalar>();
            rhs.free_color = node["free_color"].template as<cv::Scalar>();
            return true;
        }
    };

    template<typename Node>
    inline Emitter &
    operator<<(Emitter &out, const typename erl::geometry::OccupancyQuadtreeDrawer<Node>::Setting &rhs) {
        out << static_cast<const erl::geometry::AbstractQuadtreeDrawer::Setting &>(rhs);
        out << BeginMap;
        out << Key << "occupied_color" << Value << rhs.occupied_color;
        out << Key << "free_color" << Value << rhs.free_color;
        out << EndMap;
        return out;
    }
}  // namespace YAML
