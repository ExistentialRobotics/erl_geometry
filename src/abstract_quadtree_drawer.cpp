#include "erl_geometry/abstract_quadtree_drawer.hpp"

namespace erl::geometry {

    AbstractQuadtreeDrawer::AbstractQuadtreeDrawer(std::shared_ptr<Setting> setting)
        : m_setting_(std::move(setting)) {
        ERL_ASSERTM(m_setting_, "setting is nullptr.");
        m_grid_map_info_ = std::make_shared<common::GridMapInfo2Df>(
            m_setting_->area_min,
            m_setting_->area_max,
            Eigen::Vector2f(m_setting_->resolution, m_setting_->resolution),
            Eigen::Vector2i(m_setting_->padding, m_setting_->padding));
    }

    std::shared_ptr<const common::GridMapInfo2Df>
    AbstractQuadtreeDrawer::GetGridMapInfo() const {
        return m_grid_map_info_;
    }

    void
    AbstractQuadtreeDrawer::DrawTree(const std::string &filename) const {
        cv::Mat img;
        DrawTree(img);
        cv::imwrite(filename, img);
    }

    void
    AbstractQuadtreeDrawer::DrawLeaves(const std::string &filename) const {
        cv::Mat img;
        DrawLeaves(img);
        cv::imwrite(filename, img);
    }
}  // namespace erl::geometry
