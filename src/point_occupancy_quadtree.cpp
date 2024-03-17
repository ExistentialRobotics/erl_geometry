#include "erl_geometry/point_occupancy_quadtree.hpp"

namespace erl::geometry {
    PointOccupancyQuadtreeNode::PointOccupancyQuadtreeNode(const PointOccupancyQuadtreeNode &other)  // NOLINT(*-copy-constructor-init, *-no-recursion)
        : OccupancyQuadtreeNode(other.m_value_),                                                 // should not call the copy constructor of OccupancyQuadtreeNode
          m_points_(other.m_points_) {
        CopyChildren<PointOccupancyQuadtreeNode>(other);
    }

    std::istream &
    PointOccupancyQuadtreeNode::ReadData(std::istream &s) {
        s.read(reinterpret_cast<char *>(&m_value_), sizeof(m_value_));
        std::size_t num_points;
        s.read(reinterpret_cast<char *>(&num_points), sizeof(std::size_t));
        m_points_.resize(num_points);
        for (auto &point : m_points_) {
            s.read(reinterpret_cast<char *>(&point[0]), sizeof(double));
            s.read(reinterpret_cast<char *>(&point[1]), sizeof(double));
        }
        return s;
    }

    std::ostream &
    PointOccupancyQuadtreeNode::WriteData(std::ostream &s) const {
        s.write(reinterpret_cast<const char *>(&m_value_), sizeof(m_value_));
        std::size_t num_points = m_points_.size();
        s.write(reinterpret_cast<const char *>(&num_points), sizeof(std::size_t));
        for (const auto &point: m_points_) {
            s.write(reinterpret_cast<const char *>(&point[0]), sizeof(double));
            s.write(reinterpret_cast<const char *>(&point[1]), sizeof(double));
        }
        return s;
    }

    PointOccupancyQuadtree::PointOccupancyQuadtree(double resolution)
        : OccupancyQuadtreeBase<PointOccupancyQuadtreeNode>(resolution) {
        s_init_.EnsureLinking();
    }

    PointOccupancyQuadtree::PointOccupancyQuadtree(const std::shared_ptr<Setting> &setting)
        : OccupancyQuadtreeBase<PointOccupancyQuadtreeNode>(setting) {
        m_max_num_points_per_node_ = setting->max_num_points_per_node;
    }

    PointOccupancyQuadtree::PointOccupancyQuadtree(const std::string &filename)
        : OccupancyQuadtreeBase<PointOccupancyQuadtreeNode>(0.1) {  // resolution will be set by readBinary
        ERL_ASSERTM(this->ReadBinary(filename), "Failed to read %s from file: %s", GetTreeType().c_str(), filename.c_str());
    }

    std::shared_ptr<PointOccupancyQuadtree::Setting>
    PointOccupancyQuadtree::GetSetting() const {
            auto setting = std::make_shared<Setting>();
            setting->log_odd_min = this->GetLogOddMin();
            setting->log_odd_max = this->GetLogOddMax();
            setting->probability_hit = this->GetProbabilityHit();
            setting->probability_miss = this->GetProbabilityMiss();
            setting->probability_occupied_threshold = this->GetOccupancyThreshold();
            setting->resolution = this->GetResolution();
            setting->use_change_detection = this->m_use_change_detection_;
            setting->use_aabb_limit = this->m_use_aabb_limit_;
            setting->aabb = this->m_aabb_;
            setting->max_num_points_per_node = m_max_num_points_per_node_;
            return setting;
        }

    bool
    PointOccupancyQuadtree::IsNodeCollapsible(const erl::geometry::PointOccupancyQuadtreeNode *node) const {
        // all children must exist
        if (node->GetNumChildren() != 4) { return false; }

        auto first_child = this->GetNodeChild(node, 0);
        if (first_child->HasAnyChild()) { return false; }
        if (this->IsNodeOccupied(first_child)) { return false; }  // occupied node will store points, don't collapse

        for (uint32_t i = 1; i < 4; ++i) {
            auto child = this->GetNodeChild(node, i);
            // child should be a leaf node
            if (child->HasAnyChild() || *child != *first_child) { return false; }
        }
        return true;
    }

    void
    PointOccupancyQuadtree::InsertPointCloud(
        const Eigen::Ref<const Eigen::Matrix2Xd> &points,
        const Eigen::Ref<const Eigen::Vector2d> &sensor_origin,
        double max_range,
        bool parallel,
        bool lazy_eval,
        bool discretize) {
        Super::InsertPointCloud(points, sensor_origin, max_range, parallel, lazy_eval, discretize);
        // save points to nodes
        QuadtreeKeyVectorMap &end_point_mapping = discretize ? m_discrete_end_point_mapping_ : m_end_point_mapping_;
        for (auto &[key, point_indices]: end_point_mapping) {
            uint32_t depth = 0;  // find the deepest node
            PointOccupancyQuadtreeNode *node = this->Search(key, depth);
            if (node == nullptr) { continue; }
            if (!this->IsNodeOccupied(node)) { continue; }
            ERL_DEBUG_ASSERT(depth == this->mk_TreeDepth_, "depth is wrong for an occupied node.");
            auto it = point_indices.begin();
            while (node->GetNumPoints() < m_max_num_points_per_node_ && it != point_indices.end()) { node->AddPoint(points.col(*(it++))); }
        }
    }

    void
    PointOccupancyQuadtree::InsertPointCloudRays(
        const Eigen::Ref<const Eigen::Matrix2Xd> &points,
        const Eigen::Ref<const Eigen::Vector2d> &sensor_origin,
        double max_range,
        bool parallel,
        bool lazy_eval) {
        Super::InsertPointCloudRays(points, sensor_origin, max_range, parallel, lazy_eval);
        // save points to nodes
        long num_points = points.cols();
        for (long i = 0; i < num_points; ++i) {
            const auto &kP = points.col(i);
            double range = (kP - sensor_origin).norm();
            if (max_range <= 0. || (max_range > 0. && range <= max_range)) {
                QuadtreeKey key;
                if (!this->CoordToKeyChecked(kP[0], kP[1], key)) { continue; }
                uint32_t depth = 0;  // find the deepest node
                PointOccupancyQuadtreeNode *node = this->Search(key, depth);
                if (node == nullptr) { continue; }
                if (!this->IsNodeOccupied(node)) { continue; }
                ERL_DEBUG_ASSERT(depth == this->mk_TreeDepth_, "depth is wrong for an occupied node.");
                if (node->GetNumPoints() < m_max_num_points_per_node_) { node->AddPoint(kP); }
            }
        }
    }

    bool
    PointOccupancyQuadtree::InsertRay(double sx, double sy, double ex, double ey, double max_range, bool lazy_eval) {
        double dx = ex - sx;
        double dy = ey - sy;
        double range = std::sqrt(dx * dx + dy * dy);
        auto &key_ray = this->m_key_rays_[0];
        if (max_range > 0 && range > max_range) {  // cut ray at max_range
            double r = max_range / range;
            ex = sx + dx * r;
            ey = sy + dy * r;
            if (!this->ComputeRayKeys(sx, sy, ex, ey, key_ray)) { return false; }
            for (auto &key: key_ray) { UpdateNode(key, false, lazy_eval); }
            return true;
        }

        if (!this->ComputeRayKeys(sx, sy, ex, ey, key_ray)) { return false; }
        for (auto &key: key_ray) { UpdateNode(key, false, lazy_eval); }
        QuadtreeKey key;
        if (this->CoordToKeyChecked(ex, ey, key)) {
            UpdateNode(key, true, lazy_eval);
            uint32_t depth = 0;  // find the deepest node
            PointOccupancyQuadtreeNode *node = this->Search(key, depth);
            if (node != nullptr && this->IsNodeOccupied(node)) {
                ERL_DEBUG_ASSERT(depth == this->mk_TreeDepth_, "depth is wrong for an occupied node.");
                if (node->GetNumPoints() < m_max_num_points_per_node_) { node->AddPoint(Eigen::Vector2d(ex, ey)); }
            }
        }
        return true;
    }
}  // namespace erl::geometry
