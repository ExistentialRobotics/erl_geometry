#include "erl_geometry/ray_occupancy_octree.hpp"

namespace erl::geometry {
    RayOccupancyOctreeNode::RayOccupancyOctreeNode(const erl::geometry::RayOccupancyOctreeNode &other)  // NOLINT(*-copy-constructor-init, *-no-recursion)
        : OccupancyOctreeNode(other.m_value_),  // should not call the copy constructor of OccupancyOctreeNode
          m_start_points_(other.m_start_points_),
          m_end_points_(other.m_end_points_),
          m_directions_inv_(other.m_directions_inv_) {
        CopyChildren<erl::geometry::RayOccupancyOctreeNode>(other);
    }

    std::istream &
    RayOccupancyOctreeNode::ReadData(std::istream &s) {
        s.read(reinterpret_cast<char *>(&m_value_), sizeof(m_value_));
        std::size_t num_rays;
        s.read(reinterpret_cast<char *>(&num_rays), sizeof(std::size_t));
        m_start_points_.resize(num_rays);
        m_end_points_.resize(num_rays);
        m_directions_inv_.resize(num_rays);
        for (std::size_t i = 0; i < num_rays; ++i) {
            Eigen::Vector3d &start_point = m_start_points_[i];
            Eigen::Vector3d &end_point = m_end_points_[i];
            Eigen::Vector3d &direction_inv = m_directions_inv_[i];
            s.read(reinterpret_cast<char *>(&start_point[0]), sizeof(double));
            s.read(reinterpret_cast<char *>(&start_point[1]), sizeof(double));
            s.read(reinterpret_cast<char *>(&start_point[2]), sizeof(double));
            s.read(reinterpret_cast<char *>(&end_point[0]), sizeof(double));
            s.read(reinterpret_cast<char *>(&end_point[1]), sizeof(double));
            s.read(reinterpret_cast<char *>(&end_point[2]), sizeof(double));
            direction_inv = end_point - start_point;
            double norm = direction_inv.norm();
            direction_inv = norm / direction_inv.array();
        }
        return s;
    }

    std::ostream &
    RayOccupancyOctreeNode::WriteData(std::ostream &s) const {
        s.write(reinterpret_cast<const char *>(&m_value_), sizeof(m_value_));
        std::size_t num_rays = m_start_points_.size();
        s.write(reinterpret_cast<const char *>(&num_rays), sizeof(std::size_t));
        for (std::size_t i = 0; i < num_rays; ++i) {
            const Eigen::Vector3d &start_point = m_start_points_[i];
            const Eigen::Vector3d &end_point = m_end_points_[i];
            s.write(reinterpret_cast<const char *>(&start_point[0]), sizeof(double));
            s.write(reinterpret_cast<const char *>(&start_point[1]), sizeof(double));
            s.write(reinterpret_cast<const char *>(&start_point[2]), sizeof(double));
            s.write(reinterpret_cast<const char *>(&end_point[0]), sizeof(double));
            s.write(reinterpret_cast<const char *>(&end_point[1]), sizeof(double));
            s.write(reinterpret_cast<const char *>(&end_point[2]), sizeof(double));
        }
        return s;
    }

    RayOccupancyOctree::RayOccupancyOctree(double resolution)
        : OccupancyOctreeBase<erl::geometry::RayOccupancyOctreeNode>(resolution) {
        s_init_.EnsureLinking();
    }

    RayOccupancyOctree::RayOccupancyOctree(const std::shared_ptr<Setting> &setting)
        : OccupancyOctreeBase<erl::geometry::RayOccupancyOctreeNode>(setting) {
        m_max_num_rays_per_node_ = setting->max_num_rays_per_node;
    }

    RayOccupancyOctree::RayOccupancyOctree(const std::string &filename)
        : OccupancyOctreeBase<erl::geometry::RayOccupancyOctreeNode>(0.1) {  // resolution will be set by readBinary
        ERL_ASSERTM(this->ReadBinary(filename), "Failed to read %s from file: %s", GetTreeType().c_str(), filename.c_str());
    }

    std::shared_ptr<RayOccupancyOctree::Setting>
    RayOccupancyOctree::GetSetting() const {
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
        setting->max_num_rays_per_node = m_max_num_rays_per_node_;
        return setting;
    }

    bool
    RayOccupancyOctree::IsNodeCollapsible(const erl::geometry::RayOccupancyOctreeNode *node) const {
        if (node->GetNumChildren() != 8) { return false; }  // all children must exist

        auto first_child = this->GetNodeChild(node, 0);
        if (first_child->HasAnyChild()) { return false; }
        if (this->IsNodeOccupied(first_child)) { return false; }  // occupied node will store rays, don't collapse

        for (uint32_t i = 1; i < 8; ++i) {
            auto child = this->GetNodeChild(node, i);
            if (child->HasAnyChild() || *child != *first_child) { return false; }  // should be a leaf node
        }
        return true;
    }

    void
    RayOccupancyOctree::InsertPointCloud(
        const Eigen::Ref<const Eigen::Matrix3Xd> &points,
        const Eigen::Ref<const Eigen::Vector3d> &sensor_origin,
        double max_range,
        bool parallel,
        bool lazy_eval,
        bool discretize) {
        Super::InsertPointCloud(points, sensor_origin, max_range, parallel, lazy_eval, discretize);
        // save rays to nodes
        OctreeKeyVectorMap &end_point_mapping = discretize ? m_discrete_end_point_mapping_ : m_end_point_mapping_;
        for (auto &[key, ray_indices]: end_point_mapping) {
            uint32_t depth = 0;
            RayOccupancyOctreeNode *node = this->Search(key, depth);
            if (node == nullptr) { continue; }
            if (!this->IsNodeOccupied(node)) { continue; }
            ERL_DEBUG_ASSERT(depth == this->mk_TreeDepth_, "depth is wrong for an occupied node.");
            auto it = ray_indices.begin();
            while (node->GetNumRays() < m_max_num_rays_per_node_ && it != ray_indices.end()) { node->AddRay(sensor_origin, points.col(*(it++))); }
        }
    }

    void
    RayOccupancyOctree::InsertPointCloudRays(
        const Eigen::Ref<const Eigen::Matrix3Xd> &points,
        const Eigen::Ref<const Eigen::Vector3d> &sensor_origin,
        double max_range,
        bool parallel,
        bool lazy_eval) {
        Super::InsertPointCloudRays(points, sensor_origin, max_range, parallel, lazy_eval);
        // save points to nodes
        long num_rays = points.cols();
        for (long i = 0; i < num_rays; ++i) {
            const auto &kP = points.col(i);
            double range = (kP - sensor_origin).norm();
            if (max_range <= 0. || (max_range > 0. && range <= max_range)) {
                OctreeKey key;
                if (!this->CoordToKeyChecked(kP[0], kP[1], kP[2], key)) { continue; }
                uint32_t depth = 0;  // find the deepest node
                RayOccupancyOctreeNode *node = this->Search(key, depth);
                if (node == nullptr) { continue; }
                if (!this->IsNodeOccupied(node)) { continue; }
                ERL_DEBUG_ASSERT(depth == this->mk_TreeDepth_, "depth is wrong for an occupied node.");
                if (node->GetNumRays() < m_max_num_rays_per_node_) { node->AddRay(sensor_origin, kP); }
            }
        }
    }

    bool
    RayOccupancyOctree::InsertRay(double sx, double sy, double sz, double ex, double ey, double ez, double max_range, bool lazy_eval) {
        double dx = ex - sx;
        double dy = ey - sy;
        double dz = ez - sz;
        double range = std::sqrt(dx * dx + dy * dy + dz * dz);
        auto &key_ray = this->m_key_rays_[0];
        if (max_range > 0 && range > max_range) {  // cut ray at max_range
            double r = max_range / range;
            ex = sx + dx * r;
            ey = sy + dy * r;
            ez = sz + dz * r;
            if (!this->ComputeRayKeys(sx, sy, sz, ex, ey, ez, key_ray)) { return false; }
            for (auto &key: key_ray) { UpdateNode(key, false, lazy_eval); }
            return true;
        }

        if (!this->ComputeRayKeys(sx, sy, sz, ex, ey, ez, key_ray)) { return false; }
        for (auto &key: key_ray) { UpdateNode(key, false, lazy_eval); }
        OctreeKey key;
        if (this->CoordToKeyChecked(ex, ey, ez, key)) {
            UpdateNode(key, true, lazy_eval);
            uint32_t depth = 0;  // find the deepest node
            RayOccupancyOctreeNode *node = this->Search(key, depth);
            if (node != nullptr && this->IsNodeOccupied(node)) {
                ERL_DEBUG_ASSERT(depth == this->mk_TreeDepth_, "depth is wrong for an occupied node.");
                if (node->GetNumRays() < m_max_num_rays_per_node_) { node->AddRay(Eigen::Vector3d(sx, sy, sz), Eigen::Vector3d(ex, ey, ez)); }
            }
        }
        return true;
    }
}  // namespace erl::geometry
