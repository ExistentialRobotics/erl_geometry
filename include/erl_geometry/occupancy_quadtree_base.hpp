#pragma once

#include <omp.h>
#include <list>
#include "abstract_occupancy_quadtree.hpp"
#include "quadtree_impl.hpp"
#include "aabb.hpp"

namespace erl::geometry {

    template<class Node>
    class OccupancyQuadtreeBase : public QuadtreeImpl<Node, AbstractOccupancyQuadtree> {
        static_assert(std::is_base_of_v<OccupancyQuadtreeNode, Node>);

    public:
        struct Setting : public common::Yamlable<Setting> {
            float log_odd_min = 0.0;
            float log_odd_max = 0.0;
            float probability_hit = 0.0;
            float probability_miss = 0.0;
            float probability_occupied = 0.0;
            double resolution = 0.0;
            bool use_change_detection = false;
            bool use_aabb_limit = false;
            Aabb2D aabb = {};
        };

    protected:
        bool m_use_change_detection_ = false;
        QuadtreeKeyBoolMap m_changed_keys_ = {};
        bool m_use_aabb_limit_ = false;
        Aabb2D m_aabb_ = {};
        // QuadtreeKey m_aabb_min_key_ = {};
        // QuadtreeKey m_aabb_max_key_ = {};

    public:
        using ImplType = OccupancyQuadtreeBase<Node>;

        explicit OccupancyQuadtreeBase(double resolution)
            : QuadtreeImpl<Node, AbstractOccupancyQuadtree>(resolution) {}

        explicit OccupancyQuadtreeBase(const std::shared_ptr<Setting>& setting)
            : QuadtreeImpl<Node, AbstractOccupancyQuadtree>(0.1) {
            SetSetting(setting);
        }

        OccupancyQuadtreeBase(const ImplType& other)
            : QuadtreeImpl<Node, AbstractOccupancyQuadtree>(other),
              m_use_change_detection_(other.m_use_change_detection_),
              m_changed_keys_(other.m_changed_keys_),
              m_use_aabb_limit_(other.m_use_aabb_limit_),
              m_aabb_(other.m_aabb_)
        // m_aabb_min_key_(other.m_aabb_min_key_),
        // m_aabb_max_key_(other.m_aabb_max_key_)
        {
            this->m_log_odd_hit_ = other.m_log_odd_hit_;
            this->m_log_odd_max_ = other.m_log_odd_max_;
            this->m_log_odd_min_ = other.m_log_odd_min_;
            this->m_log_odd_miss_ = other.m_log_odd_miss_;
            this->m_log_odd_occ_threshold_ = other.m_log_odd_occ_threshold_;
        }

        // YAML setting interface
        std::shared_ptr<Setting>
        GetSetting() const {
            auto setting = std::make_shared<Setting>();
            setting->log_odd_min = this->GetLogOddMin();
            setting->log_odd_max = this->GetLogOddMax();
            setting->probability_hit = this->GetProbabilityHit();
            setting->probability_miss = this->GetProbabilityMiss();
            setting->probability_occupied = this->GetOccupancyThreshold();
            setting->resolution = this->m_resolution_;
            setting->use_change_detection = this->m_use_change_detection_;
            setting->use_aabb_limit = this->m_use_aabb_limit_;
            setting->aabb = this->m_aabb_;
            return setting;
        }

        void
        SetSetting(const std::shared_ptr<Setting>& setting) {
            ERL_ASSERTM(setting != nullptr, "Setting is nullptr.");
            this->SetLogOddMin(setting->log_odd_min);
            this->SetLogOddMax(setting->log_odd_max);
            this->SetProbabilityHit(setting->probability_hit);
            this->SetProbabilityMiss(setting->probability_miss);
            this->SetOccupancyThreshold(setting->probability_occupied);
            this->SetResolution(setting->resolution);
            this->m_use_change_detection_ = setting->use_change_detection;
            this->m_use_aabb_limit_ = setting->use_aabb_limit;
            this->m_aabb_ = setting->aabb;
        }

        //-- implement abstract methods

        bool
        IsNodeCollapsible(const std::shared_ptr<Node>& node) const override {
            // all children must exist
            if (node->GetNumChildren() != 4) { return false; }

            auto first_child = this->GetNodeChild(node, 0);
            if (first_child->HasAnyChild()) { return false; }

            for (unsigned int i = 1; i < 4; ++i) {
                auto child = this->GetNodeChild(node, i);
                // child should be a leaf node
                if (child->HasAnyChild() || *child != *first_child) { return false; }
            }

            return true;
        }

        void
        OnExpandNode(std::shared_ptr<Node>& node, std::shared_ptr<Node>& child) override {
            child->SetLogOdds(node->GetLogOdds());  // copy log odds from parent to child
        }

        void
        OnPruneNode(std::shared_ptr<Node>& node) override {
            node->SetLogOdds(this->GetNodeChild(node, 0)->GetLogOdds());  // copy log odds from child to parent
        }

        void
        OnDeleteNodeChild(std::shared_ptr<Node>& node, unsigned int /* deleted_child_idx */) override {
            node->SetLogOdds(node->GetMaxChildLogOdds());  // copy log odds from child to parent
        }

        //-- insert point cloud
        /**
         * Insert a point cloud in the world frame. Multiple points may fall into the same voxel that is updated only once, and occupied nodes are preferred
         * than free ones. This avoids holes and is more efficient than the plain ray insertion of InsertPointCloudRays().
         * @param points 2xN matrix of points in the world frame
         * @param sensor_origin 2D vector of the sensor origin in the world frame
         * @param max_range maximum range of the sensor. Points beyond this range are ignored. Non-positive value means no limit.
         * @param lazy_eval whether to update the occupancy of the nodes immediately. If true, the occupancy is not updated until UpdateInnerOccupancy() is
         * called.
         * @param discretize
         */
        void
        InsertPointCloud(
            const Eigen::Ref<const Eigen::Matrix2Xd>& points,
            const Eigen::Ref<const Eigen::Vector2d>& sensor_origin,
            double max_range,
            bool parallel,
            bool lazy_eval,
            bool discretize
        ) {

            QuadtreeKeySet free_cells, occupied_cells;
            // compute cells to update
            if (discretize) {
                ComputeDiscreteUpdateForPointCloud(points, sensor_origin, max_range, parallel, free_cells, occupied_cells);
            } else {
                ComputeUpdateForPointCloud(points, sensor_origin, max_range, parallel, free_cells, occupied_cells);
            }

            // insert data into tree
            // update free cells
            for (auto free_cell: free_cells) { UpdateNode(free_cell, false, lazy_eval); }
            // update occupied cells
            for (auto occupied_cell: occupied_cells) { UpdateNode(occupied_cell, true, lazy_eval); }
        }

    protected:
        /**
         * Compute keys of the cells to update for a point cloud up to the resolution.
         * @param points 2xN matrix of points in the world frame, points falling into the same voxel are merged to the first appearance.
         * @param sensor_origin 2D vector of the sensor origin in the world frame
         * @param max_range maximum range of the sensor. Points beyond this range are ignored. Non-positive value means no limit.
         * @param parallel whether to use parallel computation
         * @param free_cells keys of the free cells to update
         * @param occupied_cells keys of the occupied cells to update
         */
        void
        ComputeDiscreteUpdateForPointCloud(
            const Eigen::Ref<const Eigen::Matrix2Xd>& points,
            const Eigen::Ref<const Eigen::Vector2d>& sensor_origin,
            double max_range,
            bool parallel,
            QuadtreeKeySet& free_cells,
            QuadtreeKeySet& occupied_cells
        ) {

            long num_points = points.cols();
            if (num_points == 0) { return; }

            Eigen::Matrix2Xd new_points(2, num_points);
            QuadtreeKeySet end_points;
            for (long i = 0; i < num_points; ++i) {
                const auto& kP = points.col(i);
                QuadtreeKey key = this->CoordToKey(kP[0], kP[1]);
                std::pair<QuadtreeKeySet::iterator, bool> ret = end_points.insert(key);
                if (ret.second) { new_points.col(long(end_points.size()) - 1) = kP; }  // new end point!
            }
            new_points.conservativeResize(2, long(end_points.size()));
            ComputeUpdateForPointCloud(new_points, sensor_origin, max_range, parallel, free_cells, occupied_cells);
        }

        void
        ComputeUpdateForPointCloud(
            const Eigen::Ref<const Eigen::Matrix2Xd>& points,
            const Eigen::Ref<const Eigen::Vector2d>& sensor_origin,
            double max_range,
            bool parallel,
            QuadtreeKeySet& free_cells,
            QuadtreeKeySet& occupied_cells
        ) {

            long num_points = points.cols();
            if (num_points == 0) { return; }

            omp_set_num_threads(this->m_key_rays_.size());
#pragma omp parallel for if (parallel) default(none) shared(num_points, points, sensor_origin, max_range, free_cells, occupied_cells) schedule(guided)
            for (long i = 0; i < num_points; ++i) {
                const auto& kP = points.col(i);
                unsigned int thread_idx = omp_get_thread_num();
                QuadtreeKeyRay& key_ray = this->m_key_rays_[thread_idx];

                double dx = kP[0] - sensor_origin[0];
                double dy = kP[1] - sensor_origin[1];
                double range = std::sqrt(dx * dx + dy * dy);
                if (m_use_aabb_limit_) {  // bounding box is specified
                    if (m_aabb_.contains(kP) && (max_range < 0. || range <= max_range)) {
                        // insert occupied endpoint
                        QuadtreeKey key;
                        if (this->CoordToKeyChecked(kP[0], kP[1], key)) {
#pragma omp critical(occupied_insert)
                            occupied_cells.insert(key);
                        }
                    }

                    // insert free cells up to the bounding box boundary
                    double ex = kP[0];
                    double ey = kP[1];
                    if ((max_range >= 0.) && (range > max_range)) {
                        double r = max_range / range;
                        ex = sensor_origin[0] + dx * r;
                        ey = sensor_origin[1] + dy * r;
                    }
                    if (this->ComputeRayKeys(sensor_origin[0], sensor_origin[1], ex, ey, key_ray)) {
#pragma omp critical(free_insert)
                        free_cells.insert(key_ray.begin(), key_ray.end());
                    }

                } else {  // bounding box is not specified
                    if (max_range < 0. || (range <= max_range)) {
                        // insert free cells
                        if (this->ComputeRayKeys(sensor_origin[0], sensor_origin[1], kP[0], kP[1], key_ray)) {
#pragma omp critical(free_insert)
                            free_cells.insert(key_ray.begin(), key_ray.end());
                        }
                        // insert occupied endpoint
                        QuadtreeKey key;
                        if (this->CoordToKeyChecked(kP[0], kP[1], key)) {
#pragma omp critical(occupied_insert)
                            occupied_cells.insert(key);
                        }
                    } else {  // the point is out of range
                        double r = max_range / range;
                        // insert free cells
                        if (this->ComputeRayKeys(sensor_origin[0], sensor_origin[1], sensor_origin[0] + dx * r, sensor_origin[1] + dy * r, key_ray)) {
#pragma omp critical(free_insert)
                            free_cells.insert(key_ray.begin(), key_ray.end());
                        }
                    }
                }
            }

            // prefer occupied cells to free ones
            // due to the resolution, the same cell may appear in both free_cells and occupied_cells.
            for (auto it = free_cells.begin(), end = free_cells.end(); it != end;) {
                if (occupied_cells.find(*it) != occupied_cells.end()) {
                    it = free_cells.erase(it);
                } else {
                    ++it;
                }
            }
        }

    public:
        /**
         * Insert a point cloud ray by ray. Some cells may be updated multiple times.
         * @param points 2xN matrix of ray end points in the world frame.
         * @param sensor_origin 2D vector of the sensor origin in the world frame.
         * @param max_range maximum range of the sensor. Points beyond this range are ignored. Non-positive value means no limit.
         * @param lazy_eval whether to update the occupancy of the nodes immediately. If true, the occupancy is not updated until UpdateInnerOccupancy() is
         * called.
         */
        void
        InsertPointCloudRays(
            const Eigen::Ref<const Eigen::Matrix2Xd>& points, const Eigen::Ref<const Eigen::Vector2d>& sensor_origin, double max_range, bool lazy_eval
        ) {

            long num_points = points.cols();
            if (num_points == 0) { return; }

            omp_set_num_threads(this->m_key_rays_.size());
#pragma omp parallel for default(none) shared(num_points, points, sensor_origin, max_range, lazy_eval) schedule(guided)
            for (long i = 0; i < num_points; ++i) {
                const auto& kP = points.col(i);
                unsigned int thread_idx = omp_get_thread_num();
                QuadtreeKeyRay& key_ray = this->m_key_rays_[thread_idx];

                if (this->ComputeRayKeys(sensor_origin[0], sensor_origin[1], kP[0], kP[1], key_ray)) {
#pragma omp critical
                    {
                        for (auto& key: key_ray) { UpdateNode(key, false, lazy_eval); }
                        double range = (kP - sensor_origin).norm();
                        if (max_range <= 0. || (max_range > 0. && range <= max_range)) { UpdateNode(kP[0], kP[1], true, lazy_eval); }
                    }
                }
            }
        }

        //-- insert ray
        /**
         * Insert a ray from (sx, sy) to (ex, ey) into the tree. The ray is cut at max_range if it is positive.
         * @param sx metric x coordinate of the start point
         * @param sy metric y coordinate of the start point
         * @param ex metric x coordinate of the end point
         * @param ey metric y coordinate of the end point
         * @param max_range maximum range after which the ray is cut. Non-positive value means no limit.
         * @param lazy_eval whether to update the occupancy of the nodes immediately. If true, the occupancy is not updated until UpdateInnerOccupancy() is
         * called.
         * @return
         */
        bool
        InsertRay(double sx, double sy, double ex, double ey, double max_range, bool lazy_eval) {
            double dx = ex - sx;
            double dy = ey - sy;
            double range = std::sqrt(dx * dx + dy * dy);
            auto& key_ray = this->m_key_rays_[0];
            if (max_range > 0 && range > max_range) {  // cut ray at max_range
                double r = max_range / range;
                ex = sx + dx * r;
                ey = sy + dy * r;
                if (!this->ComputeRayKeys(sx, sy, ex, ey, key_ray)) { return false; }
                for (auto& key: key_ray) { UpdateNode(key, false, lazy_eval); }
                return true;
            }

            if (!this->ComputeRayKeys(sx, sy, ex, ey, key_ray)) { return false; }
            for (auto& key: key_ray) { UpdateNode(key, false, lazy_eval); }
            UpdateNode(ex, ey, true, lazy_eval);
            return true;
        }

        //-- cast ray
        /**
         * Cast a ray starting from (px, py) along (vx, vy) and get the hit surface point (ex, ey) if the ray hits one.
         * @param px metric x coordinate of the start point
         * @param py metric y coordinate of the start point
         * @param vx x component of the ray direction
         * @param vy y component of the ray direction
         * @param ignore_unknown whether unknown cells are ignored, i.e. treated as free. If false, the ray casting aborts when an unknown cell is hit and
         * returns false.
         * @param max_range maximum range after which the ray casting is aborted. Non-positive value means no limit.
         * @param ex metric x coordinate of the hit leaf cell
         * @param ey metric y coordinate of the hit leaf cell
         * @return true if the ray hits an occupied cell, false otherwise.
         */
        bool
        CastRay(double px, double py, double vx, double vy, bool ignore_unknown, double max_range, double& ex, double& ey) const {
            // Similar to QuadtreeImpl::ComputeRayKeys, but with extra hitting checks

            QuadtreeKey current_key;
            if (!this->CoordToKeyChecked(px, py, current_key)) {
                ERL_WARN("Ray starting from (%f, %f) is out of range.\n", px, py);
                return false;
            }

            // initialization
            std::shared_ptr<const Node> starting_node = this->Search(current_key);
            if (starting_node != nullptr) {
                if (this->IsNodeOccupied(starting_node)) {  // (px, py) is in occupied
                    this->KeyToCoord(current_key, ex, ey);
                    return true;
                }
            } else if (!ignore_unknown) {  // (px, py) is in unknown
                this->KeyToCoord(current_key, ex, ey);
                return false;
            }

            double v_norm = std::sqrt(vx * vx + vy * vy);
            vx /= v_norm;
            vy /= v_norm;
            bool max_range_set = max_range > 0.;

            // compute step direction
            int step[2];
            if (vx > 0) {
                step[0] = 1;
            } else if (vx < 0) {
                step[0] = -1;
            } else {
                step[0] = 0;
            }
            if (vy > 0) {
                step[1] = 1;
            } else if (vy < 0) {
                step[1] = -1;
            } else {
                step[1] = 0;
            }
            if (step[0] == 0 && step[1] == 0) {
                ERL_WARN("Ray casting in direction (0, 0) is impossible!");
                return false;
            }

            // compute t_max and t_delta
            double t_max[2];
            double t_delta[2];
            if (step[0] == 0) {
                t_max[0] = std::numeric_limits<double>::infinity();
                t_delta[0] = std::numeric_limits<double>::infinity();
            } else {
                double voxel_border = this->KeyToCoord(current_key[0]) + double(step[0]) * 0.5 * this->m_resolution_;
                t_max[0] = (voxel_border - px) / vx;
                t_delta[0] = this->m_resolution_ / std::abs(vx);
            }
            if (step[1] == 0) {
                t_max[1] = std::numeric_limits<double>::infinity();
                t_delta[1] = std::numeric_limits<double>::infinity();
            } else {
                double voxel_border = this->KeyToCoord(current_key[1]) + double(step[1]) * 0.5 * this->m_resolution_;
                t_max[1] = (voxel_border - py) / vy;
                t_delta[1] = this->m_resolution_ / std::abs(vy);
            }

            // incremental phase
            double max_range_sq = max_range * max_range;
            unsigned int max_key_val = (this->mk_TreeKeyOffset_ << 1) - 1;
            while (true) {
                if (t_max[0] < t_max[1]) {
                    t_max[0] += t_delta[0];
                    current_key[0] += step[0];
                    // check overflow
                    if ((step[0] < 0 && current_key[0] == 0) || (step[0] > 0 && current_key[0] == max_key_val)) {
                        ERL_DEBUG("x coordinate hits boundary, aborting ray cast.");
                        this->KeyToCoord(current_key, ex, ey);
                        return false;
                    }
                } else {
                    t_max[1] += t_delta[1];
                    current_key[1] += step[1];
                    // check overflow
                    if ((step[1] < 0 && current_key[1] == 0) || (step[1] > 0 && current_key[1] == max_key_val)) {
                        ERL_DEBUG("y coordinate hits boundary, aborting ray cast.");
                        this->KeyToCoord(current_key, ex, ey);
                        return false;
                    }
                }
                // generate world coordinates from key
                this->KeyToCoord(current_key, ex, ey);
                // check if max_range is reached
                if (max_range_set) {
                    double dx = ex - px;
                    double dy = ey - py;
                    if ((dx * dx + dy * dy) > max_range_sq) { return false; }
                }
                // search node of the new key
                std::shared_ptr<const Node> current_node = this->Search(current_key);
                if (current_node != nullptr) {
                    if (this->IsNodeOccupied(current_node)) { return true; }
                } else if (!ignore_unknown) {
                    return false;
                }
            }
        }

        //-- trace ray
        class OccupiedLeafOnRayIterator : public QuadtreeImpl<Node, AbstractOccupancyQuadtree>::LeafOnRayIterator {
            using Super = typename QuadtreeImpl<Node, AbstractOccupancyQuadtree>::LeafOnRayIterator;

        public:
            OccupiedLeafOnRayIterator() = default;

            OccupiedLeafOnRayIterator(
                double px, double py, double vx, double vy, double max_range, bool bidirectional, const ImplType* tree, unsigned int max_leaf_depth
            )
                : Super(px, py, vx, vy, max_range, bidirectional, tree, max_leaf_depth) {
                if (!this->m_stack_.empty() && !this->m_tree_->IsNodeOccupied(this->m_stack_.back().node)) { ++(*this); }
            }

            // post-increment
            OccupiedLeafOnRayIterator
            operator++(int) {
                const OccupiedLeafOnRayIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            OccupiedLeafOnRayIterator&
            operator++() {
                if (this->m_stack_.empty()) { return *this; }
                do { this->SingleIncrement(); } while (!this->m_stack_.empty() && !this->m_tree_->IsNodeOccupied(this->m_stack_.back().node));
                return *this;
            }
        };

        [[nodiscard]] inline OccupiedLeafOnRayIterator
        BeginOccupiedLeafOnRay(double px, double py, double vx, double vy, double max_range = -1, bool bidirectional = false, unsigned int max_leaf_depth = 0)
            const {
            return OccupiedLeafOnRayIterator(px, py, vx, vy, max_range, bidirectional, this, max_leaf_depth);
        }

        [[nodiscard]] inline OccupiedLeafOnRayIterator
        EndOccupiedLeafOnRay() const {
            return OccupiedLeafOnRayIterator();
        }

        //-- update nodes' occupancy
        std::shared_ptr<OccupancyQuadtreeNode>
        UpdateNode(double x, double y, bool occupied, bool lazy_eval) override {
            QuadtreeKey key;
            if (!this->CoordToKeyChecked(x, y, key)) { return nullptr; }
            return UpdateNode(key, occupied, lazy_eval);
        }

        /**
         * Update occupancy measurement of a given node
         * @param key of the node to update
         * @param occupied whether the node is observed occupied or not
         * @param lazy_eval whether update of inner nodes is omitted and only leaf nodes are updated. This speeds up the intersection, but you need to call
         * UpdateInnerOccupancy() after all updates are done.
         */
        std::shared_ptr<OccupancyQuadtreeNode>
        UpdateNode(const QuadtreeKey& key, bool occupied, bool lazy_eval) override {
            float log_odds_delta = occupied ? this->m_log_odd_hit_ : this->m_log_odd_miss_;
            return UpdateNode(key, log_odds_delta, lazy_eval);
        }

        std::shared_ptr<OccupancyQuadtreeNode>
        UpdateNode(double x, double y, float log_odds_delta, bool lazy_eval) override {
            QuadtreeKey key;
            if (!this->CoordToKeyChecked(x, y, key)) { return nullptr; }
            return UpdateNode(key, log_odds_delta, lazy_eval);
        }

        std::shared_ptr<OccupancyQuadtreeNode>
        UpdateNode(const QuadtreeKey& key, float log_odds_delta, bool lazy_eval) override {
            auto leaf = std::static_pointer_cast<OccupancyQuadtreeNode>(this->Search(key));
            // early abort, no change will happen: node already at threshold or its log-odds is locked.
            if (leaf) {
                if (!leaf->AllowUpdateLogOdds(log_odds_delta)) { return leaf; }
                if (log_odds_delta >= 0 && leaf->GetLogOdds() >= this->m_log_odd_max_) { return leaf; }
                if (log_odds_delta <= 0 && leaf->GetLogOdds() <= this->m_log_odd_min_) { return leaf; }
            }

            bool create_root = false;
            if (this->m_root_ == nullptr) {
                this->m_root_ = std::make_shared<Node>();
                this->m_tree_size_++;
                create_root = true;
            }

            struct StackElement {
                std::shared_ptr<Node> node = nullptr;
                bool node_just_created = false;
                unsigned int depth = 0;
                bool visited = false;

                StackElement() = default;

                StackElement(std::shared_ptr<Node> node, bool node_just_created, unsigned int depth, bool visited)
                    : node(node),
                      node_just_created(node_just_created),
                      depth(depth),
                      visited(visited) {}
            };

            std::shared_ptr<OccupancyQuadtreeNode> returned_node = nullptr;
            std::list<StackElement> stack;
            stack.emplace_back(this->m_root_, create_root, 0, false);
            while (!stack.empty()) {
                StackElement& s = stack.back();

                if (s.visited) {  // inner node, re-visiting
                    if (this->PruneNode(s.node)) {
                        // returned node is pruned, current node is the parent of returned node
                        returned_node = std::static_pointer_cast<OccupancyQuadtreeNode>(s.node);
                    } else {
                        UpdateInnerNodeOccupancy(s.node);
                    }
                    stack.pop_back();
                    continue;
                }

                bool created_node = false;
                if (s.depth < this->mk_TreeDepth_) {  // follow down to last level

                    unsigned int pos = ComputeChildIndex(key, this->mk_TreeDepth_ - 1 - s.depth);
                    if (!s.node->HasChild(pos)) {  // child node does not exist
                        if (!s.node->HasAnyChild() && !s.node_just_created) {
                            // current node does not have any child, and it is not a new node
                            this->ExpandNode(s.node);
                        } else {
                            this->CreateNodeChild(s.node, pos);
                            created_node = true;
                        }
                    }

                    StackElement new_element(this->GetNodeChild(s.node, pos), created_node, s.depth + 1, false);
                    if (lazy_eval) {
                        stack.pop_back();  // we will not update inner node until UpdateInnerOccupancy() is called.
                    } else {
                        s.visited = true;  // else: // will update inner node when it is re-visited.
                    }

                    // add child node to the stack
                    stack.push_back(new_element);
                } else {  // we reach the last level
                    if (m_use_change_detection_) {
                        bool occ_before = this->IsNodeOccupied(s.node);
                        UpdateNodeLogOdds(s.node, log_odds_delta);

                        if (s.node_just_created) {
                            m_changed_keys_.emplace(key, true);
                        } else if (occ_before != this->IsNodeOccupied(s.node)) {  // occupancy changed, track it
                            auto it = m_changed_keys_.find(key);
                            if (it == m_changed_keys_.end()) {  // not found
                                m_changed_keys_.emplace(key, false);
                            } else if (!it->second) {
                                m_changed_keys_.erase(it);
                            }
                        }
                    } else {
                        UpdateNodeLogOdds(s.node, log_odds_delta);
                        returned_node = s.node;  // return the leaf node
                    }
                    stack.pop_back();
                }
            }

            return returned_node;
        }

    protected:
        void
        UpdateNodeLogOdds(const std::shared_ptr<OccupancyQuadtreeNode>& node, const float& log_odd_delta) {
            node->AddLogOdds(log_odd_delta);
            float l = node->GetLogOdds();
            if (l < this->m_log_odd_min_) {
                node->SetLogOdds(this->m_log_odd_min_);
                return;
            }

            if (l > this->m_log_odd_max_) {
                node->SetLogOdds(this->m_log_odd_max_);
                return;
            }
        }

    public:
        void
        UpdateInnerOccupancy() override {
            if (this->m_root_ == nullptr) { return; }

            // only update inner nodes
            struct StackElement {
                std::shared_ptr<Node> node = nullptr;
                unsigned int depth = 0;
                bool visited = false;

                StackElement() = default;

                StackElement(std::shared_ptr<Node> node, unsigned int depth, bool visited)
                    : node(node),
                      depth(depth),
                      visited(visited) {}
            };

            std::list<StackElement> stack;
            stack.emplace_back(std::static_pointer_cast<Node>(this->m_root_), 0, false);
            while (!stack.empty()) {
                auto& s = stack.back();

                if (s.visited) {
                    UpdateInnerNodeOccupancy(s.node);
                    stack.pop_back();
                    continue;
                }

                if (s.node->HasAnyChild()) {
                    if (s.depth + 1 < this->mk_TreeDepth_) {  // the child is also inner node
                        for (unsigned int i = 0; i < 4; ++i) {
                            auto child = this->GetNodeChild(s.node, i);
                            if (child == nullptr) { continue; }
                            stack.emplace_back(child, s.depth + 1, false);
                        }
                    }
                    s.visited = true;
                } else {  // no child, drop it directly
                    stack.pop_back();
                }
            }
        }

    protected:
        inline void
        UpdateInnerNodeOccupancy(std::shared_ptr<Node>& node) {
            node->SetLogOdds(node->GetMaxChildLogOdds());
        }

    public:
        /**
         * Set all nodes' log odds according to their current max likelihood of occupancy.
         */
        void
        ToMaxLikelihood() override {
            if (this->m_root_ == nullptr) { return; }

            std::vector<std::shared_ptr<Node>> stack;
            stack.emplace_back(std::static_pointer_cast<Node>(this->m_root_));
            while (!stack.empty()) {
                auto node = stack.back();
                stack.pop_back();

                if (this->IsNodeOccupied(node)) {
                    node->SetLogOdds(this->m_log_odd_max_);
                } else {
                    node->SetLogOdds(this->m_log_odd_min_);
                }

                if (node->HasAnyChild()) {
                    for (unsigned int i = 0; i < 4; ++i) {
                        auto child = this->GetNodeChild(node, i);
                        if (child == nullptr) { continue; }
                        stack.emplace_back(child);
                    }
                }
            }
        }

        //--file IO
        std::istream&
        ReadBinaryData(std::istream& s) override {
            if (this->m_root_ != nullptr) {
                ERL_WARN("Trying to read into an existing tree.");
                return s;
            }

            this->m_root_ = std::make_shared<Node>();
            this->m_tree_size_ = 1;
            char child_char;

            std::vector<std::pair<std::shared_ptr<Node>, bool>> stack;  // node, is_new_node
            stack.emplace_back(this->m_root_, true);

            while (!stack.empty()) {
                auto& top = stack.back();
                std::shared_ptr<Node> node = top.first;
                bool& is_new_node = top.second;

                if (!is_new_node) {
                    node->SetLogOdds(node->GetMaxChildLogOdds());
                    stack.pop_back();
                    continue;
                }

                is_new_node = false;
                s.read(&child_char, sizeof(char));
                std::bitset<8> child((unsigned long long) child_char);
                bool has_inner_node_child = false;
                for (int i = 3; i >= 0; --i) {
                    // 0b10: free leaf
                    // 0b01: occupied leaf
                    // 0b11: inner node
                    bool bit0 = child[i * 2];
                    bool bit1 = child[i * 2 + 1];
                    std::shared_ptr<Node> child_node = nullptr;
                    if (bit0) {
                        if (bit1) {  // 0b11, inner node
                            child_node = this->CreateNodeChild(node, i);
                            child_node->SetLogOdds(-200);
                            has_inner_node_child = true;
                            stack.emplace_back(child_node, true);
                        } else {  // 0b01, occupied leaf
                            child_node = this->CreateNodeChild(node, i);
                            child_node->SetLogOdds(this->m_log_odd_max_);
                        }
                    } else if (bit1) {  // 0b10, free leaf
                        child_node = this->CreateNodeChild(node, i);
                        child_node->SetLogOdds(this->m_log_odd_min_);
                    }
                    // else: 0b00, child is unknown, we leave it uninitialized
                }

                if (!has_inner_node_child) {
                    node->SetLogOdds(node->GetMaxChildLogOdds());
                    stack.pop_back();
                    continue;
                }
            }

            return s;
        }

        std::ostream&
        WriteBinaryData(std::ostream& s) const override {
            if (this->m_root_ == nullptr) {
                ERL_WARN("Trying to write an empty tree.");
                return s;
            }

            std::vector<std::shared_ptr<const Node>> nodes_stack;  // node
            nodes_stack.emplace_back(this->m_root_);

            while (!nodes_stack.empty()) {
                auto node = nodes_stack.back();
                nodes_stack.pop_back();

                std::bitset<8> child;
                for (int i = 3; i >= 0; --i) {
                    auto child_node = this->GetNodeChild(node, i);
                    if (child_node == nullptr) {  // 0b00, unknown
                        child[i * 2] = false;
                        child[i * 2 + 1] = false;
                        continue;
                    }

                    if (child_node->HasAnyChild()) {  // 0b11, inner node
                        child[i * 2] = true;
                        child[i * 2 + 1] = true;
                        nodes_stack.emplace_back(child_node);
                        continue;
                    }

                    if (this->IsNodeOccupied(child_node)) {  // 0b01, occupied leaf
                        child[i * 2] = true;
                        child[i * 2 + 1] = false;
                        continue;
                    }

                    // 0b10, free leaf
                    child[i * 2] = false;
                    child[i * 2 + 1] = true;
                }
                char child_char = (char) child.to_ulong();
                s.write(&child_char, sizeof(char));
            }

            return s;
        }

    protected:
        /**
         * Constructor to enable der
         * @param resolution
         * @param tree_depth
         * @param tree_key_offset
         */
        OccupancyQuadtreeBase(double resolution, unsigned int tree_depth, unsigned int tree_key_offset)
            : QuadtreeImpl<Node, AbstractOccupancyQuadtree>(resolution, tree_depth, tree_key_offset) {}
    };

}  // namespace erl::geometry

namespace YAML {
    template<typename Setting>
    struct ConvertOccupancyQuadtreeBaseSetting {
        inline static Node
        encode(const Setting& rhs) {
            Node node;
            node["log_odd_min"] = rhs.log_odd_min;
            node["log_odd_max"] = rhs.log_odd_max;
            node["probability_hit"] = rhs.probability_hit;
            node["probability_miss"] = rhs.probability_miss;
            node["probability_occupied"] = rhs.probability_occupied;
            node["resolution"] = rhs.resolution;
            node["use_change_detection"] = rhs.use_change_detection;
            node["use_aabb_limit"] = rhs.use_aabb_limit;
            node["aabb"] = rhs.aabb;
            return node;
        }

        inline static bool
        decode(const Node& node, Setting& rhs) {
            if (!node.IsMap()) { return false; }
            rhs.log_odd_min = node["log_odd_min"].as<float>();
            rhs.log_odd_max = node["log_odd_max"].as<float>();
            rhs.probability_hit = node["probability_hit"].as<float>();
            rhs.probability_miss = node["probability_miss"].as<float>();
            rhs.probability_occupied = node["probability_occupied"].as<float>();
            rhs.resolution = node["resolution"].as<double>();
            rhs.use_change_detection = node["use_change_detection"].as<bool>();
            rhs.use_aabb_limit = node["use_aabb_limit"].as<bool>();
            rhs.aabb = node["aabb"].as<erl::geometry::Aabb2D>();
            return true;
        }
    };

    template<typename Setting>
    Emitter&
    PrintOccupancyQuadtreeBaseSetting(Emitter& out, const Setting& rhs) {
        out << BeginMap;
        out << Key << "log_odd_min" << Value << rhs.log_odd_min;
        out << Key << "log_odd_max" << Value << rhs.log_odd_max;
        out << Key << "probability_hit" << Value << rhs.probability_hit;
        out << Key << "probability_miss" << Value << rhs.probability_miss;
        out << Key << "probability_occupied" << Value << rhs.probability_occupied;
        out << Key << "resolution" << Value << rhs.resolution;
        out << Key << "use_change_detection" << Value << rhs.use_change_detection;
        out << Key << "use_aabb_limit" << Value << rhs.use_aabb_limit;
        out << Key << "aabb" << Value << rhs.aabb;
        out << EndMap;
        return out;
    }
}  // namespace YAML
