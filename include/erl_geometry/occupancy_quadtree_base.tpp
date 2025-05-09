#pragma once

#include "erl_common/random.hpp"

#include <omp.h>

#include <list>

namespace erl::geometry {

    template<typename Dtype, class Node, class Setting>
    OccupancyQuadtreeBase<Dtype, Node, Setting>::OccupancyQuadtreeBase(
        const std::shared_ptr<Setting> &setting)
        : QuadtreeImpl<Node, AbstractOccupancyQuadtree<Dtype>, Setting>(setting),
          m_setting_(std::static_pointer_cast<OccupancyQuadtreeBaseSetting>(setting)) {}

    template<typename Dtype, class Node, class Setting>
    OccupancyQuadtreeBase<Dtype, Node, Setting>::OccupancyQuadtreeBase(
        std::shared_ptr<Setting> setting,
        const std::shared_ptr<common::GridMapInfo2D<Dtype>> &map_info,
        const cv::Mat &image_map,
        const Dtype occupied_threshold,
        const int padding)
        : QuadtreeImpl<Node, AbstractOccupancyQuadtree<Dtype>, Setting>(
              [&map_info, &setting]() -> std::shared_ptr<Setting> {
                  if (setting == nullptr) { setting = std::make_shared<Setting>(); }
                  setting->resolution = map_info->Resolution().mean();
                  setting->log_odd_max = 10.0f;
                  setting->SetProbabilityHit(0.95f);   // log_odd_hit = 3
                  setting->SetProbabilityMiss(0.49f);  // log_odd_miss = 0
                  return setting;
              }()),
          m_setting_(std::static_pointer_cast<OccupancyQuadtreeBaseSetting>(std::move(setting))) {
        ERL_ASSERTM(image_map.channels() == 1, "Image map must be a single channel image.");
        cv::Mat obstacle_map;
        cv::threshold(image_map, obstacle_map, occupied_threshold, 255, cv::THRESH_BINARY);
        for (int gx = 0; gx < obstacle_map.rows; ++gx) {
            for (int gy = 0; gy < obstacle_map.cols; ++gy) {
                const Dtype x = map_info->GridToMeterForValue(gx, 0);
                const Dtype y = map_info->GridToMeterForValue(gy, 1);
                this->UpdateNode(
                    x,
                    y,
                    /*occupied*/ obstacle_map.at<uint8_t>(gx, gy) > 0,
                    /*lazy_eval*/ false);
            }
        }

        // add padding
        for (int gx = -padding; gx < 0; ++gx) {
            for (int gy = -padding; gy < obstacle_map.cols + padding; ++gy) {
                const Dtype x = map_info->GridToMeterForValue(gx, 0);
                const Dtype y = map_info->GridToMeterForValue(gy, 1);
                this->UpdateNode(x, y, /*occupied*/ true, /*lazy_eval*/ false);
            }
        }
        for (int gx = obstacle_map.rows; gx < obstacle_map.rows + padding; ++gx) {
            for (int gy = -padding; gy < obstacle_map.cols + padding; ++gy) {
                const Dtype x = map_info->GridToMeterForValue(gx, 0);
                const Dtype y = map_info->GridToMeterForValue(gy, 1);
                this->UpdateNode(x, y, /*occupied*/ true, /*lazy_eval*/ false);
            }
        }
        for (int gx = 0; gx < obstacle_map.rows; ++gx) {
            for (int gy = -padding; gy < 0; ++gy) {
                const Dtype x = map_info->GridToMeterForValue(gx, 0);
                const Dtype y = map_info->GridToMeterForValue(gy, 1);
                this->UpdateNode(x, y, /*occupied*/ true, /*lazy_eval*/ false);
            }
        }
        for (int gx = 0; gx < obstacle_map.rows; ++gx) {
            for (int gy = obstacle_map.cols; gy < obstacle_map.cols + padding; ++gy) {
                const Dtype x = map_info->GridToMeterForValue(gx, 0);
                const Dtype y = map_info->GridToMeterForValue(gy, 1);
                this->UpdateNode(x, y, /*occupied*/ true, /*lazy_eval*/ false);
            }
        }
    }

    template<typename Dtype, class Node, class Setting>
    std::shared_ptr<AbstractQuadtree<Dtype>>
    OccupancyQuadtreeBase<Dtype, Node, Setting>::Clone() const {
        std::shared_ptr<AbstractQuadtree<Dtype>> tree =
            QuadtreeImpl<Node, AbstractOccupancyQuadtree<Dtype>, Setting>::Clone();
        std::shared_ptr<OccupancyQuadtreeBase> occupancy_tree =
            std::dynamic_pointer_cast<OccupancyQuadtreeBase>(tree);
        occupancy_tree->m_changed_keys_ = m_changed_keys_;
        occupancy_tree->m_discrete_end_point_mapping_ = m_discrete_end_point_mapping_;
        occupancy_tree->m_end_point_mapping_ = m_end_point_mapping_;
        return tree;
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyQuadtreeBase<Dtype, Node, Setting>::OnDeleteNodeChild(
        Node *node,
        Node *child,
        const QuadtreeKey & /*quadtree_key*/) {
        node->SetLogOdds(std::max(node->GetLogOdds(), child->GetLogOdds()));  // update log odds
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyQuadtreeBase<Dtype, Node, Setting>::SamplePositions(
        const std::size_t num_positions,
        std::vector<Vector2> &positions) const {
        positions.clear();
        positions.reserve(num_positions);
        Dtype min_x, min_y, max_x, max_y;
        this->GetMetricMinMax(min_x, min_y, max_x, max_y);
        std::uniform_real_distribution<Dtype> uniform_x(min_x, max_x);
        std::uniform_real_distribution<Dtype> uniform_y(min_y, max_y);
        std::size_t num_sampled_positions = 0;
        while (num_sampled_positions < num_positions) {
            Dtype x = uniform_x(common::g_random_engine);
            Dtype y = uniform_y(common::g_random_engine);
            const Node *node = this->Search(x, y);
            if (node == nullptr || this->IsNodeOccupied(node)) { continue; }
            positions.emplace_back(x, y);
            num_sampled_positions++;
        }
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyQuadtreeBase<Dtype, Node, Setting>::InsertPointCloud(
        const Eigen::Ref<const Matrix2X> &points,
        const Eigen::Ref<const Vector2> &sensor_origin,
        const Dtype max_range,
        const bool parallel,
        const bool lazy_eval,
        const bool discretize) {
        static QuadtreeKeyVector free_cells, occupied_cells;  // static to avoid memory allocation
        // compute cells to update
        if (discretize) {
            ComputeDiscreteUpdateForPointCloud(
                points,
                sensor_origin,
                max_range,
                parallel,
                free_cells,
                occupied_cells);
        } else {
            ComputeUpdateForPointCloud(
                points,
                sensor_origin,
                max_range,
                parallel,
                free_cells,
                occupied_cells);
        }
        // insert data into the tree
        for (const QuadtreeKey &free_cell: free_cells) {
            this->UpdateNode(free_cell, false, lazy_eval);
        }
        for (const QuadtreeKey &occupied_cell: occupied_cells) {
            this->UpdateNode(occupied_cell, true, lazy_eval);
        }
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyQuadtreeBase<Dtype, Node, Setting>::ComputeDiscreteUpdateForPointCloud(
        const Eigen::Ref<const Matrix2X> &points,
        const Eigen::Ref<const Vector2> &sensor_origin,
        const Dtype max_range,
        const bool parallel,
        QuadtreeKeyVector &free_cells,
        QuadtreeKeyVector &occupied_cells) {

        const long num_points = points.cols();
        if (num_points == 0) { return; }

        Matrix2X new_points(2, num_points);
        m_discrete_end_point_mapping_.clear();
        for (long i = 0; i < num_points; ++i) {
            const auto &point = points.col(i);
            QuadtreeKey key = this->CoordToKey(point[0], point[1]);
            auto &indices = m_discrete_end_point_mapping_[key];
            if (indices.empty()) {
                new_points.col(static_cast<long>(m_discrete_end_point_mapping_.size()) - 1)
                    << point;
            }
            indices.push_back(i);
        }
        new_points.conservativeResize(2, static_cast<long>(m_discrete_end_point_mapping_.size()));
        this->ComputeUpdateForPointCloud(
            new_points,
            sensor_origin,
            max_range,
            parallel,
            free_cells,
            occupied_cells);
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyQuadtreeBase<Dtype, Node, Setting>::ComputeUpdateForPointCloud(
        const Eigen::Ref<const Matrix2X> &points,
        const Eigen::Ref<const Vector2> &sensor_origin,
        const Dtype max_range,
        bool parallel,
        QuadtreeKeyVector &free_cells,
        QuadtreeKeyVector &occupied_cells) {

        const long num_points = points.cols();
        if (num_points == 0) { return; }

        QuadtreeKeySet &free_cells_set = this->m_key_sets_[0];

        free_cells_set.clear();
        m_end_point_mapping_.clear();
        free_cells.clear();
        occupied_cells.clear();

        std::vector<Dtype> ranges(num_points);
        std::vector<std::array<Dtype, 2>> diffs(num_points);
        omp_set_num_threads(this->m_key_rays_.size());

        // insert occupied endpoint
        const bool aabb_limit = m_setting_->use_aabb_limit;
        const auto &aabb = m_setting_->aabb;
        for (long i = 0; i < num_points; ++i) {
            const auto &p = points.col(i);

            Dtype &dx = diffs[i][0];
            Dtype &dy = diffs[i][1];
            Dtype &range = ranges[i];

            dx = p[0] - sensor_origin[0];
            dy = p[1] - sensor_origin[1];
            range = std::sqrt(dx * dx + dy * dy);

            QuadtreeKey key;
            if (aabb_limit) {
                if (aabb.contains(p.template cast<double>()) &&
                    (max_range < 0. || range <= max_range) &&
                    this->CoordToKeyChecked(p[0], p[1], key)) {
                    // 1. inside bounding box and range limit
                    // 2. key is valid
                    auto &indices = m_end_point_mapping_[key];
                    if (indices.empty()) { occupied_cells.push_back(key); }  // new key!
                    indices.push_back(i);
                }
            } else {
                if ((max_range < 0. || range <= max_range) &&    // range limit
                    this->CoordToKeyChecked(p[0], p[1], key)) {  // key is valid
                    auto &indices = m_end_point_mapping_[key];
                    if (indices.empty()) { occupied_cells.push_back(key); }  // new key!
                    indices.push_back(i);
                }
            }
        }

        const auto num_threads = static_cast<long>(this->m_key_rays_.size());
        if (num_threads <= 1 || num_points < 100 * num_threads) { parallel = false; }
        if (parallel) {
            const auto batch_size = num_points / num_threads + 1;
#pragma omp parallel for default(none) \
    shared(num_threads, batch_size, num_points, points, ranges, diffs, sensor_origin, max_range)
            for (long tid = 0; tid < num_threads; ++tid) {
                const long start_idx = tid * batch_size;
                const long end_idx = std::min(start_idx + batch_size, num_points);
                QuadtreeKeyRay &key_ray = this->m_key_rays_[tid];
                QuadtreeKeySet &key_set = this->m_key_sets_[tid];
                QuadtreeKeyVector &free_cells_tid = this->m_key_vectors_[tid];
                key_set.clear();
                free_cells_tid.clear();
                // clear key_set and free_cells_tid before checking if this thread is used.
                if (start_idx >= num_points) { continue; }
                const Dtype sx = sensor_origin[0];
                const Dtype sy = sensor_origin[1];
                for (long i = start_idx; i < end_idx; ++i) {
                    const auto &point = points.col(i);
                    const Dtype &range = ranges[i];
                    Dtype ex = point[0];
                    Dtype ey = point[1];
                    if ((max_range >= 0.0f) && (range > max_range)) {  // crop ray at max_range
                        const Dtype r = max_range / range;
                        ex = sx + diffs[i][0] * r;
                        ey = sy + diffs[i][1] * r;
                    }
                    if (!this->ComputeRayKeys(sx, sy, ex, ey, key_ray)) { continue; }
                    for (auto &key: key_ray) {
                        // skip keys marked as occupied or already inserted
                        if (m_end_point_mapping_.contains(key)) { continue; }
                        if (!key_set.emplace(key).second) { continue; }
                        free_cells_tid.push_back(key);
                    }
                }
            }

            // we do a stride-2 merge in advance with the following benefits:
            // 1. we can parallelize it;
            // 2. we don't need to do any set insertion so that rehashing and memory reallocation
            // are avoided;
            // 3. we can avoid the overhead of the critical section.
            // this part may be less helpful when chunks have small overlaps between each other
            // because it saves few insertion tries in the final merge step.
#pragma omp parallel for default(none) shared(num_threads)
            for (long i = 1; i < num_threads; i += 2) {  // 1, 3, 5, ...
                if (i + 1 >= num_threads) { continue; }
                QuadtreeKeySet &key_set0 = this->m_key_sets_[i];
                QuadtreeKeyVector &free_cells0 = this->m_key_vectors_[i];
                QuadtreeKeyVector &free_cells1 = this->m_key_vectors_[i + 1];
                if (free_cells1.empty()) { continue; }
                free_cells0.reserve(free_cells0.size() + free_cells1.size());
                for (auto &key: free_cells1) {
                    if (key_set0.contains(key)) { continue; }  // no insertion is needed
                    free_cells0.push_back(key);
                }
                free_cells1.clear();
            }

            const std::size_t max_num_free_cells = std::accumulate(
                this->m_key_vectors_.begin(),
                this->m_key_vectors_.end(),
                0,
                [](const std::size_t sum, const QuadtreeKeyVector &vec) {
                    return sum + vec.size();
                });
            free_cells.reserve(max_num_free_cells);
            free_cells_set.reserve(max_num_free_cells);
            free_cells.insert(
                free_cells.end(),
                this->m_key_vectors_[0].begin(),
                this->m_key_vectors_[0].end());
            // free_cells_set is the reference of m_key_sets_[0]
            for (long i = 1; i < num_threads; i += 2) {
                for (auto &key: this->m_key_vectors_[i]) {
                    if (!free_cells_set.emplace(key).second) { continue; }
                    free_cells.push_back(key);
                }
            }
            return;  // done.
        }

        const Dtype sx = sensor_origin[0];
        const Dtype sy = sensor_origin[1];
        for (long i = 0; i < num_points; ++i) {
            const auto &p = points.col(i);
            uint32_t thread_idx = omp_get_thread_num();
            QuadtreeKeyRay &key_ray = this->m_key_rays_[thread_idx];

            const Dtype &range = ranges[i];
            Dtype ex = p[0];
            Dtype ey = p[1];
            if (max_range >= 0.0f && range > max_range) {  // crop ray at max_range
                const Dtype r = max_range / range;
                ex = sx + diffs[i][0] * r;
                ey = sy + diffs[i][1] * r;
            }

            if (!this->ComputeRayKeys(sx, sy, ex, ey, key_ray)) { continue; }
            for (auto &key: key_ray) {
                // skip keys marked as occupied or already inserted
                if (m_end_point_mapping_.find(key) != m_end_point_mapping_.end()) { continue; }
                if (!free_cells_set.emplace(key).second) { continue; }
                free_cells.push_back(key);
            }
        }
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyQuadtreeBase<Dtype, Node, Setting>::InsertPointCloudRays(
        const Eigen::Ref<const Matrix2X> &points,
        const Eigen::Ref<const Vector2> &sensor_origin,
        const Dtype max_range,
        const bool parallel,
        const bool lazy_eval) {

        const long num_points = points.cols();
        if (num_points == 0) { return; }

        omp_set_num_threads(this->m_key_rays_.size());
#pragma omp parallel for if (parallel) default(none) \
    shared(num_points, points, sensor_origin, max_range, lazy_eval) schedule(guided)
        for (long i = 0; i < num_points; ++i) {
            const auto &point = points.col(i);
            uint32_t thread_idx = omp_get_thread_num();
            QuadtreeKeyRay &key_ray = this->m_key_rays_[thread_idx];
            if (!this->ComputeRayKeys(
                    sensor_origin[0],
                    sensor_origin[1],
                    point[0],
                    point[1],
                    key_ray)) {
                continue;
            }

#pragma omp critical
            {
                for (auto &key: key_ray) { UpdateNode(key, false, lazy_eval); }
                if (max_range <= 0. || (point - sensor_origin).norm() <= max_range) {
                    UpdateNode(point[0], point[1], true, lazy_eval);
                }
            }
        }
    }

    template<typename Dtype, class Node, class Setting>
    bool
    OccupancyQuadtreeBase<Dtype, Node, Setting>::InsertRay(
        Dtype sx,
        Dtype sy,
        Dtype ex,
        Dtype ey,
        const Dtype max_range,
        const bool lazy_eval) {
        const Dtype dx = ex - sx;
        const Dtype dy = ey - sy;
        const Dtype range = std::sqrt(dx * dx + dy * dy);
        auto &key_ray = this->m_key_rays_[0];
        if (max_range > 0 && range > max_range) {  // cut ray at max_range
            const Dtype r = max_range / range;
            ex = sx + dx * r;
            ey = sy + dy * r;
            if (!this->ComputeRayKeys(sx, sy, ex, ey, key_ray)) { return false; }
            for (auto &key: key_ray) { this->UpdateNode(key, false, lazy_eval); }
            return true;
        }

        if (!this->ComputeRayKeys(sx, sy, ex, ey, key_ray)) { return false; }
        for (auto &key: key_ray) { this->UpdateNode(key, false, lazy_eval); }
        this->UpdateNode(ex, ey, true, lazy_eval);
        return true;
    }

    template<typename Dtype, class Node, class Setting>
    OccupancyNdTreeBatchRayCaster<OccupancyQuadtreeBase<Dtype, Node, Setting>, 2>
    OccupancyQuadtreeBase<Dtype, Node, Setting>::GetBatchRayCaster(
        Matrix2X origins,
        Matrix2X directions,
        const VectorX &max_ranges,
        const VectorX &node_paddings,
        const Eigen::VectorXb &bidirectional_flags,
        const Eigen::VectorXb &leaf_only_flags,
        const Eigen::VectorXi &min_node_depths,
        const Eigen::VectorXi &max_node_depths) const {
        return OccupancyNdTreeBatchRayCaster<OccupancyQuadtreeBase, 2>(
            this,
            origins,
            directions,
            max_ranges,
            node_paddings,
            bidirectional_flags,
            leaf_only_flags,
            min_node_depths,
            max_node_depths);
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyQuadtreeBase<Dtype, Node, Setting>::CastRays(
        const Eigen::Ref<const Vector2> &position,
        const Eigen::Ref<const Matrix2> &rotation,
        const Eigen::Ref<const VectorX> &angles,
        const bool ignore_unknown,
        const Dtype max_range,
        const bool prune_rays,
        const bool parallel,
        std::vector<long> &hit_ray_indices,
        std::vector<Vector2> &hit_positions,
        std::vector<const Node *> &hit_nodes) const {

        (void) parallel;
        if (angles.size() == 0) { return; }
        long num_rays = angles.size();

        hit_ray_indices.clear();
        hit_positions.clear();
        hit_nodes.clear();

        hit_positions.resize(num_rays);
        hit_nodes.resize(num_rays, nullptr);

#pragma omp parallel for if (parallel) default(none) \
    shared(num_rays,                                 \
               position,                             \
               rotation,                             \
               angles,                               \
               ignore_unknown,                       \
               max_range,                            \
               hit_ray_indices,                      \
               hit_positions,                        \
               hit_nodes)
        for (long i = 0; i < num_rays; ++i) {
            const Dtype &kAngle = angles[i];
            Vector2 direction(std::cos(kAngle), std::sin(kAngle));
            direction = rotation * direction;
            Vector2 &hit_position = hit_positions[i];
            hit_nodes[i] = this->CastRay(
                position[0],
                position[1],
                direction[0],
                direction[1],
                ignore_unknown,
                max_range,
                hit_position[0],
                hit_position[1]);
        }

        absl::flat_hash_set<const Node *> hit_nodes_set;

        std::vector<Vector2> filtered_hit_positions;
        std::vector<const Node *> filtered_hit_nodes;
        hit_ray_indices.reserve(num_rays);
        filtered_hit_positions.reserve(num_rays);
        filtered_hit_nodes.reserve(num_rays);

        // remove rays that hit nothing or hit the same node if prune_rays is true
        for (long i = 0; i < num_rays; ++i) {
            const Node *&hit_node = hit_nodes[i];
            if (hit_node == nullptr) { continue; }
            if (!prune_rays || hit_nodes_set.insert(hit_node).second) {
                hit_ray_indices.push_back(i);
                filtered_hit_positions.push_back(hit_positions[i]);
                filtered_hit_nodes.push_back(hit_node);
            }
        }

        hit_positions = std::move(filtered_hit_positions);
        hit_nodes = std::move(filtered_hit_nodes);
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyQuadtreeBase<Dtype, Node, Setting>::CastRays(
        const Eigen::Ref<const Matrix2X> &positions,
        const Eigen::Ref<const Matrix2X> &directions,
        const bool ignore_unknown,
        const Dtype max_range,
        const bool prune_rays,
        const bool parallel,
        std::vector<long> &hit_ray_indices,
        std::vector<Vector2> &hit_positions,
        std::vector<const Node *> &hit_nodes) const {

        (void) parallel;
        long num_rays = 0;
        if (positions.cols() != 1 && directions.cols() != 1) {
            ERL_ASSERTM(
                positions.cols() == directions.cols(),
                "positions.cols() != directions.cols() when both are not 1.");
            num_rays = positions.cols();
            hit_positions.resize(num_rays);
            hit_nodes.resize(num_rays, nullptr);
#pragma omp parallel for if (parallel) default(none) \
    shared(num_rays, positions, directions, ignore_unknown, max_range, hit_positions, hit_nodes)
            for (long i = 0; i < num_rays; ++i) {
                hit_nodes[i] = CastRay(
                    positions(0, i),
                    positions(1, i),
                    directions(0, i),
                    directions(1, i),
                    ignore_unknown,
                    max_range,
                    hit_positions[i][0],
                    hit_positions[i][1]);
            }
        }
        if (positions.cols() == 1) {
            num_rays = directions.cols();
            hit_positions.resize(num_rays);
            hit_nodes.resize(num_rays, nullptr);
#pragma omp parallel for if (parallel) default(none) \
    shared(num_rays, positions, directions, ignore_unknown, max_range, hit_positions, hit_nodes)
            for (long i = 0; i < num_rays; ++i) {
                hit_nodes[i] = CastRay(
                    positions(0, 0),
                    positions(1, 0),
                    directions(0, i),
                    directions(1, i),
                    ignore_unknown,
                    max_range,
                    hit_positions[i][0],
                    hit_positions[i][1]);
            }
        }
        if (directions.cols() == 1) {
            num_rays = positions.cols();
            hit_positions.resize(num_rays);
            hit_nodes.resize(num_rays, nullptr);
#pragma omp parallel for if (parallel) default(none) \
    shared(num_rays, positions, directions, ignore_unknown, max_range, hit_positions, hit_nodes)
            for (long i = 0; i < num_rays; ++i) {
                hit_nodes[i] = CastRay(
                    positions(0, i),
                    positions(1, i),
                    directions(0, 0),
                    directions(1, 0),
                    ignore_unknown,
                    max_range,
                    hit_positions[i][0],
                    hit_positions[i][1]);
            }
        }

        if (num_rays == 0) { return; }

        absl::flat_hash_set<const Node *> hit_nodes_set;

        std::vector<Vector2> filtered_hit_positions;
        std::vector<const Node *> filtered_hit_nodes;
        hit_ray_indices.reserve(num_rays);
        filtered_hit_positions.reserve(num_rays);
        filtered_hit_nodes.reserve(num_rays);

        // remove rays that hit nothing or hit the same node if prune_rays is true
        for (long i = 0; i < num_rays; ++i) {
            const Node *&hit_node = hit_nodes[i];
            if (hit_node == nullptr) { continue; }
            if (!prune_rays || hit_nodes_set.insert(hit_node).second) {
                hit_ray_indices.push_back(i);
                filtered_hit_positions.push_back(hit_positions[i]);
                filtered_hit_nodes.push_back(hit_node);
            }
        }

        hit_positions = std::move(filtered_hit_positions);
        hit_nodes = std::move(filtered_hit_nodes);
    }

    template<typename Dtype, class Node, class Setting>
    const OccupancyQuadtreeNode *
    OccupancyQuadtreeBase<Dtype, Node, Setting>::GetHitOccupiedNode(
        const Dtype px,
        const Dtype py,
        const Dtype vx,
        const Dtype vy,
        const bool ignore_unknown,
        const Dtype max_range,
        Dtype &ex,
        Dtype &ey) const {
        return static_cast<const OccupancyQuadtreeNode *>(
            CastRay(px, py, vx, vy, ignore_unknown, max_range, ex, ey));
    }

    template<typename Dtype, class Node, class Setting>
    const Node *
    OccupancyQuadtreeBase<Dtype, Node, Setting>::CastRay(
        const Dtype px,
        const Dtype py,
        Dtype vx,
        Dtype vy,
        const bool ignore_unknown,
        const Dtype max_range,
        Dtype &ex,
        Dtype &ey) const {
        // Similar to QuadtreeImpl::ComputeRayKeys, but with extra hitting checks

        QuadtreeKey current_key;
        if (!this->CoordToKeyChecked(px, py, current_key)) {
            ERL_WARN("Ray starting from ({}, {}) is out of range.\n", px, py);
            return nullptr;
        }

        // initialization
        const Node *starting_node = this->Search(current_key);
        if (starting_node != nullptr) {
            if (this->IsNodeOccupied(starting_node)) {  // (px, py) is in occupied
                this->KeyToCoord(current_key, ex, ey);
                return starting_node;
            }
        } else if (!ignore_unknown) {  // (px, py) is in unknown
            this->KeyToCoord(current_key, ex, ey);
            return nullptr;
        }

        const Dtype v_norm = std::sqrt(vx * vx + vy * vy);
        vx /= v_norm;
        vy /= v_norm;
        const bool max_range_set = max_range > 0.;

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
            return nullptr;
        }

        // compute t_max and t_delta
        const Dtype &resolution = this->m_setting_->resolution;
        Dtype t_max[2];
        Dtype t_delta[2];
        if (step[0] == 0) {
            t_max[0] = std::numeric_limits<Dtype>::infinity();
            t_delta[0] = std::numeric_limits<Dtype>::infinity();
        } else {
            const Dtype voxel_border =
                this->KeyToCoord(current_key[0]) + static_cast<Dtype>(step[0]) * 0.5 * resolution;
            t_max[0] = (voxel_border - px) / vx;
            t_delta[0] = resolution / std::abs(vx);
        }
        if (step[1] == 0) {
            t_max[1] = std::numeric_limits<Dtype>::infinity();
            t_delta[1] = std::numeric_limits<Dtype>::infinity();
        } else {
            const Dtype voxel_border =
                this->KeyToCoord(current_key[1]) + static_cast<Dtype>(step[1]) * 0.5 * resolution;
            t_max[1] = (voxel_border - py) / vy;
            t_delta[1] = resolution / std::abs(vy);
        }

        // incremental phase
        const Dtype max_range_sq = max_range * max_range;
        const long max_key_val = (this->m_tree_key_offset_ << 1) - 1;
        while (true) {
            if (t_max[0] < t_max[1]) {
                t_max[0] += t_delta[0];
                const long next_key_val = static_cast<long>(current_key[0]) + step[0];
                // check overflow
                if ((step[0] < 0 && next_key_val <= 0) ||
                    (step[0] > 0 && next_key_val >= max_key_val)) {
                    ERL_DEBUG("x coordinate hits boundary, aborting ray cast.");
                    current_key[0] = next_key_val < 0 ? 0 : max_key_val;  // set to boundary
                    this->KeyToCoord(current_key, ex, ey);
                    return nullptr;
                }
                current_key[0] = next_key_val;
            } else {
                t_max[1] += t_delta[1];
                const long next_key_val = static_cast<long>(current_key[1]) + step[1];
                // check overflow
                if ((step[1] < 0 && next_key_val <= 0) ||
                    (step[1] > 0 && next_key_val >= max_key_val)) {
                    ERL_DEBUG("y coordinate hits boundary, aborting ray cast.");
                    current_key[1] = next_key_val < 0 ? 0 : max_key_val;
                    this->KeyToCoord(current_key, ex, ey);
                    return nullptr;
                }
                current_key[1] = next_key_val;
            }

            // generate world coordinates from the key
            this->KeyToCoord(current_key, ex, ey);
            // check if max_range is reached
            if (max_range_set) {
                if (const Dtype dx = ex - px, dy = ey - py; dx * dx + dy * dy > max_range_sq) {
                    return nullptr;
                }
            }
            // search node of the new key
            const Node *current_node = this->Search(current_key);
            if (current_node != nullptr) {
                if (this->IsNodeOccupied(current_node)) { return current_node; }
            } else if (!ignore_unknown) {
                return nullptr;
            }
        }
    }

    template<typename Dtype, class Node, class Setting>
    const QuadtreeKeyBoolMap &
    OccupancyQuadtreeBase<Dtype, Node, Setting>::GetChangedKeys() const {
        ERL_WARN_COND(
            !this->m_setting_->use_change_detection,
            "use_change_detection is false in setting. No changes are tracked.");
        return m_changed_keys_;
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyQuadtreeBase<Dtype, Node, Setting>::ClearChangedKeys() {
        m_changed_keys_.clear();
    }

    template<typename Dtype, class Node, class Setting>
    const QuadtreeKeyVectorMap &
    OccupancyQuadtreeBase<Dtype, Node, Setting>::GetEndPointMaps() const {
        return m_end_point_mapping_;
    }

    template<typename Dtype, class Node, class Setting>
    const QuadtreeKeyVectorMap &
    OccupancyQuadtreeBase<Dtype, Node, Setting>::GetDiscreteEndPointMaps() const {
        return m_discrete_end_point_mapping_;
    }

    template<typename Dtype, class Node, class Setting>
    Node *
    OccupancyQuadtreeBase<Dtype, Node, Setting>::UpdateNode(
        Dtype x,
        Dtype y,
        const bool occupied,
        const bool lazy_eval) {
        QuadtreeKey key;
        if (!this->CoordToKeyChecked(x, y, key)) { return nullptr; }
        return UpdateNode(key, occupied, lazy_eval);
    }

    template<typename Dtype, class Node, class Setting>
    Node *
    OccupancyQuadtreeBase<Dtype, Node, Setting>::UpdateNode(
        const QuadtreeKey &key,
        const bool occupied,
        const bool lazy_eval) {
        const float log_odds_delta = occupied ? m_setting_->log_odd_hit : m_setting_->log_odd_miss;
        return UpdateNode(key, log_odds_delta, lazy_eval);
    }

    template<typename Dtype, class Node, class Setting>
    Node *
    OccupancyQuadtreeBase<Dtype, Node, Setting>::UpdateNode(
        Dtype x,
        Dtype y,
        const float log_odds_delta,
        const bool lazy_eval) {
        QuadtreeKey key;
        if (!this->CoordToKeyChecked(x, y, key)) { return nullptr; }
        return UpdateNode(key, log_odds_delta, lazy_eval);
    }

    template<typename Dtype, class Node, class Setting>
    Node *
    OccupancyQuadtreeBase<Dtype, Node, Setting>::UpdateNode(
        const QuadtreeKey &key,
        float log_odds_delta,
        const bool lazy_eval) {
        if (std::vector<const Node *> nodes = this->SearchNodes(key); nodes.size() > 1) {
            // if the size of nodes is 1, there is just the root node.
            // when we enter this branch, it is more efficient because we will have a shallower
            // recursive call of UpdateNodeRecurs.
            Node *node = const_cast<Node *>(nodes.back());
            if (!node->HasAnyChild()) {
                // early abort, no change will happen: node already at threshold or its log-odds is
                // locked.
                if (!node->AllowUpdateLogOdds(log_odds_delta)) { return node; }
                const float cur_log_odds = node->GetLogOdds();
                if (log_odds_delta >= 0 && cur_log_odds >= m_setting_->log_odd_max) { return node; }
                if (log_odds_delta <= 0 && cur_log_odds <= m_setting_->log_odd_min) { return node; }
            }
            if (lazy_eval) {  // no pruning
                return static_cast<Node *>(
                    this->UpdateNodeRecurs(node, false, false, key, log_odds_delta, lazy_eval));
            }
            // when we reach here, we assume that all the inner nodes have the correct occupancy
            // before the update because `lazy_eval` is false. if the tree is ever updated with
            // `lazy_eval` set to true, we need to update the occupancy of the inner nodes before
            // updating the tree with `lazy_eval` set to false. the user should make it right.
            const bool deepest = node->GetDepth() == this->m_setting_->tree_depth;
            bool will_prune = deepest || node->HasAnyChild();
            // if the node is not leaf, we will create a child node for the key. prune will happen.
            // if the node is a leaf but not the deepest, we will expand the node. no prune then.
            ERL_DEBUG_ASSERT(
                node->HasAnyChild() || node == this->Search(key),
                "search stack does not end with the deepest node.");
            // the returned node is the deepest node after the update. and it is guaranteed that
            // `returned_node` is a descendent of `node`. i.e., `returned_node` is deeper than
            // `node`.
            Node *returned_node =
                this->UpdateNodeRecurs(node, false, false, key, log_odds_delta, lazy_eval);
            // even though `node` is not a leaf before this update, it might be a leaf after the
            // update. because the only missing child may be created and then pruned immediately,
            // which makes `node` become a leaf node. and there is no guarantee that `returned_node`
            // and `node` are the same.
            will_prune = will_prune && returned_node == node;
            // there might be pruning. if `node` and `returned_node` are the same, that means `node`
            // is the deepest or `node` is pruned and has no child. if they are different, the
            // pruning stops somewhere deeper under `node`. we don't need to prune anymore.
            if (will_prune) {
                // ERL_DEBUG_ASSERT(node == returned_node, "returned node is not leaf.");
                bool prune_failed = false;
                for (auto it = nodes.rbegin() + 1; it != nodes.rend(); ++it) {
                    auto cur_node = const_cast<Node *>(*it);
                    if (!prune_failed && this->PruneNode(cur_node)) {
                        returned_node = cur_node;
                    } else {
                        prune_failed = true;  // no more pruning once failed
                        this->UpdateInnerNodeOccupancy(cur_node);
                    }
                }
                return returned_node;
            }
            // we start from a node that will be expanded, no pruning will happen.
            // but we need to update inner node occupancy.
            for (auto it = nodes.rbegin() + 1; it != nodes.rend(); ++it) {
                this->UpdateInnerNodeOccupancy(const_cast<Node *>(*it));
            }
            return returned_node;
        }

        const bool create_root = this->m_root_ == nullptr;
        if (create_root) {
            this->m_root_ = std::make_shared<Node>();
            ++this->m_tree_size_;
            ERL_DEBUG_ASSERT(this->m_tree_size_ == 1, "tree size is not 1 after root creation.");
        }
        return static_cast<Node *>(this->UpdateNodeRecurs(
            this->m_root_.get(),
            create_root /*node_just_created*/,
            false /*node_from_expansion*/,
            key,
            log_odds_delta,
            lazy_eval));
    }

    template<typename Dtype, class Node, class Setting>
    Node *
    OccupancyQuadtreeBase<Dtype, Node, Setting>::UpdateNodeRecurs(
        Node *node,
        const bool node_just_created,
        bool node_from_expansion,
        const QuadtreeKey &key,
        const float log_odds_delta,
        const bool lazy_eval) {
        ERL_DEBUG_ASSERT(node != nullptr, "node is nullptr.");

        const uint32_t depth = node->GetDepth();
        if (const uint32_t &tree_depth = this->m_setting_->tree_depth; depth < tree_depth) {
            // follow down to the last level
            bool created_node = false;
            int pos = QuadtreeKey::ComputeChildIndex(key, tree_depth - 1 - depth);
            if (!node->HasChild(pos)) {  // child node does not exist
                if (!node->HasAnyChild() && !node_just_created) {
                    // the current node has no child and is not new: expand the pruned node
                    this->ExpandNode(node);
                    node_from_expansion = true;
                } else {
                    this->CreateNodeChild(node, pos);
                    created_node = true;
                }
            }

            if (lazy_eval) {
                return this->UpdateNodeRecurs(
                    this->GetNodeChild(node, pos),
                    created_node,
                    node_from_expansion,
                    key,
                    log_odds_delta,
                    lazy_eval);
            }
            Node *returned_node = this->UpdateNodeRecurs(
                this->GetNodeChild(node, pos),
                created_node,
                node_from_expansion,
                key,
                log_odds_delta,
                lazy_eval);
            // if the node is created from expansion, we don't need to prune it. because a deeper
            // node will block the pruning.
            if (!node_from_expansion && this->PruneNode(node)) {
                returned_node = node;  // returned_node is pruned, return its parent instead
            } else {
                this->UpdateInnerNodeOccupancy(node);
            }
            return returned_node;
        }
        // last level
        if (this->m_setting_->use_change_detection) {
            bool occ_before = this->IsNodeOccupied(node);
            UpdateNodeLogOdds(node, log_odds_delta);
            if (node_just_created) {
                m_changed_keys_.emplace(key, true);
            } else if (occ_before != this->IsNodeOccupied(node)) {  // occupancy changed, track it
                if (const auto it = m_changed_keys_.find(key); it == m_changed_keys_.end()) {
                    m_changed_keys_.emplace(key, false);  // not found
                } else if (!it->second) {
                    m_changed_keys_.erase(it);
                }
            }
        } else {
            UpdateNodeLogOdds(node, log_odds_delta);
        }
        return node;
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyQuadtreeBase<Dtype, Node, Setting>::UpdateNodeLogOdds(
        Node *node,
        float log_odd_delta) {
        node->AddLogOdds(log_odd_delta);
        const float l = node->GetLogOdds();
        const float log_odd_min = m_setting_->log_odd_min;
        const float log_odd_max = m_setting_->log_odd_max;
        if (l < log_odd_min) {
            node->SetLogOdds(log_odd_min);
            return;
        }

        if (l > log_odd_max) { node->SetLogOdds(log_odd_max); }
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyQuadtreeBase<Dtype, Node, Setting>::UpdateInnerOccupancy() {
        if (this->m_root_ == nullptr) { return; }
        UpdateInnerOccupancyRecurs(this->m_root_.get(), 0);
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyQuadtreeBase<Dtype, Node, Setting>::UpdateInnerOccupancyRecurs(
        Node *node,
        uint32_t depth) {
        ERL_DEBUG_ASSERT(node != nullptr, "node is nullptr.");
        if (!node->HasAnyChild()) { return; }
        // only recurse and update for inner nodes
        if (depth < this->m_setting_->tree_depth) {
            for (int i = 0; i < 4; ++i) {
                Node *child = this->GetNodeChild(node, i);
                if (child == nullptr) { continue; }
                UpdateInnerOccupancyRecurs(child, depth + 1);
            }
        }
        UpdateInnerNodeOccupancy(node);
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyQuadtreeBase<Dtype, Node, Setting>::UpdateInnerNodeOccupancy(Node *node) {
        node->SetLogOdds(node->GetMaxChildLogOdds());
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyQuadtreeBase<Dtype, Node, Setting>::ToMaxLikelihood() {
        if (this->m_root_ == nullptr) { return; }
        std::list<Node *> stack;
        stack.emplace_back(static_cast<Node *>(this->m_root_.get()));
        const Dtype log_odd_min = m_setting_->log_odd_min;
        const Dtype log_odd_max = m_setting_->log_odd_max;
        while (!stack.empty()) {
            Node *node = stack.back();
            stack.pop_back();

            if (this->IsNodeOccupied(node)) {
                node->SetLogOdds(log_odd_max);
            } else {
                node->SetLogOdds(log_odd_min);
            }

            if (node->HasAnyChild()) {
                for (uint32_t i = 0; i < 4; ++i) {
                    auto child = this->GetNodeChild(node, i);
                    if (child == nullptr) { continue; }
                    stack.emplace_back(child);
                }
            }
        }
    }

    template<typename Dtype, class Node, class Setting>
    bool
    OccupancyQuadtreeBase<Dtype, Node, Setting>::ReadBinaryData(std::istream &s) {
        if (this->m_root_ != nullptr) {
            ERL_WARN("Trying to read into an existing tree.");
            return false;
        }

        this->m_root_ = std::make_shared<Node>();
        this->m_tree_size_ = 1;
        char child_record;

        std::list<std::pair<Node *, bool>> stack;  // node, is_new_node
        stack.emplace_back(this->m_root_.get(), true);

        const Dtype log_odd_min = m_setting_->log_odd_min;
        const Dtype log_odd_max = m_setting_->log_odd_max;

        while (!stack.empty()) {
            auto &top = stack.back();
            Node *node = top.first;
            bool &is_new_node = top.second;

            if (!is_new_node) {
                node->SetLogOdds(node->GetMaxChildLogOdds());
                stack.pop_back();
                continue;
            }

            is_new_node = false;
            s.read(&child_record, sizeof(char));
            std::bitset<8> child(static_cast<unsigned long long>(child_record));
            bool has_inner_node_child = false;
            for (int i = 3; i >= 0; --i) {
                // 0b10: free leaf
                // 0b01: occupied leaf
                // 0b11: inner node
                const bool bit0 = child[i * 2];
                const bool bit1 = child[i * 2 + 1];
                Node *child_node = nullptr;
                if (bit0) {
                    if (bit1) {  // 0b11, inner node
                        child_node = this->CreateNodeChild(node, i);
                        child_node->SetLogOdds(-200);
                        has_inner_node_child = true;
                        stack.emplace_back(child_node, true);
                    } else {  // 0b01, occupied leaf
                        child_node = this->CreateNodeChild(node, i);
                        child_node->SetLogOdds(log_odd_max);
                    }
                } else if (bit1) {  // 0b10, free leaf
                    child_node = this->CreateNodeChild(node, i);
                    child_node->SetLogOdds(log_odd_min);
                }
                // else: 0b00, child is unknown, we leave it uninitialized
            }

            if (!has_inner_node_child) {
                node->SetLogOdds(node->GetMaxChildLogOdds());
                stack.pop_back();
            }
        }

        return s.good();
    }

    template<typename Dtype, class Node, class Setting>
    bool
    OccupancyQuadtreeBase<Dtype, Node, Setting>::WriteBinaryData(std::ostream &s) const {
        if (this->m_root_ == nullptr) { return s.good(); }

        std::list<const Node *> nodes_stack;  // node
        nodes_stack.push_back(this->m_root_.get());

        while (!nodes_stack.empty()) {
            const Node *node = nodes_stack.back();
            nodes_stack.pop_back();

            std::bitset<8> child;
            for (int i = 3; i >= 0; --i) {
                const Node *child_node = this->GetNodeChild(node, i);
                if (child_node == nullptr) {  // 0b00, unknown
                    child[i * 2] = false;
                    child[i * 2 + 1] = false;
                    continue;
                }

                if (child_node->HasAnyChild()) {  // 0b11, inner node
                    child[i * 2] = true;
                    child[i * 2 + 1] = true;
                    nodes_stack.push_back(child_node);
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
            char child_record = static_cast<char>(child.to_ulong());
            s.write(&child_record, sizeof(char));
        }

        return s.good();
    }
}  // namespace erl::geometry
