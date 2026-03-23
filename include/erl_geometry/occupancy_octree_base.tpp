#pragma once

#include "erl_common/random.hpp"

#include <omp.h>

#include <list>

namespace erl::geometry {
    template<typename Dtype, class Node, class Setting>
    OccupancyOctreeBase<Dtype, Node, Setting>::OccupancyOctreeBase(
        const std::shared_ptr<Setting> &setting)
        : Super(setting),
          m_setting_(std::static_pointer_cast<OccupancyOctreeBaseSetting>(setting)) {}

    template<typename Dtype, class Node, class Setting>
    std::shared_ptr<AbstractOctree<Dtype>>
    OccupancyOctreeBase<Dtype, Node, Setting>::Clone() const {
        std::shared_ptr<AbstractOctree<Dtype>> tree = Super::Clone();
        std::shared_ptr<OccupancyOctreeBase> occupancy_tree =
            std::dynamic_pointer_cast<OccupancyOctreeBase>(tree);
        occupancy_tree->m_changed_keys_ = m_changed_keys_;
        occupancy_tree->m_end_point_mapping_ = m_end_point_mapping_;
        return tree;
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyOctreeBase<Dtype, Node, Setting>::OnDeleteNodeChild(
        Node *node,
        Node *child,
        const OctreeKey & /*key*/) {
        node->SetLogOdds(std::max(node->GetLogOdds(), child->GetLogOdds()));  // update log odds
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyOctreeBase<Dtype, Node, Setting>::SamplePositions(
        const std::size_t num_positions,
        std::vector<Vector3> &positions) const {
        positions.clear();
        positions.reserve(num_positions);
        Dtype min_x, min_y, min_z, max_x, max_y, max_z;
        this->GetMetricMinMax(min_x, min_y, min_z, max_x, max_y, max_z);
        std::uniform_real_distribution<Dtype> uniform_x(min_x, max_x);
        std::uniform_real_distribution<Dtype> uniform_y(min_y, max_y);
        std::uniform_real_distribution<Dtype> uniform_z(min_z, max_z);
        std::size_t num_sampled_positions = 0;
        while (num_sampled_positions < num_positions) {
            Dtype x = uniform_x(common::g_random_engine);
            Dtype y = uniform_y(common::g_random_engine);
            Dtype z = uniform_z(common::g_random_engine);
            const Node *node = this->Search(x, y, z);
            if (node == nullptr || this->IsNodeOccupied(node)) { continue; }
            positions.emplace_back(x, y, z);
            num_sampled_positions++;
        }
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyOctreeBase<Dtype, Node, Setting>::ComputeOccupiedCells(
        const Eigen::Ref<const Matrix3X> &points,
        const Eigen::Ref<const Vector3> &sensor_origin,
        const Dtype min_range,
        const Dtype max_range,
        const bool discrete,
        std::vector<Dtype> &ranges,
        std::vector<std::array<Dtype, 3>> &diffs,
        Matrix3X &filtered_points,
        OctreeKeyVector &occupied_cells) {

        const long num_points = points.cols();
        if (num_points == 0) { return; }

        const bool aabb_limit = m_setting_->use_aabb_limit;
        const auto &aabb = m_setting_->aabb.cast<Dtype>();

        ranges.resize(num_points);
        diffs.resize(num_points);
        m_end_point_mapping_.clear();
        occupied_cells.clear();

        if (discrete) {
            filtered_points.resize(3, num_points);

            long cnt = 0;
            for (long i = 0; i < num_points; ++i) {
                const auto &point = points.col(i);

                Dtype &dx = diffs[cnt][0];
                Dtype &dy = diffs[cnt][1];
                Dtype &dz = diffs[cnt][2];
                Dtype &r = ranges[cnt];

                dx = point[0] - sensor_origin[0];
                dy = point[1] - sensor_origin[1];
                dz = point[2] - sensor_origin[2];
                r = std::sqrt(dx * dx + dy * dy + dz * dz);

                OctreeKey key;
                const bool key_valid = this->CoordToKeyChecked(point[0], point[1], point[2], key);

                if (!m_end_point_mapping_.contains(key)) { filtered_points.col(cnt++) << point; }
                auto &indices = m_end_point_mapping_[key];  // insert

                if (!key_valid) { continue; }
                if (((max_range > 0.0f) && (r > max_range)) || (r <= min_range)) { continue; }
                if (aabb_limit && !aabb.contains(point)) { continue; }
                indices.push_back(i);
                if (indices.size() == 1) { occupied_cells.push_back(key); }  // first time seen
            }
            filtered_points.conservativeResize(3, cnt);
            ranges.resize(cnt);
            diffs.resize(cnt);
            return;
        }

        filtered_points = points;
        for (long i = 0; i < num_points; ++i) {
            const auto &point = points.col(i);

            Dtype &dx = diffs[i][0];
            Dtype &dy = diffs[i][1];
            Dtype &dz = diffs[i][2];
            Dtype &r = ranges[i];

            dx = point[0] - sensor_origin[0];
            dy = point[1] - sensor_origin[1];
            dz = point[2] - sensor_origin[2];
            r = std::sqrt(dx * dx + dy * dy + dz * dz);

            OctreeKey key;
            if (!this->CoordToKeyChecked(point[0], point[1], point[2], key)) { continue; }
            if (((max_range > 0.0f) && (r > max_range)) || (r <= min_range)) { continue; }
            if (aabb_limit && !aabb.contains(point)) { continue; }
            auto indices = m_end_point_mapping_[key];
            indices.push_back(i);
            if (indices.size() == 1) { occupied_cells.push_back(key); }
        }
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyOctreeBase<Dtype, Node, Setting>::ComputeFreeCells(
        const Eigen::Ref<const Matrix3X> &points,
        const Eigen::Ref<const Vector3> &sensor_origin,
        const std::vector<Dtype> &ranges,
        const std::vector<std::array<Dtype, 3>> &diffs,
        const Dtype max_range,
        const bool with_count,
        bool parallel) {

        const long num_points = points.cols();
        if (num_points == 0) { return; }

        const auto num_threads = static_cast<long>(this->m_key_rays_.size());
        // single-threaded or too few points to parallelize
        if (num_threads <= 1 || num_points < 100 * num_threads) { parallel = false; }
        if (parallel) {
            const auto batch_size = num_points / num_threads + 1;
#pragma omp parallel for default(none) \
    shared(num_threads,                \
               batch_size,             \
               num_points,             \
               points,                 \
               ranges,                 \
               diffs,                  \
               sensor_origin,          \
               max_range,              \
               with_count)
            for (long tid = 0; tid < num_threads; ++tid) {
                const long start_idx = tid * batch_size;
                const long end_idx = std::min(start_idx + batch_size, num_points);
                OctreeKeyRay &key_ray = this->m_key_rays_[tid];
                OctreeKeyLongMap &key_map = this->m_key_long_maps_[tid];  // keep the count
                OctreeKeyVector &free_cells = this->m_key_vectors_[tid];  // keep the topology order
                key_map.clear();
                free_cells.clear();
                // clear key_map before checking if this thread is used.
                if (start_idx >= num_points) { continue; }
                const Dtype sx = sensor_origin[0];
                const Dtype sy = sensor_origin[1];
                const Dtype sz = sensor_origin[2];
                for (long i = start_idx; i < end_idx; ++i) {
                    const auto &point = points.col(i);
                    const Dtype &range = ranges[i];
                    Dtype ex = point[0];
                    Dtype ey = point[1];
                    Dtype ez = point[2];
                    if ((max_range >= 0.0f) && (range > max_range)) {  // crop ray at max_range
                        const Dtype r = max_range / range;
                        ex = sx + diffs[i][0] * r;
                        ey = sy + diffs[i][1] * r;
                        ez = sz + diffs[i][2] * r;
                    }
                    if (!this->ComputeRayKeys(sx, sy, sz, ex, ey, ez, key_ray)) { continue; }
                    for (auto &key: key_ray) {
                        // skip keys marked as occupied: exist in m_end_point_mapping_ and not empty
                        if (auto it = m_end_point_mapping_.find(key);
                            it != m_end_point_mapping_.end() && !it->second.empty()) {
                            continue;
                        }
                        auto [it, inserted] = key_map.try_emplace(key, 1);
                        if (inserted) {
                            free_cells.push_back(key);
                        } else {
                            ++it->second;
                        }
                    }
                }
            }

            // merge thread-local maps into the main free_cells map
            // step 1: do stride-2 merge to combine pairs of thread-local maps
#pragma omp parallel for default(none) shared(num_threads, with_count)
            for (long i = 1; i < num_threads; i += 2) {  // 1, 3, 5, ...
                if (i + 1 >= num_threads) { continue; }
                OctreeKeyLongMap &key_map0 = this->m_key_long_maps_[i];
                OctreeKeyLongMap &key_map1 = this->m_key_long_maps_[i + 1];
                OctreeKeyVector &free_cells0 = this->m_key_vectors_[i];
                OctreeKeyVector &free_cells1 = this->m_key_vectors_[i + 1];
                if (key_map1.empty()) { continue; }
                // merge
                key_map0.reserve(key_map0.size() + key_map1.size());
                free_cells0.reserve(free_cells0.size() + free_cells1.size());
                for (auto &key: free_cells1) {
                    const long cnt1 = key_map1[key];
                    if (with_count) {
                        auto [it, inserted] = key_map0.try_emplace(key, cnt1);
                        if (inserted) {
                            free_cells0.push_back(key);
                        } else {
                            it->second += cnt1;
                        }
                    } else {
                        // count will be inaccurate, but faster
                        if (!key_map0.contains(key)) { free_cells0.push_back(key); }
                    }
                }
                key_map1.clear();
                free_cells1.clear();
            }

            // step 2: merge all remaining maps into the final result
            // start with thread 0's map
            OctreeKeyLongMap &key_map = this->m_key_long_maps_[0];
            OctreeKeyVector &free_cells = this->m_key_vectors_[0];
            const std::size_t max_num_free_cells = std::accumulate(
                this->m_key_long_maps_.begin(),
                this->m_key_long_maps_.end(),
                0,
                [](const std::size_t sum, const OctreeKeyLongMap &map) {
                    return sum + map.size();
                });
            key_map.reserve(max_num_free_cells);
            free_cells.reserve(max_num_free_cells);
            // merge odd-indexed maps (which now contain accumulated counts)
            for (long i = 1; i < num_threads; i += 2) {
                for (const auto &[key, count]: this->m_key_long_maps_[i]) {
                    auto [it, inserted] = key_map.try_emplace(key, count);
                    if (inserted) {
                        free_cells.push_back(key);
                    } else {
                        it->second += count;
                    }
                }
            }
            return;  // done.
        }

        const Dtype sx = sensor_origin[0];
        const Dtype sy = sensor_origin[1];
        const Dtype sz = sensor_origin[2];
        const uint32_t thread_idx = omp_get_thread_num();
        OctreeKeyLongMap &key_map = this->m_key_long_maps_[0];
        OctreeKeyVector &free_cells = this->m_key_vectors_[0];
        OctreeKeyRay &key_ray = this->m_key_rays_[thread_idx];
        key_map.clear();
        free_cells.clear();
        key_ray.clear();
        for (long i = 0; i < num_points; ++i) {
            const auto &point = points.col(i);

            const Dtype &range = ranges[i];
            Dtype ex = point[0];
            Dtype ey = point[1];
            Dtype ez = point[2];
            if ((max_range >= 0.0f) && (range > max_range)) {  // crop ray at max_range
                const Dtype r = max_range / range;
                ex = sx + diffs[i][0] * r;
                ey = sy + diffs[i][1] * r;
                ez = sz + diffs[i][2] * r;
            }
            // key is invalid
            if (!this->ComputeRayKeys(sx, sy, sz, ex, ey, ez, key_ray)) { continue; }
            for (auto &key: key_ray) {
                // skip keys marked as occupied: exist in m_end_point_mapping_ and not empty
                if (auto it = m_end_point_mapping_.find(key);
                    it != m_end_point_mapping_.end() && !it->second.empty()) {
                    continue;
                }
                auto [it, inserted] = key_map.try_emplace(key, 1);
                if (inserted) {
                    free_cells.push_back(key);
                } else {
                    ++it->second;
                }
            }
        }
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyOctreeBase<Dtype, Node, Setting>::InsertPointCloud(
        const Eigen::Ref<const Matrix3X> &points,
        const Eigen::Ref<const Vector3> &sensor_origin,
        const Dtype min_range,
        const Dtype max_range,
        const bool with_count,
        const bool parallel,
        const bool lazy_eval,
        const bool discrete) {

        if (points.cols() == 0) { return; }

        std::vector<Dtype> ranges;
        std::vector<std::array<Dtype, 3>> diffs;
        Matrix3X new_points;
        OctreeKeyVector occupied_cells;
        occupied_cells.reserve(points.cols());
        ComputeOccupiedCells(
            points,
            sensor_origin,
            min_range,
            max_range,
            discrete,
            ranges,
            diffs,
            new_points,
            occupied_cells);
        ComputeFreeCells(new_points, sensor_origin, ranges, diffs, max_range, with_count, parallel);

        // insert data into the tree
        if (with_count) {
            for (const OctreeKey &free_cell: this->m_key_vectors_[0]) {
                const long cnt = this->m_key_long_maps_[0][free_cell];
                if (cnt <= 0) { continue; }  // should not happen
                float log_odds_delta = m_setting_->log_odd_miss * static_cast<float>(cnt);
                this->UpdateNode(free_cell, log_odds_delta, lazy_eval);
            }
            for (const OctreeKey &occupied_cell: occupied_cells) {
                const auto &indices = m_end_point_mapping_[occupied_cell];
                if (indices.empty()) { continue; }  // should not happen
                float log_odds_delta = m_setting_->log_odd_hit * static_cast<float>(indices.size());
                this->UpdateNode(occupied_cell, log_odds_delta, lazy_eval);
            }
            return;
        }

        // update free cells
        for (const OctreeKey &free_cell: this->m_key_vectors_[0]) {
            this->UpdateNode(free_cell, false, lazy_eval);
        }
        // update occupied cells
        for (const OctreeKey &occupied_cell: occupied_cells) {
            this->UpdateNode(occupied_cell, true, lazy_eval);
        }
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyOctreeBase<Dtype, Node, Setting>::InsertPointCloudRays(
        const Eigen::Ref<const Matrix3X> &points,
        const Eigen::Ref<const Vector3> &sensor_origin,
        const Dtype min_range,
        const Dtype max_range,
        const bool parallel,
        const bool lazy_eval) {

        const long num_points = points.cols();
        if (num_points == 0) { return; }

        omp_set_num_threads(this->m_key_rays_.size());
#pragma omp parallel for if (parallel) default(none) \
    shared(num_points, points, sensor_origin, min_range, max_range, lazy_eval) schedule(guided)
        for (long i = 0; i < num_points; ++i) {
            const auto point = points.col(i);
            uint32_t thread_idx = omp_get_thread_num();
            OctreeKeyRay &key_ray = this->m_key_rays_[thread_idx];
            if (!this->ComputeRayKeys(
                    sensor_origin[0],
                    sensor_origin[1],
                    sensor_origin[2],
                    point[0],
                    point[1],
                    point[2],
                    key_ray)) {
                continue;
            }

#pragma omp critical
            {
                for (auto &key: key_ray) { this->UpdateNode(key, false, lazy_eval); }
                if (const Dtype range = (point - sensor_origin).norm();
                    (max_range <= 0.0f || range <= max_range) && (range > min_range)) {
                    this->UpdateNode(point[0], point[1], point[2], true, lazy_eval);
                }
            }
        }
    }

    template<typename Dtype, class Node, class Setting>
    bool
    OccupancyOctreeBase<Dtype, Node, Setting>::InsertRay(
        Dtype sx,
        Dtype sy,
        Dtype sz,
        Dtype ex,
        Dtype ey,
        Dtype ez,
        const Dtype min_range,
        const Dtype max_range,
        bool lazy_eval) {
        const Dtype dx = ex - sx;
        const Dtype dy = ey - sy;
        const Dtype dz = ez - sz;
        const Dtype range = std::sqrt(dx * dx + dy * dy + dz * dz);
        auto &key_ray = this->m_key_rays_[0];

        bool hit = max_range <= 0.0f || range <= max_range;
        if (!hit) {  // cut ray at max_range
            const Dtype r = max_range / range;
            ex = sx + dx * r;
            ey = sy + dy * r;
            ez = sz + dz * r;
        }

        if (!this->ComputeRayKeys(sx, sy, sz, ex, ey, ez, key_ray)) { return false; }
        for (auto &key: key_ray) { this->UpdateNode(key, false, lazy_eval); }

        hit = hit && (range > min_range);
        if (!hit) { return true; }  // finished

        this->UpdateNode(ex, ey, ez, true, lazy_eval);
        return true;
    }

    template<typename Dtype, class Node, class Setting>
    OccupancyNdTreeBatchRayCaster<OccupancyOctreeBase<Dtype, Node, Setting>, 3>
    OccupancyOctreeBase<Dtype, Node, Setting>::GetBatchRayCaster(
        Matrix3X origins,
        Matrix3X directions,
        const VectorX &max_ranges,
        const VectorX &node_paddings,
        const Eigen::VectorXb &bidirectional_flags,
        const Eigen::VectorXb &leaf_only_flags,
        const Eigen::VectorXi &min_node_depths,
        const Eigen::VectorXi &max_node_depths) const {
        return OccupancyNdTreeBatchRayCaster<OccupancyOctreeBase, 3>(
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
    OccupancyOctreeBase<Dtype, Node, Setting>::CastRays(
        const Eigen::Ref<const Vector3> &position,
        const Eigen::Ref<const Matrix3> &rotation,
        const Eigen::Ref<const VectorX> &azimuth_angles,
        const Eigen::Ref<const VectorX> &elevation_angles,
        const bool ignore_unknown,
        const Dtype max_range,
        const bool prune_rays,
        const bool parallel,
        std::vector<std::pair<long, long>> &hit_ray_indices,
        std::vector<Vector3> &hit_positions,
        std::vector<const Node *> &hit_nodes) const {

        (void) parallel;
        const long num_azimuths = azimuth_angles.size();
        const long num_elevations = elevation_angles.size();
        if (num_azimuths == 0 || num_elevations == 0) { return; }
        long num_rays = num_azimuths * num_elevations;

        hit_ray_indices.clear();
        hit_positions.clear();
        hit_nodes.clear();

        hit_ray_indices.resize(num_rays);
        hit_positions.resize(num_rays);
        hit_nodes.resize(num_rays);

#pragma omp parallel for if (parallel) default(none) \
    shared(position,                                 \
               rotation,                             \
               num_azimuths,                         \
               num_elevations,                       \
               azimuth_angles,                       \
               elevation_angles,                     \
               ignore_unknown,                       \
               max_range,                            \
               hit_ray_indices,                      \
               hit_positions,                        \
               hit_nodes)
        for (long i = 0; i < num_azimuths; ++i) {
            const long idx_base = i * num_elevations;
            for (long j = 0; j < num_elevations; ++j) {
                long idx = idx_base + j;
                const Dtype cos_elevation = std::cos(elevation_angles[j]);
                Vector3 direction(
                    std::cos(azimuth_angles[i]) * cos_elevation,
                    std::sin(azimuth_angles[i]) * cos_elevation,
                    std::sin(elevation_angles[j]));
                direction = rotation * direction;
                Vector3 &hit_position = hit_positions[idx];
                hit_nodes[idx] = this->CastRay(
                    position[0],
                    position[1],
                    position[2],
                    direction[0],
                    direction[1],
                    direction[2],
                    ignore_unknown,
                    max_range,
                    hit_position[0],
                    hit_position[1],
                    hit_position[2]);
            }
        }

        absl::flat_hash_set<const Node *> hit_nodes_set;

        std::vector<std::pair<long, long>> filtered_hit_ray_indices;
        std::vector<Vector3> filtered_hit_positions;
        std::vector<const Node *> filtered_hit_nodes;

        filtered_hit_ray_indices.reserve(num_rays);
        filtered_hit_positions.reserve(num_rays);
        filtered_hit_nodes.reserve(num_rays);

        // remove rays that hit nothing or hit the same node if prune_rays is true
        for (long i = 0; i < num_rays; ++i) {
            const Node *&hit_node = hit_nodes[i];
            if (hit_node == nullptr) { continue; }
            if (!prune_rays || hit_nodes_set.insert(hit_node).second) {
                filtered_hit_ray_indices.push_back(hit_ray_indices[i]);
                filtered_hit_positions.push_back(hit_positions[i]);
                filtered_hit_nodes.push_back(hit_node);
            }
        }

        filtered_hit_ray_indices.shrink_to_fit();
        filtered_hit_positions.shrink_to_fit();
        filtered_hit_nodes.shrink_to_fit();

        std::swap(hit_ray_indices, filtered_hit_ray_indices);
        std::swap(hit_positions, filtered_hit_positions);
        std::swap(hit_nodes, filtered_hit_nodes);
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyOctreeBase<Dtype, Node, Setting>::CastRays(
        const Eigen::Ref<const Matrix3X> &positions,
        const Eigen::Ref<const Matrix3X> &directions,
        const bool ignore_unknown,
        const Dtype max_range,
        const bool prune_rays,
        const bool parallel,
        std::vector<long> &hit_ray_indices,
        std::vector<Vector3> &hit_positions,
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
                    positions(2, i),
                    directions(0, i),
                    directions(1, i),
                    directions(2, i),
                    ignore_unknown,
                    max_range,
                    hit_positions[i](0),
                    hit_positions[i](1),
                    hit_positions[i](2));
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
                    positions(2, 0),
                    directions(0, i),
                    directions(1, i),
                    directions(2, i),
                    ignore_unknown,
                    max_range,
                    hit_positions[i](0),
                    hit_positions[i](1),
                    hit_positions[i](2));
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
                    positions(2, i),
                    directions(0, 0),
                    directions(1, 0),
                    directions(2, 0),
                    ignore_unknown,
                    max_range,
                    hit_positions[i](0),
                    hit_positions[i](1),
                    hit_positions[i](2));
            }
        }

        if (num_rays == 0) { return; }

        absl::flat_hash_set<const Node *> hit_nodes_set;

        std::vector<long> filtered_hit_ray_indices;
        std::vector<Vector3> filtered_hit_positions;
        std::vector<const Node *> filtered_hit_nodes;
        filtered_hit_ray_indices.reserve(num_rays);
        filtered_hit_positions.reserve(num_rays);
        filtered_hit_nodes.reserve(num_rays);

        // remove rays that hit nothing or hit the same node if prune_rays is true
        for (long i = 0; i < num_rays; ++i) {
            const Node *&hit_node = hit_nodes[i];
            if (hit_node == nullptr) { continue; }
            if (!prune_rays || hit_nodes_set.insert(hit_node).second) {
                filtered_hit_ray_indices.push_back(i);
                filtered_hit_positions.push_back(hit_positions[i]);
                filtered_hit_nodes.push_back(hit_node);
            }
        }

        filtered_hit_ray_indices.shrink_to_fit();
        filtered_hit_positions.shrink_to_fit();
        filtered_hit_nodes.shrink_to_fit();

        std::swap(hit_ray_indices, filtered_hit_ray_indices);
        std::swap(hit_positions, filtered_hit_positions);
        std::swap(hit_nodes, filtered_hit_nodes);
    }

    template<typename Dtype, class Node, class Setting>
    const OccupancyOctreeNode *
    OccupancyOctreeBase<Dtype, Node, Setting>::GetHitOccupiedNode(
        const Dtype px,
        const Dtype py,
        const Dtype pz,
        const Dtype vx,
        const Dtype vy,
        const Dtype vz,
        const bool ignore_unknown,
        const Dtype max_range,
        Dtype &ex,
        Dtype &ey,
        Dtype &ez) const {
        return static_cast<const OccupancyOctreeNode *>(
            CastRay(px, py, pz, vx, vy, vz, ignore_unknown, max_range, ex, ey, ez));
    }

    template<typename Dtype, class Node, class Setting>
    const Node *
    OccupancyOctreeBase<Dtype, Node, Setting>::CastRay(
        Dtype px,
        Dtype py,
        Dtype pz,
        Dtype vx,
        Dtype vy,
        Dtype vz,
        const bool ignore_unknown,
        const Dtype max_range,
        Dtype &ex,
        Dtype &ey,
        Dtype &ez) const {
        // Similar to OctreeImpl::ComputeRayKeys, but with extra hitting checks

        OctreeKey current_key;
        if (!this->CoordToKeyChecked(px, py, pz, current_key)) {
            ERL_WARN("Ray starting from ({}, {}, {}) is out of range.\n", px, py, pz);
            return nullptr;
        }

        // initialization
        const Node *starting_node = this->Search(current_key);
        if (starting_node != nullptr) {
            if (this->IsNodeOccupied(starting_node)) {  // (px, py, pz) is in occupied
                this->KeyToCoord(current_key, ex, ey, ez);
                return starting_node;
            }
        } else if (!ignore_unknown) {  // (px, py, pz) is in unknown
            this->KeyToCoord(current_key, ex, ey, ez);
            return nullptr;
        }

        const Dtype v_norm = std::sqrt(vx * vx + vy * vy + vz * vz);
        vx /= v_norm;
        vy /= v_norm;
        vz /= v_norm;
        const bool max_range_set = max_range > 0.0f;

        // compute step direction
        int step[3];
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
        if (vz > 0) {
            step[2] = 1;
        } else if (vz < 0) {
            step[2] = -1;
        } else {
            step[2] = 0;
        }
        if (step[0] == 0 && step[1] == 0 && step[2] == 0) {
            ERL_WARN("Ray casting in direction (0, 0, 0) is impossible!");
            return nullptr;
        }

        // compute t_max and t_delta
        const Dtype resolution = this->m_setting_->resolution;
        Dtype t_max[3];
        Dtype t_delta[3];
        if (step[0] == 0) {
            t_max[0] = std::numeric_limits<Dtype>::infinity();
            t_delta[0] = std::numeric_limits<Dtype>::infinity();
        } else {
            const Dtype voxel_border =
                this->KeyToCoord(current_key[0]) + static_cast<Dtype>(step[0]) * 0.5f * resolution;
            t_max[0] = (voxel_border - px) / vx;
            t_delta[0] = resolution / std::abs(vx);
        }
        if (step[1] == 0) {
            t_max[1] = std::numeric_limits<Dtype>::infinity();
            t_delta[1] = std::numeric_limits<Dtype>::infinity();
        } else {
            const Dtype voxel_border =
                this->KeyToCoord(current_key[1]) + static_cast<Dtype>(step[1]) * 0.5f * resolution;
            t_max[1] = (voxel_border - py) / vy;
            t_delta[1] = resolution / std::abs(vy);
        }
        if (step[2] == 0) {
            t_max[2] = std::numeric_limits<Dtype>::infinity();
            t_delta[2] = std::numeric_limits<Dtype>::infinity();
        } else {
            const Dtype voxel_border =
                this->KeyToCoord(current_key[2]) + static_cast<Dtype>(step[2]) * 0.5f * resolution;
            t_max[2] = (voxel_border - pz) / vz;
            t_delta[2] = resolution / std::abs(vz);
        }

        // incremental phase
        const Dtype max_range_sq = max_range * max_range;
        const long max_key_val = (this->m_tree_key_offset_ << 1) - 1;
        while (true) {
            int idx = 0;
            if (t_max[1] < t_max[0]) { idx = 1; }
            if (t_max[2] < t_max[idx]) { idx = 2; }

            t_max[idx] += t_delta[idx];
            const long next_key_val = static_cast<long>(current_key[idx]) + step[idx];
            // check overflow
            if ((step[idx] < 0 && next_key_val <= 0) ||
                (step[idx] > 0 && next_key_val >= max_key_val)) {
                ERL_DEBUG("coordinate hits boundary, aborting ray cast.");
                current_key[idx] = next_key_val < 0 ? 0 : max_key_val;  // set to boundary
                this->KeyToCoord(current_key, ex, ey, ez);
                return nullptr;
            }
            current_key[idx] = next_key_val;

            // generate world coordinates from the key
            this->KeyToCoord(current_key, ex, ey, ez);
            // check if max_range is reached
            if (max_range_set) {
                if (const Dtype dx = ex - px, dy = ey - py, dz = ez - pz;
                    (dx * dx + dy * dy + dz * dz) > max_range_sq) {
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
    const OctreeKeyBoolMap &
    OccupancyOctreeBase<Dtype, Node, Setting>::GetChangedKeys() const {
        ERL_WARN_COND(
            !this->m_setting_->use_change_detection,
            "use_change_detection is false in setting. No changes are tracked.");
        return m_changed_keys_;
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyOctreeBase<Dtype, Node, Setting>::ClearChangedKeys() {
        m_changed_keys_.clear();
    }

    template<typename Dtype, class Node, class Setting>
    const OctreeKeyVectorMap &
    OccupancyOctreeBase<Dtype, Node, Setting>::GetEndPointMaps() const {
        return m_end_point_mapping_;
    }

    template<typename Dtype, class Node, class Setting>
    Node *
    OccupancyOctreeBase<Dtype, Node, Setting>::UpdateNode(
        Dtype x,
        Dtype y,
        Dtype z,
        const bool occupied,
        const bool lazy_eval) {
        OctreeKey key;
        if (!this->CoordToKeyChecked(x, y, z, key)) { return nullptr; }
        return this->UpdateNode(key, occupied, lazy_eval);
    }

    template<typename Dtype, class Node, class Setting>
    Node *
    OccupancyOctreeBase<Dtype, Node, Setting>::UpdateNode(
        const OctreeKey &key,
        const bool occupied,
        const bool lazy_eval) {
        const float log_odds_delta = occupied ? m_setting_->log_odd_hit : m_setting_->log_odd_miss;
        return this->UpdateNode(key, log_odds_delta, lazy_eval);
    }

    template<typename Dtype, class Node, class Setting>
    Node *
    OccupancyOctreeBase<Dtype, Node, Setting>::UpdateNode(
        Dtype x,
        Dtype y,
        Dtype z,
        const float log_odds_delta,
        const bool lazy_eval) {
        OctreeKey key;
        if (!this->CoordToKeyChecked(x, y, z, key)) { return nullptr; }
        return this->UpdateNode(key, log_odds_delta, lazy_eval);
    }

    template<typename Dtype, class Node, class Setting>
    Node *
    OccupancyOctreeBase<Dtype, Node, Setting>::UpdateNode(
        const OctreeKey &key,
        float log_odds_delta,
        const bool lazy_eval) {
        if (std::vector<const Node *> nodes = this->SearchNodes(key); nodes.size() > 1) {
            // if the size of nodes is 1, there is just the root node.
            // when we enter this branch, it is more efficient because we will have a shallower
            // recursive call of UpdateNodeRecurs.
            Node *node = const_cast<Node *>(nodes.back());
            if (!node->HasAnyChild()) {  // node is a leaf node
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
            create_root,
            false,
            key,
            log_odds_delta,
            lazy_eval));
    }

    template<typename Dtype, class Node, class Setting>
    Node *
    OccupancyOctreeBase<Dtype, Node, Setting>::UpdateNodeRecurs(
        Node *node,
        const bool node_just_created,
        bool node_from_expansion,
        const OctreeKey &key,
        const Dtype log_odds_delta,
        const bool lazy_eval) {
        ERL_DEBUG_ASSERT(node != nullptr, "node is nullptr.");

        const uint32_t depth = node->GetDepth();
        if (const uint32_t &tree_depth = this->m_setting_->tree_depth; depth < tree_depth) {
            // follow down to the last level
            bool created_node = false;
            int pos = OctreeKey::ComputeChildIndex(key, tree_depth - 1 - depth);
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
            this->UpdateNodeLogOdds(node, log_odds_delta);
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
            this->UpdateNodeLogOdds(node, log_odds_delta);
        }
        return node;
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyOctreeBase<Dtype, Node, Setting>::UpdateNodeLogOdds(Node *node, float log_odd_delta) {
        node->AddLogOdds(log_odd_delta);
        const float l = node->GetLogOdds();
        const Dtype log_odd_min = m_setting_->log_odd_min;
        const Dtype log_odd_max = m_setting_->log_odd_max;
        if (l < log_odd_min) {
            node->SetLogOdds(log_odd_min);
            return;
        }

        if (l > log_odd_max) { node->SetLogOdds(log_odd_max); }
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyOctreeBase<Dtype, Node, Setting>::UpdateInnerOccupancy() {
        if (this->m_root_ == nullptr) { return; }
        UpdateInnerOccupancyRecurs(this->m_root_.get(), 0);
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyOctreeBase<Dtype, Node, Setting>::UpdateInnerOccupancyRecurs(
        Node *node,
        uint32_t depth) {
        ERL_DEBUG_ASSERT(node != nullptr, "node is nullptr.");
        if (!node->HasAnyChild()) { return; }
        // only recurse and update for inner nodes
        if (depth < this->m_setting_->tree_depth) {
            for (int i = 0; i < 8; ++i) {
                Node *child = this->GetNodeChild(node, i);
                if (child == nullptr) { continue; }
                UpdateInnerOccupancyRecurs(child, depth + 1);
            }
        }
        UpdateInnerNodeOccupancy(node);
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyOctreeBase<Dtype, Node, Setting>::UpdateInnerNodeOccupancy(Node *node) {
        node->SetLogOdds(node->GetMaxChildLogOdds());
    }

    template<typename Dtype, class Node, class Setting>
    void
    OccupancyOctreeBase<Dtype, Node, Setting>::ToMaxLikelihood() {
        if (this->m_root_ == nullptr) { return; }
        std::list<Node *> stack;
        stack.emplace_back(static_cast<Node *>(this->m_root_.get()));
        const auto log_odd_min = static_cast<Dtype>(m_setting_->log_odd_min);
        const auto log_odd_max = static_cast<Dtype>(m_setting_->log_odd_max);
        while (!stack.empty()) {
            Node *node = stack.back();
            stack.pop_back();

            if (this->IsNodeOccupied(node)) {
                node->SetLogOdds(log_odd_max);
            } else {
                node->SetLogOdds(log_odd_min);
            }

            if (node->HasAnyChild()) {
                for (uint32_t i = 0; i < 8; ++i) {
                    auto child = this->GetNodeChild(node, i);
                    if (child == nullptr) { continue; }
                    stack.emplace_back(child);
                }
            }
        }
    }

    template<typename Dtype, class Node, class Setting>
    bool
    OccupancyOctreeBase<Dtype, Node, Setting>::ReadBinaryData(std::istream &s) {
        if (this->m_root_ != nullptr) {
            ERL_WARN("Trying to read into an existing tree.");
            return false;
        }

        this->m_root_ = std::make_shared<Node>();
        this->m_tree_size_ = 1;
        uint16_t child_record;

        std::list<std::pair<Node *, bool>> stack;  // node, is_new_node
        stack.emplace_back(this->m_root_.get(), true);

        const auto log_odd_min = static_cast<Dtype>(m_setting_->log_odd_min);
        const auto log_odd_max = static_cast<Dtype>(m_setting_->log_odd_max);

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
            s.read(reinterpret_cast<char *>(&child_record), sizeof(uint16_t));
            std::bitset<16> child(child_record);
            bool has_inner_node_child = false;
            for (int i = 7; i >= 0; --i) {
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
    OccupancyOctreeBase<Dtype, Node, Setting>::WriteBinaryData(std::ostream &s) const {
        if (this->m_root_ == nullptr) { return s.good(); }

        std::list<const Node *> nodes_stack;  // node
        nodes_stack.push_back(this->m_root_.get());

        while (!nodes_stack.empty()) {
            const Node *node = nodes_stack.back();
            nodes_stack.pop_back();

            std::bitset<16> child;
            for (int i = 7; i >= 0; --i) {
                const Node *child_node = this->GetNodeChild(node, i);
                if (child_node == nullptr) {  // 0b00, unknown
                    child[i << 1] = false;
                    child[(i << 1) + 1] = false;
                    continue;
                }

                if (child_node->HasAnyChild()) {  // 0b11, inner node
                    child[i << 1] = true;
                    child[(i << 1) + 1] = true;
                    nodes_stack.push_back(child_node);
                    continue;
                }

                if (this->IsNodeOccupied(child_node)) {  // 0b01, occupied leaf
                    child[i << 1] = true;
                    child[(i << 1) + 1] = false;
                    continue;
                }

                // 0b10, free leaf
                child[i << 1] = false;
                child[(i << 1) + 1] = true;
            }
            auto child_record = static_cast<uint16_t>(child.to_ulong());
            s.write(reinterpret_cast<char *>(&child_record), sizeof(uint16_t));
        }

        return s.good();
    }

    // ==================== Frontier Extraction ====================

    template<typename Dtype, class Node, class Setting>
    template<typename LeafIterator>
    std::vector<typename OccupancyOctreeBase<Dtype, Node, Setting>::Frontier>
    OccupancyOctreeBase<Dtype, Node, Setting>::ExtractFrontiersImpl(
        LeafIterator it,
        LeafIterator it_end,
        const std::size_t min_num_triangles,
        const bool sort_by_area) const {

        struct FaceQuad {
            int fixed_dim;
            uint32_t fixed_val;
            int d1_dim;
            int d2_dim;
            uint32_t d1_min, d1_max, d2_min, d2_max;
            bool flip_winding;
        };

        const auto setting = this->template GetSetting<OccupancyNdTreeSetting>();
        const uint32_t tree_depth = setting->tree_depth;

        // --- Step 1: Collect frontier face quads ---

        std::vector<FaceQuad> quads;

        // For each face of a free leaf, find uncovered (unknown) rectangular regions
        // via a 2D sweep. Each uncovered rectangle becomes a frontier quad.
        const auto emit_frontier_quads = [&](auto nit,
                                             auto nit_end,
                                             int fixed_dim,
                                             uint32_t fixed_val,
                                             int d1_dim,
                                             int d2_dim,
                                             uint32_t d1_min,
                                             uint32_t d1_max,
                                             uint32_t d2_min,
                                             uint32_t d2_max,
                                             bool flip_winding) {
            struct Rect {
                uint32_t d1_min, d1_max, d2_min, d2_max;
            };
            std::vector<Rect> neighbor_rects;

            for (; nit != nit_end; ++nit) {
                const uint32_t nd = nit.GetDepth();
                const uint32_t nl = tree_depth - nd;
                const OctreeKey &nk = nit.GetKey();
                const uint32_t cell_size = 1u << nl;

                uint32_t n_d1_lo = (nk[d1_dim] >> nl) << nl;
                uint32_t n_d1_hi = n_d1_lo + cell_size;
                uint32_t n_d2_lo = (nk[d2_dim] >> nl) << nl;
                uint32_t n_d2_hi = n_d2_lo + cell_size;

                // Clip to face extent
                n_d1_lo = std::max(n_d1_lo, d1_min);
                n_d1_hi = std::min(n_d1_hi, d1_max);
                n_d2_lo = std::max(n_d2_lo, d2_min);
                n_d2_hi = std::min(n_d2_hi, d2_max);

                if (n_d1_lo < n_d1_hi && n_d2_lo < n_d2_hi) {
                    neighbor_rects.push_back({n_d1_lo, n_d1_hi, n_d2_lo, n_d2_hi});
                }
            }

            // If no neighbors found, the entire face is frontier.
            if (neighbor_rects.empty()) {
                quads.push_back(
                    {fixed_dim,
                     fixed_val,
                     d1_dim,
                     d2_dim,
                     d1_min,
                     d1_max,
                     d2_min,
                     d2_max,
                     flip_winding});
                return;
            }

            if (neighbor_rects.size() == 1 && neighbor_rects[0].d1_min == d1_min &&
                neighbor_rects[0].d1_max == d1_max && neighbor_rects[0].d2_min == d2_min &&
                neighbor_rects[0].d2_max == d2_max) {
                // The only neighbor fully covers the face, no frontier.
                return;
            }

            // 2D sweep: split along d1 breakpoints, then 1D sweep along d2 per band.
            std::vector<uint32_t> d1_breaks;
            d1_breaks.push_back(d1_min);
            d1_breaks.push_back(d1_max);
            for (const auto &r: neighbor_rects) {
                d1_breaks.push_back(r.d1_min);
                d1_breaks.push_back(r.d1_max);
            }
            std::sort(d1_breaks.begin(), d1_breaks.end());
            d1_breaks.erase(std::unique(d1_breaks.begin(), d1_breaks.end()), d1_breaks.end());

            for (std::size_t bi = 0; bi + 1 < d1_breaks.size(); ++bi) {
                const uint32_t band_lo = d1_breaks[bi];
                const uint32_t band_hi = d1_breaks[bi + 1];

                std::vector<std::pair<uint32_t, uint32_t>> covered;
                for (const auto &r: neighbor_rects) {
                    if (r.d1_min <= band_lo && r.d1_max >= band_hi) {
                        covered.emplace_back(r.d2_min, r.d2_max);
                    }
                }
                std::sort(covered.begin(), covered.end());

                uint32_t current = d2_min;
                for (const auto &[c_lo, c_hi]: covered) {
                    if (c_lo > current) {
                        quads.push_back(
                            {fixed_dim,
                             fixed_val,
                             d1_dim,
                             d2_dim,
                             band_lo,
                             band_hi,
                             current,
                             c_lo,
                             flip_winding});
                    }
                    current = std::max(current, c_hi);
                }
                if (current < d2_max) {
                    quads.push_back(
                        {fixed_dim,
                         fixed_val,
                         d1_dim,
                         d2_dim,
                         band_lo,
                         band_hi,
                         current,
                         d2_max,
                         flip_winding});
                }
            }
        };

        for (; it != it_end; ++it) {
            const Node *node = it.GetNode();
            if (this->IsNodeOccupied(node)) { continue; }

            const OctreeKey &key = it.GetKey();
            const uint32_t depth = it.GetDepth();
            const uint32_t level = tree_depth - depth;
            const uint32_t cell_size = 1u << level;
            const uint32_t kx_min = (key[0] >> level) << level;
            const uint32_t ky_min = (key[1] >> level) << level;
            const uint32_t kz_min = (key[2] >> level) << level;
            const uint32_t kx_max = kx_min + cell_size;
            const uint32_t ky_max = ky_min + cell_size;
            const uint32_t kz_max = kz_min + cell_size;

            OctreeKey neighbor_key;

            // Winding flip rule:
            // Natural cross(d1_axis, d2_axis) gives +X for (Y,Z), -Y for (X,Z), +Z for (X,Y).
            // Outward normal points from free cell toward unknown space.
            // West(-X): natural +X, want -X → flip.  East(+X): natural +X, want +X → no flip.
            // South(-Y): natural -Y, want -Y → no flip.  North(+Y): natural -Y, want +Y → flip.
            // Bottom(-Z): natural +Z, want -Z → flip.  Top(+Z): natural +Z, want +Z → no flip.

            if (this->ComputeWestNeighborKey(key, depth, neighbor_key)) {
                emit_frontier_quads(
                    this->BeginWestLeafNeighbor(key, depth),
                    this->EndWestLeafNeighbor(),
                    0,
                    kx_min,
                    1,
                    2,
                    ky_min,
                    ky_max,
                    kz_min,
                    kz_max,
                    /*flip=*/true);
            }
            if (this->ComputeEastNeighborKey(key, depth, neighbor_key)) {
                emit_frontier_quads(
                    this->BeginEastLeafNeighbor(key, depth),
                    this->EndEastLeafNeighbor(),
                    0,
                    kx_max,
                    1,
                    2,
                    ky_min,
                    ky_max,
                    kz_min,
                    kz_max,
                    /*flip=*/false);
            }
            if (this->ComputeSouthNeighborKey(key, depth, neighbor_key)) {
                emit_frontier_quads(
                    this->BeginSouthLeafNeighbor(key, depth),
                    this->EndSouthLeafNeighbor(),
                    1,
                    ky_min,
                    0,
                    2,
                    kx_min,
                    kx_max,
                    kz_min,
                    kz_max,
                    /*flip=*/false);
            }
            if (this->ComputeNorthNeighborKey(key, depth, neighbor_key)) {
                emit_frontier_quads(
                    this->BeginNorthLeafNeighbor(key, depth),
                    this->EndNorthLeafNeighbor(),
                    1,
                    ky_max,
                    0,
                    2,
                    kx_min,
                    kx_max,
                    kz_min,
                    kz_max,
                    /*flip=*/true);
            }
            if (this->ComputeBottomNeighborKey(key, depth, neighbor_key)) {
                emit_frontier_quads(
                    this->BeginBottomLeafNeighbor(key, depth),
                    this->EndBottomLeafNeighbor(),
                    2,
                    kz_min,
                    0,
                    1,
                    kx_min,
                    kx_max,
                    ky_min,
                    ky_max,
                    /*flip=*/true);
            }
            if (this->ComputeTopNeighborKey(key, depth, neighbor_key)) {
                emit_frontier_quads(
                    this->BeginTopLeafNeighbor(key, depth),
                    this->EndTopLeafNeighbor(),
                    2,
                    kz_max,
                    0,
                    1,
                    kx_min,
                    kx_max,
                    ky_min,
                    ky_max,
                    /*flip=*/false);
            }
        }

        if (quads.empty()) { return {}; }

        // --- Step 2: Build corner keys for each quad and group by shared edges ---

        // Compute 4 corner OctreeKeys for a quad.
        // Order: p0=(d1_min,d2_min), p1=(d1_max,d2_min), p2=(d1_max,d2_max), p3=(d1_min,d2_max)
        // If flip_winding, swap p1 and p3 to reverse winding.
        const auto get_corners = [](const FaceQuad &q) -> std::array<OctreeKey, 4> {
            std::array<OctreeKey, 4> c;
            for (int i = 0; i < 4; ++i) {
                c[i][q.fixed_dim] = q.fixed_val;
                c[i][q.d1_dim] = (i == 1 || i == 2) ? q.d1_max : q.d1_min;
                c[i][q.d2_dim] = (i == 2 || i == 3) ? q.d2_max : q.d2_min;
            }
            if (q.flip_winding) { std::swap(c[1], c[3]); }
            return c;
        };

        // Build edge adjacency for connected component grouping.
        // An edge is (min_key, max_key) pair ordered canonically.
        struct KeyPair {
            OctreeKey a, b;

            bool
            operator==(const KeyPair &other) const {
                return a == other.a && b == other.b;
            }
        };

        struct KeyPairHash {
            std::size_t
            operator()(const KeyPair &kp) const {
                auto h1 = OctreeKey::KeyHash{}(kp.a);
                auto h2 = OctreeKey::KeyHash{}(kp.b);
                return h1 ^ (h2 * 2654435761u);
            }
        };

        auto make_edge = [](const OctreeKey &a, const OctreeKey &b) -> KeyPair {
            return (a < b) ? KeyPair{a, b} : KeyPair{b, a};
        };

        // edge -> list of quad indices
        absl::flat_hash_map<KeyPair, std::vector<std::size_t>, KeyPairHash> edge_to_quads;
        std::vector<std::array<OctreeKey, 4>> all_corners(quads.size());

        for (std::size_t qi = 0; qi < quads.size(); ++qi) {
            all_corners[qi] = get_corners(quads[qi]);
            const auto &c = all_corners[qi];
            // 4 edges: (c0,c1), (c1,c2), (c2,c3), (c3,c0)
            for (int ei = 0; ei < 4; ++ei) {
                auto edge = make_edge(c[ei], c[(ei + 1) % 4]);
                edge_to_quads[edge].push_back(qi);
            }
        }

        // BFS to find connected components
        std::vector<int> component(quads.size(), -1);
        int num_components = 0;
        std::vector<std::size_t> bfs_queue;

        for (std::size_t qi = 0; qi < quads.size(); ++qi) {
            if (component[qi] >= 0) { continue; }
            const int comp_id = num_components++;
            component[qi] = comp_id;
            bfs_queue.clear();
            bfs_queue.push_back(qi);

            for (std::size_t head = 0; head < bfs_queue.size(); ++head) {
                const std::size_t cur = bfs_queue[head];
                const auto &c = all_corners[cur];
                for (int ei = 0; ei < 4; ++ei) {
                    auto edge = make_edge(c[ei], c[(ei + 1) % 4]);
                    for (const std::size_t neighbor_qi: edge_to_quads[edge]) {
                        if (component[neighbor_qi] < 0) {
                            component[neighbor_qi] = comp_id;
                            bfs_queue.push_back(neighbor_qi);
                        }
                    }
                }
            }
        }

        // --- Step 3: Build indexed meshes per component ---

        std::vector<Frontier> frontiers(num_components);

        // Per-component vertex deduplication maps (OctreeKey -> vertex index)
        std::vector<absl::flat_hash_map<OctreeKey, int>> comp_vertex_map(num_components);

        for (std::size_t qi = 0; qi < quads.size(); ++qi) {
            const int comp_id = component[qi];
            auto &frontier = frontiers[comp_id];
            auto &vertex_map = comp_vertex_map[comp_id];
            const auto &c = all_corners[qi];

            auto get_or_add_vertex = [&](const OctreeKey &k) -> int {
                auto [iter, inserted] =
                    vertex_map.emplace(k, static_cast<int>(frontier.vertices.size()));
                if (inserted) { frontier.vertices.push_back(this->KeyToVertexCoord(k)); }
                return iter->second;
            };

            int v0 = get_or_add_vertex(c[0]);
            int v1 = get_or_add_vertex(c[1]);
            int v2 = get_or_add_vertex(c[2]);
            int v3 = get_or_add_vertex(c[3]);

            frontier.faces.push_back({v0, v1, v2});
            frontier.faces.push_back({v0, v2, v3});
        }

        // Remove empty frontiers and apply min_num_triangles filter
        std::vector<Frontier> result;
        for (auto &f: frontiers) {
            if (f.faces.size() >= min_num_triangles) { result.push_back(std::move(f)); }
        }

        if (sort_by_area) {
            // Compute total surface area per frontier and sort descending
            std::vector<std::pair<std::size_t, Dtype>> index_area(result.size());
            for (std::size_t i = 0; i < result.size(); ++i) {
                Dtype area = 0;
                for (const auto &face: result[i].faces) {
                    const Vector3 &a = result[i].vertices[face[0]];
                    const Vector3 &b = result[i].vertices[face[1]];
                    const Vector3 &c = result[i].vertices[face[2]];
                    area += (b - a).cross(c - a).norm() * static_cast<Dtype>(0.5);
                }
                index_area[i] = {i, area};
            }
            std::sort(index_area.begin(), index_area.end(), [](const auto &a, const auto &b) {
                return a.second > b.second;
            });
            std::vector<Frontier> sorted;
            sorted.reserve(result.size());
            for (const auto &[i, area]: index_area) { sorted.push_back(std::move(result[i])); }
            result = std::move(sorted);
        }

        return result;
    }

    template<typename Dtype, class Node, class Setting>
    std::vector<typename OccupancyOctreeBase<Dtype, Node, Setting>::Frontier>
    OccupancyOctreeBase<Dtype, Node, Setting>::ExtractFrontiers(
        const std::size_t min_num_triangles,
        const bool sort_by_area) const {
        return ExtractFrontiersImpl(
            this->BeginLeaf(),
            this->EndLeaf(),
            min_num_triangles,
            sort_by_area);
    }

    template<typename Dtype, class Node, class Setting>
    std::vector<typename OccupancyOctreeBase<Dtype, Node, Setting>::Frontier>
    OccupancyOctreeBase<Dtype, Node, Setting>::ExtractFrontiers(
        const Dtype aabb_min_x,
        const Dtype aabb_min_y,
        const Dtype aabb_min_z,
        const Dtype aabb_max_x,
        const Dtype aabb_max_y,
        const Dtype aabb_max_z,
        const std::size_t min_num_triangles,
        const bool sort_by_area) const {
        return ExtractFrontiersImpl(
            this->BeginLeafInAabb(
                aabb_min_x,
                aabb_min_y,
                aabb_min_z,
                aabb_max_x,
                aabb_max_y,
                aabb_max_z),
            this->EndLeafInAabb(),
            min_num_triangles,
            sort_by_area);
    }

}  // namespace erl::geometry
