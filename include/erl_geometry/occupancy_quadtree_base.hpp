#pragma once

#include <omp.h>
#include <list>
#include "erl_common/random.hpp"
#include "abstract_occupancy_quadtree.hpp"
#include "occupancy_nd_tree_setting.hpp"
#include "quadtree_impl.hpp"
#include "aabb.hpp"

namespace erl::geometry {

    struct OccupancyQuadtreeBaseSetting : public common::OverrideYamlable<OccupancyNdTreeSetting, OccupancyQuadtreeBaseSetting> {
        bool use_change_detection = false;
        bool use_aabb_limit = false;
        Aabb2D aabb = {};

        inline bool
        operator==(const NdTreeSetting& rhs) const override {
            if (typeid(*this) != typeid(rhs)) return false;
            auto that = reinterpret_cast<const OccupancyQuadtreeBaseSetting&>(rhs);
            return use_change_detection == that.use_change_detection &&  //
                   use_aabb_limit == that.use_aabb_limit &&              //
                   aabb == that.aabb && OccupancyNdTreeSetting::operator==(rhs);
        }
    };

    template<class Node>
    class OccupancyQuadtreeBase : public QuadtreeImpl<Node, AbstractOccupancyQuadtree> {
        static_assert(std::is_base_of_v<OccupancyQuadtreeNode, Node>);

    public:
        using Super = QuadtreeImpl<Node, AbstractOccupancyQuadtree>;
        using Self = OccupancyQuadtreeBase<Node>;

    protected:
        std::shared_ptr<OccupancyQuadtreeBaseSetting> m_setting_ = std::make_shared<OccupancyQuadtreeBaseSetting>();
        QuadtreeKeyBoolMap m_changed_keys_ = {};
        QuadtreeKeyVectorMap m_discrete_end_point_mapping_ = {};  // buffer used for inserting point cloud to track the end points
        QuadtreeKeyVectorMap m_end_point_mapping_ = {};           // buffer used for inserting point cloud to track the end points

    public:
        OccupancyQuadtreeBase() = delete;  // no default constructor

        explicit OccupancyQuadtreeBase(const std::shared_ptr<OccupancyQuadtreeBaseSetting>& setting)
            : QuadtreeImpl<Node, AbstractOccupancyQuadtree>(setting),
              m_setting_(setting) {}

        OccupancyQuadtreeBase(const Self& other) = delete;  // no copy constructor

        //-- implement abstract methods
        inline void
        OnDeleteNodeChild(Node* node, uint32_t /* deleted_child_idx */) override {
            node->SetLogOdds(node->GetMaxChildLogOdds());  // copy log odds from child to parent
        }

        //-- Sample position
        /**
         * Sample positions from the free space.
         */
        void
        SamplePositions(std::size_t num_positions, std::vector<Eigen::Vector2d>& positions) const {
            positions.clear();
            positions.reserve(num_positions);
            double min_x, min_y, max_x, max_y;
            this->GetMetricMinMax(min_x, min_y, max_x, max_y);
            std::uniform_real_distribution<double> uniform_x(min_x, max_x);
            std::uniform_real_distribution<double> uniform_y(min_y, max_y);
            while (positions.size() < num_positions) {
                double x = uniform_x(common::g_random_engine);
                double y = uniform_y(common::g_random_engine);
                const Node* node = this->Search(x, y);
                if (node == nullptr || this->IsNodeOccupied(node)) { continue; }
                positions.emplace_back(x, y);
            }
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
        virtual void
        InsertPointCloud(
            const Eigen::Ref<const Eigen::Matrix2Xd>& points,
            const Eigen::Ref<const Eigen::Vector2d>& sensor_origin,
            double max_range,
            bool parallel,
            bool lazy_eval,
            bool discretize) {

            static QuadtreeKeyVector free_cells, occupied_cells;  // static to avoid memory allocation
            // compute cells to update
            if (discretize) {
                ComputeDiscreteUpdateForPointCloud(points, sensor_origin, max_range, parallel, free_cells, occupied_cells);
            } else {
                ComputeUpdateForPointCloud(points, sensor_origin, max_range, parallel, free_cells, occupied_cells);
            }
            // insert data into tree
            for (QuadtreeKey& free_cell: free_cells) { UpdateNode(free_cell, false, lazy_eval); }
            for (QuadtreeKey& occupied_cell: occupied_cells) { UpdateNode(occupied_cell, true, lazy_eval); }
        }

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
            QuadtreeKeyVector& free_cells,
            QuadtreeKeyVector& occupied_cells) {

            long num_points = points.cols();
            if (num_points == 0) { return; }

            Eigen::Matrix2Xd new_points(2, num_points);
            m_end_point_mapping_.clear();
            for (long i = 0; i < num_points; ++i) {
                const auto& kP = points.col(i);
                QuadtreeKey key = this->CoordToKey(kP[0], kP[1]);
                auto& indices = m_discrete_end_point_mapping_[key];
                if (indices.empty()) { new_points.col(long(m_discrete_end_point_mapping_.size()) - 1) << kP; }  // new end point!
                indices.push_back(i);
            }
            new_points.conservativeResize(2, long(m_discrete_end_point_mapping_.size()));
            ComputeUpdateForPointCloud(new_points, sensor_origin, max_range, parallel, free_cells, occupied_cells);
        }

        void
        ComputeUpdateForPointCloud(
            const Eigen::Ref<const Eigen::Matrix2Xd>& points,
            const Eigen::Ref<const Eigen::Vector2d>& sensor_origin,
            double max_range,
            bool parallel,
            QuadtreeKeyVector& free_cells,
            QuadtreeKeyVector& occupied_cells) {

            long num_points = points.cols();
            if (num_points == 0) { return; }

            static QuadtreeKeySet free_cells_set;  // static to avoid memory allocation

            free_cells_set.clear();
            m_end_point_mapping_.clear();
            free_cells.clear();
            occupied_cells.clear();

            std::vector<double> ranges(num_points);
            std::vector<std::array<double, 2>> diffs(num_points);
            omp_set_num_threads(this->m_key_rays_.size());

            // insert occupied endpoint
            bool& use_aabb_limit = m_setting_->use_aabb_limit;
            Aabb2D& aabb = m_setting_->aabb;
            for (long i = 0; i < num_points; ++i) {
                const auto& kP = points.col(i);

                double& dx = diffs[i][0];
                double& dy = diffs[i][1];
                double& range = ranges[i];

                dx = kP[0] - sensor_origin[0];
                dy = kP[1] - sensor_origin[1];
                range = std::sqrt(dx * dx + dy * dy);

                QuadtreeKey key;
                if (use_aabb_limit) {                                                     // bounding box is specified
                    if ((aabb.contains(kP) && (max_range < 0. || range <= max_range)) &&  // inside bounding box and range limit
                        this->CoordToKeyChecked(kP[0], kP[1], key)) {                     // key is valid
                        auto& indices = m_end_point_mapping_[key];
                        if (indices.empty()) { occupied_cells.push_back(key); }  // new key!
                        indices.push_back(i);
                    }
                } else {
                    if ((max_range < 0. || (range <= max_range)) &&    // range limit
                        this->CoordToKeyChecked(kP[0], kP[1], key)) {  // key is valid
                        auto& indices = m_end_point_mapping_[key];
                        if (indices.empty()) { occupied_cells.push_back(key); }  // new key!
                        indices.push_back(i);
                    }
                }
            }

            const double& sx = sensor_origin[0];
            const double& sy = sensor_origin[1];

            // insert free cells
#pragma omp parallel for if (parallel) default(none) shared(num_points, points, sensor_origin, max_range, sx, sy, ranges, diffs, free_cells, free_cells_set)
            for (long i = 0; i < num_points; ++i) {
                const auto& kP = points.col(i);
                uint32_t thread_idx = omp_get_thread_num();
                QuadtreeKeyRay& key_ray = this->m_key_rays_[thread_idx];

                double& dx = diffs[i][0];
                double& dy = diffs[i][1];
                double& range = ranges[i];

                double ex = kP[0];
                double ey = kP[1];
                if ((max_range >= 0.) && (range > max_range)) {  // crop ray at max_range
                    double r = max_range / range;
                    ex = sx + dx * r;
                    ey = sy + dy * r;
                }

                if (!this->ComputeRayKeys(sx, sy, ex, ey, key_ray)) { continue; }  // key is invalid
#pragma omp critical(free_insert)
                {
                    for (auto& key: key_ray) {
                        if (m_end_point_mapping_.find(key) != m_end_point_mapping_.end()) { continue; }  // skip keys marked as occupied
                        auto it = free_cells_set.emplace(key);
                        if (it.second) { free_cells.push_back(key); }
                    }
                }
            }
        }

        /**
         * Insert a point cloud ray by ray. Some cells may be updated multiple times.
         * @param points 2xN matrix of ray end points in the world frame.
         * @param sensor_origin 2D vector of the sensor origin in the world frame.
         * @param max_range maximum range of the sensor. Points beyond this range are ignored. Non-positive value means no limit.
         * @param parallel whether to use parallel computation
         * @param lazy_eval whether to update the occupancy of the nodes immediately. If true, the occupancy is not updated until UpdateInnerOccupancy() is
         * called.
         */
        virtual void
        InsertPointCloudRays(
            const Eigen::Ref<const Eigen::Matrix2Xd>& points,
            const Eigen::Ref<const Eigen::Vector2d>& sensor_origin,
            double max_range,
            bool parallel,
            bool lazy_eval) {

            long num_points = points.cols();
            if (num_points == 0) { return; }

            omp_set_num_threads(this->m_key_rays_.size());
#pragma omp parallel for if (parallel) default(none) shared(num_points, points, sensor_origin, max_range, lazy_eval) schedule(guided)
            for (long i = 0; i < num_points; ++i) {
                const auto& kP = points.col(i);
                uint32_t thread_idx = omp_get_thread_num();
                QuadtreeKeyRay& key_ray = this->m_key_rays_[thread_idx];
                if (!this->ComputeRayKeys(sensor_origin[0], sensor_origin[1], kP[0], kP[1], key_ray)) { continue; }

#pragma omp critical
                {
                    for (auto& key: key_ray) { UpdateNode(key, false, lazy_eval); }
                    double range = (kP - sensor_origin).norm();
                    if (max_range <= 0. || (max_range > 0. && range <= max_range)) { UpdateNode(kP[0], kP[1], true, lazy_eval); }
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
        virtual bool
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
        void
        CastRays(
            const Eigen::Ref<const Eigen::Vector2d>& position,
            const Eigen::Ref<const Eigen::Matrix2d>& rotation,
            const Eigen::Ref<const Eigen::VectorXd>& angles,
            bool ignore_unknown,
            double max_range,
            bool prune_rays,
            bool parallel,
            std::vector<long>& hit_ray_indices,
            std::vector<Eigen::Vector2d>& hit_positions,
            std::vector<const Node*>& hit_nodes,
            std::vector<uint32_t>& node_depths) const {

            if (angles.size() == 0) { return; }
            long num_rays = angles.size();

            hit_ray_indices.clear();
            hit_positions.clear();
            hit_nodes.clear();
            node_depths.clear();

            hit_ray_indices.resize(num_rays);
            hit_positions.resize(num_rays);
            hit_nodes.resize(num_rays);
            node_depths.resize(num_rays);

#pragma omp parallel for if (parallel) default(none) \
    shared(num_rays, position, rotation, angles, ignore_unknown, max_range, hit_ray_indices, hit_positions, hit_nodes, node_depths)
            for (long i = 0; i < num_rays; ++i) {
                const double& kAngle = angles[i];
                Eigen::Vector2d direction(std::cos(kAngle), std::sin(kAngle));
                direction = rotation * direction;
                Eigen::Vector2d& hit_position = hit_positions[i];
                hit_nodes[i] = this->CastRay(
                    position[0],
                    position[1],
                    direction[0],
                    direction[1],
                    ignore_unknown,
                    max_range,
                    hit_position[0],
                    hit_position[1],
                    node_depths[i]);
            }

            absl::flat_hash_set<const Node*> hit_nodes_set;

            std::vector<long> filtered_hit_ray_indices;
            std::vector<Eigen::Vector2d> filtered_hit_positions;
            std::vector<const Node*> filtered_hit_nodes;
            std::vector<uint32_t> filtered_node_depths;

            filtered_hit_ray_indices.reserve(num_rays);
            filtered_hit_positions.reserve(num_rays);
            filtered_hit_nodes.reserve(num_rays);
            filtered_node_depths.reserve(num_rays);

            // remove rays that hit nothing or hit the same node if prune_rays is true
            for (long i = 0; i < num_rays; ++i) {
                const Node*& hit_node = hit_nodes[i];
                if (hit_node == nullptr) { continue; }
                if (!prune_rays || hit_nodes_set.insert(hit_node).second) {
                    filtered_hit_ray_indices.push_back(hit_ray_indices[i]);
                    filtered_hit_positions.push_back(hit_positions[i]);
                    filtered_hit_nodes.push_back(hit_node);
                    filtered_node_depths.push_back(node_depths[i]);
                }
            }

            filtered_hit_ray_indices.shrink_to_fit();
            filtered_hit_positions.shrink_to_fit();
            filtered_hit_nodes.shrink_to_fit();
            filtered_node_depths.shrink_to_fit();

            std::swap(hit_ray_indices, filtered_hit_ray_indices);
            std::swap(hit_positions, filtered_hit_positions);
            std::swap(hit_nodes, filtered_hit_nodes);
            std::swap(node_depths, filtered_node_depths);
        }

        void
        CastRays(
            const Eigen::Ref<const Eigen::Matrix2Xd>& positions,
            const Eigen::Ref<const Eigen::Matrix2Xd>& directions,
            bool ignore_unknown,
            double max_range,
            bool prune_rays,
            bool parallel,
            std::vector<long>& hit_ray_indices,
            std::vector<Eigen::Vector2d>& hit_positions,
            std::vector<const Node*>& hit_nodes,
            std::vector<uint32_t>& node_depths) const {
            long num_rays = 0;
            if (positions.cols() != 1 && directions.cols() != 1) {
                ERL_ASSERTM(positions.cols() == directions.cols(), "positions.cols() != directions.cols() when both are not 1.");
                num_rays = positions.cols();
                hit_positions.resize(num_rays);
                hit_nodes.resize(num_rays, nullptr);
                node_depths.resize(num_rays, 0);
#pragma omp parallel for if (parallel) default(none) shared(num_rays, positions, directions, ignore_unknown, max_range, hit_positions, hit_nodes, node_depths)
                for (long i = 0; i < num_rays; ++i) {
                    Eigen::Vector2d& hit_position = hit_positions[i];
                    hit_nodes[i] = CastRay(
                        positions(0, i),
                        positions(1, i),
                        directions(0, i),
                        directions(1, i),
                        ignore_unknown,
                        max_range,
                        hit_position[0],
                        hit_position[1],
                        node_depths[i]);
                }
            }
            if (positions.cols() == 1) {
                num_rays = directions.cols();
                hit_positions.resize(num_rays);
                hit_nodes.resize(num_rays, nullptr);
                node_depths.resize(num_rays, 0);
#pragma omp parallel for if (parallel) default(none) shared(num_rays, positions, directions, ignore_unknown, max_range, hit_positions, hit_nodes, node_depths)
                for (long i = 0; i < num_rays; ++i) {
                    Eigen::Vector2d& hit_position = hit_positions[i];
                    hit_nodes[i] = CastRay(
                        positions(0, 0),
                        positions(1, 0),
                        directions(0, i),
                        directions(1, i),
                        ignore_unknown,
                        max_range,
                        hit_position[0],
                        hit_position[1],
                        node_depths[i]);
                }
            }
            if (directions.cols() == 1) {
                num_rays = positions.cols();
                hit_positions.resize(num_rays);
                hit_nodes.resize(num_rays, nullptr);
                node_depths.resize(num_rays, 0);
#pragma omp parallel for if (parallel) default(none) shared(num_rays, positions, directions, ignore_unknown, max_range, hit_positions, hit_nodes, node_depths)
                for (long i = 0; i < num_rays; ++i) {
                    Eigen::Vector2d& hit_position = hit_positions[i];
                    hit_nodes[i] = CastRay(
                        positions(0, i),
                        positions(1, i),
                        directions(0, 0),
                        directions(1, 0),
                        ignore_unknown,
                        max_range,
                        hit_position[0],
                        hit_position[1],
                        node_depths[i]);
                }
            }

            if (num_rays == 0) { return; }

            absl::flat_hash_set<const Node*> hit_nodes_set;

            std::vector<long> filtered_hit_ray_indices;
            std::vector<Eigen::Vector2d> filtered_hit_positions;
            std::vector<const Node*> filtered_hit_nodes;
            std::vector<uint32_t> filtered_node_depths;

            filtered_hit_ray_indices.reserve(num_rays);
            filtered_hit_positions.reserve(num_rays);
            filtered_hit_nodes.reserve(num_rays);
            filtered_node_depths.reserve(num_rays);

            // remove rays that hit nothing or hit the same node if prune_rays is true
            for (long i = 0; i < num_rays; ++i) {
                const Node*& hit_node = hit_nodes[i];
                if (hit_node == nullptr) { continue; }
                if (!prune_rays || hit_nodes_set.insert(hit_node).second) {
                    filtered_hit_ray_indices.push_back(i);
                    filtered_hit_positions.push_back(hit_positions[i]);
                    filtered_hit_nodes.push_back(hit_node);
                    filtered_node_depths.push_back(node_depths[i]);
                }
            }

            filtered_hit_ray_indices.shrink_to_fit();
            filtered_hit_positions.shrink_to_fit();
            filtered_hit_nodes.shrink_to_fit();
            filtered_node_depths.shrink_to_fit();

            std::swap(hit_ray_indices, filtered_hit_ray_indices);
            std::swap(hit_positions, filtered_hit_positions);
            std::swap(hit_nodes, filtered_hit_nodes);
            std::swap(node_depths, filtered_node_depths);
        }

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
         * @return node pointer if the ray hits an occupied cell, nullptr otherwise.
         */
        const Node*
        CastRay(double px, double py, double vx, double vy, bool ignore_unknown, double max_range, double& ex, double& ey, uint32_t& depth) const {
            // Similar to QuadtreeImpl::ComputeRayKeys, but with extra hitting checks

            QuadtreeKey current_key;
            if (!this->CoordToKeyChecked(px, py, current_key)) {
                ERL_WARN("Ray starting from (%f, %f) is out of range.\n", px, py);
                return nullptr;
            }

            // initialization
            const Node* starting_node = this->Search(current_key, depth);
            if (starting_node != nullptr) {
                if (this->IsNodeOccupied(starting_node)) {  // (px, py) is in occupied
                    this->KeyToCoord(current_key, ex, ey);
                    return starting_node;
                }
            } else if (!ignore_unknown) {  // (px, py) is in unknown
                this->KeyToCoord(current_key, ex, ey);
                return nullptr;
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
                return nullptr;
            }

            // compute t_max and t_delta
            auto& resolution = m_setting_->resolution;
            double t_max[2];
            double t_delta[2];
            if (step[0] == 0) {
                t_max[0] = std::numeric_limits<double>::infinity();
                t_delta[0] = std::numeric_limits<double>::infinity();
            } else {
                double voxel_border = this->KeyToCoord(current_key[0]) + double(step[0]) * 0.5 * resolution;
                t_max[0] = (voxel_border - px) / vx;
                t_delta[0] = resolution / std::abs(vx);
            }
            if (step[1] == 0) {
                t_max[1] = std::numeric_limits<double>::infinity();
                t_delta[1] = std::numeric_limits<double>::infinity();
            } else {
                double voxel_border = this->KeyToCoord(current_key[1]) + double(step[1]) * 0.5 * resolution;
                t_max[1] = (voxel_border - py) / vy;
                t_delta[1] = resolution / std::abs(vy);
            }

            // incremental phase
            double max_range_sq = max_range * max_range;
            long max_key_val = (this->m_tree_key_offset_ << 1) - 1;
            while (true) {
                if (t_max[0] < t_max[1]) {
                    t_max[0] += t_delta[0];
                    long next_key_val = long(current_key[0]) + step[0];
                    // check overflow
                    if ((step[0] < 0 && next_key_val <= 0) || (step[0] > 0 && next_key_val >= max_key_val)) {
                        ERL_DEBUG("x coordinate hits boundary, aborting ray cast.");
                        current_key[0] = next_key_val < 0 ? 0 : max_key_val;  // set to boundary
                        this->KeyToCoord(current_key, ex, ey);
                        return nullptr;
                    } else {
                        current_key[0] = next_key_val;
                    }
                } else {
                    t_max[1] += t_delta[1];
                    long next_key_val = long(current_key[1]) + step[1];
                    // check overflow
                    if ((step[1] < 0 && next_key_val <= 0) || (step[1] > 0 && next_key_val >= max_key_val)) {
                        ERL_DEBUG("y coordinate hits boundary, aborting ray cast.");
                        current_key[1] = next_key_val < 0 ? 0 : max_key_val;
                        this->KeyToCoord(current_key, ex, ey);
                        return nullptr;
                    } else {
                        current_key[1] = next_key_val;
                    }
                }

                // generate world coordinates from key
                this->KeyToCoord(current_key, ex, ey);
                // check if max_range is reached
                if (max_range_set) {
                    double dx = ex - px;
                    double dy = ey - py;
                    if ((dx * dx + dy * dy) > max_range_sq) { return nullptr; }
                }
                // search node of the new key
                depth = 0;
                const Node* current_node = this->Search(current_key, depth);
                if (current_node != nullptr) {
                    if (this->IsNodeOccupied(current_node)) { return current_node; }
                } else if (!ignore_unknown) {
                    return nullptr;
                }
            }
        }

        //-- trace ray
        class OccupiedLeafOnRayIterator : public QuadtreeImpl<Node, AbstractOccupancyQuadtree>::LeafOnRayIterator {
            using Super = typename QuadtreeImpl<Node, AbstractOccupancyQuadtree>::LeafOnRayIterator;

        public:
            OccupiedLeafOnRayIterator() = default;

            OccupiedLeafOnRayIterator(
                double px,
                double py,
                double vx,
                double vy,
                double max_range,
                bool bidirectional,
                const Self* tree,
                uint32_t max_leaf_depth)
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
        BeginOccupiedLeafOnRay(double px, double py, double vx, double vy, double max_range = -1, bool bidirectional = false, uint32_t max_leaf_depth = 0)
            const {
            return OccupiedLeafOnRayIterator(px, py, vx, vy, max_range, bidirectional, this, max_leaf_depth);
        }

        [[nodiscard]] inline OccupiedLeafOnRayIterator
        EndOccupiedLeafOnRay() const {
            return OccupiedLeafOnRayIterator();
        }

        [[nodiscard]] inline QuadtreeKeyBoolMap::const_iterator
        BeginChangedKey() const {
            if (!m_setting_->use_change_detection) {
                ERL_WARN("use_change_detection is false in setting. No changes are tracked.");
                return m_changed_keys_.end();
            }
            return m_changed_keys_.begin();
        }

        [[nodiscard]] inline QuadtreeKeyBoolMap::const_iterator
        EndChangedKey() const {
            return m_changed_keys_.end();
        }

        inline void
        ClearChangedKeys() {
            m_changed_keys_.clear();
        }

        //-- update nodes' occupancy
        /**
         * Update the node at the given key with the given log-odds delta.
         * @param key of the node to update
         * @param log_odds_delta to be added to the node's log-odds value
         * @param lazy_eval whether update of inner nodes is omitted and only leaf nodes are updated. This speeds up the intersection, but you need to call
         * UpdateInnerOccupancy() after all updates are done.
         * @return
         */
        inline Node*
        UpdateNode(double x, double y, bool occupied, bool lazy_eval) {
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
        inline Node*
        UpdateNode(const QuadtreeKey& key, bool occupied, bool lazy_eval) {
            float log_odds_delta = occupied ? m_setting_->log_odd_hit : m_setting_->log_odd_miss;
            return UpdateNode(key, log_odds_delta, lazy_eval);
        }

        inline Node*
        UpdateNode(double x, double y, float log_odds_delta, bool lazy_eval) {
            QuadtreeKey key;
            if (!this->CoordToKeyChecked(x, y, key)) { return nullptr; }
            return UpdateNode(key, log_odds_delta, lazy_eval);
        }

        Node*
        UpdateNode(const QuadtreeKey& key, float log_odds_delta, bool lazy_eval) {
            auto leaf = static_cast<Node*>(this->Search(key));
            auto log_odds_delta_ = double(log_odds_delta);
            // early abort, no change will happen: node already at threshold or its log-odds is locked.
            if (leaf) {
                if (!leaf->AllowUpdateLogOdds(log_odds_delta_)) { return leaf; }
                if (log_odds_delta_ >= 0 && leaf->GetLogOdds() >= m_setting_->log_odd_max) { return leaf; }
                if (log_odds_delta_ <= 0 && leaf->GetLogOdds() <= m_setting_->log_odd_min) { return leaf; }
            }

            bool create_root = this->m_root_ == nullptr;
            if (create_root) {
                this->m_root_ = std::make_shared<Node>();
                this->m_tree_size_++;
            }
            return static_cast<Node*>(UpdateNodeRecurs(this->m_root_.get(), create_root, key, 0, log_odds_delta_, lazy_eval));
        }

    private:
        Node*
        UpdateNodeRecurs(Node* node, bool node_just_created, const QuadtreeKey& key, uint32_t depth, double log_odds_delta, bool lazy_eval) {
            bool created_node = false;
            ERL_DEBUG_ASSERT(node != nullptr, "node is nullptr.");

            auto& tree_depth = m_setting_->tree_depth;
            if (depth < tree_depth) {  // follow down to last level
                int pos = QuadtreeKey::ComputeChildIndex(key, tree_depth - 1 - depth);
                if (!node->HasChild(pos)) {                            // child node does not exist
                    if (!node->HasAnyChild() && !node_just_created) {  // current node has no child and is not new
                        this->ExpandNode(node);                        // expand pruned node
                    } else {
                        this->CreateNodeChild(node, pos);
                        created_node = true;
                    }
                }

                if (lazy_eval) {
                    return UpdateNodeRecurs(this->GetNodeChild(node, pos), created_node, key, depth + 1, log_odds_delta, lazy_eval);
                } else {
                    Node* returned_node = UpdateNodeRecurs(this->GetNodeChild(node, pos), created_node, key, depth + 1, log_odds_delta, lazy_eval);
                    if (this->PruneNode(node)) {
                        returned_node = node;  // returned_node is pruned, return its parent instead
                    } else {
                        UpdateInnerNodeOccupancy(node);
                    }
                    return returned_node;
                }
            } else {  // last level
                if (m_setting_->use_change_detection) {
                    bool occ_before = this->IsNodeOccupied(node);
                    UpdateNodeLogOdds(node, log_odds_delta);
                    if (node_just_created) {
                        m_changed_keys_.emplace(key, true);
                        // m_changed_keys_.insert(std::make_pair(key, true));
                    } else if (occ_before != this->IsNodeOccupied(node)) {  // occupancy changed, track it
                        auto it = m_changed_keys_.find(key);
                        if (it == m_changed_keys_.end()) {  // not found
                            m_changed_keys_.emplace(key, false);
                            // m_changed_keys_.insert(std::make_pair(key, false));
                        } else if (!it->second) {
                            m_changed_keys_.erase(it);
                        }
                    }
                } else {
                    UpdateNodeLogOdds(node, log_odds_delta);
                }
                return node;
            }
        }

    protected:
        void
        UpdateNodeLogOdds(Node* node, float log_odd_delta) {
            node->AddLogOdds(log_odd_delta);
            float l = node->GetLogOdds();
            auto& log_odd_min = m_setting_->log_odd_min;
            auto& log_odd_max = m_setting_->log_odd_max;
            if (l < log_odd_min) {
                node->SetLogOdds(log_odd_min);
                return;
            }

            if (l > log_odd_max) {
                node->SetLogOdds(log_odd_max);
                return;
            }
        }

    public:
        void
        UpdateInnerOccupancy() {
            if (this->m_root_ == nullptr) { return; }
            UpdateInnerOccupancyRecurs(this->m_root_.get(), 0);
        }

    protected:
        void
        UpdateInnerOccupancyRecurs(Node* node, uint32_t depth) {
            ERL_DEBUG_ASSERT(node != nullptr, "node is nullptr.");
            if (!node->HasAnyChild()) { return; }
            // only recurse and update for inner nodes
            if (depth < m_setting_->tree_depth) {
                for (int i = 0; i < 4; ++i) {
                    Node* child = this->GetNodeChild(node, i);
                    if (child == nullptr) { continue; }
                    UpdateInnerOccupancyRecurs(child, depth + 1);
                }
            }
            UpdateInnerNodeOccupancy(node);
        }

        inline void
        UpdateInnerNodeOccupancy(Node* node) {
            node->SetLogOdds(node->GetMaxChildLogOdds());
        }

    public:
        /**
         * Set all nodes' log odds according to their current max likelihood of occupancy.
         */
        void
        ToMaxLikelihood() override {
            if (this->m_root_ == nullptr) { return; }
            std::list<Node*> stack;
            stack.emplace_back(static_cast<Node*>(this->m_root_.get()));
            auto& log_odd_min = m_setting_->log_odd_min;
            auto& log_odd_max = m_setting_->log_odd_max;
            while (!stack.empty()) {
                Node* node = stack.back();
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

        //--file IO
        std::istream&
        ReadBinaryData(std::istream& s) override {
            if (this->m_root_ != nullptr) {
                ERL_WARN("Trying to read into an existing tree.");
                return s;
            }

            this->m_root_ = std::make_shared<Node>();
            this->m_tree_size_ = 1;
            char child_record;

            std::list<std::pair<Node*, bool>> stack;  // node, is_new_node
            stack.emplace_back(this->m_root_.get(), true);

            while (!stack.empty()) {
                auto& top = stack.back();
                Node* node = top.first;
                bool& is_new_node = top.second;

                if (!is_new_node) {
                    node->SetLogOdds(node->GetMaxChildLogOdds());
                    stack.pop_back();
                    continue;
                }

                is_new_node = false;
                s.read(&child_record, sizeof(char));
                std::bitset<8> child((unsigned long long) child_record);
                bool has_inner_node_child = false;
                for (int i = 3; i >= 0; --i) {
                    // 0b10: free leaf
                    // 0b01: occupied leaf
                    // 0b11: inner node
                    bool bit0 = child[i * 2];
                    bool bit1 = child[i * 2 + 1];
                    Node* child_node = nullptr;
                    if (bit0) {
                        if (bit1) {  // 0b11, inner node
                            child_node = this->CreateNodeChild(node, i);
                            child_node->SetLogOdds(-200);
                            has_inner_node_child = true;
                            stack.emplace_back(child_node, true);
                        } else {  // 0b01, occupied leaf
                            child_node = this->CreateNodeChild(node, i);
                            child_node->SetLogOdds(m_setting_->log_odd_max);
                        }
                    } else if (bit1) {  // 0b10, free leaf
                        child_node = this->CreateNodeChild(node, i);
                        child_node->SetLogOdds(m_setting_->log_odd_min);
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

            std::list<const Node*> nodes_stack;  // node
            nodes_stack.push_back(this->m_root_.get());

            while (!nodes_stack.empty()) {
                const Node* node = nodes_stack.back();
                nodes_stack.pop_back();

                std::bitset<8> child;
                for (int i = 3; i >= 0; --i) {
                    const Node* child_node = this->GetNodeChild(node, i);
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
                char child_record = (char) child.to_ulong();
                s.write(&child_record, sizeof(char));
            }

            return s;
        }
    };
}  // namespace erl::geometry

namespace YAML {
    template<>
    struct convert<erl::geometry::OccupancyQuadtreeBaseSetting> {
        inline static Node
        encode(const erl::geometry::OccupancyQuadtreeBaseSetting& rhs) {
            Node node = convert<erl::geometry::OccupancyNdTreeSetting>::encode(rhs);
            node["use_change_detection"] = rhs.use_change_detection;
            node["use_aabb_limit"] = rhs.use_aabb_limit;
            node["aabb"] = rhs.aabb;
            return node;
        }

        inline static bool
        decode(const Node& node, erl::geometry::OccupancyQuadtreeBaseSetting& rhs) {
            if (!node.IsMap()) { return false; }
            if (!convert<erl::geometry::OccupancyNdTreeSetting>::decode(node, rhs)) { return false; }
            rhs.use_change_detection = node["use_change_detection"].as<bool>();
            rhs.use_aabb_limit = node["use_aabb_limit"].as<bool>();
            rhs.aabb = node["aabb"].as<erl::geometry::Aabb2D>();
            return true;
        }
    };
}  // namespace YAML
