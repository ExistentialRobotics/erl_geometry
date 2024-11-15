#pragma once

#include "aabb.hpp"
#include "abstract_occupancy_octree.hpp"
#include "occupancy_nd_tree_batch_ray_caster.hpp"
#include "occupancy_nd_tree_setting.hpp"
#include "octree_impl.hpp"

#include "erl_common/random.hpp"

#include <omp.h>

#include <list>

namespace erl::geometry {

    struct OccupancyOctreeBaseSetting : public common::Yamlable<OccupancyOctreeBaseSetting, OccupancyNdTreeSetting> {
        bool use_change_detection = false;
        bool use_aabb_limit = false;
        Aabb3D aabb = {};

        bool
        operator==(const NdTreeSetting& rhs) const override {
            if (OccupancyNdTreeSetting::operator==(rhs)) {
                const auto that = reinterpret_cast<const OccupancyOctreeBaseSetting&>(rhs);
                return use_change_detection == that.use_change_detection &&  //
                       use_aabb_limit == that.use_aabb_limit &&              //
                       aabb == that.aabb;
            }
            return false;
        }
    };

    ERL_REGISTER_YAMLABLE(OccupancyOctreeBaseSetting);

    template<class Node, class Setting>
    class OccupancyOctreeBase : public OctreeImpl<Node, AbstractOccupancyOctree, Setting> {
        static_assert(std::is_base_of_v<OccupancyOctreeNode, Node>);
        static_assert(std::is_base_of_v<OccupancyOctreeBaseSetting, Setting>);

        std::shared_ptr<OccupancyOctreeBaseSetting> m_setting_ = nullptr;

    protected:
        OctreeKeyBoolMap m_changed_keys_ = {};
        OctreeKeyVectorMap m_discrete_end_point_mapping_ = {};  // buffer used for inserting point cloud to track the end points
        OctreeKeyVectorMap m_end_point_mapping_ = {};           // buffer used for inserting point cloud to track the end points

    public:
        OccupancyOctreeBase() = delete;  // no default constructor

        explicit OccupancyOctreeBase(const std::shared_ptr<Setting>& setting)
            : OctreeImpl<Node, AbstractOccupancyOctree, Setting>(setting),
              m_setting_(std::static_pointer_cast<OccupancyOctreeBaseSetting>(setting)) {}

        OccupancyOctreeBase(const OccupancyOctreeBase& other) = default;
        OccupancyOctreeBase&
        operator=(const OccupancyOctreeBase& other) = default;
        OccupancyOctreeBase(OccupancyOctreeBase&& other) noexcept = default;
        OccupancyOctreeBase&
        operator=(OccupancyOctreeBase&& other) noexcept = default;

        [[nodiscard]] std::shared_ptr<AbstractOctree>
        Clone() const override {
            std::shared_ptr<AbstractOctree> tree = OctreeImpl<Node, AbstractOccupancyOctree, Setting>::Clone();
            std::shared_ptr<OccupancyOctreeBase> occupancy_tree = std::dynamic_pointer_cast<OccupancyOctreeBase>(tree);
            occupancy_tree->m_changed_keys_ = m_changed_keys_;
            occupancy_tree->m_discrete_end_point_mapping_ = m_discrete_end_point_mapping_;
            occupancy_tree->m_end_point_mapping_ = m_end_point_mapping_;
            return tree;
        }

        //-- implement abstract methods
        void
        OnDeleteNodeChild(Node* node, Node* child, const OctreeKey& /*key*/) override {
            node->SetLogOdds(std::max(node->GetLogOdds(), child->GetLogOdds()));  // update log odds
        }

        //-- Sample position
        /**
         * Sample positions from the free space.
         */
        void
        SamplePositions(const std::size_t num_positions, std::vector<Eigen::Vector3d>& positions) const {
            positions.clear();
            positions.reserve(num_positions);
            double min_x, min_y, min_z, max_x, max_y, max_z;
            this->GetMetricMinMax(min_x, min_y, min_z, max_x, max_y, max_z);
            std::uniform_real_distribution<double> uniform_x(min_x, max_x);
            std::uniform_real_distribution<double> uniform_y(min_y, max_y);
            std::uniform_real_distribution<double> uniform_z(min_z, max_z);
            while (positions.size() < num_positions) {
                double x = uniform_x(common::g_random_engine);
                double y = uniform_y(common::g_random_engine);
                double z = uniform_z(common::g_random_engine);
                const Node* node = this->Search(x, y, z);
                if (node == nullptr || this->IsNodeOccupied(node)) { continue; }
                positions.emplace_back(x, y, z);
            }
        }

        //-- insert point cloud
        /**
         * Insert a point cloud in the world frame. Multiple points may fall into the same voxel that is updated only once, and occupied nodes are preferred
         * than free ones. This avoids holes and is more efficient than the plain ray insertion of InsertPointCloudRays().
         * @param points 3xN matrix of points in the world frame
         * @param sensor_origin 3D vector of the sensor origin in the world frame
         * @param max_range maximum range of the sensor. Points beyond this range are ignored. Non-positive value means no limit.
         * @param parallel whether to use parallel computation
         * @param lazy_eval whether to update the occupancy of the nodes later. If true, the occupancy is not updated until UpdateInnerOccupancy() is called.
         * @param discretize
         */
        virtual void
        InsertPointCloud(
            const Eigen::Ref<const Eigen::Matrix3Xd>& points,
            const Eigen::Ref<const Eigen::Vector3d>& sensor_origin,
            const double max_range,
            const bool parallel,
            const bool lazy_eval,
            const bool discretize) {
            static OctreeKeyVector free_cells, occupied_cells;  // static to avoid memory allocation
            // compute cells to update
            if (discretize) {
                this->ComputeDiscreteUpdateForPointCloud(points, sensor_origin, max_range, parallel, free_cells, occupied_cells);
            } else {
                this->ComputeUpdateForPointCloud(points, sensor_origin, max_range, parallel, free_cells, occupied_cells);
            }
            // insert data into tree
            for (const OctreeKey& free_cell: free_cells) { this->UpdateNode(free_cell, false, lazy_eval); }
            for (const OctreeKey& occupied_cell: occupied_cells) { this->UpdateNode(occupied_cell, true, lazy_eval); }
        }

        /**
         * Compute keys of the cells to update for a point cloud up to the resolution.
         * @param points 3xN matrix of points in the world frame, points falling into the same voxel are merged to the first appearance.
         * @param sensor_origin 3D vector of the sensor origin in the world frame
         * @param max_range maximum range of the sensor. Points beyond this range are ignored. Non-positive value means no limit.
         * @param parallel whether to use parallel computation
         * @param free_cells keys of the free cells to update
         * @param occupied_cells keys of the occupied cells to update
         */
        void
        ComputeDiscreteUpdateForPointCloud(
            const Eigen::Ref<const Eigen::Matrix3Xd>& points,
            const Eigen::Ref<const Eigen::Vector3d>& sensor_origin,
            const double max_range,
            const bool parallel,
            OctreeKeyVector& free_cells,
            OctreeKeyVector& occupied_cells) {

            const long num_points = points.cols();
            if (num_points == 0) { return; }

            Eigen::Matrix3Xd new_points(3, num_points);
            m_discrete_end_point_mapping_.clear();
            for (long i = 0; i < num_points; ++i) {
                const auto& point = points.col(i);
                OctreeKey key = this->CoordToKey(point[0], point[1], point[2]);
                auto& indices = m_discrete_end_point_mapping_[key];
                if (indices.empty()) { new_points.col(static_cast<long>(m_discrete_end_point_mapping_.size()) - 1) << point; }  // new end point!
                indices.push_back(i);
            }
            new_points.conservativeResize(3, static_cast<long>(m_discrete_end_point_mapping_.size()));
            this->ComputeUpdateForPointCloud(new_points, sensor_origin, max_range, parallel, free_cells, occupied_cells);
        }

        void
        ComputeUpdateForPointCloud(
            const Eigen::Ref<const Eigen::Matrix3Xd>& points,
            const Eigen::Ref<const Eigen::Vector3d>& sensor_origin,
            const double max_range,
            const bool parallel,
            OctreeKeyVector& free_cells,
            OctreeKeyVector& occupied_cells) {

            const long num_points = points.cols();
            if (num_points == 0) { return; }

            static OctreeKeySet free_cells_set;  // static to avoid memory allocation

            free_cells_set.clear();
            m_end_point_mapping_.clear();
            free_cells.clear();
            occupied_cells.clear();

            std::vector<double> ranges(num_points);
            std::vector<std::array<double, 3>> diffs(num_points);
            omp_set_num_threads(this->m_key_rays_.size());

            // insert occupied endpoint
            const bool aabb_limit = m_setting_->use_aabb_limit;
            const Aabb3D& aabb = m_setting_->aabb;
            for (long i = 0; i < num_points; ++i) {
                const auto& point = points.col(i);

                double& dx = diffs[i][0];
                double& dy = diffs[i][1];
                double& dz = diffs[i][2];
                double& range = ranges[i];

                dx = point[0] - sensor_origin[0];
                dy = point[1] - sensor_origin[1];
                dz = point[2] - sensor_origin[2];
                range = std::sqrt(dx * dx + dy * dy + dz * dz);

                OctreeKey key;
                if (aabb_limit) {                                                            // bounding box is specified
                    if ((aabb.contains(point) && (max_range < 0. || range <= max_range)) &&  // inside bounding box and range limit
                        this->CoordToKeyChecked(point[0], point[1], point[2], key)) {        // key is valid
                        auto& indices = m_end_point_mapping_[key];
                        if (indices.empty()) { occupied_cells.push_back(key); }  // new key!
                        indices.push_back(i);
                    }
                } else {
                    if ((max_range < 0. || (range <= max_range)) &&                    // range limit
                        this->CoordToKeyChecked(point[0], point[1], point[2], key)) {  // key is valid
                        auto& indices = m_end_point_mapping_[key];
                        if (indices.empty()) { occupied_cells.push_back(key); }  // new key!
                        indices.push_back(i);
                    }
                }
            }

            const double& sx = sensor_origin[0];
            const double& sy = sensor_origin[1];
            const double& sz = sensor_origin[2];

            // we may need to handle thousands of rays, it is inefficient to create a key ray buffer for each ray
            // which may cause memory allocation and deallocation frequently
            // insert free cells
#pragma omp parallel for if (parallel) default(none) shared(num_points, points, sensor_origin, max_range, sx, sy, sz, ranges, diffs, free_cells, free_cells_set)
            for (long i = 0; i < num_points; ++i) {
                const auto& point = points.col(i);
                uint32_t thread_idx = omp_get_thread_num();
                OctreeKeyRay& key_ray = this->m_key_rays_[thread_idx];

                const double& range = ranges[i];
                double ex = point[0];
                double ey = point[1];
                double ez = point[2];
                if ((max_range >= 0.) && (range > max_range)) {  // crop ray at max_range
                    const double r = max_range / range;
                    ex = sx + diffs[i][0] * r;
                    ey = sy + diffs[i][1] * r;
                    ez = sz + diffs[i][2] * r;
                }

                if (!this->ComputeRayKeys(sx, sy, sz, ex, ey, ez, key_ray)) { continue; }  // key is invalid
#pragma omp critical(free_insert)
                {
                    // slower than ComputeRayKeys, there is always a thread busy with this critical section
                    for (auto& key: key_ray) {
                        if (m_end_point_mapping_.find(key) != m_end_point_mapping_.end()) { continue; }  // skip keys marked as occupied
                        if (const auto [_, new_key] = free_cells_set.emplace(key); new_key) { free_cells.push_back(key); }
                    }
                }
            }
        }

        /**
         * Insert a point cloud ray by ray. Some cells may be updated multiple times. Benchmark shows that this is slower and less accurate than
         * InsertPointCloud.
         * @param points 3xN matrix of ray end points in the world frame.
         * @param sensor_origin 3D vector of the sensor origin in the world frame.
         * @param max_range maximum range of the sensor. Points beyond this range are ignored. Non-positive value means no limit.
         * @param parallel whether to use parallel computation
         * @param lazy_eval whether to update the occupancy of the nodes immediately. If true, the occupancy is not updated until UpdateInnerOccupancy() is
         * called.
         */
        virtual void
        InsertPointCloudRays(
            const Eigen::Ref<const Eigen::Matrix3Xd>& points,
            const Eigen::Ref<const Eigen::Vector3d>& sensor_origin,
            const double max_range,
            bool parallel,
            const bool lazy_eval) {

            const long num_points = points.cols();
            if (num_points == 0) { return; }

            omp_set_num_threads(this->m_key_rays_.size());
#pragma omp parallel for if (parallel) default(none) shared(num_points, points, sensor_origin, max_range, lazy_eval) schedule(guided)
            for (long i = 0; i < num_points; ++i) {
                const auto point = points.col(i);
                uint32_t thread_idx = omp_get_thread_num();
                OctreeKeyRay& key_ray = this->m_key_rays_[thread_idx];
                if (!this->ComputeRayKeys(sensor_origin[0], sensor_origin[1], sensor_origin[2], point[0], point[1], point[2], key_ray)) { continue; }

#pragma omp critical
                {
                    for (auto& key: key_ray) { this->UpdateNode(key, false, lazy_eval); }
                    if (max_range <= 0. || (point - sensor_origin).norm() <= max_range) { this->UpdateNode(point[0], point[1], point[2], true, lazy_eval); }
                }
            }
        }

        //-- insert ray
        /**
         * Insert a ray from (sx, sy) to (ex, ey) into the tree. The ray is cut at max_range if it is positive.
         * @param sx metric x coordinate of the start point
         * @param sy metric y coordinate of the start point
         * @param sz metric z coordinate of the start point
         * @param ex metric x coordinate of the end point
         * @param ey metric y coordinate of the end point
         * @param ez metric z coordinate of the end point
         * @param max_range maximum range after which the ray is cut. Non-positive value means no limit.
         * @param lazy_eval whether to update the occupancy of the nodes immediately. If true, the occupancy is not updated until UpdateInnerOccupancy() is
         * called.
         * @return
         */
        virtual bool
        InsertRay(double sx, double sy, double sz, double ex, double ey, double ez, const double max_range, bool lazy_eval) {
            const double dx = ex - sx;
            const double dy = ey - sy;
            const double dz = ez - sz;
            const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
            auto& key_ray = this->m_key_rays_[0];
            if (max_range > 0 && range > max_range) {  // cut ray at max_range
                const double r = max_range / range;
                ex = sx + dx * r;
                ey = sy + dy * r;
                ez = sz + dz * r;
                if (!this->ComputeRayKeys(sx, sy, sz, ex, ey, ez, key_ray)) { return false; }
                for (auto& key: key_ray) { this->UpdateNode(key, false, lazy_eval); }
                return true;
            }

            if (!this->ComputeRayKeys(sx, sy, sz, ex, ey, ez, key_ray)) { return false; }
            for (auto& key: key_ray) { this->UpdateNode(key, false, lazy_eval); }
            this->UpdateNode(ex, ey, ez, true, lazy_eval);
            return true;
        }

        //-- cast ray
        OccupancyNdTreeBatchRayCaster<OccupancyOctreeBase, 3>
        GetBatchRayCaster(
            Eigen::Matrix3Xd origins,
            Eigen::Matrix3Xd directions,
            const Eigen::VectorXd& max_ranges,
            const Eigen::VectorXd& node_paddings,
            const Eigen::VectorXb& bidirectional_flags,
            const Eigen::VectorXb& leaf_only_flags,
            const Eigen::VectorXi& min_node_depths,
            const Eigen::VectorXi& max_node_depths) const {
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

        void
        CastRays(
            const Eigen::Ref<const Eigen::Vector3d>& position,
            const Eigen::Ref<const Eigen::Matrix3d>& rotation,
            const Eigen::Ref<const Eigen::VectorXd>& azimuth_angles,
            const Eigen::Ref<const Eigen::VectorXd>& elevation_angles,
            const bool ignore_unknown,
            const double max_range,
            const bool prune_rays,  // whether to prune rays after the first hit of the same occupied node
            const bool parallel,
            std::vector<std::pair<long, long>>& hit_ray_indices,  // (azimuth_idx, elevation_idx) of the hit rays
            std::vector<Eigen::Vector3d>& hit_positions,
            std::vector<const Node*>& hit_nodes) const {

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
                    const double cos_elevation = std::cos(elevation_angles[j]);
                    Eigen::Vector3d direction(
                        std::cos(azimuth_angles[i]) * cos_elevation,
                        std::sin(azimuth_angles[i]) * cos_elevation,
                        std::sin(elevation_angles[j]));
                    direction = rotation * direction;
                    Eigen::Vector3d& hit_position = hit_positions[idx];
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

            absl::flat_hash_set<const Node*> hit_nodes_set;

            std::vector<std::pair<long, long>> filtered_hit_ray_indices;
            std::vector<Eigen::Vector3d> filtered_hit_positions;
            std::vector<const Node*> filtered_hit_nodes;

            filtered_hit_ray_indices.reserve(num_rays);
            filtered_hit_positions.reserve(num_rays);
            filtered_hit_nodes.reserve(num_rays);

            // remove rays that hit nothing or hit the same node if prune_rays is true
            for (long i = 0; i < num_rays; ++i) {
                const Node*& hit_node = hit_nodes[i];
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

        void
        CastRays(
            const Eigen::Ref<const Eigen::Matrix3Xd>& positions,
            const Eigen::Ref<const Eigen::Matrix3Xd>& directions,
            const bool ignore_unknown,
            const double max_range,
            const bool prune_rays,
            const bool parallel,
            std::vector<long>& hit_ray_indices,
            std::vector<Eigen::Vector3d>& hit_positions,
            std::vector<const Node*>& hit_nodes) const {
            long num_rays = 0;
            if (positions.cols() != 1 && directions.cols() != 1) {
                ERL_ASSERTM(positions.cols() == directions.cols(), "positions.cols() != directions.cols() when both are not 1.");
                num_rays = positions.cols();
                hit_positions.resize(num_rays);
                hit_nodes.resize(num_rays, nullptr);
#pragma omp parallel for if (parallel) default(none) shared(num_rays, positions, directions, ignore_unknown, max_range, hit_positions, hit_nodes)
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
#pragma omp parallel for if (parallel) default(none) shared(num_rays, positions, directions, ignore_unknown, max_range, hit_positions, hit_nodes)
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
#pragma omp parallel for if (parallel) default(none) shared(num_rays, positions, directions, ignore_unknown, max_range, hit_positions, hit_nodes)
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

            absl::flat_hash_set<const Node*> hit_nodes_set;

            std::vector<long> filtered_hit_ray_indices;
            std::vector<Eigen::Vector3d> filtered_hit_positions;
            std::vector<const Node*> filtered_hit_nodes;
            filtered_hit_ray_indices.reserve(num_rays);
            filtered_hit_positions.reserve(num_rays);
            filtered_hit_nodes.reserve(num_rays);

            // remove rays that hit nothing or hit the same node if prune_rays is true
            for (long i = 0; i < num_rays; ++i) {
                const Node*& hit_node = hit_nodes[i];
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

        const OccupancyOctreeNode*
        GetHitOccupiedNode(
            const double px,
            const double py,
            const double pz,
            const double vx,
            const double vy,
            const double vz,
            const bool ignore_unknown,
            const double max_range,
            double& ex,
            double& ey,
            double& ez) const override {
            return static_cast<const OccupancyOctreeNode*>(CastRay(px, py, pz, vx, vy, vz, ignore_unknown, max_range, ex, ey, ez));
        }

        /**
         * Cast a ray starting from (px, py) along (vx, vy) and get the hit surface point (ex, ey) if the ray hits one.
         * @param px metric x coordinate of the start point
         * @param py metric y coordinate of the start point
         * @param pz metric z coordinate of the start point
         * @param vx x component of the ray direction
         * @param vy y component of the ray direction
         * @param vz z component of the ray direction
         * @param ignore_unknown whether unknown cells are ignored, i.e. treated as free. If false, the ray casting aborts when an unknown cell is hit and
         * returns false.
         * @param max_range maximum range after which the ray casting is aborted. Non-positive value means no limit.
         * @param ex metric x coordinate of the hit leaf cell
         * @param ey metric y coordinate of the hit leaf cell
         * @param ez metric z coordinate of the hit leaf cell
         * @return node pointer if the ray hits an occupied cell, nullptr otherwise.
         */
        const Node*
        CastRay(
            double px,
            double py,
            double pz,
            double vx,
            double vy,
            double vz,
            const bool ignore_unknown,
            const double max_range,
            double& ex,
            double& ey,
            double& ez) const {
            // Similar to OctreeImpl::ComputeRayKeys, but with extra hitting checks

            OctreeKey current_key;
            if (!this->CoordToKeyChecked(px, py, pz, current_key)) {
                ERL_WARN("Ray starting from ({}, {}, {}) is out of range.\n", px, py, pz);
                return nullptr;
            }

            // initialization
            const Node* starting_node = this->Search(current_key);
            if (starting_node != nullptr) {
                if (this->IsNodeOccupied(starting_node)) {  // (px, py, pz) is in occupied
                    this->KeyToCoord(current_key, ex, ey, ez);
                    return starting_node;
                }
            } else if (!ignore_unknown) {  // (px, py, pz) is in unknown
                this->KeyToCoord(current_key, ex, ey, ez);
                return nullptr;
            }

            const double v_norm = std::sqrt(vx * vx + vy * vy + vz * vz);
            vx /= v_norm;
            vy /= v_norm;
            vz /= v_norm;
            const bool max_range_set = max_range > 0.;

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
            const double resolution = this->m_setting_->resolution;
            double t_max[3];
            double t_delta[3];
            if (step[0] == 0) {
                t_max[0] = std::numeric_limits<double>::infinity();
                t_delta[0] = std::numeric_limits<double>::infinity();
            } else {
                const double voxel_border = this->KeyToCoord(current_key[0]) + static_cast<double>(step[0]) * 0.5 * resolution;
                t_max[0] = (voxel_border - px) / vx;
                t_delta[0] = resolution / std::abs(vx);
            }
            if (step[1] == 0) {
                t_max[1] = std::numeric_limits<double>::infinity();
                t_delta[1] = std::numeric_limits<double>::infinity();
            } else {
                const double voxel_border = this->KeyToCoord(current_key[1]) + static_cast<double>(step[1]) * 0.5 * resolution;
                t_max[1] = (voxel_border - py) / vy;
                t_delta[1] = resolution / std::abs(vy);
            }
            if (step[2] == 0) {
                t_max[2] = std::numeric_limits<double>::infinity();
                t_delta[2] = std::numeric_limits<double>::infinity();
            } else {
                const double voxel_border = this->KeyToCoord(current_key[2]) + static_cast<double>(step[2]) * 0.5 * resolution;
                t_max[2] = (voxel_border - pz) / vz;
                t_delta[2] = resolution / std::abs(vz);
            }

            // incremental phase
            const double max_range_sq = max_range * max_range;
            const long max_key_val = (this->m_tree_key_offset_ << 1) - 1;
            while (true) {
                int idx = 0;
                if (t_max[1] < t_max[0]) { idx = 1; }
                if (t_max[2] < t_max[idx]) { idx = 2; }

                t_max[idx] += t_delta[idx];
                const long next_key_val = static_cast<long>(current_key[idx]) + step[idx];
                // check overflow
                if ((step[idx] < 0 && next_key_val <= 0) || (step[idx] > 0 && next_key_val >= max_key_val)) {
                    ERL_DEBUG("coordinate hits boundary, aborting ray cast.");
                    current_key[idx] = next_key_val < 0 ? 0 : max_key_val;  // set to boundary
                    this->KeyToCoord(current_key, ex, ey, ez);
                    return nullptr;
                }
                current_key[idx] = next_key_val;

                // generate world coordinates from key
                this->KeyToCoord(current_key, ex, ey, ez);
                // check if max_range is reached
                if (max_range_set) {
                    if (const double dx = ex - px, dy = ey - py, dz = ez - pz; (dx * dx + dy * dy + dz * dz) > max_range_sq) { return nullptr; }
                }
                // search node of the new key
                const Node* current_node = this->Search(current_key);
                if (current_node != nullptr) {
                    if (this->IsNodeOccupied(current_node)) { return current_node; }
                } else if (!ignore_unknown) {
                    return nullptr;
                }
            }
        }

        //-- trace ray
        [[nodiscard]] OctreeKeyBoolMap::const_iterator
        BeginChangedKey() const {
            if (!reinterpret_cast<OccupancyOctreeBaseSetting*>(this->m_setting_.get())->use_change_detection) {
                ERL_WARN("use_change_detection is false in setting. No changes are tracked.");
                return m_changed_keys_.end();
            }
            return m_changed_keys_.begin();
        }

        [[nodiscard]] OctreeKeyBoolMap::const_iterator
        EndChangedKey() const {
            return m_changed_keys_.end();
        }

        void
        ClearChangedKey() {
            m_changed_keys_.clear();
        }

        //-- update nodes' occupancy
        /**
         * Update the node at the given key with the given log-odds delta.
         * @param x
         * @param y
         * @param z
         * @param occupied
         * @param lazy_eval whether update of inner nodes is omitted and only leaf nodes are updated. This speeds up the intersection, but you need to call
         * UpdateInnerOccupancy() after all updates are done.
         * @return
         */
        Node*
        UpdateNode(double x, double y, double z, const bool occupied, const bool lazy_eval) {
            OctreeKey key;
            if (!this->CoordToKeyChecked(x, y, z, key)) { return nullptr; }
            return this->UpdateNode(key, occupied, lazy_eval);
        }

        /**
         * Update occupancy measurement of a given node
         * @param key of the node to update
         * @param occupied whether the node is observed occupied or not
         * @param lazy_eval whether update of inner nodes is omitted and only leaf nodes are updated. This speeds up the intersection, but you need to call
         * UpdateInnerOccupancy() after all updates are done.
         */
        Node*
        UpdateNode(const OctreeKey& key, const bool occupied, const bool lazy_eval) {
            const float log_odds_delta = occupied ? m_setting_->log_odd_hit : m_setting_->log_odd_miss;
            return this->UpdateNode(key, log_odds_delta, lazy_eval);
        }

        Node*
        UpdateNode(double x, double y, double z, const float log_odds_delta, const bool lazy_eval) {
            OctreeKey key;
            if (!this->CoordToKeyChecked(x, y, z, key)) { return nullptr; }
            return this->UpdateNode(key, log_odds_delta, lazy_eval);
        }

        Node*
        UpdateNode(const OctreeKey& key, const float log_odds_delta, const bool lazy_eval) {
            auto leaf = const_cast<Node*>(this->Search(key));
            auto log_odds_delta_double = static_cast<double>(log_odds_delta);
            // early abort, no change will happen: node already at threshold or its log-odds is locked.
            if (leaf) {
                if (!leaf->AllowUpdateLogOdds(log_odds_delta_double)) { return leaf; }
                if (log_odds_delta_double >= 0 && leaf->GetLogOdds() >= m_setting_->log_odd_max) { return leaf; }
                if (log_odds_delta_double <= 0 && leaf->GetLogOdds() <= m_setting_->log_odd_min) { return leaf; }
            }

            const bool create_root = this->m_root_ == nullptr;
            if (create_root) {
                this->m_root_ = std::make_shared<Node>();
                ++this->m_tree_size_;
                ERL_DEBUG_ASSERT(this->m_tree_size_ == 1, "tree size is not 1 after root creation.");
            }
            return static_cast<Node*>(this->UpdateNodeRecurs(this->m_root_.get(), create_root, key, log_odds_delta_double, lazy_eval));
        }

    private:
        Node*
        UpdateNodeRecurs(Node* node, const bool node_just_created, const OctreeKey& key, const double log_odds_delta, const bool lazy_eval) {
            ERL_DEBUG_ASSERT(node != nullptr, "node is nullptr.");

            const uint32_t depth = node->GetDepth();
            if (const uint32_t& tree_depth = this->m_setting_->tree_depth; depth < tree_depth) {  // follow down to last level
                bool created_node = false;
                int pos = OctreeKey::ComputeChildIndex(key, tree_depth - 1 - depth);
                if (!node->HasChild(pos)) {                            // child node does not exist
                    if (!node->HasAnyChild() && !node_just_created) {  // current node has no child and is not new
                        this->ExpandNode(node);                        // expand pruned node
                    } else {
                        this->CreateNodeChild(node, pos);
                        created_node = true;
                    }
                }

                if (lazy_eval) { return this->UpdateNodeRecurs(this->GetNodeChild(node, pos), created_node, key, log_odds_delta, lazy_eval); }
                Node* returned_node = this->UpdateNodeRecurs(this->GetNodeChild(node, pos), created_node, key, log_odds_delta, lazy_eval);
                if (this->PruneNode(node)) {
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
                } else if (occ_before != this->IsNodeOccupied(node)) {                             // occupancy changed, track it
                    if (const auto it = m_changed_keys_.find(key); it == m_changed_keys_.end()) {  // not found
                        m_changed_keys_.emplace(key, false);
                    } else if (!it->second) {
                        m_changed_keys_.erase(it);
                    }
                }
            } else {
                this->UpdateNodeLogOdds(node, log_odds_delta);
            }
            return node;
        }

    protected:
        void
        UpdateNodeLogOdds(Node* node, float log_odd_delta) {
            node->AddLogOdds(log_odd_delta);
            const float l = node->GetLogOdds();
            const double log_odd_min = m_setting_->log_odd_min;
            const double log_odd_max = m_setting_->log_odd_max;
            if (l < log_odd_min) {
                node->SetLogOdds(log_odd_min);
                return;
            }

            if (l > log_odd_max) { node->SetLogOdds(log_odd_max); }
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
            if (depth < this->m_setting_->tree_depth) {
                for (int i = 0; i < 8; ++i) {
                    Node* child = this->GetNodeChild(node, i);
                    if (child == nullptr) { continue; }
                    UpdateInnerOccupancyRecurs(child, depth + 1);
                }
            }
            UpdateInnerNodeOccupancy(node);
        }

        void
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
            const double log_odd_min = m_setting_->log_odd_min;
            const double log_odd_max = m_setting_->log_odd_max;
            while (!stack.empty()) {
                Node* node = stack.back();
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

    protected:
        //--file IO
        std::istream&
        ReadBinaryData(std::istream& s) override {
            if (this->m_root_ != nullptr) {
                ERL_WARN("Trying to read into an existing tree.");
                return s;
            }

            this->m_root_ = std::make_shared<Node>();
            this->m_tree_size_ = 1;
            uint16_t child_record;

            std::list<std::pair<Node*, bool>> stack;  // node, is_new_node
            stack.emplace_back(this->m_root_.get(), true);

            const double log_odd_min = m_setting_->log_odd_min;
            const double log_odd_max = m_setting_->log_odd_max;

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
                s.read(reinterpret_cast<char*>(&child_record), sizeof(uint16_t));
                std::bitset<16> child((unsigned long long) child_record);
                bool has_inner_node_child = false;
                for (int i = 7; i >= 0; --i) {
                    // 0b10: free leaf
                    // 0b01: occupied leaf
                    // 0b11: inner node
                    const bool bit0 = child[i * 2];
                    const bool bit1 = child[i * 2 + 1];
                    Node* child_node = nullptr;
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

                std::bitset<16> child;
                for (int i = 7; i >= 0; --i) {
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
                auto child_record = static_cast<uint16_t>(child.to_ulong());
                s.write(reinterpret_cast<char*>(&child_record), sizeof(uint16_t));
            }

            return s;
        }
    };
}  // namespace erl::geometry

// ReSharper disable CppInconsistentNaming
template<>
struct YAML::convert<erl::geometry::OccupancyOctreeBaseSetting> {
    static Node
    encode(const erl::geometry::OccupancyOctreeBaseSetting& rhs) {
        Node node = convert<erl::geometry::OccupancyNdTreeSetting>::encode(rhs);
        node["use_change_detection"] = rhs.use_change_detection;
        node["use_aabb_limit"] = rhs.use_aabb_limit;
        node["aabb"] = rhs.aabb;
        return node;
    }

    static bool
    decode(const Node& node, erl::geometry::OccupancyOctreeBaseSetting& rhs) {
        if (!node.IsMap()) { return false; }
        if (!convert<erl::geometry::OccupancyNdTreeSetting>::decode(node, rhs)) { return false; }
        rhs.use_change_detection = node["use_change_detection"].as<bool>();
        rhs.use_aabb_limit = node["use_aabb_limit"].as<bool>();
        rhs.aabb = node["aabb"].as<erl::geometry::Aabb3D>();
        return true;
    }
};  // namespace YAML

// ReSharper restore CppInconsistentNaming
