#include <utility>
#include <absl/container/flat_hash_map.h>

#include "erl_geometry/lidar_frame_3d.hpp"
#include "erl_geometry/azimuth_elevation.hpp"
#include "erl_geometry/hidden_point_removal.hpp"
#include "erl_common/random.hpp"

namespace erl::geometry {

    void
    LidarFrame3D::Update(
        const Eigen::Ref<const Eigen::Matrix3d> &rotation,
        const Eigen::Ref<const Eigen::Vector3d> &translation,
        const Eigen::Ref<Eigen::VectorXd> &azimuths,
        const Eigen::Ref<Eigen::VectorXd> &elevations,
        Eigen::MatrixXd ranges,
        bool partition_rays) {
        m_rotation_ << rotation;
        m_translation_ << translation;
        m_azimuth_frame_ = azimuths.replicate(1, elevations.size());
        m_elevation_frame_ = elevations.transpose().replicate(azimuths.size(), 1);
        m_ranges_ = std::move(ranges);
        m_partitions_.clear();
        m_partitioned_ = false;
        m_kd_tree_->Clear();

        long num_azimuths = GetNumAzimuthLines();
        long num_elevations = GetNumElevationLines();
        ERL_ASSERTM(num_azimuths == m_ranges_.rows(), "num of azimuths (%ld) does not match rows of ranges (%ld).", num_azimuths, m_ranges_.rows());
        ERL_ASSERTM(num_elevations == m_ranges_.cols(), "num of elevations (%ld) does not match cols of ranges (%ld).", num_elevations, m_ranges_.cols());
        ERL_ASSERTM(num_azimuths > 0, "no azimuth angle.");
        ERL_ASSERTM(num_elevations > 0, "no elevation angle.");

        m_dirs_frame_.resize(num_azimuths, num_elevations);
        m_dirs_world_.resize(num_azimuths, num_elevations);
        m_end_pts_frame_.resize(num_azimuths, num_elevations);
        m_end_pts_world_.resize(num_azimuths, num_elevations);

        m_mask_hit_.setConstant(num_azimuths, num_elevations, false);
        m_hit_ray_indices_.resize(2, num_azimuths * num_elevations);
        m_hit_points_world_.resize(3, num_azimuths * num_elevations);

        // compute directions and end points
#pragma omp parallel for collapse(2) default(none) shared(num_azimuths, num_elevations, Eigen::Dynamic)
        for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
            for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
                double &range = m_ranges_(azimuth_idx, elevation_idx);
                if (range == 0 || std::isnan(range)) { continue; }  // zero or nan depth! Not allowed.
                double &azimuth = m_azimuth_frame_(azimuth_idx, elevation_idx);
                double &elevation = m_elevation_frame_(azimuth_idx, elevation_idx);

                // directions
                Eigen::Vector3d &dir_frame = m_dirs_frame_(azimuth_idx, elevation_idx);
                dir_frame << AzimuthElevationToDirection(azimuth, elevation);
                m_dirs_world_(azimuth_idx, elevation_idx) << m_rotation_ * dir_frame;

                // end points
                Eigen::Vector3d &end_pt_frame = m_end_pts_frame_(azimuth_idx, elevation_idx);
                end_pt_frame << range * dir_frame;
                m_end_pts_world_(azimuth_idx, elevation_idx) << m_rotation_ * end_pt_frame + m_translation_;

                // max valid range
                // cannot move this line to the outer loop, because directions are computed in the inner loop
                if ((azimuth < m_setting_->valid_azimuth_min) || (azimuth > m_setting_->valid_azimuth_max)) { continue; }
                if ((range < m_setting_->valid_range_min) || (range > m_setting_->valid_range_max)) { continue; }
                m_mask_hit_(azimuth_idx, elevation_idx) = true;
            }
        }

        m_max_valid_range_ = 0.0;
        long num_hit_points = 0;
        for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
            for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
                if (!m_mask_hit_(azimuth_idx, elevation_idx)) { continue; }
                double &range = m_ranges_(azimuth_idx, elevation_idx);
                if (range > m_max_valid_range_) { m_max_valid_range_ = range; }
                m_hit_ray_indices_.col(num_hit_points) << azimuth_idx, elevation_idx;
                m_hit_points_world_.col(num_hit_points++) << m_end_pts_world_(azimuth_idx, elevation_idx);
            }
        }
        m_hit_ray_indices_.conservativeResize(2, num_hit_points);
        m_hit_points_world_.conservativeResize(3, num_hit_points);

        if (!partition_rays) { return; }  // do not partition rays
        PartitionRays();
    }

    void
    LidarFrame3D::ComputeClosestEndPoint(
        const Eigen::Ref<const Eigen::Vector3d> &position_world,
        long &end_point_azimuth_index,
        long &end_point_elevation_index,
        double &distance,
        bool brute_force) const {
        if (brute_force) {
            end_point_azimuth_index = -1;
            end_point_elevation_index = -1;
            distance = std::numeric_limits<double>::infinity();
            long n_azimuths = GetNumAzimuthLines();
            long n_elevations = GetNumElevationLines();
            for (long azimuth_idx = 0; azimuth_idx < n_azimuths; ++azimuth_idx) {
                for (long elevation_idx = 0; elevation_idx < n_elevations; ++elevation_idx) {
                    double d = (m_end_pts_world_(azimuth_idx, elevation_idx) - position_world).squaredNorm();
                    if (d < distance) {
                        end_point_azimuth_index = azimuth_idx;
                        end_point_elevation_index = elevation_idx;
                        distance = d;
                    }
                }
            }
            distance = std::sqrt(distance);
            return;
        }

        if (!m_kd_tree_->Ready()) { std::const_pointer_cast<KdTree3d>(m_kd_tree_)->SetDataMatrix(m_end_pts_world_.data()->data(), m_end_pts_world_.size()); }
        long end_point_index = -1;
        distance = std::numeric_limits<double>::infinity();
        m_kd_tree_->Knn(1, position_world, end_point_index, distance);
        // end_point_index = end_point_elevation_index * m_azimuth_frame_.size() + end_point_azimuth_index (column-major)
        long num_azimuths = GetNumAzimuthLines();
        end_point_elevation_index = end_point_index / num_azimuths;
        end_point_azimuth_index = end_point_index - end_point_elevation_index * num_azimuths;
        distance = std::sqrt(distance);
    }

    void
    LidarFrame3D::SampleAlongRays(
        long num_samples_per_ray,
        double max_in_obstacle_dist,
        double sampled_rays_ratio,
        Eigen::Matrix3Xd &positions_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances) const {

        std::vector<long> ray_indices = common::GenerateShuffledIndices<long>(m_hit_ray_indices_.cols(), sampled_rays_ratio);
        auto n_rays = long(ray_indices.size());
        long n_samples = n_rays * num_samples_per_ray;
        positions_world.resize(3, n_samples);
        directions_world.resize(3, n_samples);
        distances.resize(n_samples);

        long index = 0;
        for (long &ray_idx: ray_indices) {
            const long &kAzimuthIdx = m_hit_ray_indices_(0, ray_idx);
            const long &kElevationIdx = m_hit_ray_indices_(1, ray_idx);
            double range = m_ranges_(kAzimuthIdx, kElevationIdx);
            double range_step = (range + max_in_obstacle_dist) / double(num_samples_per_ray);
            const Eigen::Vector3d &kDirWorld = m_dirs_world_(kAzimuthIdx, kElevationIdx);

            positions_world.col(index) << m_translation_;  // operator<< is at least 2x faster than operator= for Eigen matrix
            directions_world.col(index) << kDirWorld;      // operator<< is almost the same fast as element-wise assignment
            distances[index++] = range;                    // for vector, element-wise assignment is faster than operator<<

            Eigen::Vector3d shift = range_step * kDirWorld;
            for (long i = 1; i < num_samples_per_ray; ++i) {
                range -= range_step;
                positions_world.col(index) << positions_world.col(index - 1) + shift;
                directions_world.col(index) << kDirWorld;
                distances[index++] = range;
            }
        }
    }

    void
    LidarFrame3D::SampleAlongRays(
        double range_step,
        double max_in_obstacle_dist,
        double sampled_rays_ratio,
        Eigen::Matrix3Xd &positions_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances) const {

        std::vector<long> ray_indices = common::GenerateShuffledIndices<long>(m_hit_ray_indices_.cols(), sampled_rays_ratio);
        auto n_rays = long(ray_indices.size());
        long n_samples = 0;
        std::vector<std::pair<std::pair<long, long>, long>> n_samples_per_ray;
        n_samples_per_ray.reserve(n_rays);
        for (long &ray_idx: ray_indices) {
            const long &kAzimuthIdx = m_hit_ray_indices_(0, ray_idx);
            const long &kElevationIdx = m_hit_ray_indices_(1, ray_idx);
            auto n = long(std::floor((m_ranges_(kAzimuthIdx, kElevationIdx) + max_in_obstacle_dist) / range_step)) + 1;
            n_samples_per_ray.emplace_back(std::make_pair(kAzimuthIdx, kElevationIdx), n);
            n_samples += n;
        }
        positions_world.resize(3, n_samples);
        directions_world.resize(3, n_samples);
        distances.resize(n_samples);

        long sample_idx = 0;
        for (auto &[ray_idx, n_samples_of_ray]: n_samples_per_ray) {
            auto &[azimuth_idx, elevation_idx] = ray_idx;

            double range = m_ranges_(azimuth_idx, elevation_idx);
            const Eigen::Vector3d &kDirWorld = m_dirs_world_(azimuth_idx, elevation_idx);
            positions_world.col(sample_idx) << m_translation_;
            directions_world.col(sample_idx) << kDirWorld;
            distances[sample_idx++] = range;

            Eigen::Vector3d shift = range_step * kDirWorld;
            for (long sample_idx_of_ray = 1; sample_idx_of_ray < n_samples_of_ray; ++sample_idx_of_ray) {
                range -= range_step;
                positions_world.col(sample_idx) << positions_world.col(sample_idx - 1) + shift;
                directions_world.col(sample_idx) << kDirWorld;
                distances[sample_idx++] = range;
            }
        }
    }

    void
    LidarFrame3D::SampleNearSurface(
        long num_samples_per_ray,
        double max_offset,
        double sampled_rays_ratio,
        Eigen::Matrix3Xd &positions_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances) const {
        std::vector<long> hit_ray_indices = common::GenerateShuffledIndices<long>(m_hit_ray_indices_.cols(), sampled_rays_ratio);
        auto n_rays = long(hit_ray_indices.size());
        long n_samples = n_rays * num_samples_per_ray;
        positions_world.resize(3, n_samples);
        directions_world.resize(3, n_samples);
        distances.resize(n_samples);

        std::uniform_real_distribution<double> uniform(-max_offset, max_offset);
        long sample_idx = 0;
        for (long &hit_ray_idx: hit_ray_indices) {
            const long &kAzimuthIdx = m_hit_ray_indices_(0, hit_ray_idx);
            const long &kElevationIdx = m_hit_ray_indices_(1, hit_ray_idx);
            const Eigen::Vector3d &kDirWorld = m_dirs_world_(kAzimuthIdx, kElevationIdx);
            for (long i = 0; i < num_samples_per_ray; ++i) {
                double offset = uniform(erl::common::g_random_engine);
                positions_world.col(sample_idx) << m_translation_ + (m_ranges_(kAzimuthIdx, kElevationIdx) + offset) * kDirWorld;
                directions_world.col(sample_idx) << kDirWorld;
                distances[sample_idx++] = -offset;
            }
        }
    }

    void
    LidarFrame3D::SampleInRegionHpr(
        long num_positions,
        long num_along_ray_samples_per_ray,
        long num_near_surface_samples_per_ray,
        double max_in_obstacle_dist,
        Eigen::Matrix3Xd &positions_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances,
        bool parallel) const {
        ERL_ASSERTM(num_positions > 0, "num_positions (%ld) must be positive.", num_positions);

        if (parallel) {
            uint32_t num_threads = std::thread::hardware_concurrency();
            long num_positions_per_thread = num_positions / num_threads;
            long leftover = num_positions - num_positions_per_thread * num_threads;
            std::vector<std::thread> threads;
            threads.reserve(num_threads);
            std::vector<Eigen::Matrix3Xd> positions_world_buffers(num_threads);
            std::vector<Eigen::Matrix3Xd> directions_world_buffers(num_threads);
            std::vector<Eigen::VectorXd> distances_buffers(num_threads);
            uint64_t seed = erl::common::g_random_engine();
            for (uint32_t thread_idx = 0; thread_idx < num_threads; ++thread_idx) {
                threads.emplace_back(
                    &LidarFrame3D::SampleInRegionHprThread,
                    this,
                    seed + thread_idx,
                    num_positions_per_thread + thread_idx < leftover ? 1 : 0,
                    num_along_ray_samples_per_ray,
                    num_near_surface_samples_per_ray,
                    max_in_obstacle_dist,
                    &positions_world_buffers[thread_idx],
                    &directions_world_buffers[thread_idx],
                    &distances_buffers[thread_idx]);
            }
            for (auto &thread: threads) { thread.join(); }
            threads.clear();
            long num_samples = 0;
            for (auto &positions: positions_world_buffers) { num_samples += positions.cols(); }
            positions_world.resize(3, num_samples);
            directions_world.resize(3, num_samples);
            distances.resize(num_samples);
            long copied_samples = 0;
            for (uint32_t thread_idx = 0; thread_idx < num_threads; ++thread_idx) {
                auto &positions = positions_world_buffers[thread_idx];
                auto &directions = directions_world_buffers[thread_idx];
                long num_samples_to_copy = positions.cols();
                if (num_samples_to_copy == 0) { continue; }
                positions_world.block(0, copied_samples, 3, num_samples_to_copy) << positions;
                directions_world.block(0, copied_samples, 3, num_samples_to_copy) << directions;
                distances.segment(copied_samples, num_samples_to_copy) << distances_buffers[thread_idx];
                copied_samples += num_samples_to_copy;
            }
        } else {
            SampleInRegionHprThread(
                erl::common::g_random_engine(),
                num_positions,
                num_along_ray_samples_per_ray,
                num_near_surface_samples_per_ray,
                max_in_obstacle_dist,
                &positions_world,
                &directions_world,
                &distances);
        }
        ERL_DEBUG("%ld positions, %ld samples collected.", num_positions, positions_world.cols());
    }

    void
    LidarFrame3D::SampleInRegionVrs(
        long num_hit_points,
        long num_samples_per_azimuth_segment,
        long num_azimuth_segments,
        Eigen::Matrix3Xd &positions_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances,
        bool parallel) const {

        ERL_ASSERTM(num_hit_points > 0, "num_hit_points (%ld) must be positive.", num_hit_points);

        std::vector<long> selected_hit_ray_indices(m_hit_ray_indices_.cols());
        std::iota(selected_hit_ray_indices.begin(), selected_hit_ray_indices.end(), 0);
        num_hit_points = std::min(num_hit_points, m_hit_ray_indices_.cols());
        if (num_hit_points < long(selected_hit_ray_indices.size())) {
            std::shuffle(selected_hit_ray_indices.begin(), selected_hit_ray_indices.end(), common::g_random_engine);
            selected_hit_ray_indices.resize(num_hit_points);
        }

        if (parallel) {
            uint32_t num_threads = std::thread::hardware_concurrency();
            long num_positions_per_thread = num_hit_points / num_threads;
            long leftover = num_hit_points - num_positions_per_thread * num_threads;
            std::vector<std::thread> threads;
            threads.reserve(num_threads);
            std::vector<Eigen::Matrix3Xd> positions_world_buffers(num_threads);
            std::vector<Eigen::Matrix3Xd> directions_world_buffers(num_threads);
            std::vector<Eigen::VectorXd> distances_buffers(num_threads);
            uint64_t seed = erl::common::g_random_engine();
            for (long thread_idx = 0, start = 0; thread_idx < num_threads; ++thread_idx) {
                long end = start + num_positions_per_thread + (thread_idx < leftover ? 1 : 0);
                threads.emplace_back(
                    &LidarFrame3D::SampleInRegionVrsThread,
                    this,
                    seed + thread_idx,
                    selected_hit_ray_indices.data() + start,
                    selected_hit_ray_indices.data() + end,
                    num_samples_per_azimuth_segment,
                    num_azimuth_segments,
                    &positions_world_buffers[thread_idx],
                    &directions_world_buffers[thread_idx],
                    &distances_buffers[thread_idx]);
                start = end;
            }
            for (auto &thread: threads) { thread.join(); }
            threads.clear();
            long num_samples = 0;
            for (auto &positions: positions_world_buffers) { num_samples += positions.cols(); }
            positions_world.resize(3, num_samples);
            directions_world.resize(3, num_samples);
            distances.resize(num_samples);
            long copied_samples = 0;
            for (uint32_t thread_idx = 0; thread_idx < num_threads; ++thread_idx) {
                auto &positions = positions_world_buffers[thread_idx];
                auto &directions = directions_world_buffers[thread_idx];
                long num_samples_to_copy = positions.cols();
                if (num_samples_to_copy == 0) { continue; }
                positions_world.block(0, copied_samples, 3, num_samples_to_copy) << positions;
                directions_world.block(0, copied_samples, 3, num_samples_to_copy) << directions;
                distances.segment(copied_samples, num_samples_to_copy) << distances_buffers[thread_idx];
                copied_samples += num_samples_to_copy;
            }
        } else {
            SampleInRegionVrsThread(
                erl::common::g_random_engine(),
                selected_hit_ray_indices.data(),
                selected_hit_ray_indices.data() + num_hit_points,
                num_samples_per_azimuth_segment,
                num_azimuth_segments,
                &positions_world,
                &directions_world,
                &distances);
        }
    }

    void
    LidarFrame3D::ComputeRaysAt(
        const Eigen::Ref<const Eigen::Vector3d> &position_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances,
        std::vector<long> &visible_hit_point_indices) const {
        ERL_ASSERTM(!std::isinf(m_max_valid_range_), "max valid range is not set.");
        double radius = (m_max_valid_range_ + (position_world - m_translation_).norm()) * 10.0;
        visible_hit_point_indices.clear();
        HiddenPointRemoval(m_hit_points_world_, position_world, radius, visible_hit_point_indices, true, false);
        auto num_visible_hit_points = long(visible_hit_point_indices.size());
        if (directions_world.cols() < num_visible_hit_points) {
            directions_world.resize(3, num_visible_hit_points);
            distances.resize(num_visible_hit_points);
        }
        long sample_idx = 0;
        for (long &index: visible_hit_point_indices) {
            auto dir = directions_world.col(sample_idx);
            double &distance = distances[sample_idx++];

            dir << m_hit_points_world_.col(index) - position_world;
            distance = dir.norm();
            dir /= distance;
        }
    }

    void
    LidarFrame3D::PartitionRays() {
        long num_azimuths = GetNumAzimuthLines();
        long num_elevations = GetNumElevationLines();
        // detect discontinuities, out-of-max-range measurements
        m_mask_continuous_.setConstant(num_azimuths, num_elevations, true);
        m_mask_continuous_.row(0).setConstant(false);
        m_mask_continuous_.col(0).setConstant(false);
        m_mask_continuous_.bottomRows<1>().setConstant(false);
        m_mask_continuous_.rightCols<1>().setConstant(false);
        double rolling_range_diff = 0.0;
        double gamma1 = m_setting_->rolling_diff_discount;
        double gamma2 = 1 - gamma1;
        for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
            for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
                if (!m_mask_hit_(azimuth_idx, elevation_idx)) { continue; }
                double &azimuth = m_azimuth_frame_(azimuth_idx, elevation_idx);
                double &elevation = m_elevation_frame_(azimuth_idx, elevation_idx);
                const double &kRange = m_ranges_(azimuth_idx, elevation_idx);
                if (azimuth_idx == 0) {
                    if (elevation_idx == 0) { continue; }
                    double range_diff =
                        std::abs((kRange - m_ranges_(azimuth_idx, elevation_idx - 1)) / (elevation - m_elevation_frame_(azimuth_idx, elevation_idx - 1)));
                    if (range_diff > m_setting_->discontinuity_factor * rolling_range_diff) { m_mask_continuous_(azimuth_idx, elevation_idx - 1) = false; }
                    if (rolling_range_diff == 0.0) { rolling_range_diff = range_diff; }
                    rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * range_diff;
                } else {
                    double range_diff =
                        std::abs((kRange - m_ranges_(azimuth_idx - 1, elevation_idx)) / (azimuth - m_azimuth_frame_(azimuth_idx - 1, elevation_idx)));
                    if (range_diff > m_setting_->discontinuity_factor * rolling_range_diff) { m_mask_continuous_(azimuth_idx - 1, elevation_idx) = false; }
                    rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * range_diff;
                    if (elevation_idx == 0) { continue; }
                    range_diff =
                        std::abs((kRange - m_ranges_(azimuth_idx, elevation_idx - 1)) / (elevation - m_elevation_frame_(azimuth_idx, elevation_idx - 1)));
                    if (range_diff > m_setting_->discontinuity_factor * rolling_range_diff) { m_mask_continuous_(azimuth_idx, elevation_idx - 1) = false; }
                    rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * range_diff;
                }
            }
        }

        throw std::runtime_error("Implementation for partitioning rays is not complete.");
    }

    void
    LidarFrame3D::SampleInRegionHprThread(
        uint64_t seed,
        long num_positions,
        long num_along_ray_samples_per_ray,
        long num_near_surface_samples_per_ray,
        double max_in_obstacle_dist,
        Eigen::Matrix3Xd *positions_world_ptr,
        Eigen::Matrix3Xd *directions_world_ptr,
        Eigen::VectorXd *distances_ptr) const {
        ERL_ASSERTM(m_hit_points_world_.cols() > 0, "no hit points. cannot sample in region.");

        std::uniform_int_distribution<long> uniform_int_dist(0, m_hit_points_world_.cols() - 1);
        std::uniform_real_distribution<double> uniform_real_dist(0.1, 0.8);  // avoid getting too close to either the sensor or the hit point

        long max_num_samples = num_positions * m_hit_ray_indices_.cols() * (num_along_ray_samples_per_ray + num_near_surface_samples_per_ray);
        Eigen::Matrix3Xd &positions_samples = *positions_world_ptr;
        Eigen::Matrix3Xd &directions_samples = *directions_world_ptr;
        Eigen::VectorXd &distances_samples = *distances_ptr;
        positions_samples.resize(3, max_num_samples);
        directions_samples.resize(3, max_num_samples);
        distances_samples.resize(max_num_samples);

        std::vector<long> visible_hit_point_indices;
        std::mt19937 random_engine(seed);
        std::uniform_real_distribution<double> uniform_ns(-max_in_obstacle_dist, max_in_obstacle_dist);

        long sample_cnt = 0;
        for (long position_idx = 0; position_idx < num_positions; ++position_idx) {
            long hit_index = uniform_int_dist(random_engine);
            double r = uniform_real_dist(random_engine);
            const long &kHitAzimuthIndex = m_hit_ray_indices_(0, hit_index);
            const long &kHitElevationIndex = m_hit_ray_indices_(1, hit_index);
            const double &kRange = m_ranges_(kHitAzimuthIndex, kHitElevationIndex);
            r *= kRange;

            // synthesize a lidar scan
            Eigen::Vector3d position_scan = m_translation_ + r * m_dirs_world_(kHitAzimuthIndex, kHitElevationIndex);
            double radius = (m_max_valid_range_ + (position_scan - m_translation_).norm()) * 10.0;
            visible_hit_point_indices.clear();
            HiddenPointRemoval(m_hit_points_world_, position_scan, radius, visible_hit_point_indices, true, false);

            auto num_rays = long(visible_hit_point_indices.size());
            if (num_rays == 0) {
                position_idx--;  // retry
                continue;
            }

            for (long &index: visible_hit_point_indices) {
                Eigen::Vector3d dir = m_hit_points_world_.col(index) - position_scan;
                double range = dir.norm();
                dir /= range;

                // sample near surface with this lidar scan
                for (long i = 0; i < num_near_surface_samples_per_ray; ++i) {
                    double offset = uniform_ns(random_engine);
                    positions_samples.col(sample_cnt) << position_scan + (range + offset) * dir;
                    directions_samples.col(sample_cnt) << dir;
                    distances_samples[sample_cnt++] = -offset;
                }

                // sample along rays with this lidar scan
                positions_samples.col(sample_cnt) << position_scan;
                directions_samples.col(sample_cnt) << dir;
                distances_samples[sample_cnt++] = range;
                double range_step = (range + max_in_obstacle_dist) / double(num_along_ray_samples_per_ray);
                Eigen::Vector3d shift = range_step * dir;
                for (long i = 1; i < num_along_ray_samples_per_ray; ++i) {
                    range -= range_step;
                    positions_samples.col(sample_cnt) << positions_samples.col(sample_cnt - 1) + shift;
                    directions_samples.col(sample_cnt) << dir;
                    distances_samples[sample_cnt++] = range;
                }
            }
        }

        positions_samples.conservativeResize(3, sample_cnt);
        directions_samples.conservativeResize(3, sample_cnt);
        distances_samples.conservativeResize(sample_cnt);
    }

    void
    LidarFrame3D::SampleInRegionVrsThread(
        uint64_t seed,
        const long *hit_point_index_start,
        const long *hit_point_index_end,
        long num_samples_per_azimuth_segment,
        long num_azimuth_segments,
        Eigen::Matrix3Xd *positions_world_ptr,
        Eigen::Matrix3Xd *directions_world_ptr,
        Eigen::VectorXd *distances_ptr) const {
        ERL_ASSERTM(m_hit_points_world_.cols() > 0, "no hit points. cannot sample in region.");

        Eigen::Matrix3Xd &positions_samples = *positions_world_ptr;
        Eigen::Matrix3Xd &directions_samples = *directions_world_ptr;
        Eigen::VectorXd &distances_samples = *distances_ptr;
        long max_num_samples = num_azimuth_segments * num_samples_per_azimuth_segment * (hit_point_index_end - hit_point_index_start);
        positions_samples.resize(3, max_num_samples);
        directions_samples.resize(3, max_num_samples);
        distances_samples.resize(max_num_samples);
        long sample_idx = 0;
        std::mt19937 random_engine(seed);
        std::uniform_real_distribution<double> uniform_range_ratio(0.1, 0.9);

        struct RayInfo {
            double ray_azimuth = 0.0;
            double ray_elevation = 0.0;
            double end_point_elevation = 0.0;
            double range = 0.0;
            Eigen::Vector3d dir_world = {};
        };

        for (const long *hit_point_index_ptr = hit_point_index_start; hit_point_index_ptr < hit_point_index_end; ++hit_point_index_ptr) {
            // make the hit point the origin, and the viewing direction along the -z axis
            const long kHitAzimuthIndex = m_hit_ray_indices_(0, *hit_point_index_ptr);
            const long kHitElevationIndex = m_hit_ray_indices_(1, *hit_point_index_ptr);
            const double &kHitRange = m_ranges_(kHitAzimuthIndex, kHitElevationIndex);
            const Eigen::Vector3d &kViewingDir = m_dirs_world_(kHitAzimuthIndex, kHitElevationIndex);
            Eigen::Vector3d axis = -kViewingDir.cross(Eigen::Vector3d(0.0, 0.0, 1.0)).normalized();
            double angle = std::acos(-kViewingDir.dot(Eigen::Vector3d(0.0, 0.0, 1.0)));
            Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle, axis).matrix();
            double azimuth_resolution = 2 * M_PI / double(num_azimuth_segments);

            // 1. transform the rays to the hit point's frame, and partition the rays into azimuth segments
            absl::flat_hash_map<long, std::vector<RayInfo>> azimuth_rays;
            // 2. remove hit rays behind the viewing position, compute spherical coordinates, and partition the points into azimuth segments
            std::vector<std::pair<double, double>> spherical_coords;  // azimuth, elevation
            spherical_coords.reserve(m_hit_points_world_.cols());
            // 3. calculate max end_point_elevation in each azimuth segment
            Eigen::VectorXd max_elevations = Eigen::VectorXd::Constant(num_azimuth_segments, -M_PI_2);
            for (long j = 0; j < m_ranges_.cols(); ++j) {
                for (long i = 0; i < m_ranges_.rows(); ++i) {
                    if (i == kHitAzimuthIndex && j == kHitElevationIndex) { continue; }  // skip the hit point
                    if (std::isinf(m_ranges_(i, j))) { continue; }                       // skip rays that hit nothing
                    if (!m_mask_hit_(i, j)) { continue; }
                    if (m_dirs_world_(i, j).dot(kViewingDir) <= 0.0) { continue; }  // behind the viewing position

                    RayInfo ray_info;
                    ray_info.range = m_ranges_(i, j);
                    ray_info.dir_world << m_dirs_world_(i, j);
                    // 1.
                    Eigen::Vector3d direction = rotation * ray_info.dir_world;
                    DirectionToAzimuthElevation(direction, ray_info.ray_azimuth, ray_info.ray_elevation);
                    auto azimuth_index = long((ray_info.ray_azimuth + M_PI) / azimuth_resolution) % num_azimuth_segments;
                    // 2.
                    Eigen::Vector3d point = rotation * (m_end_pts_world_(i, j) - m_hit_points_world_.col(*hit_point_index_ptr));
                    ray_info.end_point_elevation = std::asin(point.z() / point.norm());
                    spherical_coords.emplace_back(ray_info.ray_azimuth, ray_info.end_point_elevation);
                    // 3.
                    double &max_elevation = max_elevations[azimuth_index];
                    if (ray_info.end_point_elevation > max_elevation) { max_elevation = ray_info.end_point_elevation; }
                    azimuth_rays[azimuth_index].emplace_back(std::move(ray_info));
                }
            }

            // sample along rays in each azimuth segment
            for (auto &[azimuth_index, rays]: azimuth_rays) {
                double &max_elevation = max_elevations[azimuth_index];
                double cos_max_elevation = std::cos(max_elevation) * kHitRange;
                std::uniform_int_distribution<std::size_t> uniform_ray_index(0, rays.size() - 1);
                for (long cnt_samples = 0; cnt_samples < num_samples_per_azimuth_segment; ++cnt_samples) {
                    std::size_t ray_index = uniform_ray_index(random_engine);
                    RayInfo &ray = rays[ray_index];
                    double r = uniform_range_ratio(random_engine);
                    double elevation_diff = max_elevation - ray.ray_elevation;
                    double max_range = std::min(ray.range, cos_max_elevation / std::sin(elevation_diff));  // calculate max sampling range along the ray
                    Eigen::Vector3d position = m_translation_ + r * max_range * ray.dir_world;
                    Eigen::Vector3d direction = m_hit_points_world_.col(*hit_point_index_ptr) - position;
                    double distance = direction.norm();
                    direction /= distance;
                    positions_samples.col(sample_idx) << position;
                    directions_samples.col(sample_idx) << direction;
                    distances_samples[sample_idx++] = distance;
                }
            }
        }
        positions_samples.conservativeResize(3, sample_idx);
        directions_samples.conservativeResize(3, sample_idx);
        distances_samples.conservativeResize(sample_idx);
    }
}  // namespace erl::geometry
