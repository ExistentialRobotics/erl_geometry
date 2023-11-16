#include <utility>

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
        bool partition_rays
    ) {
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
        for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
            for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
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
        bool brute_force
    ) const {
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
        Eigen::Matrix3Xd &positions_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances,
        double sampled_rays_ratio
    ) const {
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
        Eigen::Matrix3Xd &positions_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances,
        double sampled_rays_ratio
    ) const {
        std::vector<long> ray_indices = common::GenerateShuffledIndices<long>(m_hit_ray_indices_.cols(), sampled_rays_ratio);
        auto n_rays = long(ray_indices.size());
        long n_samples = 0;
        Eigen::VectorXl n_samples_per_ray(n_rays);
        for (long idx = 0; idx < n_rays; ++idx) {
            long ray_idx = ray_indices[idx];
            const long &kAzimuthIdx = m_hit_ray_indices_(0, ray_idx);
            const long &kElevationIdx = m_hit_ray_indices_(1, ray_idx);
            long &n = n_samples_per_ray[idx];
            n = long(std::floor((m_ranges_(kAzimuthIdx, kElevationIdx) + max_in_obstacle_dist) / range_step)) + 1;
            n_samples += n;
        }
        positions_world.resize(3, n_samples);
        directions_world.resize(3, n_samples);
        distances.resize(n_samples);

        long sample_idx = 0;
        for (long idx = 0; idx < n_rays; ++idx) {
            long ray_idx = ray_indices[idx];
            const long &kAzimuthIdx = m_hit_ray_indices_(0, ray_idx);
            const long &kElevationIdx = m_hit_ray_indices_(1, ray_idx);
            long &n_samples_of_ray = n_samples_per_ray[idx];

            double range = m_ranges_(kAzimuthIdx, kElevationIdx);
            const Eigen::Vector3d &kDirWorld = m_dirs_world_(kAzimuthIdx, kElevationIdx);
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
        Eigen::Matrix3Xd &positions_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances,
        double sampled_rays_ratio
    ) const {
        std::vector<long> ray_indices = common::GenerateShuffledIndices<long>(m_hit_ray_indices_.cols(), sampled_rays_ratio);
        auto n_rays = long(ray_indices.size());
        long n_samples = n_rays * num_samples_per_ray;
        positions_world.resize(3, n_samples);
        directions_world.resize(3, n_samples);
        distances.resize(n_samples);

        std::uniform_real_distribution<double> uniform(-max_offset, max_offset);
        long sample_idx = 0;
        for (long &ray_idx: ray_indices) {
            const long &kAzimuthIdx = m_hit_ray_indices_(0, ray_idx);
            const long &kElevationIdx = m_hit_ray_indices_(1, ray_idx);
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
    LidarFrame3D::SampleInRegion(
        long num_samples,
        long num_samples_per_iter,
        Eigen::Matrix3Xd &positions_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances,
        bool parallel
    ) const {
        ERL_ASSERTM(num_samples > 0, "num_samples (%ld) must be positive.", num_samples);
        if (num_samples_per_iter < 0 || num_samples_per_iter >= num_samples) { num_samples_per_iter = num_samples; }

        if (parallel) {
            uint32_t num_threads = std::thread::hardware_concurrency();
            long num_samples_per_thread = num_samples / num_threads + 1;  // make sure we have enough samples
            std::vector<std::thread> threads;
            threads.reserve(num_threads);
            std::vector<Eigen::Matrix3Xd> positions_world_buffers(num_threads);
            std::vector<Eigen::Matrix3Xd> directions_world_buffers(num_threads);
            std::vector<Eigen::VectorXd> distances_buffers(num_threads);
            std::vector<long> iter_cnts(num_threads);
            uint64_t seed = erl::common::g_random_engine();
            for (uint32_t thread_idx = 0; thread_idx < num_threads; ++thread_idx) {
                threads.emplace_back(
                    &LidarFrame3D::SampleInRegionThread,
                    this,
                    seed + thread_idx,
                    num_samples_per_thread,
                    num_samples_per_iter,
                    &positions_world_buffers[thread_idx],
                    &directions_world_buffers[thread_idx],
                    &distances_buffers[thread_idx],
                    &iter_cnts[thread_idx]
                );
            }
            for (auto &thread: threads) { thread.join(); }
            threads.clear();
            ERL_DEBUG("%ld samples collected in %ld iterations.", num_samples_per_thread * num_threads, std::accumulate(iter_cnts.begin(), iter_cnts.end(), 0L));
            positions_world.resize(3, num_samples);
            directions_world.resize(3, num_samples);
            distances.resize(num_samples);
            long copied_samples = 0;
            for (uint32_t thread_idx = 0; thread_idx < num_threads; ++thread_idx) {
                long num_samples_to_copy = std::min(num_samples - copied_samples, num_samples_per_thread);
                positions_world.block(0, copied_samples, 3, num_samples_to_copy) << positions_world_buffers[thread_idx].leftCols(num_samples_to_copy);
                directions_world.block(0, copied_samples, 3, num_samples_to_copy) << directions_world_buffers[thread_idx].leftCols(num_samples_to_copy);
                distances.segment(copied_samples, num_samples_to_copy) << distances_buffers[thread_idx].head(num_samples_to_copy);
                copied_samples += num_samples_to_copy;
            }
        } else {
            long iter_cnt = 0;
            SampleInRegionThread(erl::common::g_random_engine(), num_samples, num_samples_per_iter, &positions_world, &directions_world, &distances, &iter_cnt);
            ERL_DEBUG("%ld samples collected in %ld iterations.", num_samples, iter_cnt);
        }
    }

    void
    LidarFrame3D::ComputeRaysAt(
        const Eigen::Ref<const Eigen::Vector3d> &position_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances,
        std::vector<long> &visible_hit_point_indices
    ) const {
        double radius = (m_max_valid_range_ + (position_world - m_translation_).norm()) * 10.0;
        visible_hit_point_indices.clear();
        HiddenPointRemoval(m_hit_points_world_, position_world, radius, visible_hit_point_indices, false);
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
                double &range = m_ranges_(azimuth_idx, elevation_idx);
                if (azimuth_idx == 0) {
                    if (elevation_idx == 0) { continue; }
                    double range_diff =
                        std::abs((range - m_ranges_(azimuth_idx, elevation_idx - 1)) / (elevation - m_elevation_frame_(azimuth_idx, elevation_idx - 1)));
                    if (range_diff > m_setting_->discontinuity_factor * rolling_range_diff) { m_mask_continuous_(azimuth_idx, elevation_idx - 1) = false; }
                    if (rolling_range_diff == 0.0) { rolling_range_diff = range_diff; }
                    rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * range_diff;
                } else {
                    double range_diff =
                        std::abs((range - m_ranges_(azimuth_idx - 1, elevation_idx)) / (azimuth - m_azimuth_frame_(azimuth_idx - 1, elevation_idx)));
                    if (range_diff > m_setting_->discontinuity_factor * rolling_range_diff) { m_mask_continuous_(azimuth_idx - 1, elevation_idx) = false; }
                    rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * range_diff;
                    if (elevation_idx == 0) { continue; }
                    range_diff =
                        std::abs((range - m_ranges_(azimuth_idx, elevation_idx - 1)) / (elevation - m_elevation_frame_(azimuth_idx, elevation_idx - 1)));
                    if (range_diff > m_setting_->discontinuity_factor * rolling_range_diff) { m_mask_continuous_(azimuth_idx, elevation_idx - 1) = false; }
                    rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * range_diff;
                }
            }
        }

        throw std::runtime_error("Implementation for partitioning rays is not complete.");
    }

    void
    LidarFrame3D::SampleInRegionThread(
        uint64_t seed,
        long num_samples,
        long num_samples_per_iter,
        Eigen::Matrix3Xd *positions_world_ptr,
        Eigen::Matrix3Xd *directions_world_ptr,
        Eigen::VectorXd *distances_ptr,
        long *iter_cnt_ptr
    ) const {
        if (num_samples_per_iter < 0 || num_samples_per_iter >= num_samples) { num_samples_per_iter = num_samples; }

        std::uniform_int_distribution<long> uniform_int_dist(0, m_hit_points_world_.cols() - 1);
        std::uniform_real_distribution<double> uniform_real_dist(0.1, 0.8);  // avoid getting too close to either the sensor or the hit point
        Eigen::Matrix3Xd &positions_world = *positions_world_ptr;
        Eigen::Matrix3Xd &directions_world = *directions_world_ptr;
        Eigen::VectorXd &distances = *distances_ptr;
        long &iter_cnt = *iter_cnt_ptr;
        positions_world.resize(3, num_samples);
        directions_world.resize(3, num_samples);
        distances.resize(num_samples);
        iter_cnt = 0;

        long collected_samples = 0;
        Eigen::Vector3d position_world;
        Eigen::Matrix3Xd directions_world_buf;
        Eigen::VectorXd distances_world_buf;
        std::vector<long> visible_hit_point_indices;
        std::mt19937 random_engine(seed);
        while (collected_samples < num_samples) {
            iter_cnt++;
            long hit_index = uniform_int_dist(random_engine);
            double r = uniform_real_dist(random_engine);
            const long &kHitAzimuthIndex = m_hit_ray_indices_(0, hit_index);
            const long &kHitElevationIndex = m_hit_ray_indices_(1, hit_index);
            const double &kRange = m_ranges_(kHitAzimuthIndex, kHitElevationIndex);
            r *= kRange;
            position_world << m_translation_ + r * m_dirs_world_(kHitAzimuthIndex, kHitElevationIndex);

            ComputeRaysAt(position_world, directions_world_buf, distances_world_buf, visible_hit_point_indices);
            auto num_rays = long(visible_hit_point_indices.size());
            if (num_rays == 0) { continue; }

            long num_samples_to_collect = std::min(num_samples - collected_samples, num_samples_per_iter);
            if (num_samples_to_collect < num_rays) {
                std::vector<long> indices = common::GenerateShuffledIndices<long>(num_rays, random_engine);

                for (long i = 0; i < num_samples_to_collect; ++i) {
                    long &ray_idx = indices[i];
                    positions_world.col(collected_samples) << position_world;
                    directions_world.col(collected_samples) << directions_world_buf.col(ray_idx);
                    distances[collected_samples++] = distances_world_buf[ray_idx];
                    if (collected_samples >= num_samples) { break; }
                }
            } else {
                for (long ray_idx = 0; ray_idx < num_rays; ++ray_idx) {
                    positions_world.col(collected_samples) << position_world;
                    directions_world.col(collected_samples) << directions_world_buf.col(ray_idx);
                    distances[collected_samples++] = distances_world_buf[ray_idx];
                    if (collected_samples >= num_samples) { break; }
                }
            }
        }
    }
}  // namespace erl::geometry
