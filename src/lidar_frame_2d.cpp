#include "erl_geometry/lidar_frame_2d.hpp"

#include "erl_common/logging.hpp"
#include "erl_geometry/utils.hpp"
#include "erl_geometry/winding_number.hpp"

namespace erl::geometry {

    LidarFrame2D::LidarFrame2D(std::shared_ptr<Setting> setting)
        : m_setting_(std::move(setting)) {
        ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");

        m_angles_frame_ = Eigen::VectorXd::LinSpaced(m_setting_->num_rays, m_setting_->angle_min, m_setting_->angle_max);
        m_dirs_frame_.clear();
        m_dirs_frame_.reserve(m_setting_->num_rays);
        for (long i = 0; i < m_setting_->num_rays; ++i) { m_dirs_frame_.emplace_back(std::cos(m_angles_frame_[i]), std::sin(m_angles_frame_[i])); }
    }

    void
    LidarFrame2D::UpdateRanges(
        const Eigen::Ref<const Eigen::Matrix2d> &rotation,
        const Eigen::Ref<const Eigen::Vector2d> &translation,
        Eigen::VectorXd ranges,
        const bool partition_rays) {

        m_rotation_ = rotation;
        m_rotation_angle_ = Eigen::Rotation2Dd(m_rotation_).angle();
        m_translation_ = translation;
        m_ranges_ = std::move(ranges);
        m_partitions_.clear();
        m_partitioned_ = false;
        m_kd_tree_->Clear();

        const long n = m_angles_frame_.size();
        ERL_ASSERTM(n == m_ranges_.size(), "angles and ranges have different sizes.");
        ERL_ASSERTM(n > 0, "angles and ranges are empty.");

        m_angles_world_.resize(n);

        // compute directions, end points, max valid range and hit mask
        m_dirs_world_.resize(n);
        m_end_pts_frame_.resize(n);
        m_end_pts_world_.resize(n);
        m_mask_hit_.setConstant(n, false);

        m_hit_ray_indices_.clear();
        m_hit_ray_indices_.reserve(n);
        m_hit_points_world_.clear();
        m_hit_points_world_.reserve(n);

        // #pragma omp parallel for default(none) shared(n, Eigen::Dynamic)
        for (long i = 0; i < n; ++i) {
            const double &range = m_ranges_[i];
            if (range == 0 || std::isnan(range) || std::isinf(range)) { continue; }
            const double &angle = m_angles_frame_[i];
            m_angles_world_[i] = common::WrapAnglePi(angle + m_rotation_angle_);

            // directions
            const Eigen::Vector2d &dir_frame = m_dirs_frame_[i];
            m_dirs_world_[i] << m_rotation_ * dir_frame;

            // end points
            Eigen::Vector2d &end_pt_frame = m_end_pts_frame_[i];
            end_pt_frame << range * dir_frame;
            m_end_pts_world_[i] << m_rotation_ * end_pt_frame + m_translation_;

            // max valid range
            if (range < m_setting_->valid_range_min || range > m_setting_->valid_range_max) { continue; }
            m_mask_hit_[i] = true;
        }

        m_max_valid_range_ = 0.0;
        for (long i = 0; i < n; ++i) {
            if (!m_mask_hit_[i]) { continue; }
            if (const double &range = m_ranges_[i]; range > m_max_valid_range_) { m_max_valid_range_ = range; }
            m_hit_ray_indices_.emplace_back(i);
            m_hit_points_world_.emplace_back(m_end_pts_world_[i]);
        }

        if (!partition_rays) { return; }  // do not partition rays
        PartitionRays();
    }

    void
    LidarFrame2D::ComputeClosestEndPoint(
        const Eigen::Ref<const Eigen::Vector2d> &position_world,
        long &end_point_index,
        double &distance,
        const bool brute_force) {
        if (brute_force) {
            end_point_index = -1;
            distance = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < m_end_pts_world_.size(); ++i) {
                if (const double d = (m_end_pts_world_[i] - position_world).squaredNorm(); d < distance) {
                    end_point_index = static_cast<long>(i);
                    distance = d;
                }
            }
            distance = std::sqrt(distance);
            return;
        }

        if (!m_kd_tree_->Ready()) {
            std::const_pointer_cast<KdTree2d>(m_kd_tree_)->SetDataMatrix(m_end_pts_world_[0].data(), static_cast<long>(m_end_pts_world_.size()));
        }
        end_point_index = -1;
        distance = std::numeric_limits<double>::infinity();
        m_kd_tree_->Nearest(position_world, end_point_index, distance);
        distance = std::sqrt(distance);
    }

    void
    LidarFrame2D::SampleAlongRays(
        const long n_samples_per_ray,
        const double max_in_obstacle_dist,
        const double sampled_rays_ratio,
        Eigen::Matrix2Xd &positions_world,
        Eigen::Matrix2Xd &directions_world,
        Eigen::VectorXd &distances) const {
        std::vector<long> hit_ray_indices = common::GenerateShuffledIndices<long>(m_hit_ray_indices_.size(), sampled_rays_ratio);
        const auto n_rays = static_cast<long>(hit_ray_indices.size());
        const long n_samples = n_rays * n_samples_per_ray;
        positions_world.resize(2, n_samples);
        directions_world.resize(2, n_samples);
        distances.resize(n_samples);

        long index = 0;
        for (const long &hit_ray_idx: hit_ray_indices) {
            const long ray_idx = m_hit_ray_indices_[hit_ray_idx];
            double range = m_ranges_[ray_idx];
            double range_step = (range + max_in_obstacle_dist) / static_cast<double>(n_samples_per_ray);
            const Eigen::Vector2d &dir_world = m_dirs_world_[ray_idx];

            positions_world.col(index) << m_translation_;
            directions_world.col(index) << dir_world;
            distances[index++] = range;

            Eigen::Vector2d shift = range_step * dir_world;
            for (long sample_idx_of_ray = 1; sample_idx_of_ray < n_samples_per_ray; ++sample_idx_of_ray) {
                range -= range_step;
                positions_world.col(index) << positions_world.col(index - 1) + shift;
                directions_world.col(index) << dir_world;
                distances[index++] = range;
            }
        }
    }

    void
    LidarFrame2D::SampleAlongRays(
        const double range_step,
        const double max_in_obstacle_dist,
        const double sampled_rays_ratio,
        Eigen::Matrix2Xd &positions_world,
        Eigen::Matrix2Xd &directions_world,
        Eigen::VectorXd &distances) const {
        std::vector<long> ray_indices = common::GenerateShuffledIndices<long>(m_hit_ray_indices_.size(), sampled_rays_ratio);
        const auto n_rays = static_cast<long>(ray_indices.size());
        long num_samples = 0;
        std::vector<std::pair<long, long>> n_samples_per_ray;
        n_samples_per_ray.reserve(n_rays);
        for (const long &idx: ray_indices) {
            auto ray_idx = m_hit_ray_indices_[idx];
            auto n = static_cast<long>(std::floor((m_ranges_[ray_idx] + max_in_obstacle_dist) / range_step)) + 1;
            n_samples_per_ray.emplace_back(ray_idx, n);
            num_samples += n;
        }
        positions_world.resize(2, num_samples);
        directions_world.resize(2, num_samples);
        distances.resize(num_samples);

        long sample_idx = 0;
        for (auto &[ray_idx, n_samples_of_ray]: n_samples_per_ray) {
            double range = m_ranges_[ray_idx];
            const Eigen::Vector2d &dir_world = m_dirs_world_[ray_idx];
            positions_world.col(sample_idx) << m_translation_;
            directions_world.col(sample_idx) << dir_world;
            distances[sample_idx++] = range;

            Eigen::Vector2d shift = range_step * dir_world;
            for (long sample_idx_of_ray = 1; sample_idx_of_ray < n_samples_of_ray; ++sample_idx_of_ray) {
                range -= range_step;
                positions_world.col(sample_idx) << positions_world.col(sample_idx - 1) + shift;
                directions_world.col(sample_idx) << dir_world;
                distances[sample_idx++] = range;
            }
        }
    }

    void
    LidarFrame2D::SampleNearSurface(
        const long num_samples_per_ray,
        const double max_offset,
        const double sampled_rays_ratio,
        Eigen::Matrix2Xd &positions_world,
        Eigen::Matrix2Xd &directions_world,
        Eigen::VectorXd &distances) const {
        std::vector<long> ray_indices = common::GenerateShuffledIndices<long>(m_hit_ray_indices_.size(), sampled_rays_ratio);
        const auto n_rays = static_cast<long>(ray_indices.size());
        const long n_samples = num_samples_per_ray * n_rays;
        positions_world.resize(2, n_samples);
        directions_world.resize(2, n_samples);
        distances.resize(n_samples);

        std::uniform_real_distribution<double> uniform(-max_offset, max_offset);
        long sample_idx = 0;
        for (const long &hit_ray_idx: ray_indices) {
            const long ray_idx = m_hit_ray_indices_[hit_ray_idx];
            const double range = m_ranges_[ray_idx];
            const Eigen::Vector2d &dir_world = m_dirs_world_[ray_idx];
            for (long ray_sample_idx = 0; ray_sample_idx < num_samples_per_ray; ++ray_sample_idx) {
                const double offset = uniform(common::g_random_engine);
                positions_world.col(sample_idx) << m_translation_ + (range + offset) * dir_world;
                directions_world.col(sample_idx) << dir_world;
                distances[sample_idx++] = -offset;
            }
        }
    }

    void
    LidarFrame2D::SampleInRegion(
        const long num_positions,
        const long num_along_ray_samples_per_ray,
        const long num_near_surface_samples_per_ray,
        const double max_in_obstacle_dist,
        Eigen::Matrix2Xd &positions_world,
        Eigen::Matrix2Xd &directions_world,
        Eigen::VectorXd &distances) const {
        ERL_ASSERTM(num_positions > 0, "num_positions ({}) must be positive.", num_positions);

        std::uniform_int_distribution<long> uniform_int_dist(0, static_cast<long>(m_hit_points_world_.size() - 1));
        std::uniform_real_distribution<double> uniform_real_dist(0.1, 0.8);
        std::uniform_real_distribution<double> uniform_ns(-max_in_obstacle_dist, max_in_obstacle_dist);

        const long max_num_samples = num_positions * static_cast<long>(m_hit_ray_indices_.size())  //
                                     * (num_along_ray_samples_per_ray + num_near_surface_samples_per_ray);
        positions_world.resize(2, max_num_samples);
        directions_world.resize(2, max_num_samples);
        distances.resize(max_num_samples);

        long sample_cnt = 0;
        Eigen::Matrix2Xd dirs;
        Eigen::VectorXd dists;
        std::vector<long> visible_hit_point_indices;
        for (long position_idx = 0; position_idx < num_positions; ++position_idx) {
            // synthesize a lidar scan
            const long hit_index = uniform_int_dist(common::g_random_engine);
            const long hit_ray_index = m_hit_ray_indices_[hit_index];
            double r = uniform_real_dist(common::g_random_engine);
            r *= m_ranges_[hit_ray_index];
            Eigen::Vector2d position_scan = m_translation_ + r * m_dirs_world_[hit_ray_index];
            ComputeRaysAt(position_scan, dirs, dists, visible_hit_point_indices);

            const auto num_rays = static_cast<long>(visible_hit_point_indices.size());
            if (num_rays == 0) {
                position_idx--;  // retry
                continue;
            }

            for (long ray_idx = 0; ray_idx < num_rays; ++ray_idx) {
                auto dir = dirs.col(ray_idx);
                double &range = dists[ray_idx];

                // sample near surface with this lidar scan
                for (long i = 0; i < num_near_surface_samples_per_ray; ++i) {
                    const double offset = uniform_ns(common::g_random_engine);
                    positions_world.col(sample_cnt) << position_scan + (range + offset) * dir;
                    directions_world.col(sample_cnt) << dir;
                    distances[sample_cnt++] = -offset;
                }

                // sample along rays with this lidar scan
                positions_world.col(sample_cnt) << position_scan;
                directions_world.col(sample_cnt) << dir;
                distances[sample_cnt++] = range;
                double range_step = (range + max_in_obstacle_dist) / static_cast<double>(num_along_ray_samples_per_ray);
                Eigen::Vector2d shift = range_step * dir;
                for (long i = 1; i < num_along_ray_samples_per_ray; ++i) {
                    range -= range_step;
                    positions_world.col(sample_cnt) << positions_world.col(sample_cnt - 1) + shift;
                    directions_world.col(sample_cnt) << dir;
                    distances[sample_cnt++] = range;
                }
            }
        }

        positions_world.conservativeResize(2, sample_cnt);
        directions_world.conservativeResize(2, sample_cnt);
        distances.conservativeResize(sample_cnt);
    }

    void
    LidarFrame2D::ComputeRaysAt(
        const Eigen::Ref<const Eigen::Vector2d> &position_world,
        Eigen::Matrix2Xd &directions_world,
        Eigen::VectorXd &distances,
        std::vector<long> &visible_hit_point_indices) const {
        visible_hit_point_indices.clear();
        const auto max_num_rays = static_cast<long>(m_end_pts_world_.size());
        Eigen::Matrix2Xd area_vertices(2, max_num_rays + 1);
        area_vertices.block(0, 0, 2, max_num_rays) << Eigen::Map<const Eigen::Matrix2Xd>(m_end_pts_world_.data()->data(), 2, max_num_rays);
        area_vertices.col(max_num_rays) << m_translation_;
        if (WindingNumber(position_world, area_vertices) <= 0) { return; }  // not inside

        if (directions_world.cols() < max_num_rays) { directions_world.resize(2, max_num_rays); }
        if (distances.size() < max_num_rays) { distances.resize(max_num_rays); }

        visible_hit_point_indices.reserve(max_num_rays);
        for (long ray_idx = 0; ray_idx < max_num_rays; ++ray_idx) {
            Eigen::Vector2d vec = m_end_pts_world_[ray_idx] - position_world;
            double min_dist = vec.norm();
            vec /= min_dist;
            double lam, dist;
            ComputeIntersectionBetweenRayAndSegment2D(position_world, vec, m_translation_, m_end_pts_world_[0], lam, dist);
            if (lam >= 0 && lam <= 1 && dist > 0 && dist < min_dist) { continue; }  // invalid ray
            ComputeIntersectionBetweenRayAndSegment2D(position_world, vec, m_end_pts_world_[max_num_rays - 1], m_translation_, lam, dist);
            if (lam >= 0 && lam <= 1 && dist > 0 && dist < min_dist) { continue; }  // invalid ray

            long arg_min = ray_idx;
            for (long ray_idx2 = 1; ray_idx2 < max_num_rays; ++ray_idx2) {
                if (ray_idx2 == ray_idx || ray_idx2 == ray_idx + 1) { continue; }        // skip neighboring edges
                if (!m_mask_hit_[ray_idx2 - 1] || !m_mask_hit_[ray_idx2]) { continue; }  // the vertex is not a hit
                ComputeIntersectionBetweenRayAndSegment2D(position_world, vec, m_end_pts_world_[ray_idx2 - 1], m_end_pts_world_[ray_idx2], lam, dist);
                if (lam < 0 || lam > 1) { continue; }  // the intersection is not on the segment
                if (dist > 0 && dist < min_dist) {
                    min_dist = dist;
                    arg_min = ray_idx2;
                    break;  // find closer intersection
                }
            }
            if (arg_min != ray_idx) { continue; }  // not the closest intersection

            const auto num_valid_rays = static_cast<long>(visible_hit_point_indices.size());
            directions_world.col(num_valid_rays) << vec;
            distances[num_valid_rays] = min_dist;
            visible_hit_point_indices.push_back(ray_idx);
        }
    }

    void
    LidarFrame2D::PartitionRays() {
        const long n = m_angles_frame_.size();
        // detect discontinuities, out-of-max-range measurements
        m_mask_continuous_.setConstant(n, true);
        m_mask_continuous_[0] = false;
        m_mask_continuous_[n - 1] = false;
        double rolling_range_diff = 0.0;
        const double gamma1 = m_setting_->rolling_diff_discount;
        const double gamma2 = 1 - gamma1;
        for (long i = 0; i < n; ++i) {
            const double angle = common::WrapAnglePi(m_angles_frame_[i]);
            const double range = m_ranges_[i];
            if (i == 0 || !m_mask_hit_[i]) { continue; }

            const double range_diff = std::abs((range - m_ranges_[i - 1]) / (angle - m_angles_frame_[i - 1]));  // range difference per angle
            if (range_diff > m_setting_->discontinuity_factor * rolling_range_diff) { m_mask_continuous_[i - 1] = false; }
            if (rolling_range_diff == 0.0) { rolling_range_diff = range_diff; }
            rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * range_diff;
        }
        // partition the sensor frame
        long j = 0;  // beginning index of the next partition, j-th ray must hit something
        for (long i = 0; i < n; ++i) {
            const long m = i - j + 1;
            if (!m_mask_hit_[i]) {
                if (m >= m_setting_->min_partition_size) {
                    m_partitions_.emplace_back(this, j, i - 1);  // do not include i-th ray, which does not hit anything
                }
                j = i + 1;  // maybe the next ray hit something
                continue;
            }
            if (!m_mask_continuous_[i]) {
                if (m >= m_setting_->min_partition_size) { m_partitions_.emplace_back(this, j, i); }
                j = i + 1;
            }
        }
        m_partitioned_ = true;
    }

}  // namespace erl::geometry
