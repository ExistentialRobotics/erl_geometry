#include "erl_common/assert.hpp"
#include "erl_geometry/lidar_frame_2d.hpp"
#include "erl_geometry/utils.hpp"
#include "erl_geometry/winding_number.hpp"

namespace erl::geometry {

    void
    LidarFrame2D::Update(
        const Eigen::Ref<const Eigen::Matrix2d> &rotation,
        const Eigen::Ref<const Eigen::Vector2d> &translation,
        Eigen::VectorXd angles,
        Eigen::VectorXd ranges,
        bool partition_rays
    ) {
        m_rotation_ = rotation;
        m_translation_ = translation;
        m_angles_frame_ = std::move(angles);
        m_ranges_ = std::move(ranges);
        m_partitions_.clear();
        m_partitioned_ = false;
        m_kd_tree_->Clear();

        long n = m_angles_frame_.size();
        ERL_ASSERTM(n == m_ranges_.size(), "angles and ranges have different sizes.");
        ERL_ASSERTM(n > 0, "angles and ranges are empty.");

        m_angles_world_.resize(n);
        m_rotation_angle_ = Eigen::Rotation2Dd(m_rotation_).angle();

        // compute directions, end points, max valid range and hit mask
        m_dirs_frame_.resize(2, n);
        m_dirs_world_.resize(2, n);
        m_end_pts_frame_.resize(2, n);
        m_end_pts_world_.resize(2, n);
        m_mask_hit_.setConstant(n, false);
        m_hit_ray_indices_.resize(n);
        m_hit_points_world_.resize(2, n);

        m_max_valid_range_ = 0.0;
        long num_hit_points = 0;
        for (long i = 0; i < n; ++i) {
            double angle = common::ClipAngle(m_angles_frame_[i]);
            double &range = m_ranges_[i];
            m_angles_world_[i] = common::ClipAngle(angle + m_rotation_angle_);

            // directions
            auto dir_frame = m_dirs_frame_.col(i);
            dir_frame << std::cos(angle), std::sin(angle);
            m_dirs_world_.col(i) << m_rotation_ * dir_frame;

            // end points
            auto end_pt_frame = m_end_pts_frame_.col(i);
            end_pt_frame << range * dir_frame;
            m_end_pts_world_.col(i) << m_rotation_ * end_pt_frame + m_translation_;

            // max valid range
            if (std::isnan(range) || (range < m_setting_->valid_range_min) || (range > m_setting_->valid_range_max)) { continue; }
            if ((angle < m_setting_->valid_angle_min) || (angle > m_setting_->valid_angle_max)) { continue; }
            m_mask_hit_[i] = true;
            if (range > m_max_valid_range_) { m_max_valid_range_ = range; }
            m_hit_ray_indices_[num_hit_points] = i;
            m_hit_points_world_.col(num_hit_points++) << m_end_pts_world_.col(i);
        }
        m_hit_ray_indices_.conservativeResize(num_hit_points);
        m_hit_points_world_.conservativeResize(2, num_hit_points);

        if (!partition_rays) { return; }  // do not partition rays
        PartitionRays();
    }

    void
    LidarFrame2D::ComputeClosestEndPoint(const Eigen::Ref<const Eigen::Vector2d> &position_world, long &end_point_index, double &distance, bool brute_force)
        const {
        if (brute_force) {
            end_point_index = -1;
            distance = std::numeric_limits<double>::infinity();
            long n_vertices = m_end_pts_world_.cols();
            for (long i = 0; i < n_vertices; ++i) {
                double d = (m_end_pts_world_.col(i) - position_world).squaredNorm();
                if (d < distance) {
                    end_point_index = i;
                    distance = d;
                }
            }
            distance = std::sqrt(distance);
            return;
        }

        if (!m_kd_tree_->Ready()) { std::const_pointer_cast<KdTree2d>(m_kd_tree_)->SetDataMatrix(m_end_pts_world_); }
        end_point_index = -1;
        distance = std::numeric_limits<double>::infinity();
        m_kd_tree_->Knn(1, position_world, end_point_index, distance);
        distance = std::sqrt(distance);
    }

    void
    LidarFrame2D::SampleAlongRays(
        long n_samples_per_ray,
        double max_in_obstacle_dist,
        double sampled_rays_ratio,
        Eigen::Matrix2Xd &positions_world,
        Eigen::Matrix2Xd &directions_world,
        Eigen::VectorXd &distances
    ) const {
        std::vector<long> hit_ray_indices = common::GenerateShuffledIndices<long>(m_hit_ray_indices_.size(), sampled_rays_ratio);
        auto n_rays = long(hit_ray_indices.size());
        long n_samples = n_rays * n_samples_per_ray;
        positions_world.resize(2, n_samples);
        directions_world.resize(2, n_samples);
        distances.resize(n_samples);

        long index = 0;
        for (long &hit_ray_idx: hit_ray_indices) {
            const long &kRayIdx = m_hit_ray_indices_[hit_ray_idx];
            double range = m_ranges_[kRayIdx];
            double range_step = (range + max_in_obstacle_dist) / double(n_samples_per_ray);
            const Eigen::Vector2d &kDirWorld = m_dirs_world_.col(kRayIdx);

            positions_world.col(index) << m_translation_;
            directions_world.col(index) << kDirWorld;
            distances[index++] = range;

            Eigen::Vector2d shift = range_step * kDirWorld;
            for (long sample_idx_of_ray = 1; sample_idx_of_ray < n_samples_per_ray; ++sample_idx_of_ray) {
                range -= range_step;
                positions_world.col(index) << positions_world.col(index - 1) + shift;
                directions_world.col(index) << kDirWorld;
                distances[index++] = range;
            }
        }
    }

    void
    LidarFrame2D::SampleAlongRays(
        double range_step,
        double max_in_obstacle_dist,
        double sampled_rays_ratio,
        Eigen::Matrix2Xd &positions_world,
        Eigen::Matrix2Xd &directions_world,
        Eigen::VectorXd &distances
    ) const {
        std::vector<long> ray_indices = common::GenerateShuffledIndices<long>(m_hit_ray_indices_.size(), sampled_rays_ratio);
        auto n_rays = long(ray_indices.size());
        long num_samples = 0;
        std::vector<std::pair<long, long>> n_samples_per_ray;
        n_samples_per_ray.reserve(n_rays);
        for (long &idx: ray_indices) {
            auto ray_idx = m_hit_ray_indices_[idx];
            auto n = long(std::floor((m_ranges_[ray_idx] + max_in_obstacle_dist) / range_step)) + 1;
            n_samples_per_ray.emplace_back(ray_idx, n);
            num_samples += n;
        }
        positions_world.resize(2, num_samples);
        directions_world.resize(2, num_samples);
        distances.resize(num_samples);

        long sample_idx = 0;
        for (auto &[ray_idx, n_samples_of_ray]: n_samples_per_ray) {
            double range = m_ranges_[ray_idx];
            const Eigen::Vector2d &kDirWorld = m_dirs_world_.col(ray_idx);
            positions_world.col(sample_idx) << m_translation_;
            directions_world.col(sample_idx) << kDirWorld;
            distances[sample_idx++] = range;

            Eigen::Vector2d shift = range_step * kDirWorld;
            for (long sample_idx_of_ray = 1; sample_idx_of_ray < n_samples_of_ray; ++sample_idx_of_ray) {
                range -= range_step;
                positions_world.col(sample_idx) << positions_world.col(sample_idx - 1) + shift;
                directions_world.col(sample_idx) << kDirWorld;
                distances[sample_idx++] = range;
            }
        }
    }

    void
    LidarFrame2D::SampleNearSurface(
        long num_samples_per_ray,
        double max_offset,
        double sampled_rays_ratio,
        Eigen::Matrix2Xd &positions_world,
        Eigen::Matrix2Xd &directions_world,
        Eigen::VectorXd &distances
    ) const {
        std::vector<long> ray_indices = common::GenerateShuffledIndices<long>(m_hit_ray_indices_.size(), sampled_rays_ratio);
        auto n_rays = long(ray_indices.size());
        long n_samples = num_samples_per_ray * n_rays;
        positions_world.resize(2, n_samples);
        directions_world.resize(2, n_samples);
        distances.resize(n_samples);

        std::uniform_real_distribution<double> uniform(-max_offset, max_offset);
        long sample_idx = 0;
        for (long &hit_ray_idx: ray_indices) {
            const long &kRayIdx = m_hit_ray_indices_[hit_ray_idx];
            const double &kRange = m_ranges_[kRayIdx];
            auto dir_world = m_dirs_world_.col(kRayIdx);
            for (long ray_sample_idx = 0; ray_sample_idx < num_samples_per_ray; ++ray_sample_idx) {
                double offset = uniform(erl::common::g_random_engine);
                positions_world.col(sample_idx) << m_translation_ + (kRange + offset) * dir_world;
                directions_world.col(sample_idx) << dir_world;
                distances[sample_idx++] = -offset;
            }
        }
    }

    void
    LidarFrame2D::SampleInRegion(
        long num_samples, long num_samples_per_iter, Eigen::Matrix2Xd &positions_world, Eigen::Matrix2Xd &directions_world, Eigen::VectorXd &distances
    ) const {
        ERL_ASSERTM(num_samples > 0, "num_samples (%ld) must be positive.", num_samples);
        if (num_samples_per_iter < 0) { num_samples_per_iter = num_samples; }

        std::uniform_real_distribution<double> distribution(-m_max_valid_range_, m_max_valid_range_);
        positions_world.resize(2, num_samples);
        directions_world.resize(2, num_samples);
        distances.resize(num_samples);

        long collected_samples = 0;
        Eigen::Vector2d position;
        Eigen::Matrix2Xd dirs;
        Eigen::VectorXd dists;
        std::vector<long> visible_hit_point_indices;
        while (collected_samples < num_samples) {
            position << distribution(common::g_random_engine) + m_translation_[0], distribution(common::g_random_engine) + m_translation_[1];

            ComputeRaysAt(position, dirs, dists, visible_hit_point_indices);
            auto num_rays = long(visible_hit_point_indices.size());
            if (num_rays == 0) { continue; }

            long num_samples_to_collect = std::min(num_samples - collected_samples, num_samples_per_iter);
            if (num_samples_to_collect < num_rays) {  // too many samples
                std::vector<long> indices(num_rays);
                std::iota(indices.begin(), indices.end(), 0);
                std::shuffle(indices.begin(), indices.end(), common::g_random_engine);
                for (long i = 0; i < num_samples_to_collect; ++i) {
                    long &ray_idx = indices[i];
                    positions_world.col(collected_samples) << position;
                    directions_world.col(collected_samples) << dirs.col(ray_idx);
                    distances(collected_samples++) = dists[ray_idx];
                    if (collected_samples >= num_samples) { break; }
                }
            } else {
                for (long ray_idx = 0; ray_idx < num_rays; ++ray_idx) {
                    positions_world.col(collected_samples) << position;
                    directions_world.col(collected_samples) << dirs.col(ray_idx);
                    distances(collected_samples++) = dists[ray_idx];
                    if (collected_samples >= num_samples) { break; }
                }
            }
        }
    }

    void
    LidarFrame2D::ComputeRaysAt(
        const Eigen::Ref<const Eigen::Vector2d> &position_world,
        Eigen::Matrix2Xd &directions_world,
        Eigen::VectorXd &distances,
        std::vector<long> &visible_hit_point_indices
    ) const {
        visible_hit_point_indices.clear();
        long max_num_rays = m_end_pts_world_.cols();
        Eigen::Matrix2Xd area_vertices(2, max_num_rays + 1);
        area_vertices.block(0, 0, 2, max_num_rays) << m_end_pts_world_;
        area_vertices.col(max_num_rays) << m_translation_;
        bool inside = WindingNumber(position_world, area_vertices) > 0;
        if (!inside) { return; }

        if (directions_world.cols() < max_num_rays) { directions_world.resize(2, max_num_rays); }
        if (distances.size() < max_num_rays) { distances.resize(max_num_rays); }

        visible_hit_point_indices.reserve(max_num_rays);
        for (long ray_idx = 0; ray_idx < max_num_rays; ++ray_idx) {
            Eigen::Vector2d vec = m_end_pts_world_.col(ray_idx) - position_world;
            double min_dist = vec.norm();
            vec /= min_dist;
            double lam, dist;
            ComputeIntersectionBetweenRayAndSegment2D(position_world, vec, m_translation_, m_end_pts_world_.col(0), lam, dist);
            if (lam >= 0 && lam <= 1 && dist > 0 && dist < min_dist) { continue; }  // invalid ray
            ComputeIntersectionBetweenRayAndSegment2D(position_world, vec, m_end_pts_world_.col(max_num_rays - 1), m_translation_, lam, dist);
            if (lam >= 0 && lam <= 1 && dist > 0 && dist < min_dist) { continue; }  // invalid ray

            long arg_min = ray_idx;
            for (long ray_idx2 = 1; ray_idx2 < max_num_rays; ++ray_idx2) {
                if (ray_idx2 == ray_idx || ray_idx2 == ray_idx + 1) { continue; }        // skip neighboring edges
                if (!m_mask_hit_[ray_idx2 - 1] || !m_mask_hit_[ray_idx2]) { continue; }  // the vertex is not a hit
                ComputeIntersectionBetweenRayAndSegment2D(position_world, vec, m_end_pts_world_.col(ray_idx2 - 1), m_end_pts_world_.col(ray_idx2), lam, dist);
                if (lam < 0 || lam > 1) { continue; }  // the intersection is not on the segment
                if (dist > 0 && dist < min_dist) {
                    min_dist = dist;
                    arg_min = ray_idx2;
                    break;  // find closer intersection
                }
            }
            if (arg_min != ray_idx) { continue; }  // not the closest intersection

            auto num_valid_rays = long(visible_hit_point_indices.size());
            directions_world.col(num_valid_rays) << vec;
            distances[num_valid_rays] = min_dist;
            visible_hit_point_indices.push_back(ray_idx);
        }
    }

    void
    LidarFrame2D::PartitionRays() {
        long n = m_angles_frame_.size();
        // detect discontinuities, out-of-max-range measurements
        m_mask_continuous_.setConstant(n, true);
        m_mask_continuous_[0] = false;
        m_mask_continuous_[n - 1] = false;
        double rolling_range_diff = 0.0;
        double gamma1 = m_setting_->rolling_diff_discount;
        double gamma2 = 1 - gamma1;
        for (long i = 0; i < n; ++i) {
            double angle = common::ClipAngle(m_angles_frame_[i]);
            double &range = m_ranges_[i];
            if (i == 0 || !m_mask_hit_[i]) { continue; }

            double range_diff = std::abs((range - m_ranges_[i - 1]) / (angle - m_angles_frame_[i - 1]));  // range difference per angle
            if (range_diff > m_setting_->discontinuity_factor * rolling_range_diff) { m_mask_continuous_[i - 1] = false; }
            if (rolling_range_diff == 0.0) { rolling_range_diff = range_diff; }
            rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * range_diff;
        }
        // partition the sensor frame
        long j = 0;  // beginning index of the next partition, j-th ray must hit something
        for (long i = 0; i < n; ++i) {
            long m = i - j + 1;
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
