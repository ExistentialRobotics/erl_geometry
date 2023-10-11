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
        Eigen::VectorXd ranges) {

        m_rotation_ = rotation;
        m_translation_ = translation;
        m_angles_frame_ = std::move(angles);
        m_ranges_ = std::move(ranges);
        m_partitions_.clear();

        long n = m_angles_frame_.size();
        ERL_DEBUG_ASSERT(n == m_ranges_.size(), "angles and ranges have different sizes.");
        ERL_DEBUG_ASSERT(n > 0, "angles and ranges are empty.");

        m_angles_world_.resize(n);
        m_rotation_angle_ = Eigen::Rotation2Dd(m_rotation_).angle();

        m_mask_hit_.setConstant(n, false);
        m_mask_continuous_.setConstant(n, true);
        m_mask_continuous_[0] = false;
        m_mask_continuous_[n - 1] = false;

        // detect discontinuities, out-of-max-range measurements
        double rolling_range_diff = 0.0;
        for (long i = 0; i < n; ++i) {
            double angle = common::ClipAngle(m_angles_frame_[i]);
            double &range = m_ranges_[i];
            m_angles_world_[i] = common::ClipAngle(angle + m_rotation_angle_);

            if (std::isnan(range) || (range < m_setting_->valid_range_min) || (range > m_setting_->valid_range_max)) { continue; }
            if ((angle < m_setting_->valid_angle_min) || (angle > m_setting_->valid_angle_max)) { continue; }

            m_mask_hit_[i] = true;
            if (range > m_max_valid_range_) { m_max_valid_range_ = range; }

            if (i == 0) { continue; }

            double range_diff = std::abs(range - m_ranges_[i - 1]);
            if (range_diff > m_setting_->discontinuity_factor * rolling_range_diff) { m_mask_continuous_[i - 1] = false; }
            rolling_range_diff = 0.9 * rolling_range_diff + 0.1 * range_diff;
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

        // compute directions, end points
        m_dirs_frame_.resize(2, n);
        m_dirs_world_.resize(2, n);
        m_end_pts_frame_.resize(2, n);
        m_end_pts_world_.resize(2, n);
        for (long i = 0; i < n; ++i) {
            double &u = m_dirs_frame_(0, i);
            double &v = m_dirs_frame_(1, i);
            u = std::cos(m_angles_frame_[i]);
            v = std::sin(m_angles_frame_[i]);
            m_dirs_world_(0, i) = m_rotation_(0, 0) * u + m_rotation_(0, 1) * v;
            m_dirs_world_(1, i) = m_rotation_(1, 0) * u + m_rotation_(1, 1) * v;

            double &x = m_end_pts_frame_(0, i);
            double &y = m_end_pts_frame_(1, i);
            x = u * m_ranges_[i];
            y = v * m_ranges_[i];
            m_end_pts_world_(0, i) = m_rotation_(0, 0) * x + m_rotation_(0, 1) * y + m_translation_[0];
            m_end_pts_world_(1, i) = m_rotation_(1, 0) * x + m_rotation_(1, 1) * y + m_translation_[1];
        }
    }

    void
    LidarFrame2D::SampleAlongRays(
        double range_step,
        double max_in_obstacle_dist,
        Eigen::Matrix2Xd &positions,
        Eigen::Matrix2Xd &directions,
        Eigen::VectorXd &distances) const {

        long n = m_angles_frame_.size();
        std::vector<Eigen::Matrix2Xd> positions_list(n);
        std::vector<Eigen::Matrix2Xd> directions_list(n);
        std::vector<Eigen::VectorXd> distances_list(n);

        const double &x = m_translation_[0];
        const double &y = m_translation_[1];
        for (long i = 0; i < n; ++i) {
            double range = m_ranges_[i];
            auto m = long(std::floor((range + max_in_obstacle_dist) / range_step)) + 1;

            auto &positions_i = positions_list[i];
            auto &directions_i = directions_list[i];
            auto &distances_i = distances_list[i];
            const double &u = m_dirs_world_(0, i);
            const double &v = m_dirs_world_(1, i);

            positions_i.resize(2, m);
            directions_i.resize(2, m);
            distances_i.resize(m);
            positions_i(0, 0) = x;
            positions_i(1, 0) = y;
            directions_i(0, 0) = u;
            directions_i(1, 0) = v;
            distances_i[0] = range;
            double shift = 0;
            for (long j = 1; j < m; ++j) {
                range -= range_step;
                shift += range_step;
                positions_i(0, j) = x + shift * u;
                positions_i(1, j) = y + shift * v;
                directions_i(0, j) = u;
                directions_i(1, j) = v;
                distances_i[j] = range;
            }
        }

        long m = 0;
        for (long i = 0; i < n; ++i) { m += positions_list[i].cols(); }
        positions.resize(2, m);
        directions.resize(2, m);
        distances.resize(m);
        for (long i = 0, index = 0; i < n; ++i) {
            auto &positions_i = positions_list[i];
            auto &directions_i = directions_list[i];
            auto &distances_i = distances_list[i];
            positions.block(0, index, 2, positions_i.cols()) = positions_i;
            directions.block(0, index, 2, directions_i.cols()) = directions_i;
            distances.segment(index, distances_i.size()) = distances_i;
            index += positions_i.cols();
        }
    }

    void
    LidarFrame2D::SampleAlongRays(
        int num_samples_per_ray,
        double max_in_obstacle_dist,
        Eigen::Matrix2Xd &positions,
        Eigen::Matrix2Xd &directions,
        Eigen::VectorXd &distances) const {

        long n = m_angles_frame_.size();
        long m = num_samples_per_ray * n;
        positions.resize(2, m);
        directions.resize(2, m);
        distances.resize(m);

        long index = 0;
        const double &x = m_translation_[0];
        const double &y = m_translation_[1];
        for (long i = 0; i < n; ++i) {
            double range = m_angles_frame_[i];
            double range_step = (range + max_in_obstacle_dist) / num_samples_per_ray;
            positions(0, index) = x;
            positions(1, index) = y;

            const double &u = m_dirs_world_(0, i);
            const double &v = m_dirs_world_(1, i);

            directions(0, index) = u;
            directions(1, index) = v;
            distances[index++] = range;
            double shift = 0;
            for (long j = 1; j < num_samples_per_ray; ++j) {
                range -= range_step;
                shift += range_step;
                positions(0, index) = x + shift * u;
                positions(1, index) = y + shift * v;
                directions(0, index) = u;
                directions(1, index) = v;
                distances[index++] = range;
            }
        }
    }

    void
    LidarFrame2D::SampleNearSurface(
        int num_samples_per_ray,
        double max_offset,
        Eigen::Matrix2Xd &positions,
        Eigen::Matrix2Xd &directions,
        Eigen::VectorXd &distances) const {

        long n = m_angles_frame_.size();
        long m = num_samples_per_ray * n;
        positions.resize(2, m);
        directions.resize(2, m);
        distances.resize(m);

        std::uniform_real_distribution<double> uniform(-max_offset, max_offset);
        long index = 0;

        const double &x = m_translation_[0];
        const double &y = m_translation_[1];
        for (long i = 0; i < n; ++i) {
            double range = m_ranges_[i];
            for (long j = 0; j < num_samples_per_ray; ++j) {
                double offset = uniform(erl::common::g_random_engine);
                const double &u = m_dirs_world_(0, i);
                const double &v = m_dirs_world_(1, i);

                positions(0, index) = x + (range + offset) * u;
                positions(1, index) = y + (range + offset) * v;
                directions(0, index) = u;
                directions(1, index) = v;
                distances[index++] = -offset;
            }
        }
    }

    void
    LidarFrame2D::SampleInRegion(int num_samples, Eigen::Matrix2Xd &positions, Eigen::Matrix2Xd &directions, Eigen::VectorXd &distances) const {
        ERL_ASSERTM(num_samples > 0, "num_samples must be positive.");

        std::uniform_real_distribution<double> distribution(-m_max_valid_range_, m_max_valid_range_);
        positions.resize(2, num_samples);
        directions.resize(2, num_samples);
        distances.resize(num_samples);

        int collected_samples = 0;
        while (collected_samples < num_samples) {
            Eigen::Vector2d position(distribution(common::g_random_engine) + m_translation_[0], distribution(common::g_random_engine) + m_translation_[1]);
            Eigen::Matrix2Xd dirs;
            Eigen::VectorXd dists;
            ComputeRaysAt(position, dirs, dists);
            if (dirs.cols() == 0) { continue; }
            int num_samples_to_collect = std::min(num_samples - collected_samples, int(dirs.cols()));
            std::vector<int> indices(dirs.cols());
            std::iota(indices.begin(), indices.end(), 0);
            std::shuffle(indices.begin(), indices.end(), common::g_random_engine);
            for (int i = 0; i < num_samples_to_collect; ++i) {
                int j = collected_samples + i;
                int k = indices[i];
                positions.col(j) = position;
                directions.col(j) = dirs.col(k);
                distances(j) = dists(k);
            }
            collected_samples += num_samples_to_collect;
        }
    }

    void
    LidarFrame2D::ComputeRaysAt(const Eigen::Ref<const Eigen::Vector2d> &position, Eigen::Matrix2Xd &directions, Eigen::VectorXd &distances) const {
        long n = m_end_pts_world_.cols();
        Eigen::Matrix2Xd area_vertices(2, n + 1);
        area_vertices.block(0, 0, 2, n) = m_end_pts_world_;
        area_vertices.col(n) = m_translation_;
        bool inside = WindingNumber(position, area_vertices) > 0;
        if (!inside) {
            directions.resize(2, 0);
            distances.resize(0);
            return;
        }

        directions.resize(2, n);
        distances.resize(n);

        long cnt = 0;
        for (long index = 0; index < n; ++index) {
            Eigen::Vector2d vec = m_end_pts_world_.col(index) - position;
            long arg_min = index;
            double min_dist = vec.norm();
            vec /= min_dist;
            double lam, dist;
            ComputeIntersectionBetweenRayAndSegment2D(position, vec, m_translation_, m_end_pts_world_.col(0), lam, dist);
            if (lam >= 0 && lam <= 1 && dist > 0 && dist < min_dist) { continue; }
            ComputeIntersectionBetweenRayAndSegment2D(position, vec, m_end_pts_world_.col(n - 1), m_translation_, lam, dist);
            if (lam >= 0 && lam <= 1 && dist > 0 && dist < min_dist) { continue; }
            for (long i = 1; i < n; ++i) {
                if (i == index || i == index + 1) { continue; }            // skip neighboring edges
                if (!m_mask_hit_[i - 1] || !m_mask_hit_[i]) { continue; }  // the vertex is not a hit
                ComputeIntersectionBetweenRayAndSegment2D(position, vec, m_end_pts_world_.col(i - 1), m_end_pts_world_.col(i), lam, dist);
                if (lam < 0 || lam > 1) { continue; }  // the intersection is not on the segment
                if (dist > 0 && dist < min_dist) {
                    min_dist = dist;
                    arg_min = i;
                    break;  // find closer intersection
                }
            }
            if (arg_min != index) { continue; }  // not the closest intersection

            directions.col(cnt) = vec;
            distances[cnt] = min_dist;
            cnt++;
        }
        directions.conservativeResize(2, cnt);
        distances.conservativeResize(cnt);
    }

}  // namespace erl::geometry
