#pragma once

#include "lidar_frame_2d.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/logging.hpp"
#include "erl_common/random.hpp"
#include "erl_common/serialization.hpp"
#include "erl_geometry/intersection.hpp"
#include "erl_geometry/winding_number.hpp"

namespace erl::geometry {

    template<typename Dtype>
    YAML::Node
    LidarFrame2D<Dtype>::Setting::YamlConvertImpl::encode(const Setting &setting) {
        YAML::Node node;
        ERL_YAML_SAVE_ATTR(node, setting, valid_range_min);
        ERL_YAML_SAVE_ATTR(node, setting, valid_range_max);
        ERL_YAML_SAVE_ATTR(node, setting, angle_min);
        ERL_YAML_SAVE_ATTR(node, setting, angle_max);
        ERL_YAML_SAVE_ATTR(node, setting, num_rays);
        ERL_YAML_SAVE_ATTR(node, setting, discontinuity_detection);
        ERL_YAML_SAVE_ATTR(node, setting, discontinuity_factor);
        ERL_YAML_SAVE_ATTR(node, setting, rolling_diff_discount);
        ERL_YAML_SAVE_ATTR(node, setting, min_partition_size);
        return node;
    }

    template<typename Dtype>
    bool
    LidarFrame2D<Dtype>::Setting::YamlConvertImpl::decode(
        const YAML::Node &node,
        Setting &setting) {
        if (!node.IsMap()) { return false; }
        ERL_YAML_LOAD_ATTR(node, setting, valid_range_min);
        ERL_YAML_LOAD_ATTR(node, setting, valid_range_max);
        ERL_YAML_LOAD_ATTR(node, setting, angle_min);
        ERL_YAML_LOAD_ATTR(node, setting, angle_max);
        ERL_YAML_LOAD_ATTR(node, setting, num_rays);
        ERL_YAML_LOAD_ATTR(node, setting, discontinuity_detection);
        ERL_YAML_LOAD_ATTR(node, setting, discontinuity_factor);
        ERL_YAML_LOAD_ATTR(node, setting, rolling_diff_discount);
        ERL_YAML_LOAD_ATTR(node, setting, min_partition_size);
        return true;
    }

    template<typename Dtype>
    long
    LidarFrame2D<Dtype>::Setting::Resize(Dtype factor) {
        num_rays = static_cast<long>(factor * num_rays);
        return num_rays;
    }

    template<typename Dtype>
    LidarFrame2D<Dtype>::Partition::Partition(
        LidarFrame2D *frame,
        const long index_begin,
        const long index_end)
        : m_frame_(frame),
          m_index_begin_(index_begin),
          m_index_end_(index_end) {
        ERL_DEBUG_ASSERT(m_frame_ != nullptr, "frame is nullptr.");
        ERL_DEBUG_ASSERT(m_index_begin_ >= 0, "index_begin is negative.");
        ERL_DEBUG_ASSERT(m_index_end_ >= 0, "index_end is negative.");
    }

    template<typename Dtype>
    long
    LidarFrame2D<Dtype>::Partition::GetIndexBegin() const {
        return m_index_begin_;
    }

    template<typename Dtype>
    long
    LidarFrame2D<Dtype>::Partition::GetIndexEnd() const {
        return m_index_end_ + 1;
    }

    template<typename Dtype>
    bool
    LidarFrame2D<Dtype>::Partition::AngleInPartition(const Dtype angle_world) const {
        const Dtype angle_frame = common::WrapAnglePi(angle_world - m_frame_->m_rotation_angle_);
        return (angle_frame >= m_frame_->m_angles_frame_[m_index_begin_]) &&
               (angle_frame <= m_frame_->m_angles_frame_[m_index_end_]);
    }

    template<typename Dtype>
    LidarFrame2D<Dtype>::LidarFrame2D(std::shared_ptr<Setting> setting)
        : m_setting_(std::move(setting)) {
        ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");

        const long n = m_setting_->num_rays;
        m_angles_frame_ = VectorX::LinSpaced(n, m_setting_->angle_min, m_setting_->angle_max);
        m_dirs_frame_.clear();
        m_dirs_frame_.reserve(n);
        for (long i = 0; i < n; ++i) {
            m_dirs_frame_.emplace_back(std::cos(m_angles_frame_[i]), std::sin(m_angles_frame_[i]));
        }
    }

    template<typename Dtype>
    bool
    LidarFrame2D<Dtype>::AngleIsInFrame(const Dtype angle_frame) const {
        return angle_frame >= m_angles_frame_[0] &&
               angle_frame <= m_angles_frame_[m_angles_frame_.size() - 1];
    }

    template<typename Dtype>
    bool
    LidarFrame2D<Dtype>::PointIsInFrame(const Vector2 &xy_frame) const {
        if (xy_frame.norm() > m_max_valid_range_) { return false; }
        const Dtype angle_frame = std::atan2(xy_frame[1], xy_frame[0]);
        return AngleIsInFrame(angle_frame);
    }

    template<typename Dtype>
    typename LidarFrame2D<Dtype>::Vector2
    LidarFrame2D<Dtype>::DirWorldToFrame(const Vector2 &dir_world) const {
        return m_rotation_.transpose() * dir_world;
    }

    template<typename Dtype>
    typename LidarFrame2D<Dtype>::Vector2
    LidarFrame2D<Dtype>::DirFrameToWorld(const Vector2 &dir_frame) const {
        return m_rotation_ * dir_frame;
    }

    template<typename Dtype>
    typename LidarFrame2D<Dtype>::Vector2
    LidarFrame2D<Dtype>::PosWorldToFrame(const Vector2 &pos_world) const {
        return m_rotation_.transpose() * (pos_world - m_translation_);
    }

    template<typename Dtype>
    typename LidarFrame2D<Dtype>::Vector2
    LidarFrame2D<Dtype>::PosFrameToWorld(const Vector2 &pos_local) const {
        return m_rotation_ * pos_local + m_translation_;
    }

    template<typename Dtype>
    void
    LidarFrame2D<Dtype>::UpdateRanges(
        const Eigen::Ref<const Matrix2> &rotation,
        const Eigen::Ref<const Vector2> &translation,
        VectorX ranges) {

        m_rotation_ = rotation;
        m_rotation_angle_ = Eigen::Rotation2D<Dtype>(m_rotation_).angle();
        m_translation_ = translation;
        m_ranges_ = std::move(ranges);
        m_partitions_.clear();
        m_partitioned_ = false;
        m_kd_tree_->Clear();

        const long n = m_angles_frame_.size();
        ERL_ASSERTM(
            n == m_ranges_.size(),
            "angles and ranges have different sizes: {} vs {}.",
            n,
            m_ranges_.size());
        ERL_ASSERTM(n > 0, "angles and ranges are empty.");

        m_angles_world_.resize(n);

        // compute directions, end points, max valid range and hit mask
        m_dirs_world_.resize(n);
        m_end_pts_frame_.resize(n);
        m_end_pts_world_.resize(n);
        m_mask_hit_.setConstant(n, false);
        m_mask_continuous_.setConstant(n, true);
        m_mask_continuous_[0] = false;      // first ray is always discontinuous
        m_mask_continuous_[n - 1] = false;  // last ray is always discontinuous
        Dtype rolling_range_diff = 0.0;

        m_hit_ray_indices_.clear();
        m_hit_ray_indices_.reserve(n);
        m_hit_points_frame_.clear();
        m_hit_points_frame_.reserve(n);
        m_hit_points_world_.clear();
        m_hit_points_world_.reserve(n);

        const Dtype valid_range_min = m_setting_->valid_range_min;
        const Dtype valid_range_max = m_setting_->valid_range_max;
        const bool discontinuity_detection = m_setting_->discontinuity_detection;
        const Dtype gamma1 = m_setting_->rolling_diff_discount;
        const Dtype gamma2 = 1 - gamma1;
        const Dtype eta = m_setting_->discontinuity_factor;

        for (long i = 0; i < n; ++i) {
            const Dtype &range = m_ranges_[i];
            if (range == 0 || !std::isfinite(range)) { continue; }
            Dtype angle = m_angles_frame_[i];
            m_angles_world_[i] = common::WrapAnglePi(angle + m_rotation_angle_);

            // directions
            const Vector2 &dir_frame = m_dirs_frame_[i];
            m_dirs_world_[i] << m_rotation_ * dir_frame;

            // end points
            Vector2 &end_pt_frame = m_end_pts_frame_[i];
            end_pt_frame << range * dir_frame;
            m_end_pts_world_[i] << m_rotation_ * end_pt_frame + m_translation_;

            // max valid range
            if (range < valid_range_min || range > valid_range_max) { continue; }
            m_mask_hit_[i] = true;

            // discontinuity detection
            if (!discontinuity_detection || i == 0 || !m_mask_hit_[i]) { continue; }
            angle = common::WrapAnglePi(angle);
            Dtype diff = std::abs((range - m_ranges_[i - 1]) / (angle - m_angles_frame_[i - 1]));
            if (diff > eta * rolling_range_diff) {
                m_mask_continuous_[i - 1] = false;
                m_mask_continuous_[i] = false;
            }
            if (rolling_range_diff == 0.0) { rolling_range_diff = diff; }
            rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * diff;
        }

        m_max_valid_range_ = 0.0;
        for (long i = 0; i < n; ++i) {
            if (!m_mask_hit_[i]) { continue; }
            if (const Dtype &range = m_ranges_[i]; range > m_max_valid_range_) {
                m_max_valid_range_ = range;
            }
            m_hit_ray_indices_.emplace_back(i);
            m_hit_points_frame_.emplace_back(m_end_pts_frame_[i]);
            m_hit_points_world_.emplace_back(m_end_pts_world_[i]);
        }
    }

    template<typename Dtype>
    typename LidarFrame2D<Dtype>::VectorX
    LidarFrame2D<Dtype>::PointCloudToRanges(
        const Matrix2 &rotation,
        const Vector2 &translation,
        const Eigen::Ref<const Matrix2X> &points,
        const bool are_local) const {
        Eigen::VectorXi frame_coords(points.cols());
        VectorX distances(points.cols());
        const Dtype angle_min = m_setting_->angle_min;
        const long n = m_setting_->num_rays;
        const Dtype angle_res = (m_setting_->angle_max - angle_min) / static_cast<Dtype>(n - 1);
        for (long i = 0; i < points.cols(); ++i) {
            Vector2 p = are_local ? Vector2(points.col(i))
                                  : Vector2(rotation.transpose() * (points.col(i) - translation));
            distances[i] = p.norm();
            Dtype angle = std::atan2(p[1], p[0]);
            frame_coords[i] = static_cast<int>((angle - angle_min) / angle_res);
        }
        VectorX ranges(n);
        ranges.setConstant(-1.0f);
        for (long i = 0; i < n; ++i) {
            int &coord = frame_coords[i];
            if (coord < 0 || coord >= n) { continue; }
            Dtype &range = ranges[i];
            if (range > 0.0f) { continue; }
            range = distances[i];
        }
        return ranges;
    }

    template<typename Dtype>
    const std::shared_ptr<typename LidarFrame2D<Dtype>::Setting> &
    LidarFrame2D<Dtype>::GetSetting() const {
        return m_setting_;
    }

    template<typename Dtype>
    long
    LidarFrame2D<Dtype>::GetNumRays() const {
        return m_angles_frame_.size();
    }

    template<typename Dtype>
    long
    LidarFrame2D<Dtype>::GetNumHitRays() const {
        return static_cast<long>(m_hit_ray_indices_.size());
    }

    template<typename Dtype>
    const typename LidarFrame2D<Dtype>::Matrix2 &
    LidarFrame2D<Dtype>::GetRotationMatrix() const {
        return m_rotation_;
    }

    template<typename Dtype>
    Dtype
    LidarFrame2D<Dtype>::GetRotationAngle() const {
        return m_rotation_angle_;
    }

    template<typename Dtype>
    const typename LidarFrame2D<Dtype>::Vector2 &
    LidarFrame2D<Dtype>::GetTranslationVector() const {
        return m_translation_;
    }

    template<typename Dtype>
    Eigen::Matrix3<Dtype>
    LidarFrame2D<Dtype>::GetPoseMatrix() const {
        Eigen::Transform<Dtype, 2, Eigen::Isometry> pose;
        pose.linear() = m_rotation_;
        pose.translation() = m_translation_;
        return pose.matrix();
    }

    template<typename Dtype>
    const typename LidarFrame2D<Dtype>::VectorX &
    LidarFrame2D<Dtype>::GetAnglesInFrame() const {
        return m_angles_frame_;
    }

    template<typename Dtype>
    const typename LidarFrame2D<Dtype>::VectorX &
    LidarFrame2D<Dtype>::GetAnglesInWorld() const {
        return m_angles_world_;
    }

    template<typename Dtype>
    const typename LidarFrame2D<Dtype>::VectorX &
    LidarFrame2D<Dtype>::GetRanges() const {
        return m_ranges_;
    }

    template<typename Dtype>
    const std::vector<typename LidarFrame2D<Dtype>::Vector2> &
    LidarFrame2D<Dtype>::GetRayDirectionsInFrame() const {
        return m_dirs_frame_;
    }

    template<typename Dtype>
    const std::vector<typename LidarFrame2D<Dtype>::Vector2> &
    LidarFrame2D<Dtype>::GetRayDirectionsInWorld() const {
        return m_dirs_world_;
    }

    template<typename Dtype>
    const std::vector<typename LidarFrame2D<Dtype>::Vector2> &
    LidarFrame2D<Dtype>::GetEndPointsInFrame() const {
        return m_end_pts_frame_;
    }

    template<typename Dtype>
    const std::vector<typename LidarFrame2D<Dtype>::Vector2> &
    LidarFrame2D<Dtype>::GetEndPointsInWorld() const {
        return m_end_pts_world_;
    }

    template<typename Dtype>
    const Eigen::VectorXb &
    LidarFrame2D<Dtype>::GetHitMask() const {
        return m_mask_hit_;
    }

    template<typename Dtype>
    const Eigen::VectorXb &
    LidarFrame2D<Dtype>::GetContinuityMask() const {
        return m_mask_continuous_;
    }

    template<typename Dtype>
    const std::vector<long> &
    LidarFrame2D<Dtype>::GetHitRayIndices() const {
        return m_hit_ray_indices_;
    }

    template<typename Dtype>
    const std::vector<typename LidarFrame2D<Dtype>::Vector2> &
    LidarFrame2D<Dtype>::GetHitPointsFrame() const {
        return m_hit_points_frame_;
    }

    template<typename Dtype>
    const std::vector<typename LidarFrame2D<Dtype>::Vector2> &
    LidarFrame2D<Dtype>::GetHitPointsWorld() const {
        return m_hit_points_world_;
    }

    template<typename Dtype>
    Dtype
    LidarFrame2D<Dtype>::GetMaxValidRange() const {
        return m_max_valid_range_;
    }

    template<typename Dtype>
    const std::vector<typename LidarFrame2D<Dtype>::Partition> &
    LidarFrame2D<Dtype>::GetPartitions() const {
        ERL_ASSERTM(m_partitioned_, "LidarFrame2D::GetPartitions() is called before partitioning.");
        return m_partitions_;
    }

    template<typename Dtype>
    bool
    LidarFrame2D<Dtype>::IsPartitioned() const {
        return m_partitioned_;
    }

    template<typename Dtype>
    bool
    LidarFrame2D<Dtype>::IsValid() const {
        return m_max_valid_range_ > 0;
    }

    template<typename Dtype>
    void
    LidarFrame2D<Dtype>::ComputeClosestEndPoint(
        const Eigen::Ref<const Vector2> &position_world,
        long &end_point_index,
        Dtype &distance,
        const bool brute_force) {
        if (brute_force) {
            end_point_index = -1;
            distance = std::numeric_limits<Dtype>::infinity();
            for (std::size_t i = 0; i < m_end_pts_world_.size(); ++i) {
                if (const Dtype d = (m_end_pts_world_[i] - position_world).squaredNorm();
                    d < distance) {
                    end_point_index = static_cast<long>(i);
                    distance = d;
                }
            }
            distance = std::sqrt(distance);
            return;
        }

        if (!m_kd_tree_->Ready()) {
            std::const_pointer_cast<KdTree>(m_kd_tree_)
                ->SetDataMatrix(
                    m_end_pts_world_[0].data(),
                    static_cast<long>(m_end_pts_world_.size()));
        }
        end_point_index = -1;
        distance = std::numeric_limits<Dtype>::infinity();
        m_kd_tree_->Nearest(position_world, end_point_index, distance);
        distance = std::sqrt(distance);
    }

    template<typename Dtype>
    void
    LidarFrame2D<Dtype>::SampleAlongRays(
        const long n_samples_per_ray,
        const Dtype max_in_obstacle_dist,
        const Dtype sampled_rays_ratio,
        Matrix2X &positions_world,
        Matrix2X &directions_world,
        VectorX &distances) const {

        std::vector<long> hit_ray_indices =
            common::GenerateShuffledIndices<long>(m_hit_ray_indices_.size(), sampled_rays_ratio);
        const auto n_rays = static_cast<long>(hit_ray_indices.size());
        const long n_samples = n_rays * n_samples_per_ray;
        positions_world.resize(2, n_samples);
        directions_world.resize(2, n_samples);
        distances.resize(n_samples);

        long index = 0;
        for (const long &hit_ray_idx: hit_ray_indices) {
            const long ray_idx = m_hit_ray_indices_[hit_ray_idx];
            Dtype range = m_ranges_[ray_idx];
            Dtype range_step =
                (range + max_in_obstacle_dist) / static_cast<Dtype>(n_samples_per_ray);
            const Vector2 &dir_world = m_dirs_world_[ray_idx];

            positions_world.col(index) << m_translation_;
            directions_world.col(index) << dir_world;
            distances[index++] = range;

            Vector2 shift = range_step * dir_world;
            for (long sample_idx_of_ray = 1; sample_idx_of_ray < n_samples_per_ray;
                 ++sample_idx_of_ray) {
                range -= range_step;
                positions_world.col(index) << positions_world.col(index - 1) + shift;
                directions_world.col(index) << dir_world;
                distances[index++] = range;
            }
        }
    }

    template<typename Dtype>
    void
    LidarFrame2D<Dtype>::SampleAlongRays(
        const Dtype range_step,
        const Dtype max_in_obstacle_dist,
        const Dtype sampled_rays_ratio,
        Matrix2X &positions_world,
        Matrix2X &directions_world,
        VectorX &distances) const {

        std::vector<long> ray_indices =
            common::GenerateShuffledIndices<long>(m_hit_ray_indices_.size(), sampled_rays_ratio);
        const auto n_rays = static_cast<long>(ray_indices.size());
        long num_samples = 0;
        std::vector<std::pair<long, long>> n_samples_per_ray;
        n_samples_per_ray.reserve(n_rays);
        for (const long &idx: ray_indices) {
            auto ray_idx = m_hit_ray_indices_[idx];
            auto n = static_cast<long>(
                         std::floor((m_ranges_[ray_idx] + max_in_obstacle_dist) / range_step)) +
                     1;
            n_samples_per_ray.emplace_back(ray_idx, n);
            num_samples += n;
        }
        positions_world.resize(2, num_samples);
        directions_world.resize(2, num_samples);
        distances.resize(num_samples);

        long sample_idx = 0;
        for (auto &[ray_idx, n_samples_of_ray]: n_samples_per_ray) {
            Dtype range = m_ranges_[ray_idx];
            const Vector2 &dir_world = m_dirs_world_[ray_idx];
            positions_world.col(sample_idx) << m_translation_;
            directions_world.col(sample_idx) << dir_world;
            distances[sample_idx++] = range;

            Vector2 shift = range_step * dir_world;
            for (long sample_idx_of_ray = 1; sample_idx_of_ray < n_samples_of_ray;
                 ++sample_idx_of_ray) {
                range -= range_step;
                positions_world.col(sample_idx) << positions_world.col(sample_idx - 1) + shift;
                directions_world.col(sample_idx) << dir_world;
                distances[sample_idx++] = range;
            }
        }
    }

    template<typename Dtype>
    void
    LidarFrame2D<Dtype>::SampleNearSurface(
        const long num_samples_per_ray,
        const Dtype max_offset,
        const Dtype sampled_rays_ratio,
        Matrix2X &positions_world,
        Matrix2X &directions_world,
        VectorX &distances) const {
        std::vector<long> ray_indices =
            common::GenerateShuffledIndices<long>(m_hit_ray_indices_.size(), sampled_rays_ratio);
        const auto n_rays = static_cast<long>(ray_indices.size());
        const long n_samples = num_samples_per_ray * n_rays;
        positions_world.resize(2, n_samples);
        directions_world.resize(2, n_samples);
        distances.resize(n_samples);

        std::uniform_real_distribution<Dtype> uniform(-max_offset, max_offset);
        long sample_idx = 0;
        for (const long &hit_ray_idx: ray_indices) {
            const long ray_idx = m_hit_ray_indices_[hit_ray_idx];
            const Dtype range = m_ranges_[ray_idx];
            const Vector2 &dir_world = m_dirs_world_[ray_idx];
            for (long ray_sample_idx = 0; ray_sample_idx < num_samples_per_ray; ++ray_sample_idx) {
                const Dtype offset = uniform(common::g_random_engine);
                positions_world.col(sample_idx) << m_translation_ + (range + offset) * dir_world;
                directions_world.col(sample_idx) << dir_world;
                distances[sample_idx++] = -offset;
            }
        }
    }

    template<typename Dtype>
    void
    LidarFrame2D<Dtype>::SampleInRegion(
        const long num_positions,
        const long num_along_ray_samples_per_ray,
        const long num_near_surface_samples_per_ray,
        const Dtype max_in_obstacle_dist,
        Matrix2X &positions_world,
        Matrix2X &directions_world,
        VectorX &distances) const {
        ERL_ASSERTM(num_positions > 0, "num_positions ({}) must be positive.", num_positions);

        std::uniform_int_distribution<long> uniform_int_dist(
            0,
            static_cast<long>(m_hit_points_world_.size() - 1));
        std::uniform_real_distribution<Dtype> uniform_real_dist(0.1, 0.8);
        std::uniform_real_distribution<Dtype> uniform_ns(
            -max_in_obstacle_dist,
            max_in_obstacle_dist);

        const long max_num_samples =
            num_positions * static_cast<long>(m_hit_ray_indices_.size())  //
            * (num_along_ray_samples_per_ray + num_near_surface_samples_per_ray);
        positions_world.resize(2, max_num_samples);
        directions_world.resize(2, max_num_samples);
        distances.resize(max_num_samples);

        long sample_cnt = 0;
        Matrix2X dirs;
        VectorX dists;
        std::vector<long> visible_hit_point_indices;
        (void) visible_hit_point_indices;  // suppress warning
        for (long position_idx = 0; position_idx < num_positions; ++position_idx) {
            // synthesize a lidar scan
            const long hit_index = uniform_int_dist(common::g_random_engine);
            const long hit_ray_index = m_hit_ray_indices_[hit_index];
            Dtype r = uniform_real_dist(common::g_random_engine);
            r *= m_ranges_[hit_ray_index];
            Vector2 position_scan = m_translation_ + r * m_dirs_world_[hit_ray_index];
            ComputeRaysAt(position_scan, dirs, dists, visible_hit_point_indices);

            const auto num_rays = static_cast<long>(visible_hit_point_indices.size());
            if (num_rays == 0) {
                position_idx--;  // retry
                continue;
            }

            for (long ray_idx = 0; ray_idx < num_rays; ++ray_idx) {
                auto dir = dirs.col(ray_idx);
                Dtype &range = dists[ray_idx];

                // sample near surface with this lidar scan
                for (long i = 0; i < num_near_surface_samples_per_ray; ++i) {
                    const Dtype offset = uniform_ns(common::g_random_engine);
                    positions_world.col(sample_cnt) << position_scan + (range + offset) * dir;
                    directions_world.col(sample_cnt) << dir;
                    distances[sample_cnt++] = -offset;
                }

                // sample along rays with this lidar scan
                positions_world.col(sample_cnt) << position_scan;
                directions_world.col(sample_cnt) << dir;
                distances[sample_cnt++] = range;
                Dtype range_step = (range + max_in_obstacle_dist) /
                                   static_cast<Dtype>(num_along_ray_samples_per_ray);
                Vector2 shift = range_step * dir;
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

    template<typename Dtype>
    void
    LidarFrame2D<Dtype>::ComputeRaysAt(
        const Eigen::Ref<const Vector2> &position_world,
        Matrix2X &directions_world,
        VectorX &distances,
        std::vector<long> &visible_hit_point_indices) const {

        visible_hit_point_indices.clear();
        const auto max_num_rays = static_cast<long>(m_end_pts_world_.size());
        Matrix2X area_vertices(2, max_num_rays + 1);
        area_vertices.block(0, 0, 2, max_num_rays)
            << Eigen::Map<const Matrix2X>(m_end_pts_world_.data()->data(), 2, max_num_rays);
        area_vertices.col(max_num_rays) << m_translation_;
        if (WindingNumber<Dtype>(position_world, area_vertices) <= 0) { return; }  // not inside

        if (directions_world.cols() < max_num_rays) { directions_world.resize(2, max_num_rays); }
        if (distances.size() < max_num_rays) { distances.resize(max_num_rays); }

        visible_hit_point_indices.reserve(max_num_rays);
        for (long ray_idx = 0; ray_idx < max_num_rays; ++ray_idx) {
            Vector2 vec = m_end_pts_world_[ray_idx] - position_world;
            Dtype min_dist = vec.norm();
            vec /= min_dist;
            Dtype lam, dist;
            bool intersected;
            ComputeIntersectionBetweenRayAndLine2D<Dtype>(
                position_world,
                vec,
                m_translation_,
                m_end_pts_world_[0],
                lam,
                dist,
                intersected);
            if (intersected && lam >= 0 && lam <= 1 && dist > 0 && dist < min_dist) {
                continue;  // invalid ray
            }
            ComputeIntersectionBetweenRayAndLine2D<Dtype>(
                position_world,
                vec,
                m_end_pts_world_[max_num_rays - 1],
                m_translation_,
                lam,
                dist,
                intersected);
            if (intersected && lam >= 0 && lam <= 1 && dist > 0 && dist < min_dist) {
                continue;  // invalid ray
            }

            long arg_min = ray_idx;
            for (long ray_idx2 = 1; ray_idx2 < max_num_rays; ++ray_idx2) {
                if (ray_idx2 == ray_idx || ray_idx2 == ray_idx + 1) {
                    continue;  // skip neighboring edges
                }
                if (!m_mask_hit_[ray_idx2 - 1] || !m_mask_hit_[ray_idx2]) {
                    continue;  // the vertex is not a hit
                }
                ComputeIntersectionBetweenRayAndLine2D<Dtype>(
                    position_world,
                    vec,
                    m_end_pts_world_[ray_idx2 - 1],
                    m_end_pts_world_[ray_idx2],
                    lam,
                    dist,
                    intersected);
                if (!intersected || lam < 0 || lam > 1) {
                    continue;  // the intersection is not on the segment
                }
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

    template<typename Dtype>
    bool
    LidarFrame2D<Dtype>::operator==(const LidarFrame2D &other) const {
        if (m_setting_ == nullptr && other.m_setting_ != nullptr) { return false; }
        if (m_setting_ != nullptr &&
            (other.m_setting_ == nullptr || *m_setting_ != *other.m_setting_)) {
            return false;
        }
        if (m_rotation_ != other.m_rotation_) { return false; }
        if (m_rotation_angle_ != other.m_rotation_angle_) { return false; }
        if (m_translation_ != other.m_translation_) { return false; }
        using namespace common;
        if (!SafeEigenMatrixEqual(m_angles_frame_, other.m_angles_frame_)) { return false; }
        if (!SafeEigenMatrixEqual(m_angles_world_, other.m_angles_world_)) { return false; }
        if (!SafeEigenMatrixEqual(m_ranges_, other.m_ranges_)) { return false; }

        if (m_dirs_frame_ != other.m_dirs_frame_) { return false; }
        if (m_dirs_world_ != other.m_dirs_world_) { return false; }
        if (m_end_pts_frame_ != other.m_end_pts_frame_) { return false; }
        if (m_end_pts_world_ != other.m_end_pts_world_) { return false; }

        if (!SafeEigenMatrixEqual(m_mask_hit_, other.m_mask_hit_)) { return false; }
        if (!SafeEigenMatrixEqual(m_mask_continuous_, other.m_mask_continuous_)) { return false; }

        if (m_hit_ray_indices_ != other.m_hit_ray_indices_) { return false; }
        if (m_hit_points_frame_ != other.m_hit_points_frame_) { return false; }
        if (m_hit_points_world_ != other.m_hit_points_world_) { return false; }
        if (m_max_valid_range_ != other.m_max_valid_range_) { return false; }
        if (m_partitioned_ != other.m_partitioned_) { return false; }
        if (m_partitions_.size() != other.m_partitions_.size()) { return false; }
        for (std::size_t i = 0; i < m_partitions_.size(); ++i) {
            const auto &partition = m_partitions_[i];
            const auto &other_partition = other.m_partitions_[i];
            if (partition.m_index_begin_ != other_partition.m_index_begin_ ||
                partition.m_index_end_ != other_partition.m_index_end_) {
                return false;
            }
        }
        return true;
    }

    template<typename Dtype>
    bool
    LidarFrame2D<Dtype>::operator!=(const LidarFrame2D &other) const {
        return !(*this == other);
    }

    template<typename Dtype>
    bool
    LidarFrame2D<Dtype>::Write(std::ostream &s) const {
        static const common::TokenWriteFunctionPairs<LidarFrame2D> token_function_pairs = {
            {
                "setting",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    return self->m_setting_->Write(stream) && stream.good();
                },
            },
            {
                "rotation",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    return common::SaveEigenMatrixToBinaryStream(stream, self->m_rotation_) &&
                           stream.good();
                },
            },
            {
                "rotation_angle",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    stream.write(
                        reinterpret_cast<const char *>(&self->m_rotation_angle_),
                        sizeof(Dtype));
                    return stream.good();
                },
            },
            {
                "translation",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    return common::SaveEigenMatrixToBinaryStream(stream, self->m_translation_) &&
                           stream.good();
                },
            },
            {
                "angles_frame",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    return common::SaveEigenMatrixToBinaryStream(stream, self->m_angles_frame_) &&
                           stream.good();
                },
            },
            {
                "angles_world",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    return common::SaveEigenMatrixToBinaryStream(stream, self->m_angles_world_) &&
                           stream.good();
                },
            },
            {
                "ranges",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    return common::SaveEigenMatrixToBinaryStream(stream, self->m_ranges_) &&
                           stream.good();
                },
            },
            {
                "dirs_frame",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    return common::SaveVectorOfEigenMatricesToBinaryStream(
                               stream,
                               self->m_dirs_frame_) &&
                           stream.good();
                },
            },
            {
                "dirs_world",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    return common::SaveVectorOfEigenMatricesToBinaryStream(
                               stream,
                               self->m_dirs_world_) &&
                           stream.good();
                },
            },
            {
                "end_pts_frame",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    return common::SaveVectorOfEigenMatricesToBinaryStream(
                               stream,
                               self->m_end_pts_frame_) &&
                           stream.good();
                },
            },
            {
                "end_pts_world",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    return common::SaveVectorOfEigenMatricesToBinaryStream(
                               stream,
                               self->m_end_pts_world_) &&
                           stream.good();
                },
            },
            {
                "mask_hit",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    return common::SaveEigenMatrixToBinaryStream(stream, self->m_mask_hit_) &&
                           stream.good();
                },
            },
            {
                "mask_continuous",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    return common::SaveEigenMatrixToBinaryStream(
                               stream,
                               self->m_mask_continuous_) &&
                           stream.good();
                },
            },
            {
                "hit_ray_indices",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    const std::size_t n_rays = self->m_hit_ray_indices_.size();
                    stream.write(reinterpret_cast<const char *>(&n_rays), sizeof(std::size_t));
                    stream.write(
                        reinterpret_cast<const char *>(self->m_hit_ray_indices_.data()),
                        static_cast<std::streamsize>(n_rays * sizeof(long)));
                    return stream.good();
                },
            },
            {
                "hit_points_frame",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    return common::SaveVectorOfEigenMatricesToBinaryStream(
                               stream,
                               self->m_hit_points_frame_) &&
                           stream.good();
                },
            },
            {
                "hit_points_world",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    return common::SaveVectorOfEigenMatricesToBinaryStream(
                               stream,
                               self->m_hit_points_world_) &&
                           stream.good();
                },
            },
            {
                "max_valid_range",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    stream.write(
                        reinterpret_cast<const char *>(&self->m_max_valid_range_),
                        sizeof(Dtype));
                    return stream.good();
                },
            },
            {
                "partitioned",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    stream.write(
                        reinterpret_cast<const char *>(&self->m_partitioned_),
                        sizeof(bool));
                    return stream.good();
                },
            },
            {
                "partitions",
                [](const LidarFrame2D *self, std::ostream &stream) {
                    stream << self->m_partitions_.size();
                    for (const auto &partition: self->m_partitions_) {
                        stream << ' ' << partition.m_index_begin_ << ' ' << partition.m_index_end_;
                    }
                    return stream.good();
                },
            },
        };
        return common::WriteTokens(s, this, token_function_pairs);
    }

    template<typename Dtype>
    bool
    LidarFrame2D<Dtype>::Read(std::istream &s) {
        static const common::TokenReadFunctionPairs<LidarFrame2D> token_function_pairs = {
            {
                "setting",
                [](LidarFrame2D *self, std::istream &stream) {
                    return self->m_setting_->Read(stream) && stream.good();
                },
            },
            {
                "rotation",
                [](LidarFrame2D *self, std::istream &stream) {
                    return common::LoadEigenMatrixFromBinaryStream(stream, self->m_rotation_) &&
                           stream.good();
                },
            },
            {
                "rotation_angle",
                [](LidarFrame2D *self, std::istream &stream) {
                    stream.read(reinterpret_cast<char *>(&self->m_rotation_angle_), sizeof(Dtype));
                    return stream.good();
                },
            },
            {
                "translation",
                [](LidarFrame2D *self, std::istream &stream) {
                    return common::LoadEigenMatrixFromBinaryStream(stream, self->m_translation_) &&
                           stream.good();
                },
            },
            {
                "angles_frame",
                [](LidarFrame2D *self, std::istream &stream) {
                    return common::LoadEigenMatrixFromBinaryStream(stream, self->m_angles_frame_) &&
                           stream.good();
                },
            },
            {
                "angles_world",
                [](LidarFrame2D *self, std::istream &stream) {
                    return common::LoadEigenMatrixFromBinaryStream(stream, self->m_angles_world_) &&
                           stream.good();
                },
            },
            {
                "ranges",
                [](LidarFrame2D *self, std::istream &stream) {
                    return common::LoadEigenMatrixFromBinaryStream(stream, self->m_ranges_) &&
                           stream.good();
                },
            },
            {
                "dirs_frame",
                [](LidarFrame2D *self, std::istream &stream) {
                    return common::LoadVectorOfEigenMatricesFromBinaryStream(
                               stream,
                               self->m_dirs_frame_) &&
                           stream.good();
                },
            },
            {
                "dirs_world",
                [](LidarFrame2D *self, std::istream &stream) {
                    return common::LoadVectorOfEigenMatricesFromBinaryStream(
                               stream,
                               self->m_dirs_world_) &&
                           stream.good();
                },
            },
            {
                "end_pts_frame",
                [](LidarFrame2D *self, std::istream &stream) {
                    return common::LoadVectorOfEigenMatricesFromBinaryStream(
                               stream,
                               self->m_end_pts_frame_) &&
                           stream.good();
                },
            },
            {
                "end_pts_world",
                [](LidarFrame2D *self, std::istream &stream) {
                    return common::LoadVectorOfEigenMatricesFromBinaryStream(
                               stream,
                               self->m_end_pts_world_) &&
                           stream.good();
                },
            },
            {
                "mask_hit",
                [](LidarFrame2D *self, std::istream &stream) {
                    return common::LoadEigenMatrixFromBinaryStream(stream, self->m_mask_hit_) &&
                           stream.good();
                },
            },
            {
                "mask_continuous",
                [](LidarFrame2D *self, std::istream &stream) {
                    return common::LoadEigenMatrixFromBinaryStream(
                               stream,
                               self->m_mask_continuous_) &&
                           stream.good();
                },
            },
            {
                "hit_ray_indices",
                [](LidarFrame2D *self, std::istream &stream) {
                    std::size_t n;
                    stream.read(reinterpret_cast<char *>(&n), sizeof(std::size_t));
                    self->m_hit_ray_indices_.resize(n);
                    stream.read(
                        reinterpret_cast<char *>(self->m_hit_ray_indices_.data()),
                        static_cast<std::streamsize>(n * sizeof(long)));
                    return stream.good();
                },
            },
            {
                "hit_points_frame",
                [](LidarFrame2D *self, std::istream &stream) {
                    return common::LoadVectorOfEigenMatricesFromBinaryStream(
                               stream,
                               self->m_hit_points_frame_) &&
                           stream.good();
                },
            },
            {
                "hit_points_world",
                [](LidarFrame2D *self, std::istream &stream) {
                    return common::LoadVectorOfEigenMatricesFromBinaryStream(
                               stream,
                               self->m_hit_points_world_) &&
                           stream.good();
                },
            },
            {
                "max_valid_range",
                [](LidarFrame2D *self, std::istream &stream) {
                    stream.read(reinterpret_cast<char *>(&self->m_max_valid_range_), sizeof(Dtype));
                    return stream.good();
                },
            },
            {
                "partitioned",
                [](LidarFrame2D *self, std::istream &stream) {
                    stream.read(reinterpret_cast<char *>(&self->m_partitioned_), sizeof(bool));
                    return stream.good();
                },
            },
            {
                "partitions",
                [](LidarFrame2D *self, std::istream &stream) {
                    long n;
                    stream >> n;
                    self->m_partitions_.reserve(n);
                    for (long i = 0; i < n; ++i) {
                        long index_begin, index_end;
                        stream >> index_begin >> index_end;
                        self->m_partitions_.emplace_back(self, index_begin, index_end);
                    }
                    return stream.good();
                },
            },
        };
        return common::ReadTokens(s, this, token_function_pairs);
    }

    template<typename Dtype>
    void
    LidarFrame2D<Dtype>::PartitionRays() {
        if (m_hit_ray_indices_.empty()) { return; }
        const long n = m_angles_frame_.size();
        if (!m_setting_->discontinuity_detection) {
            // detect discontinuities, out-of-max-range measurements
            m_mask_continuous_.setConstant(n, true);
            m_mask_continuous_[0] = false;
            m_mask_continuous_[n - 1] = false;
            Dtype rolling_range_diff = 0.0;
            const Dtype gamma1 = m_setting_->rolling_diff_discount;
            const Dtype gamma2 = 1 - gamma1;
            const Dtype eta = m_setting_->discontinuity_factor;
            for (long i = 0; i < n; ++i) {
                const Dtype angle = common::WrapAnglePi(m_angles_frame_[i]);
                const Dtype range = m_ranges_[i];
                if (i == 0 || !m_mask_hit_[i]) { continue; }

                const Dtype range_diff =  // range difference per angle
                    std::abs((range - m_ranges_[i - 1]) / (angle - m_angles_frame_[i - 1]));
                if (range_diff > eta * rolling_range_diff) { m_mask_continuous_[i - 1] = false; }
                if (rolling_range_diff == 0.0) { rolling_range_diff = range_diff; }
                rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * range_diff;
            }
        }
        // partition the sensor frame
        long j = 0;  // beginning index of the next partition, j-th ray must hit something
        const long min_partition_size = m_setting_->min_partition_size;
        for (long i = 0; i < n; ++i) {
            const long m = i - j + 1;
            if (!m_mask_hit_[i]) {
                // do not include i-th ray, which does not hit anything
                if (m >= min_partition_size) { m_partitions_.emplace_back(this, j, i - 1); }
                j = i + 1;  // maybe the next ray hit something
                continue;
            }
            if (!m_mask_continuous_[i]) {
                if (m >= min_partition_size) { m_partitions_.emplace_back(this, j, i); }
                j = i + 1;
            }
        }
        m_partitioned_ = true;
    }

}  // namespace erl::geometry
