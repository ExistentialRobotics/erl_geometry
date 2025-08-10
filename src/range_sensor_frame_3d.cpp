#include "erl_geometry/range_sensor_frame_3d.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/random.hpp"
#include "erl_common/serialization.hpp"
#include "erl_geometry/hidden_point_removal.hpp"

namespace erl::geometry {
    template<typename Dtype>
    YAML::Node
    RangeSensorFrame3D<Dtype>::Setting::YamlConvertImpl::encode(const Setting &setting) {
        YAML::Node node;
        ERL_YAML_SAVE_ATTR(node, setting, row_margin);
        ERL_YAML_SAVE_ATTR(node, setting, col_margin);
        ERL_YAML_SAVE_ATTR(node, setting, valid_range_min);
        ERL_YAML_SAVE_ATTR(node, setting, valid_range_max);
        return node;
    }

    template<typename Dtype>
    bool
    RangeSensorFrame3D<Dtype>::Setting::YamlConvertImpl::decode(
        const YAML::Node &node,
        Setting &setting) {
        if (!node.IsMap()) { return false; }
        ERL_YAML_LOAD_ATTR(node, setting, row_margin);
        ERL_YAML_LOAD_ATTR(node, setting, col_margin);
        ERL_YAML_LOAD_ATTR(node, setting, valid_range_min);
        ERL_YAML_LOAD_ATTR(node, setting, valid_range_max);
        return true;
    }

    template<typename Dtype>
    RangeSensorFrame3D<Dtype>::RangeSensorFrame3D(std::shared_ptr<Setting> setting)
        : m_setting_(std::move(setting)) {
        ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr when creating RangeSensorFrame3D.");
    }

    template<typename Dtype>
    std::shared_ptr<RangeSensorFrame3D<Dtype>>
    RangeSensorFrame3D<Dtype>::Create(
        const std::string &type,
        const std::shared_ptr<Setting> &setting) {
        return Factory::GetInstance().Create(type, setting);
    }

    template<typename Dtype>
    long
    RangeSensorFrame3D<Dtype>::GetNumRays() const {
        return m_ranges_.size();
    }

    template<typename Dtype>
    long
    RangeSensorFrame3D<Dtype>::GetNumHitRays() const {
        return static_cast<long>(m_hit_ray_indices_.size());
    }

    template<typename Dtype>
    const typename RangeSensorFrame3D<Dtype>::Matrix3 &
    RangeSensorFrame3D<Dtype>::GetRotationMatrix() const {
        return m_rotation_;
    }

    template<typename Dtype>
    const typename RangeSensorFrame3D<Dtype>::Vector3 &
    RangeSensorFrame3D<Dtype>::GetTranslationVector() const {
        return m_translation_;
    }

    template<typename Dtype>
    typename RangeSensorFrame3D<Dtype>::Matrix4
    RangeSensorFrame3D<Dtype>::GetPoseMatrix() const {
        Eigen::Transform<Dtype, 3, Eigen::Isometry> pose;
        pose.linear() = m_rotation_;
        pose.translation() = m_translation_;
        return pose.matrix();
    }

    template<typename Dtype>
    const Eigen::MatrixX<typename RangeSensorFrame3D<Dtype>::Vector2> &
    RangeSensorFrame3D<Dtype>::GetFrameCoords() const {
        return m_frame_coords_;
    }

    template<typename Dtype>
    bool
    RangeSensorFrame3D<Dtype>::PosIsInFrame(const Vector3 &xyz_frame) const {
        Vector2 frame_coords;
        Dtype dist;
        if (!ComputeFrameCoords(xyz_frame, dist, frame_coords)) { return false; }
        return CoordsIsInFrame(frame_coords);
    }

    template<typename Dtype>
    bool
    RangeSensorFrame3D<Dtype>::CoordsIsInFrame(const Vector2 &frame_coords) const {
        const Vector2 &top_left = m_frame_coords_(m_setting_->row_margin, m_setting_->col_margin);
        const Vector2 &bottom_right = m_frame_coords_(  //
            m_frame_coords_.rows() - m_setting_->row_margin - 1,
            m_frame_coords_.cols() - m_setting_->col_margin - 1);
        return frame_coords[0] >= top_left[0] && frame_coords[0] <= bottom_right[0] &&  //
               frame_coords[1] >= top_left[1] && frame_coords[1] <= bottom_right[1];
    }

    template<typename Dtype>
    typename RangeSensorFrame3D<Dtype>::Vector3
    RangeSensorFrame3D<Dtype>::DirWorldToFrame(const Vector3 &dir_world) const {
        return m_rotation_.transpose() * dir_world;
    }

    template<typename Dtype>
    typename RangeSensorFrame3D<Dtype>::Vector3
    RangeSensorFrame3D<Dtype>::DirFrameToWorld(const Vector3 &dir_frame) const {
        return m_rotation_ * dir_frame;
    }

    template<typename Dtype>
    typename RangeSensorFrame3D<Dtype>::Vector3
    RangeSensorFrame3D<Dtype>::PosWorldToFrame(const Vector3 &pos_world) const {
        return m_rotation_.transpose() * (pos_world - m_translation_);
    }

    template<typename Dtype>
    typename RangeSensorFrame3D<Dtype>::Vector3
    RangeSensorFrame3D<Dtype>::PosFrameToWorld(const Vector3 &pos_frame) const {
        return m_rotation_ * pos_frame + m_translation_;
    }

    template<typename Dtype>
    const typename RangeSensorFrame3D<Dtype>::MatrixX &
    RangeSensorFrame3D<Dtype>::GetRanges() const {
        return m_ranges_;
    }

    template<typename Dtype>
    const Eigen::MatrixX<typename RangeSensorFrame3D<Dtype>::Vector3> &
    RangeSensorFrame3D<Dtype>::GetRayDirectionsInFrame() const {
        return m_dirs_frame_;
    }

    template<typename Dtype>
    const Eigen::MatrixX<typename RangeSensorFrame3D<Dtype>::Vector3> &
    RangeSensorFrame3D<Dtype>::GetRayDirectionsInWorld() const {
        return m_dirs_world_;
    }

    template<typename Dtype>
    const Eigen::MatrixX<typename RangeSensorFrame3D<Dtype>::Vector3> &
    RangeSensorFrame3D<Dtype>::GetEndPointsInFrame() const {
        return m_end_pts_frame_;
    }

    template<typename Dtype>
    const Eigen::MatrixX<typename RangeSensorFrame3D<Dtype>::Vector3> &
    RangeSensorFrame3D<Dtype>::GetEndPointsInWorld() const {
        return m_end_pts_world_;
    }

    template<typename Dtype>
    const std::vector<std::pair<long, long>> &
    RangeSensorFrame3D<Dtype>::GetHitRayIndices() const {
        return m_hit_ray_indices_;
    }

    template<typename Dtype>
    const std::vector<typename RangeSensorFrame3D<Dtype>::Vector3> &
    RangeSensorFrame3D<Dtype>::GetHitPointsFrame() const {
        return m_hit_points_frame_;
    }

    template<typename Dtype>
    const std::vector<typename RangeSensorFrame3D<Dtype>::Vector3> &
    RangeSensorFrame3D<Dtype>::GetHitPointsWorld() const {
        return m_hit_points_world_;
    }

    template<typename Dtype>
    Dtype
    RangeSensorFrame3D<Dtype>::GetMinValidRange() const {
        return m_setting_->valid_range_min;
    }

    template<typename Dtype>
    Dtype
    RangeSensorFrame3D<Dtype>::GetMaxValidRange() const {
        return m_max_valid_range_;
    }

    template<typename Dtype>
    const Eigen::MatrixXb &
    RangeSensorFrame3D<Dtype>::GetHitMask() const {
        return m_mask_hit_;
    }

    template<typename Dtype>
    bool
    RangeSensorFrame3D<Dtype>::IsValid() const {
        return m_max_valid_range_ > 0.0;
    }

    template<typename Dtype>
    void
    RangeSensorFrame3D<Dtype>::ComputeClosestEndPoint(
        const Eigen::Ref<const Vector3> &position_world,
        long &end_point_row_index,
        long &end_point_col_index,
        Dtype &distance,
        const bool brute_force) {
        if (brute_force) {
            end_point_row_index = -1;
            end_point_col_index = -1;
            distance = std::numeric_limits<Dtype>::infinity();
            const long rows = m_end_pts_world_.rows();
            const long cols = m_end_pts_world_.cols();
            for (long col = 0; col < cols; ++col) {
                for (long row = 0; row < rows; ++row) {
                    if (const Dtype d = (m_end_pts_world_(row, col) - position_world).squaredNorm();
                        d < distance) {
                        end_point_row_index = row;
                        end_point_col_index = col;
                        distance = d;
                    }
                }
            }
            distance = std::sqrt(distance);
            return;
        }

        if (!m_kd_tree_->Ready()) {
            std::const_pointer_cast<KdTree>(m_kd_tree_)
                ->SetDataMatrix(m_end_pts_world_.data()->data(), m_end_pts_world_.size());
        }
        long end_point_index = -1;
        distance = std::numeric_limits<Dtype>::infinity();
        m_kd_tree_->Nearest(position_world, end_point_index, distance);
        const long rows = m_end_pts_world_.rows();
        end_point_col_index = end_point_index / rows;
        end_point_row_index = end_point_index - end_point_col_index * rows;
        distance = std::sqrt(distance);
    }

    template<typename Dtype>
    void
    RangeSensorFrame3D<Dtype>::SampleAlongRays(
        const long num_samples_per_ray,
        const Dtype max_in_obstacle_dist,
        const Dtype sampled_rays_ratio,
        Matrix3X &positions_world,
        Matrix3X &directions_world,
        VectorX &distances) const {

        const std::vector<long> ray_indices =
            common::GenerateShuffledIndices<long>(GetNumHitRays(), sampled_rays_ratio);
        const auto n_rays = static_cast<long>(ray_indices.size());
        const long n_samples = n_rays * num_samples_per_ray;
        positions_world.resize(3, n_samples);
        directions_world.resize(3, n_samples);
        distances.resize(n_samples);

        long index = 0;
        for (const long &ray_idx: ray_indices) {
            const auto [azimuth_idx, elevation_idx] = m_hit_ray_indices_[ray_idx];
            Dtype range = m_ranges_(azimuth_idx, elevation_idx);
            Dtype range_step =
                (range + max_in_obstacle_dist) / static_cast<Dtype>(num_samples_per_ray);
            const Vector3 &dir_world = m_dirs_world_(azimuth_idx, elevation_idx);

            positions_world.col(index) << m_translation_;
            directions_world.col(index) << dir_world;
            distances[index++] = range;

            Vector3 shift = range_step * dir_world;
            for (long i = 1; i < num_samples_per_ray; ++i) {
                range -= range_step;
                positions_world.col(index) << positions_world.col(index - 1) + shift;
                directions_world.col(index) << dir_world;
                distances[index++] = range;
            }
        }
    }

    template<typename Dtype>
    void
    RangeSensorFrame3D<Dtype>::SampleAlongRays(
        const Dtype range_step,
        const Dtype max_in_obstacle_dist,
        const Dtype sampled_rays_ratio,
        Matrix3X &positions_world,
        Matrix3X &directions_world,
        VectorX &distances) const {

        const std::vector<long> ray_indices =
            common::GenerateShuffledIndices<long>(GetNumHitRays(), sampled_rays_ratio);
        const auto n_rays = static_cast<long>(ray_indices.size());
        long n_samples = 0;
        std::vector<std::pair<std::pair<long, long>, long>> n_samples_per_ray;
        n_samples_per_ray.reserve(n_rays);
        for (const long &ray_idx: ray_indices) {
            const auto [azimuth_idx, elevation_idx] = m_hit_ray_indices_[ray_idx];
            auto n =
                static_cast<long>(std::floor(
                    (m_ranges_(azimuth_idx, elevation_idx) + max_in_obstacle_dist) / range_step)) +
                1;
            n_samples_per_ray.emplace_back(std::make_pair(azimuth_idx, elevation_idx), n);
            n_samples += n;
        }
        positions_world.resize(3, n_samples);
        directions_world.resize(3, n_samples);
        distances.resize(n_samples);

        long sample_idx = 0;
        for (auto &[ray_idx, n_samples_of_ray]: n_samples_per_ray) {
            auto &[azimuth_idx, elevation_idx] = ray_idx;

            Dtype range = m_ranges_(azimuth_idx, elevation_idx);
            const Vector3 &dir_world = m_dirs_world_(azimuth_idx, elevation_idx);
            positions_world.col(sample_idx) << m_translation_;
            directions_world.col(sample_idx) << dir_world;
            distances[sample_idx++] = range;

            Vector3 shift = range_step * dir_world;
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
    RangeSensorFrame3D<Dtype>::SampleNearSurface(
        const long num_samples_per_ray,
        const Dtype max_offset,
        const Dtype sampled_rays_ratio,
        Matrix3X &positions_world,
        Matrix3X &directions_world,
        VectorX &distances) const {
        const std::vector<long> hit_ray_indices =
            common::GenerateShuffledIndices<long>(GetNumHitRays(), sampled_rays_ratio);
        const auto n_rays = static_cast<long>(hit_ray_indices.size());
        const long n_samples = n_rays * num_samples_per_ray;
        positions_world.resize(3, n_samples);
        directions_world.resize(3, n_samples);
        distances.resize(n_samples);

        std::uniform_real_distribution<Dtype> uniform(-max_offset, max_offset);
        long sample_idx = 0;
        for (const long &hit_ray_idx: hit_ray_indices) {
            const auto [azimuth_idx, elevation_idx] = m_hit_ray_indices_[hit_ray_idx];
            const Vector3 &dir_world = m_dirs_world_(azimuth_idx, elevation_idx);
            for (long i = 0; i < num_samples_per_ray; ++i) {
                const Dtype offset = uniform(common::g_random_engine);
                positions_world.col(sample_idx)
                    << m_translation_ +
                           (m_ranges_(azimuth_idx, elevation_idx) + offset) * dir_world;
                directions_world.col(sample_idx) << dir_world;
                distances[sample_idx++] = -offset;
            }
        }
    }

    template<typename Dtype>
    void
    RangeSensorFrame3D<Dtype>::SampleInRegionHpr(
        long num_positions,
        long num_along_ray_samples_per_ray,
        long num_near_surface_samples_per_ray,
        Dtype max_in_obstacle_dist,
        Matrix3X &positions_world,
        Matrix3X &directions_world,
        VectorX &distances,
        const bool parallel) const {
        ERL_ASSERTM(num_positions > 0, "num_positions ({}) must be positive.", num_positions);

        if (parallel) {
            const uint32_t num_threads = std::thread::hardware_concurrency();
            const long num_positions_per_thread = num_positions / num_threads;
            const long leftover = num_positions - num_positions_per_thread * num_threads;
            std::vector<std::thread> threads;
            threads.reserve(num_threads);
            std::vector<Matrix3X> positions_world_buffers(num_threads);
            std::vector<Matrix3X> directions_world_buffers(num_threads);
            std::vector<VectorX> distances_buffers(num_threads);
            const uint64_t seed = common::g_random_engine();
            for (uint32_t thread_idx = 0; thread_idx < num_threads; ++thread_idx) {
                threads.emplace_back(
                    &RangeSensorFrame3D::SampleInRegionHprThread,
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
                const long num_samples_to_copy = positions.cols();
                if (num_samples_to_copy == 0) { continue; }
                positions_world.block(0, copied_samples, 3, num_samples_to_copy) << positions;
                directions_world.block(0, copied_samples, 3, num_samples_to_copy) << directions;
                distances.segment(copied_samples, num_samples_to_copy)
                    << distances_buffers[thread_idx];
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
        ERL_DEBUG("{} positions, {} samples collected.", num_positions, positions_world.cols());
    }

    template<typename Dtype>
    void
    RangeSensorFrame3D<Dtype>::SampleInRegionVrs(
        long num_hit_points,
        long num_samples_per_azimuth_segment,
        long num_azimuth_segments,
        Matrix3X &positions_world,
        Matrix3X &directions_world,
        VectorX &distances,
        const bool parallel) const {

        ERL_ASSERTM(num_hit_points > 0, "num_hit_points ({}) must be positive.", num_hit_points);

        std::vector<long> selected_hit_ray_indices(GetNumHitRays());
        std::iota(selected_hit_ray_indices.begin(), selected_hit_ray_indices.end(), 0);
        num_hit_points = std::min(num_hit_points, GetNumHitRays());
        if (num_hit_points < static_cast<long>(selected_hit_ray_indices.size())) {
            std::shuffle(
                selected_hit_ray_indices.begin(),
                selected_hit_ray_indices.end(),
                common::g_random_engine);
            selected_hit_ray_indices.resize(num_hit_points);
        }

        if (parallel) {
            const uint32_t num_threads = std::thread::hardware_concurrency();
            const long num_positions_per_thread = num_hit_points / num_threads;
            const long leftover = num_hit_points - num_positions_per_thread * num_threads;
            std::vector<std::thread> threads;
            threads.reserve(num_threads);
            std::vector<Matrix3X> positions_world_buffers(num_threads);
            std::vector<Matrix3X> directions_world_buffers(num_threads);
            std::vector<VectorX> distances_buffers(num_threads);
            const uint64_t seed = common::g_random_engine();
            for (long thread_idx = 0, start = 0; thread_idx < num_threads; ++thread_idx) {
                const long end = start + num_positions_per_thread + (thread_idx < leftover ? 1 : 0);
                threads.emplace_back(
                    &RangeSensorFrame3D::SampleInRegionVrsThread,
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
                Matrix3X &positions = positions_world_buffers[thread_idx];
                Matrix3X &directions = directions_world_buffers[thread_idx];
                const long num_samples_to_copy = positions.cols();
                if (num_samples_to_copy == 0) { continue; }
                positions_world.block(0, copied_samples, 3, num_samples_to_copy) << positions;
                directions_world.block(0, copied_samples, 3, num_samples_to_copy) << directions;
                distances.segment(copied_samples, num_samples_to_copy)
                    << distances_buffers[thread_idx];
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

    template<typename Dtype>
    void
    RangeSensorFrame3D<Dtype>::ComputeRaysAt(
        const Eigen::Ref<const Vector3> &position_world,
        Matrix3X &directions_world,
        VectorX &distances,
        std::vector<long> &visible_hit_point_indices) const {
        ERL_ASSERTM(!std::isinf(m_max_valid_range_), "max valid range is not set.");
        Dtype radius = (m_max_valid_range_ + (position_world - m_translation_).norm()) * 10.0f;
        visible_hit_point_indices.clear();
        HiddenPointRemoval(
            m_hit_points_world_,
            position_world,
            radius,
            visible_hit_point_indices,
            true,
            false);
        if (const auto num_visible_hit_points = static_cast<long>(visible_hit_point_indices.size());
            directions_world.cols() < num_visible_hit_points) {
            directions_world.resize(3, num_visible_hit_points);
            distances.resize(num_visible_hit_points);
        }
        long sample_idx = 0;
        for (const long &index: visible_hit_point_indices) {
            auto dir = directions_world.col(sample_idx);
            Dtype &distance = distances[sample_idx++];

            dir << m_hit_points_world_[index] - position_world;
            distance = dir.norm();
            dir /= distance;
            (void) dir;
        }
    }

    template<typename Dtype>
    bool
    RangeSensorFrame3D<Dtype>::operator==(const RangeSensorFrame3D &other) const {
        if (m_setting_ == nullptr && other.m_setting_ != nullptr) { return false; }
        if (m_setting_ != nullptr &&
            (other.m_setting_ == nullptr || *m_setting_ != *other.m_setting_)) {
            return false;
        }
        if (m_rotation_ != other.m_rotation_) { return false; }
        if (m_translation_ != other.m_translation_) { return false; }

        using namespace common;
        if (!SafeEigenMatrixEqual(m_frame_coords_, other.m_frame_coords_)) { return false; }
        if (!SafeEigenMatrixEqual(m_ranges_, other.m_ranges_)) { return false; }
        if (!SafeEigenMatrixEqual(m_dirs_frame_, other.m_dirs_frame_)) { return false; }
        if (!SafeEigenMatrixEqual(m_dirs_world_, other.m_dirs_world_)) { return false; }
        if (!SafeEigenMatrixEqual(m_end_pts_frame_, other.m_end_pts_frame_)) { return false; }
        if (!SafeEigenMatrixEqual(m_end_pts_world_, other.m_end_pts_world_)) { return false; }
        if (!SafeEigenMatrixEqual(m_mask_hit_, other.m_mask_hit_)) { return false; }

        if (m_hit_ray_indices_ != other.m_hit_ray_indices_) { return false; }
        if (m_hit_points_frame_ != other.m_hit_points_frame_) { return false; }
        if (m_hit_points_world_ != other.m_hit_points_world_) { return false; }
        if (m_max_valid_range_ != other.m_max_valid_range_) { return false; }
        return true;
    }

    template<typename Dtype>
    bool
    RangeSensorFrame3D<Dtype>::operator!=(const RangeSensorFrame3D &other) const {
        return !(*this == other);
    }

    template<typename Dtype>
    bool
    RangeSensorFrame3D<Dtype>::Write(std::ostream &s) const {
        using namespace common;
        static const TokenWriteFunctionPairs<RangeSensorFrame3D> token_function_pairs = {
            {
                "setting",
                [](const RangeSensorFrame3D *self, std::ostream &stream) {
                    return self->m_setting_->Write(stream) && stream.good();
                },
            },
            {
                "rotation",
                [](const RangeSensorFrame3D *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_rotation_) &&
                           stream.good();
                },
            },
            {
                "translation",
                [](const RangeSensorFrame3D *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_translation_) &&
                           stream.good();
                },
            },
            {
                "frame_coords",
                [](const RangeSensorFrame3D *self, std::ostream &stream) {
                    return SaveEigenMatrixOfEigenMatricesToBinaryStream(
                               stream,
                               self->m_frame_coords_) &&
                           stream.good();
                },
            },
            {
                "ranges",
                [](const RangeSensorFrame3D *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_ranges_) && stream.good();
                },
            },
            {
                "dirs_frame",
                [](const RangeSensorFrame3D *self, std::ostream &stream) {
                    return SaveEigenMatrixOfEigenMatricesToBinaryStream(
                               stream,
                               self->m_dirs_frame_) &&
                           stream.good();
                },
            },
            {
                "dirs_world",
                [](const RangeSensorFrame3D *self, std::ostream &stream) {
                    return SaveEigenMatrixOfEigenMatricesToBinaryStream(
                               stream,
                               self->m_dirs_world_) &&
                           stream.good();
                },
            },
            {
                "end_pts_frame",
                [](const RangeSensorFrame3D *self, std::ostream &stream) {
                    return SaveEigenMatrixOfEigenMatricesToBinaryStream(
                               stream,
                               self->m_end_pts_frame_) &&
                           stream.good();
                },
            },
            {
                "end_pts_world",
                [](const RangeSensorFrame3D *self, std::ostream &stream) {
                    return SaveEigenMatrixOfEigenMatricesToBinaryStream(
                               stream,
                               self->m_end_pts_world_) &&
                           stream.good();
                },
            },
            {
                "mask_hit",
                [](const RangeSensorFrame3D *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_mask_hit_) &&
                           stream.good();
                },
            },
            {
                "hit_ray_indices",
                [](const RangeSensorFrame3D *self, std::ostream &stream) {
                    stream << self->m_hit_ray_indices_.size() << '\n';
                    for (const auto &[row, col]: self->m_hit_ray_indices_) {
                        stream << row << " " << col << '\n';
                    }
                    return stream.good();
                },
            },
            {
                "hit_points_frame",
                [](const RangeSensorFrame3D *self, std::ostream &stream) {
                    return SaveVectorOfEigenMatricesToBinaryStream(
                               stream,
                               self->m_hit_points_frame_) &&
                           stream.good();
                },
            },
            {
                "hit_points_world",
                [](const RangeSensorFrame3D *self, std::ostream &stream) {
                    return SaveVectorOfEigenMatricesToBinaryStream(
                               stream,
                               self->m_hit_points_world_) &&
                           stream.good();
                },
            },
            {
                "max_valid_range",
                [](const RangeSensorFrame3D *self, std::ostream &stream) {
                    stream.write(
                        reinterpret_cast<const char *>(&self->m_max_valid_range_),
                        sizeof(Dtype));
                    return stream.good();
                },
            },
        };
        return WriteTokens(s, this, token_function_pairs);
    }

    template<typename Dtype>
    bool
    RangeSensorFrame3D<Dtype>::Read(std::istream &s) {
        using namespace common;
        static const TokenReadFunctionPairs<RangeSensorFrame3D> token_function_pairs = {
            {
                "setting",
                [](RangeSensorFrame3D *self, std::istream &stream) {
                    return self->m_setting_->Read(stream) && stream.good();
                },
            },
            {
                "rotation",
                [](RangeSensorFrame3D *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_rotation_) &&
                           stream.good();
                },
            },
            {
                "translation",
                [](RangeSensorFrame3D *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_translation_) &&
                           stream.good();
                },
            },
            {
                "frame_coords",
                [](RangeSensorFrame3D *self, std::istream &stream) {
                    return LoadEigenMatrixOfEigenMatricesFromBinaryStream(
                               stream,
                               self->m_frame_coords_) &&
                           stream.good();
                },
            },
            {
                "ranges",
                [](RangeSensorFrame3D *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_ranges_) &&
                           stream.good();
                },
            },
            {
                "dirs_frame",
                [](RangeSensorFrame3D *self, std::istream &stream) {
                    return LoadEigenMatrixOfEigenMatricesFromBinaryStream(
                               stream,
                               self->m_dirs_frame_) &&
                           stream.good();
                },
            },
            {
                "dirs_world",
                [](RangeSensorFrame3D *self, std::istream &stream) {
                    return LoadEigenMatrixOfEigenMatricesFromBinaryStream(
                               stream,
                               self->m_dirs_world_) &&
                           stream.good();
                },
            },
            {
                "end_pts_frame",
                [](RangeSensorFrame3D *self, std::istream &stream) {
                    return LoadEigenMatrixOfEigenMatricesFromBinaryStream(
                               stream,
                               self->m_end_pts_frame_) &&
                           stream.good();
                },
            },
            {
                "end_pts_world",
                [](RangeSensorFrame3D *self, std::istream &stream) {
                    return LoadEigenMatrixOfEigenMatricesFromBinaryStream(
                               stream,
                               self->m_end_pts_world_) &&
                           stream.good();
                },
            },
            {
                "mask_hit",
                [](RangeSensorFrame3D *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_mask_hit_) &&
                           stream.good();
                },
            },
            {
                "hit_ray_indices",
                [](RangeSensorFrame3D *self, std::istream &stream) {
                    long num_hit_ray_indices;
                    stream >> num_hit_ray_indices;
                    self->m_hit_ray_indices_.resize(num_hit_ray_indices);
                    for (long i = 0; i < num_hit_ray_indices; ++i) {
                        stream >> self->m_hit_ray_indices_[i].first >>
                            self->m_hit_ray_indices_[i].second;
                    }
                    SkipLine(stream);
                    return stream.good();
                },
            },
            {
                "hit_points_frame",
                [](RangeSensorFrame3D *self, std::istream &stream) {
                    return LoadVectorOfEigenMatricesFromBinaryStream(
                               stream,
                               self->m_hit_points_frame_) &&
                           stream.good();
                },
            },
            {
                "hit_points_world",
                [](RangeSensorFrame3D *self, std::istream &stream) {
                    return LoadVectorOfEigenMatricesFromBinaryStream(
                               stream,
                               self->m_hit_points_world_) &&
                           stream.good();
                },
            },
            {
                "max_valid_range",
                [](RangeSensorFrame3D *self, std::istream &stream) {
                    stream.read(reinterpret_cast<char *>(&self->m_max_valid_range_), sizeof(Dtype));
                    return stream.good();
                },
            },
        };
        return ReadTokens(s, this, token_function_pairs);
    }

    template<typename Dtype>
    void
    RangeSensorFrame3D<Dtype>::SampleInRegionHprThread(
        const uint64_t seed,
        const long num_positions,
        const long num_along_ray_samples_per_ray,
        const long num_near_surface_samples_per_ray,
        const Dtype max_in_obstacle_dist,
        Matrix3X *positions_world_ptr,
        Matrix3X *directions_world_ptr,
        VectorX *distances_ptr) const {
        const long num_hit_rays = GetNumHitRays();
        ERL_ASSERTM(num_hit_rays > 0, "no hit points. cannot sample in region.");

        std::uniform_int_distribution<long> uniform_int_dist(0, num_hit_rays - 1);
        // avoid getting too close to either the sensor or the hit point
        std::uniform_real_distribution<Dtype> uniform_real_dist(0.1f, 0.8f);

        const long max_num_samples =
            num_positions * num_hit_rays *
            (num_along_ray_samples_per_ray + num_near_surface_samples_per_ray);
        Matrix3X &positions_samples = *positions_world_ptr;
        Matrix3X &directions_samples = *directions_world_ptr;
        VectorX &distances_samples = *distances_ptr;
        positions_samples.resize(3, max_num_samples);
        directions_samples.resize(3, max_num_samples);
        distances_samples.resize(max_num_samples);

        std::vector<long> visible_hit_point_indices;
        std::mt19937 random_engine(seed);
        std::uniform_real_distribution<Dtype> uniform_ns(
            -max_in_obstacle_dist,
            max_in_obstacle_dist);

        long sample_cnt = 0;
        for (long position_idx = 0; position_idx < num_positions; ++position_idx) {
            const long hit_index = uniform_int_dist(random_engine);
            Dtype r = uniform_real_dist(random_engine);
            const auto [hit_azimuth_index, hit_elevation_index] = m_hit_ray_indices_[hit_index];
            r *= m_ranges_(hit_azimuth_index, hit_elevation_index);

            // synthesize a lidar scan
            Vector3 position_scan =
                m_translation_ + r * m_dirs_world_(hit_azimuth_index, hit_elevation_index);
            Dtype radius = (m_max_valid_range_ + (position_scan - m_translation_).norm()) * 10.0f;
            visible_hit_point_indices.clear();
            HiddenPointRemoval<Dtype>(
                m_hit_points_world_,
                position_scan,
                radius,
                visible_hit_point_indices,
                true,
                false);

            if (static_cast<long>(visible_hit_point_indices.size()) == 0) {
                position_idx--;  // retry
                continue;
            }

            for (const long &index: visible_hit_point_indices) {
                Vector3 dir = m_hit_points_world_[index] - position_scan;
                Dtype range = dir.norm();
                dir /= range;

                // sample near surface with this lidar scan
                for (long i = 0; i < num_near_surface_samples_per_ray; ++i) {
                    const Dtype offset = uniform_ns(random_engine);
                    positions_samples.col(sample_cnt) << position_scan + (range + offset) * dir;
                    directions_samples.col(sample_cnt) << dir;
                    distances_samples[sample_cnt++] = -offset;
                }

                // sample along rays with this lidar scan
                positions_samples.col(sample_cnt) << position_scan;
                directions_samples.col(sample_cnt) << dir;
                distances_samples[sample_cnt++] = range;
                Dtype range_step = (range + max_in_obstacle_dist) /
                                   static_cast<Dtype>(num_along_ray_samples_per_ray);
                Vector3 shift = range_step * dir;
                for (long i = 1; i < num_along_ray_samples_per_ray; ++i) {
                    range -= range_step;
                    positions_samples.col(sample_cnt)
                        << positions_samples.col(sample_cnt - 1) + shift;
                    directions_samples.col(sample_cnt) << dir;
                    distances_samples[sample_cnt++] = range;
                }
            }
        }

        positions_samples.conservativeResize(3, sample_cnt);
        directions_samples.conservativeResize(3, sample_cnt);
        distances_samples.conservativeResize(sample_cnt);
    }

    template<typename Dtype>
    void
    RangeSensorFrame3D<Dtype>::SampleInRegionVrsThread(
        const uint64_t seed,
        const long *hit_point_index_start,
        const long *hit_point_index_end,
        long num_samples_per_azimuth_segment,
        long num_azimuth_segments,
        Matrix3X *positions_world_ptr,
        Matrix3X *directions_world_ptr,
        VectorX *distances_ptr) const {
        const long num_hit_rays = GetNumHitRays();
        ERL_ASSERTM(num_hit_rays > 0, "no hit points. cannot sample in region.");

        Matrix3X &positions_samples = *positions_world_ptr;
        Matrix3X &directions_samples = *directions_world_ptr;
        VectorX &distances_samples = *distances_ptr;
        long max_num_samples = num_azimuth_segments * num_samples_per_azimuth_segment *
                               (hit_point_index_end - hit_point_index_start);
        positions_samples.resize(3, max_num_samples);
        directions_samples.resize(3, max_num_samples);
        distances_samples.resize(max_num_samples);
        long sample_idx = 0;
        std::mt19937 random_engine(seed);
        std::uniform_real_distribution<Dtype> uniform_range_ratio(0.1f, 0.9f);

        struct RayInfo {
            Dtype ray_azimuth = 0.0f;
            Dtype ray_elevation = 0.0f;
            Dtype end_point_elevation = 0.0f;
            Dtype range = 0.0f;
            Vector3 dir_world = {};
        };

        constexpr auto kPI = static_cast<Dtype>(M_PI);

        for (const long *hit_point_index_ptr = hit_point_index_start;
             hit_point_index_ptr < hit_point_index_end;
             ++hit_point_index_ptr) {
            // make the hit point the origin, and the viewing direction along the -z axis
            const auto [hit_azimuth_index, hit_elevation_index] =
                m_hit_ray_indices_[*hit_point_index_ptr];
            const Dtype &hit_range = m_ranges_(hit_azimuth_index, hit_elevation_index);
            const Vector3 &viewing_dir = m_dirs_world_(hit_azimuth_index, hit_elevation_index);
            Vector3 axis = -viewing_dir.cross(Vector3(0.0f, 0.0f, 1.0f)).normalized();
            Dtype angle = std::acos(-viewing_dir.dot(Vector3(0.0f, 0.0f, 1.0f)));
            Matrix3 rotation = Eigen::AngleAxis<Dtype>(angle, axis).matrix();
            Dtype azimuth_resolution = 2.0f * kPI / static_cast<Dtype>(num_azimuth_segments);

            // 1. transform the rays to the hit point's frame and partition the rays into azimuth
            // segments
            absl::flat_hash_map<long, std::vector<RayInfo>> azimuth_rays;
            // 2. remove hit rays behind the viewing position, compute spherical coordinates, and
            // partition the points into azimuth segments
            std::vector<std::pair<Dtype, Dtype>> spherical_coords;  // azimuth, elevation
            spherical_coords.reserve(num_hit_rays);
            // 3. calculate max end_point_elevation in each azimuth segment
            VectorX max_elevations = VectorX::Constant(num_azimuth_segments, -kPI * 0.5f);
            for (long j = 0; j < m_ranges_.cols(); ++j) {
                for (long i = 0; i < m_ranges_.rows(); ++i) {
                    if (i == hit_azimuth_index && j == hit_elevation_index) { continue; }
                    if (std::isinf(m_ranges_(i, j))) { continue; }  // skip rays that hit nothing
                    if (!m_mask_hit_(i, j)) { continue; }

                    // behind the viewing position
                    if (m_dirs_world_(i, j).dot(viewing_dir) <= 0.0f) { continue; }

                    RayInfo ray_info;
                    ray_info.range = m_ranges_(i, j);
                    ray_info.dir_world << m_dirs_world_(i, j);
                    // 1.
                    Vector3 direction = rotation * ray_info.dir_world;
                    common::DirectionToAzimuthElevation<Dtype>(
                        direction,
                        ray_info.ray_azimuth,
                        ray_info.ray_elevation);
                    auto azimuth_index =
                        static_cast<long>((ray_info.ray_azimuth + kPI) / azimuth_resolution) %
                        num_azimuth_segments;
                    // 2.
                    Vector3 point = rotation * (m_end_pts_world_(i, j) -
                                                m_hit_points_world_[*hit_point_index_ptr]);
                    ray_info.end_point_elevation = std::asin(point.z() / point.norm());
                    spherical_coords.emplace_back(
                        ray_info.ray_azimuth,
                        ray_info.end_point_elevation);
                    // 3.
                    if (Dtype &max_elevation = max_elevations[azimuth_index];
                        ray_info.end_point_elevation > max_elevation) {
                        max_elevation = ray_info.end_point_elevation;
                    }
                    azimuth_rays[azimuth_index].emplace_back(std::move(ray_info));
                }
            }

            // sample along rays in each azimuth segment
            for (auto &[azimuth_index, rays]: azimuth_rays) {
                Dtype &max_elevation = max_elevations[azimuth_index];
                Dtype cos_max_elevation = std::cos(max_elevation) * hit_range;
                std::uniform_int_distribution<std::size_t> uniform_ray_index(0, rays.size() - 1);
                for (long cnt_samples = 0; cnt_samples < num_samples_per_azimuth_segment;
                     ++cnt_samples) {
                    std::size_t ray_index = uniform_ray_index(random_engine);
                    auto &[ray_azimuth, ray_elevation, end_point_elevation, range, dir_world] =
                        rays[ray_index];
                    Dtype r = uniform_range_ratio(random_engine);
                    Dtype elevation_diff = max_elevation - ray_elevation;
                    // calculate max sampling range along the ray
                    Dtype max_range = std::min(range, cos_max_elevation / std::sin(elevation_diff));
                    Vector3 position = m_translation_ + r * max_range * dir_world;
                    Vector3 direction = m_hit_points_world_[*hit_point_index_ptr] - position;
                    Dtype distance = direction.norm();
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

    template class RangeSensorFrame3D<double>;
    template class RangeSensorFrame3D<float>;
}  // namespace erl::geometry
