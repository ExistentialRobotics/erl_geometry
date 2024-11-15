#include "erl_geometry/range_sensor_frame_3d.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/random.hpp"
#include "erl_geometry/hidden_point_removal.hpp"

#include <absl/container/flat_hash_map.h>

namespace erl::geometry {

    std::shared_ptr<RangeSensorFrame3D>
    RangeSensorFrame3D::Create(const std::string &type, const std::shared_ptr<Setting> &setting) {
        const auto it = s_class_id_mapping_.find(type);
        if (it == s_class_id_mapping_.end()) {
            ERL_WARN("Unknown RangeSensorFrame3D type: {}. Here are the registered RangeSensorFrame3D types:", type);
            for (const auto &pair: s_class_id_mapping_) { ERL_WARN("  - {}", pair.first); }
            return nullptr;
        }
        return it->second(setting);
    }

    void
    RangeSensorFrame3D::ComputeClosestEndPoint(
        const Eigen::Ref<const Eigen::Vector3d> &position_world,
        long &end_point_row_index,
        long &end_point_col_index,
        double &distance,
        const bool brute_force) {
        if (brute_force) {
            end_point_row_index = -1;
            end_point_col_index = -1;
            distance = std::numeric_limits<double>::infinity();
            const long rows = m_end_pts_world_.rows();
            const long cols = m_end_pts_world_.cols();
            for (long col = 0; col < cols; ++col) {
                for (long row = 0; row < rows; ++row) {
                    if (const double d = (m_end_pts_world_(row, col) - position_world).squaredNorm(); d < distance) {
                        end_point_row_index = row;
                        end_point_col_index = col;
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
        m_kd_tree_->Nearest(position_world, end_point_index, distance);
        const long rows = m_end_pts_world_.rows();
        end_point_col_index = end_point_index / rows;
        end_point_row_index = end_point_index - end_point_col_index * rows;
        distance = std::sqrt(distance);
    }

    void
    RangeSensorFrame3D::SampleAlongRays(
        const long num_samples_per_ray,
        const double max_in_obstacle_dist,
        const double sampled_rays_ratio,
        Eigen::Matrix3Xd &positions_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances) const {

        std::vector<long> ray_indices = common::GenerateShuffledIndices<long>(GetNumHitRays(), sampled_rays_ratio);
        const auto n_rays = static_cast<long>(ray_indices.size());
        const long n_samples = n_rays * num_samples_per_ray;
        positions_world.resize(3, n_samples);
        directions_world.resize(3, n_samples);
        distances.resize(n_samples);

        long index = 0;
        for (const long &ray_idx: ray_indices) {
            const auto [azimuth_idx, elevation_idx] = m_hit_ray_indices_[ray_idx];
            double range = m_ranges_(azimuth_idx, elevation_idx);
            double range_step = (range + max_in_obstacle_dist) / static_cast<double>(num_samples_per_ray);
            const Eigen::Vector3d &dir_world = m_dirs_world_(azimuth_idx, elevation_idx);

            positions_world.col(index) << m_translation_;  // operator<< is at least 2x faster than operator= for Eigen matrix
            directions_world.col(index) << dir_world;      // operator<< is almost the same fast as element-wise assignment
            distances[index++] = range;                    // for vector, element-wise assignment is faster than operator<<

            Eigen::Vector3d shift = range_step * dir_world;
            for (long i = 1; i < num_samples_per_ray; ++i) {
                range -= range_step;
                positions_world.col(index) << positions_world.col(index - 1) + shift;
                directions_world.col(index) << dir_world;
                distances[index++] = range;
            }
        }
    }

    void
    RangeSensorFrame3D::SampleAlongRays(
        const double range_step,
        const double max_in_obstacle_dist,
        const double sampled_rays_ratio,
        Eigen::Matrix3Xd &positions_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances) const {

        std::vector<long> ray_indices = common::GenerateShuffledIndices<long>(GetNumHitRays(), sampled_rays_ratio);
        const auto n_rays = static_cast<long>(ray_indices.size());
        long n_samples = 0;
        std::vector<std::pair<std::pair<long, long>, long>> n_samples_per_ray;
        n_samples_per_ray.reserve(n_rays);
        for (const long &ray_idx: ray_indices) {
            const auto [azimuth_idx, elevation_idx] = m_hit_ray_indices_[ray_idx];
            auto n = static_cast<long>(std::floor((m_ranges_(azimuth_idx, elevation_idx) + max_in_obstacle_dist) / range_step)) + 1;
            n_samples_per_ray.emplace_back(std::make_pair(azimuth_idx, elevation_idx), n);
            n_samples += n;
        }
        positions_world.resize(3, n_samples);
        directions_world.resize(3, n_samples);
        distances.resize(n_samples);

        long sample_idx = 0;
        for (auto &[ray_idx, n_samples_of_ray]: n_samples_per_ray) {
            auto &[azimuth_idx, elevation_idx] = ray_idx;

            double range = m_ranges_(azimuth_idx, elevation_idx);
            const Eigen::Vector3d &dir_world = m_dirs_world_(azimuth_idx, elevation_idx);
            positions_world.col(sample_idx) << m_translation_;
            directions_world.col(sample_idx) << dir_world;
            distances[sample_idx++] = range;

            Eigen::Vector3d shift = range_step * dir_world;
            for (long sample_idx_of_ray = 1; sample_idx_of_ray < n_samples_of_ray; ++sample_idx_of_ray) {
                range -= range_step;
                positions_world.col(sample_idx) << positions_world.col(sample_idx - 1) + shift;
                directions_world.col(sample_idx) << dir_world;
                distances[sample_idx++] = range;
            }
        }
    }

    void
    RangeSensorFrame3D::SampleNearSurface(
        const long num_samples_per_ray,
        const double max_offset,
        const double sampled_rays_ratio,
        Eigen::Matrix3Xd &positions_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances) const {
        std::vector<long> hit_ray_indices = common::GenerateShuffledIndices<long>(GetNumHitRays(), sampled_rays_ratio);
        const auto n_rays = static_cast<long>(hit_ray_indices.size());
        const long n_samples = n_rays * num_samples_per_ray;
        positions_world.resize(3, n_samples);
        directions_world.resize(3, n_samples);
        distances.resize(n_samples);

        std::uniform_real_distribution<double> uniform(-max_offset, max_offset);
        long sample_idx = 0;
        for (const long &hit_ray_idx: hit_ray_indices) {
            const auto [azimuth_idx, elevation_idx] = m_hit_ray_indices_[hit_ray_idx];
            const Eigen::Vector3d &dir_world = m_dirs_world_(azimuth_idx, elevation_idx);
            for (long i = 0; i < num_samples_per_ray; ++i) {
                const double offset = uniform(common::g_random_engine);
                positions_world.col(sample_idx) << m_translation_ + (m_ranges_(azimuth_idx, elevation_idx) + offset) * dir_world;
                directions_world.col(sample_idx) << dir_world;
                distances[sample_idx++] = -offset;
            }
        }
    }

    void
    RangeSensorFrame3D::SampleInRegionHpr(
        long num_positions,
        long num_along_ray_samples_per_ray,
        long num_near_surface_samples_per_ray,
        double max_in_obstacle_dist,
        Eigen::Matrix3Xd &positions_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances,
        const bool parallel) const {
        ERL_ASSERTM(num_positions > 0, "num_positions ({}) must be positive.", num_positions);

        if (parallel) {
            const uint32_t num_threads = std::thread::hardware_concurrency();
            const long num_positions_per_thread = num_positions / num_threads;
            const long leftover = num_positions - num_positions_per_thread * num_threads;
            std::vector<std::thread> threads;
            threads.reserve(num_threads);
            std::vector<Eigen::Matrix3Xd> positions_world_buffers(num_threads);
            std::vector<Eigen::Matrix3Xd> directions_world_buffers(num_threads);
            std::vector<Eigen::VectorXd> distances_buffers(num_threads);
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
        ERL_DEBUG("{} positions, {} samples collected.", num_positions, positions_world.cols());
    }

    void
    RangeSensorFrame3D::SampleInRegionVrs(
        long num_hit_points,
        long num_samples_per_azimuth_segment,
        long num_azimuth_segments,
        Eigen::Matrix3Xd &positions_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances,
        const bool parallel) const {

        ERL_ASSERTM(num_hit_points > 0, "num_hit_points ({}) must be positive.", num_hit_points);

        std::vector<long> selected_hit_ray_indices(GetNumHitRays());
        std::iota(selected_hit_ray_indices.begin(), selected_hit_ray_indices.end(), 0);
        num_hit_points = std::min(num_hit_points, GetNumHitRays());
        if (num_hit_points < static_cast<long>(selected_hit_ray_indices.size())) {
            std::shuffle(selected_hit_ray_indices.begin(), selected_hit_ray_indices.end(), common::g_random_engine);
            selected_hit_ray_indices.resize(num_hit_points);
        }

        // TODO: rewrite parallelism with OpenMP, example: OccupancyOctreeBase::ComputeUpdateForPointCloud, which lets OpenMP handle batch processing
        if (parallel) {
            const uint32_t num_threads = std::thread::hardware_concurrency();
            const long num_positions_per_thread = num_hit_points / num_threads;
            const long leftover = num_hit_points - num_positions_per_thread * num_threads;
            std::vector<std::thread> threads;
            threads.reserve(num_threads);
            std::vector<Eigen::Matrix3Xd> positions_world_buffers(num_threads);
            std::vector<Eigen::Matrix3Xd> directions_world_buffers(num_threads);
            std::vector<Eigen::VectorXd> distances_buffers(num_threads);
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
                Eigen::Matrix3Xd &positions = positions_world_buffers[thread_idx];
                Eigen::Matrix3Xd &directions = directions_world_buffers[thread_idx];
                const long num_samples_to_copy = positions.cols();
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
    RangeSensorFrame3D::ComputeRaysAt(
        const Eigen::Ref<const Eigen::Vector3d> &position_world,
        Eigen::Matrix3Xd &directions_world,
        Eigen::VectorXd &distances,
        std::vector<long> &visible_hit_point_indices) const {
        ERL_ASSERTM(!std::isinf(m_max_valid_range_), "max valid range is not set.");
        const double radius = (m_max_valid_range_ + (position_world - m_translation_).norm()) * 10.0;
        visible_hit_point_indices.clear();
        HiddenPointRemoval(m_hit_points_world_, position_world, radius, visible_hit_point_indices, true, false);
        if (const auto num_visible_hit_points = static_cast<long>(visible_hit_point_indices.size()); directions_world.cols() < num_visible_hit_points) {
            directions_world.resize(3, num_visible_hit_points);
            distances.resize(num_visible_hit_points);
        }
        long sample_idx = 0;
        for (const long &index: visible_hit_point_indices) {
            auto dir = directions_world.col(sample_idx);
            double &distance = distances[sample_idx++];

            dir << m_hit_points_world_[index] - position_world;
            distance = dir.norm();
            dir /= distance;
        }
    }

    bool
    RangeSensorFrame3D::operator==(const RangeSensorFrame3D &other) const {
        if (m_setting_ == nullptr && other.m_setting_ != nullptr) { return false; }
        if (m_setting_ != nullptr && (other.m_setting_ == nullptr || *m_setting_ != *other.m_setting_)) { return false; }
        if (m_rotation_ != other.m_rotation_) { return false; }
        if (m_translation_ != other.m_translation_) { return false; }
        if (m_frame_coords_ != other.m_frame_coords_) { return false; }
        if (m_ranges_ != other.m_ranges_) { return false; }
        if (m_dirs_frame_ != other.m_dirs_frame_) { return false; }
        if (m_dirs_world_ != other.m_dirs_world_) { return false; }
        if (m_end_pts_frame_ != other.m_end_pts_frame_) { return false; }
        if (m_end_pts_world_ != other.m_end_pts_world_) { return false; }
        if (m_mask_hit_ != other.m_mask_hit_) { return false; }
        if (m_mask_continuous_ != other.m_mask_continuous_) { return false; }
        if (m_hit_ray_indices_ != other.m_hit_ray_indices_) { return false; }
        if (m_hit_points_world_ != other.m_hit_points_world_) { return false; }
        if (m_max_valid_range_ != other.m_max_valid_range_) { return false; }
        return true;
    }

    bool
    RangeSensorFrame3D::Write(const std::string &filename) const {
        ERL_INFO("Writing RangeSensorFrame3D to file: {}", filename);
        std::filesystem::create_directories(std::filesystem::path(filename).parent_path());
        std::ofstream file(filename, std::ios_base::out | std::ios_base::binary);
        if (!file.is_open()) {
            ERL_WARN("Failed to open file: {}", filename);
            return false;
        }

        const bool success = Write(file);
        file.close();
        return success;
    }

    static const std::string kFileHeader = "# erl::geometry::RangeSensorFrame3D";

    bool
    RangeSensorFrame3D::Write(std::ostream &s) const {
        s << kFileHeader << std::endl  //
          << "# (feel free to add / change comments, but leave the first line as it is!)" << std::endl
          << "setting" << std::endl;
        // write setting
        if (!m_setting_->Write(s)) {
            ERL_WARN("Failed to write setting.");
            return false;
        }
        // write data
        s << "rotation" << std::endl;
        if (!common::SaveEigenMatrixToBinaryStream(s, m_rotation_)) {
            ERL_WARN("Failed to write rotation.");
            return false;
        }
        s << "translation" << std::endl;
        if (!common::SaveEigenMatrixToBinaryStream(s, m_translation_)) {
            ERL_WARN("Failed to write translation.");
            return false;
        }
        s << "frame_coords" << std::endl;
        if (!common::SaveEigenMatrixOfEigenMatricesToBinaryStream(s, m_frame_coords_)) {
            ERL_WARN("Failed to write frame_coords.");
            return false;
        }
        s << "ranges" << std::endl;
        if (!common::SaveEigenMatrixToBinaryStream(s, m_ranges_)) {
            ERL_WARN("Failed to write ranges.");
            return false;
        }
        s << "dirs_frame" << std::endl;
        if (!common::SaveEigenMatrixOfEigenMatricesToBinaryStream(s, m_dirs_frame_)) {
            ERL_WARN("Failed to write dirs_frame.");
            return false;
        }
        s << "dirs_world" << std::endl;
        if (!common::SaveEigenMatrixOfEigenMatricesToBinaryStream(s, m_dirs_world_)) {
            ERL_WARN("Failed to write dirs_world.");
            return false;
        }
        s << "end_pts_frame" << std::endl;
        if (!common::SaveEigenMatrixOfEigenMatricesToBinaryStream(s, m_end_pts_frame_)) {
            ERL_WARN("Failed to write end_pts_frame.");
            return false;
        }
        s << "end_pts_world" << std::endl;
        if (!common::SaveEigenMatrixOfEigenMatricesToBinaryStream(s, m_end_pts_world_)) {
            ERL_WARN("Failed to write end_pts_world.");
            return false;
        }
        s << "mask_hit" << std::endl;
        if (!common::SaveEigenMatrixToBinaryStream(s, m_mask_hit_)) {
            ERL_WARN("Failed to write mask_hit.");
            return false;
        }
        s << "mask_continuous" << std::endl;
        if (!common::SaveEigenMatrixToBinaryStream(s, m_mask_continuous_)) {
            ERL_WARN("Failed to write mask_continuous.");
            return false;
        }
        s << "hit_ray_indices " << m_hit_ray_indices_.size() << std::endl;
        for (const auto &[row, col]: m_hit_ray_indices_) { s << row << " " << col << std::endl; }
        s << "hit_points_world" << std::endl;
        if (!common::SaveVectorOfEigenMatricesToBinaryStream(s, m_hit_points_world_)) {
            ERL_WARN("Failed to write hit_points_world.");
            return false;
        }
        s << "max_valid_range" << std::endl;
        s.write(reinterpret_cast<const char *>(&m_max_valid_range_), sizeof(double));
        s << "end_of_RangeSensorFrame3D" << std::endl;
        return s.good();
    }

    bool
    RangeSensorFrame3D::Read(const std::string &filename) {
        ERL_INFO("Reading RangeSensorFrame3D from file: {}", std::filesystem::absolute(filename));
        std::ifstream file(filename.c_str(), std::ios_base::in | std::ios_base::binary);
        if (!file.is_open()) {
            ERL_WARN("Failed to open file: {}", filename.c_str());
            return false;
        }

        const bool success = Read(file);
        file.close();
        return success;
    }

    bool
    RangeSensorFrame3D::Read(std::istream &s) {
        if (!s.good()) {
            ERL_WARN("Input stream is not ready for reading");
            return false;
        }

        // check if the first line is valid
        std::string line;
        std::getline(s, line);
        if (line.compare(0, kFileHeader.length(), kFileHeader) != 0) {  // check if the first line is valid
            ERL_WARN("Header does not start with \"{}\"", kFileHeader.c_str());
            return false;
        }

        auto skip_line = [&s]() {
            char c;
            do { c = static_cast<char>(s.get()); } while (s.good() && c != '\n');
        };

        static const char *tokens[] = {
            "setting",
            "rotation",
            "translation",
            "frame_coords",
            "ranges",
            "dirs_frame",
            "dirs_world",
            "end_pts_frame",
            "end_pts_world",
            "mask_hit",
            "mask_continuous",
            "hit_ray_indices",
            "hit_points_world",
            "max_valid_range",
            "end_of_RangeSensorFrame3D",
        };

        // read data
        std::string token;
        int token_idx = 0;
        while (s.good()) {
            s >> token;
            if (token.compare(0, 1, "#") == 0) {
                skip_line();  // comment line, skip forward until end of line
                continue;
            }
            // non-comment line
            if (token != tokens[token_idx]) {
                ERL_WARN("Expected token {}, got {}.", tokens[token_idx], token);  // check token
                return false;
            }
            // reading state machine
            switch (token_idx) {
                case 0: {  // setting
                    skip_line();
                    if (!m_setting_->Read(s)) {
                        ERL_WARN("Failed to read setting.");
                        return false;
                    }
                    break;
                }
                case 1: {  // rotation
                    skip_line();
                    if (!common::LoadEigenMatrixFromBinaryStream(s, m_rotation_)) {
                        ERL_WARN("Failed to read rotation.");
                        return false;
                    }
                    break;
                }
                case 2: {  // translation
                    skip_line();
                    if (!common::LoadEigenMatrixFromBinaryStream(s, m_translation_)) {
                        ERL_WARN("Failed to read translation.");
                        return false;
                    }
                    break;
                }
                case 3: {  // frame_coords
                    skip_line();
                    if (!common::LoadEigenMatrixOfEigenMatricesFromBinaryStream(s, m_frame_coords_)) {
                        ERL_WARN("Failed to read frame_coords.");
                        return false;
                    }
                    break;
                }
                case 4: {  // ranges
                    skip_line();
                    if (!common::LoadEigenMatrixFromBinaryStream(s, m_ranges_)) {
                        ERL_WARN("Failed to read ranges.");
                        return false;
                    }
                    break;
                }
                case 5: {  // dirs_frame
                    skip_line();
                    if (!common::LoadEigenMatrixOfEigenMatricesFromBinaryStream(s, m_dirs_frame_)) {
                        ERL_WARN("Failed to read dirs_frame.");
                        return false;
                    }
                    break;
                }
                case 6: {  // dirs_world
                    skip_line();
                    if (!common::LoadEigenMatrixOfEigenMatricesFromBinaryStream(s, m_dirs_world_)) {
                        ERL_WARN("Failed to read dirs_world.");
                        return false;
                    }
                    break;
                }
                case 7: {  // end_pts_frame
                    skip_line();
                    if (!common::LoadEigenMatrixOfEigenMatricesFromBinaryStream(s, m_end_pts_frame_)) {
                        ERL_WARN("Failed to read end_pts_frame.");
                        return false;
                    }
                    break;
                }
                case 8: {  // end_pts_world
                    skip_line();
                    if (!common::LoadEigenMatrixOfEigenMatricesFromBinaryStream(s, m_end_pts_world_)) {
                        ERL_WARN("Failed to read end_pts_world.");
                        return false;
                    }
                    break;
                }
                case 9: {  // mask_hit
                    skip_line();
                    if (!common::LoadEigenMatrixFromBinaryStream(s, m_mask_hit_)) {
                        ERL_WARN("Failed to read mask_hit.");
                        return false;
                    }
                    break;
                }
                case 10: {  // mask_continuous
                    skip_line();
                    if (!common::LoadEigenMatrixFromBinaryStream(s, m_mask_continuous_)) {
                        ERL_WARN("Failed to read mask_continuous.");
                        return false;
                    }
                    break;
                }
                case 11: {  // hit_ray_indices
                    long num_hit_ray_indices;
                    s >> num_hit_ray_indices;
                    m_hit_ray_indices_.resize(num_hit_ray_indices);
                    for (long i = 0; i < num_hit_ray_indices; ++i) { s >> m_hit_ray_indices_[i].first >> m_hit_ray_indices_[i].second; }
                    break;
                }
                case 12: {  // hit_points_world
                    skip_line();
                    if (!common::LoadVectorOfEigenMatricesFromBinaryStream(s, m_hit_points_world_)) {
                        ERL_WARN("Failed to read hit_points_world.");
                        return false;
                    }
                    break;
                }
                case 13: {  // max_valid_range
                    skip_line();
                    s.read(reinterpret_cast<char *>(&m_max_valid_range_), sizeof(double));
                    break;
                }
                case 14: {  // end of RangeSensorFrame3D
                    skip_line();
                    return true;
                }
                default: {  // should not reach here
                    ERL_FATAL("Internal error, should not reach here.");
                }
            }
            ++token_idx;
        }
        ERL_WARN("Failed to read RangeSensorFrame3D. Truncated file?");
        return false;  // should not reach here
    }

    void
    RangeSensorFrame3D::SampleInRegionHprThread(
        const uint64_t seed,
        const long num_positions,
        const long num_along_ray_samples_per_ray,
        const long num_near_surface_samples_per_ray,
        const double max_in_obstacle_dist,
        Eigen::Matrix3Xd *positions_world_ptr,
        Eigen::Matrix3Xd *directions_world_ptr,
        Eigen::VectorXd *distances_ptr) const {
        const long num_hit_rays = GetNumHitRays();
        ERL_ASSERTM(num_hit_rays > 0, "no hit points. cannot sample in region.");

        std::uniform_int_distribution<long> uniform_int_dist(0, num_hit_rays - 1);
        std::uniform_real_distribution<double> uniform_real_dist(0.1, 0.8);  // avoid getting too close to either the sensor or the hit point

        const long max_num_samples = num_positions * num_hit_rays * (num_along_ray_samples_per_ray + num_near_surface_samples_per_ray);
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
            const long hit_index = uniform_int_dist(random_engine);
            double r = uniform_real_dist(random_engine);
            const auto [hit_azimuth_index, hit_elevation_index] = m_hit_ray_indices_[hit_index];
            r *= m_ranges_(hit_azimuth_index, hit_elevation_index);

            // synthesize a lidar scan
            Eigen::Vector3d position_scan = m_translation_ + r * m_dirs_world_(hit_azimuth_index, hit_elevation_index);
            const double radius = (m_max_valid_range_ + (position_scan - m_translation_).norm()) * 10.0;
            visible_hit_point_indices.clear();
            HiddenPointRemoval(m_hit_points_world_, position_scan, radius, visible_hit_point_indices, true, false);

            if (static_cast<long>(visible_hit_point_indices.size()) == 0) {
                position_idx--;  // retry
                continue;
            }

            for (const long &index: visible_hit_point_indices) {
                Eigen::Vector3d dir = m_hit_points_world_[index] - position_scan;
                double range = dir.norm();
                dir /= range;

                // sample near surface with this lidar scan
                for (long i = 0; i < num_near_surface_samples_per_ray; ++i) {
                    const double offset = uniform_ns(random_engine);
                    positions_samples.col(sample_cnt) << position_scan + (range + offset) * dir;
                    directions_samples.col(sample_cnt) << dir;
                    distances_samples[sample_cnt++] = -offset;
                }

                // sample along rays with this lidar scan
                positions_samples.col(sample_cnt) << position_scan;
                directions_samples.col(sample_cnt) << dir;
                distances_samples[sample_cnt++] = range;
                double range_step = (range + max_in_obstacle_dist) / static_cast<double>(num_along_ray_samples_per_ray);
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
    RangeSensorFrame3D::SampleInRegionVrsThread(
        uint64_t seed,
        const long *hit_point_index_start,
        const long *hit_point_index_end,
        long num_samples_per_azimuth_segment,
        long num_azimuth_segments,
        Eigen::Matrix3Xd *positions_world_ptr,
        Eigen::Matrix3Xd *directions_world_ptr,
        Eigen::VectorXd *distances_ptr) const {
        const long num_hit_rays = GetNumHitRays();
        ERL_ASSERTM(num_hit_rays > 0, "no hit points. cannot sample in region.");

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
            const auto [hit_azimuth_index, hit_elevation_index] = m_hit_ray_indices_[*hit_point_index_ptr];
            const double &hit_range = m_ranges_(hit_azimuth_index, hit_elevation_index);
            const Eigen::Vector3d &viewing_dir = m_dirs_world_(hit_azimuth_index, hit_elevation_index);
            Eigen::Vector3d axis = -viewing_dir.cross(Eigen::Vector3d(0.0, 0.0, 1.0)).normalized();
            double angle = std::acos(-viewing_dir.dot(Eigen::Vector3d(0.0, 0.0, 1.0)));
            Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle, axis).matrix();
            double azimuth_resolution = 2.0 * M_PI / static_cast<double>(num_azimuth_segments);

            // 1. transform the rays to the hit point's frame, and partition the rays into azimuth segments
            absl::flat_hash_map<long, std::vector<RayInfo>> azimuth_rays;
            // 2. remove hit rays behind the viewing position, compute spherical coordinates, and partition the points into azimuth segments
            std::vector<std::pair<double, double>> spherical_coords;  // azimuth, elevation
            spherical_coords.reserve(num_hit_rays);
            // 3. calculate max end_point_elevation in each azimuth segment
            Eigen::VectorXd max_elevations = Eigen::VectorXd::Constant(num_azimuth_segments, -M_PI_2);
            for (long j = 0; j < m_ranges_.cols(); ++j) {
                for (long i = 0; i < m_ranges_.rows(); ++i) {
                    if (i == hit_azimuth_index && j == hit_elevation_index) { continue; }  // skip the hit point
                    if (std::isinf(m_ranges_(i, j))) { continue; }                         // skip rays that hit nothing
                    if (!m_mask_hit_(i, j)) { continue; }
                    if (m_dirs_world_(i, j).dot(viewing_dir) <= 0.0) { continue; }  // behind the viewing position

                    RayInfo ray_info;
                    ray_info.range = m_ranges_(i, j);
                    ray_info.dir_world << m_dirs_world_(i, j);
                    // 1.
                    Eigen::Vector3d direction = rotation * ray_info.dir_world;
                    common::DirectionToAzimuthElevation(direction, ray_info.ray_azimuth, ray_info.ray_elevation);
                    auto azimuth_index = static_cast<long>((ray_info.ray_azimuth + M_PI) / azimuth_resolution) % num_azimuth_segments;
                    // 2.
                    Eigen::Vector3d point = rotation * (m_end_pts_world_(i, j) - m_hit_points_world_[*hit_point_index_ptr]);
                    ray_info.end_point_elevation = std::asin(point.z() / point.norm());
                    spherical_coords.emplace_back(ray_info.ray_azimuth, ray_info.end_point_elevation);
                    // 3.
                    if (double &max_elevation = max_elevations[azimuth_index]; ray_info.end_point_elevation > max_elevation) {
                        max_elevation = ray_info.end_point_elevation;
                    }
                    azimuth_rays[azimuth_index].emplace_back(std::move(ray_info));
                }
            }

            // sample along rays in each azimuth segment
            for (auto &[azimuth_index, rays]: azimuth_rays) {
                double &max_elevation = max_elevations[azimuth_index];
                double cos_max_elevation = std::cos(max_elevation) * hit_range;
                std::uniform_int_distribution<std::size_t> uniform_ray_index(0, rays.size() - 1);
                for (long cnt_samples = 0; cnt_samples < num_samples_per_azimuth_segment; ++cnt_samples) {
                    std::size_t ray_index = uniform_ray_index(random_engine);
                    auto &[ray_azimuth, ray_elevation, end_point_elevation, range, dir_world] = rays[ray_index];
                    double r = uniform_range_ratio(random_engine);
                    double elevation_diff = max_elevation - ray_elevation;
                    double max_range = std::min(range, cos_max_elevation / std::sin(elevation_diff));  // calculate max sampling range along the ray
                    Eigen::Vector3d position = m_translation_ + r * max_range * dir_world;
                    Eigen::Vector3d direction = m_hit_points_world_[*hit_point_index_ptr] - position;
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
