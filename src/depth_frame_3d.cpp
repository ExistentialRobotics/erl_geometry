#include "erl_geometry/depth_frame_3d.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/exception.hpp"
#include "erl_common/opencv.hpp"

namespace erl::geometry {

    DepthFrame3D::DepthFrame3D(std::shared_ptr<Setting> setting)
        : RangeSensorFrame3D(setting),
          m_setting_(std::move(setting)) {
        ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        UpdateFrameCoords();
    }

    void
    DepthFrame3D::UpdateRanges(
        const Eigen::Ref<const Eigen::Matrix3d> &rotation,
        const Eigen::Ref<const Eigen::Vector3d> &translation,
        Eigen::MatrixXd depth,
        const bool partition_rays) {

        const long image_height = m_setting_->camera_intrinsic.image_height;
        const long image_width = m_setting_->camera_intrinsic.image_width;
        ERL_ASSERTM(depth.rows() == image_height, "depth image height ({}) does not match setting ({}).", depth.rows(), image_height);
        ERL_ASSERTM(depth.cols() == image_width, "depth image width ({}) does not match setting ({}).", depth.cols(), image_width);

        m_rotation_ << rotation;
        m_translation_ << translation;
        m_ranges_ = std::move(depth);

        m_partitions_.clear();
        m_partitioned_ = false;
        m_kd_tree_->Clear();

        m_dirs_world_.resize(image_height, image_width);
        m_end_pts_frame_.resize(image_height, image_width);
        m_end_pts_world_.resize(image_height, image_width);

        m_mask_hit_.setConstant(image_height, image_width, false);
        m_hit_ray_indices_.clear();
        m_hit_ray_indices_.reserve(image_height * image_width);
        m_hit_points_world_.clear();
        m_hit_points_world_.reserve(image_height * image_width);

        const double valid_range_min = m_setting_->valid_range_min;
        const double valid_range_max = m_setting_->valid_range_max;

        // compute directions and end points
#pragma omp parallel for default(none) shared(image_height, image_width, valid_range_min, valid_range_max, Eigen::Dynamic)
        for (long u = 0; u < image_width; ++u) {
            for (long v = 0; v < image_height; ++v) {
                double &range = m_ranges_(v, u);
                if (range <= 0 || !std::isfinite(range)) { continue; }  // non-positive or nan depth! Not allowed.

                // directions and end_points in frame
                Eigen::Vector3d &dir_frame = m_dirs_frame_(v, u);
                Eigen::Vector3d &end_pt_frame = m_end_pts_frame_(v, u);
                // u <--> image coordinate x
                // v <--> image coordinate y
                end_pt_frame << dir_frame * (range / dir_frame[2]);  // range is depth currently
                range = end_pt_frame.norm();                         // range is now the actual range

                // transform directions and end_points to world
                m_dirs_world_(v, u) << m_rotation_ * dir_frame;
                m_end_pts_world_(v, u) << m_rotation_ * end_pt_frame + m_translation_;

                // max valid range
                if (range < valid_range_min || range > valid_range_max) { continue; }
                m_mask_hit_(v, u) = true;
            }
        }

        m_max_valid_range_ = 0.0;
        for (long u = 0; u < image_width; ++u) {
            const bool *mask_hit_ptr = m_mask_hit_.col(u).data();
            const double *ranges_ptr = m_ranges_.col(u).data();
            const Eigen::Vector3d *end_pts_world_ptr = m_end_pts_world_.col(u).data();
            for (long v = 0; v < image_height; ++v) {
                if (!mask_hit_ptr[v]) { continue; }
                if (const double range = ranges_ptr[v]; range > m_max_valid_range_) { m_max_valid_range_ = range; }
                m_hit_ray_indices_.emplace_back(v, u);
                m_hit_points_world_.emplace_back(end_pts_world_ptr[v]);
            }
        }

        if (!partition_rays) { return; }  // do not partition rays
        PartitionRays();
    }

    bool
    DepthFrame3D::operator==(const RangeSensorFrame3D &other) const {
        if (!RangeSensorFrame3D::operator==(other)) { return false; }
        const auto *other_ptr = dynamic_cast<const DepthFrame3D *>(&other);
        if (other_ptr == nullptr) { return false; }
        if (m_setting_ == nullptr && other_ptr->m_setting_ != nullptr) { return false; }
        if (m_setting_ != nullptr && (other_ptr->m_setting_ == nullptr || *m_setting_ != *other_ptr->m_setting_)) { return false; }
        if (m_partitions_.size() != other_ptr->m_partitions_.size()) { return false; }
        // TODO: compare partitions
        if (m_partitioned_ != other_ptr->m_partitioned_) { return false; }
        return true;
    }

    bool
    DepthFrame3D::Write(const std::string &filename) const {
        ERL_INFO("Writing DepthFrame3D to file: {}", filename);
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

    static const std::string kFileHeader = "# erl::geometry::DepthFrame3D";

    bool
    DepthFrame3D::Write(std::ostream &s) const {
        if (!RangeSensorFrame3D::Write(s)) {
            ERL_WARN("Failed to write parent class RangeSensorFrame3D.");
            return false;
        }
        s << kFileHeader << std::endl  //
          << "# (feel free to add / change comments, but leave the first line as it is!)" << std::endl
          << "setting" << std::endl;
        // write setting
        if (!m_setting_->Write(s)) {
            ERL_WARN("Failed to write setting.");
            return false;
        }
        // write data
        s << "partitions " << m_partitions_.size() << std::endl;
        // TODO: write partitions
        s << "partitioned " << m_partitioned_ << std::endl;
        s << "end_of_DepthFrame3D" << std::endl;
        return s.good();
    }

    bool
    DepthFrame3D::Read(const std::string &filename) {
        ERL_INFO("Reading DepthFrame3D from file: {}", std::filesystem::absolute(filename));
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
    DepthFrame3D::Read(std::istream &s) {
        if (!RangeSensorFrame3D::Read(s)) {
            ERL_WARN("Failed to read parent class RangeSensorFrame3D.");
            return false;
        }

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
            "partitions",
            "partitioned",
            "end_of_DepthFrame3D",
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
                case 1: {  // partitions
                    long num_partitions;
                    s >> num_partitions;
                    m_partitions_.resize(num_partitions);
                    skip_line();
                    // TODO: read partitions
                    break;
                }
                case 2: {  // partitioned
                    s >> m_partitioned_;
                    break;
                }
                case 3: {  // end_of_DepthFrame3D
                    skip_line();
                    return true;
                }
                default: {  // should not reach here
                    ERL_FATAL("Internal error, should not reach here.");
                }
            }
            ++token_idx;
        }
        ERL_WARN("Failed to read DepthFrame3D. Truncated file?");
        return false;  // should not reach here
    }

    void
    // ReSharper disable once CppMemberFunctionMayBeStatic
    DepthFrame3D::PartitionRays() {  // NOLINT(*-convert-member-functions-to-static)
        throw NotImplemented(__PRETTY_FUNCTION__);
    }

}  // namespace erl::geometry
