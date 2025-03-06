#pragma once

#include "erl_common/exception.hpp"
#include "erl_common/opencv.hpp"

namespace erl::geometry {
    template<typename Dtype>
    YAML::Node
    DepthFrame3D<Dtype>::Setting::YamlConvertImpl::encode(const Setting &setting) {
        YAML::Node node = Super::Setting::YamlConvertImpl::encode(setting);
        node["camera_intrinsic"] = setting.camera_intrinsic;
        return node;
    }

    template<typename Dtype>
    bool
    DepthFrame3D<Dtype>::Setting::YamlConvertImpl::decode(const YAML::Node &node, Setting &setting) {
        if (!Super::Setting::YamlConvertImpl::decode(node, setting)) { return false; }
        setting.camera_intrinsic = node["camera_intrinsic"].as<CameraIntrinsic<Dtype>>();
        return true;
    }

    template<typename Dtype>
    DepthFrame3D<Dtype>::DepthFrame3D(std::shared_ptr<Setting> setting)
        : Super(setting),
          m_setting_(std::move(setting)) {
        ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        UpdateFrameCoords();
    }

    template<typename Dtype>
    std::shared_ptr<const typename DepthFrame3D<Dtype>::Setting>
    DepthFrame3D<Dtype>::GetSetting() const {
        return m_setting_;
    }

    template<typename Dtype>
    void
    DepthFrame3D<Dtype>::Reset() {
        Super::m_max_valid_range_ = std::numeric_limits<Dtype>::min();
        m_partitioned_ = false;
    }

    template<typename Dtype>
    long
    DepthFrame3D<Dtype>::GetImageHeight() const {
        return Super::m_frame_coords_.rows();
    }

    template<typename Dtype>
    long
    DepthFrame3D<Dtype>::GetImageWidth() const {
        return Super::m_frame_coords_.cols();
    }

    template<typename Dtype>
    bool
    DepthFrame3D<Dtype>::IsPartitioned() const {
        return m_partitioned_;
    }

    template<typename Dtype>
    bool
    DepthFrame3D<Dtype>::PointIsInFrame(const Vector3 &xyz_frame) const {
        if (xyz_frame[2] < 0) { return false; }  // behind the camera
        return Super::CoordsIsInFrame(ComputeFrameCoords(xyz_frame));
    }

    template<typename Dtype>
    typename DepthFrame3D<Dtype>::Vector2
    DepthFrame3D<Dtype>::ComputeFrameCoords(const Vector3 &dir_frame) const {
        return {dir_frame[1] / dir_frame[2], dir_frame[0] / dir_frame[2]};
    }

    template<typename Dtype>
    typename DepthFrame3D<Dtype>::MatrixX
    DepthFrame3D<Dtype>::DepthImageToDepth(const MatrixX &depth_img, const double depth_scale) {
        return depth_img / depth_scale;
    }

    template<typename Dtype>
    typename DepthFrame3D<Dtype>::MatrixX
    DepthFrame3D<Dtype>::DepthToDepthImage(const MatrixX &depth, const double depth_scale) {
        return depth * depth_scale;
    }

    template<typename Dtype>
    void
    DepthFrame3D<Dtype>::UpdateRanges(
        const Eigen::Ref<const Matrix3> &rotation,
        const Eigen::Ref<const Vector3> &translation,
        MatrixX depth,
        const bool partition_rays) {

        const long image_height = m_setting_->camera_intrinsic.image_height;
        const long image_width = m_setting_->camera_intrinsic.image_width;
        ERL_ASSERTM(depth.rows() == image_height, "depth image height ({}) does not match setting ({}).", depth.rows(), image_height);
        ERL_ASSERTM(depth.cols() == image_width, "depth image width ({}) does not match setting ({}).", depth.cols(), image_width);

        Super::m_rotation_ << rotation;
        Super::m_translation_ << translation;
        Super::m_ranges_ = std::move(depth);

        m_partitions_.clear();
        m_partitioned_ = false;
        Super::m_kd_tree_->Clear();

        Super::m_dirs_world_.resize(image_height, image_width);
        Super::m_end_pts_frame_.resize(image_height, image_width);
        Super::m_end_pts_world_.resize(image_height, image_width);

        Super::m_mask_hit_.setConstant(image_height, image_width, false);
        Super::m_hit_ray_indices_.clear();
        Super::m_hit_ray_indices_.reserve(image_height * image_width);
        Super::m_hit_points_world_.clear();
        Super::m_hit_points_world_.reserve(image_height * image_width);

        const Dtype valid_range_min = m_setting_->valid_range_min;
        const Dtype valid_range_max = m_setting_->valid_range_max;

// compute directions and end points
#pragma omp parallel for default(none) shared(image_height, image_width, valid_range_min, valid_range_max, Eigen::Dynamic)
        for (long u = 0; u < image_width; ++u) {
            for (long v = 0; v < image_height; ++v) {
                Dtype &range = Super::m_ranges_(v, u);
                if (range <= 0 || !std::isfinite(range)) { continue; }  // non-positive or nan depth! Not allowed.

                // directions and end_points in frame
                Vector3 &dir_frame = Super::m_dirs_frame_(v, u);
                Vector3 &end_pt_frame = Super::m_end_pts_frame_(v, u);
                // u <--> image coordinate x
                // v <--> image coordinate y
                end_pt_frame << dir_frame * (range / dir_frame[2]);  // range is depth currently
                range = end_pt_frame.norm();                         // range is now the actual range

                // transform directions and end_points to world
                Super::m_dirs_world_(v, u) << Super::m_rotation_ * dir_frame;
                Super::m_end_pts_world_(v, u) << Super::m_rotation_ * end_pt_frame + Super::m_translation_;

                // max valid range
                if (range < valid_range_min || range > valid_range_max) { continue; }
                Super::m_mask_hit_(v, u) = true;
            }
        }

        Super::m_max_valid_range_ = 0.0;
        for (long u = 0; u < image_width; ++u) {
            const bool *mask_hit_ptr = Super::m_mask_hit_.col(u).data();
            const Dtype *ranges_ptr = Super::m_ranges_.col(u).data();
            const Vector3 *end_pts_world_ptr = Super::m_end_pts_world_.col(u).data();
            for (long v = 0; v < image_height; ++v) {
                if (!mask_hit_ptr[v]) { continue; }
                if (const Dtype range = ranges_ptr[v]; range > Super::m_max_valid_range_) { Super::m_max_valid_range_ = range; }
                Super::m_hit_ray_indices_.emplace_back(v, u);
                Super::m_hit_points_world_.emplace_back(end_pts_world_ptr[v]);
            }
        }

        if (!partition_rays) { return; }  // do not partition rays
        PartitionRays();
    }

    template<typename Dtype>
    void
    DepthFrame3D<Dtype>::UpdateRanges(
        const Eigen::Ref<const Matrix3> &rotation,
        const Eigen::Ref<const Vector3> &translation,
        const std::string &depth_file,
        const double depth_scale,
        const bool partition_rays) {
        cv::Mat depth_img = cv::imread(depth_file, cv::IMREAD_UNCHANGED);
        depth_img.convertTo(depth_img, sizeof(Dtype) == 4 ? CV_32FC1 : CV_64FC1);  // convert to float or double
        MatrixX depth;
        cv::cv2eigen(depth_img, depth);
        UpdateRanges(rotation, translation, DepthImageToDepth(depth, depth_scale), partition_rays);
    }

    template<typename Dtype>
    const std::vector<DepthFramePartition3D> &
    DepthFrame3D<Dtype>::GetPartitions() const {
        ERL_ASSERTM(m_partitioned_, "LidarFrame3D::GetPartitions() is called before partitioning.");
        return m_partitions_;
    }

    template<typename Dtype>
    bool
    DepthFrame3D<Dtype>::operator==(const Super &other) const {
        if (!Super::operator==(other)) { return false; }
        const auto *other_ptr = dynamic_cast<const DepthFrame3D *>(&other);
        if (other_ptr == nullptr) { return false; }
        if (m_setting_ == nullptr && other_ptr->m_setting_ != nullptr) { return false; }
        if (m_setting_ != nullptr && (other_ptr->m_setting_ == nullptr || *m_setting_ != *other_ptr->m_setting_)) { return false; }
        if (m_partitions_.size() != other_ptr->m_partitions_.size()) { return false; }
        // TODO: compare partitions
        if (m_partitioned_ != other_ptr->m_partitioned_) { return false; }
        return true;
    }

    template<typename Dtype>
    bool
    DepthFrame3D<Dtype>::Write(const std::string &filename) const {
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

    template<typename Dtype>
    bool
    DepthFrame3D<Dtype>::Write(std::ostream &s) const {
        if (!Super::Write(s)) {
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

    template<typename Dtype>
    bool
    DepthFrame3D<Dtype>::Read(const std::string &filename) {
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

    template<typename Dtype>
    bool
    DepthFrame3D<Dtype>::Read(std::istream &s) {
        if (!Super::Read(s)) {
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

    template<typename Dtype>
    void
    DepthFrame3D<Dtype>::PartitionRays() {
        m_partitioned_ = true;
        ERL_WARN("Not implemented yet.");
    }

    template<typename Dtype>
    void
    DepthFrame3D<Dtype>::UpdateFrameCoords() {
        m_setting_->camera_intrinsic.ComputeFrameDirections(Super::m_frame_coords_, Super::m_dirs_frame_);
    }
}  // namespace erl::geometry
