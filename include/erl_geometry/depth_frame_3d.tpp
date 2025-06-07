#pragma once

#include "erl_common/opencv.hpp"

namespace erl::geometry {
    template<typename Dtype>
    YAML::Node
    DepthFrame3D<Dtype>::Setting::YamlConvertImpl::encode(const Setting &setting) {
        YAML::Node node = Super::Setting::YamlConvertImpl::encode(setting);
        ERL_YAML_SAVE_ATTR(node, setting, camera_intrinsic);
        return node;
    }

    template<typename Dtype>
    bool
    DepthFrame3D<Dtype>::Setting::YamlConvertImpl::decode(
        const YAML::Node &node,
        Setting &setting) {
        if (!Super::Setting::YamlConvertImpl::decode(node, setting)) { return false; }
        return ERL_YAML_LOAD_ATTR(node, setting, camera_intrinsic);
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
        this->m_max_valid_range_ = std::numeric_limits<Dtype>::min();
    }

    template<typename Dtype>
    long
    DepthFrame3D<Dtype>::GetImageHeight() const {
        return this->m_frame_coords_.rows();
    }

    template<typename Dtype>
    long
    DepthFrame3D<Dtype>::GetImageWidth() const {
        return this->m_frame_coords_.cols();
    }

    template<typename Dtype>
    std::pair<long, long>
    DepthFrame3D<Dtype>::GetFrameShape() const {
        return {
            m_setting_->camera_intrinsic.image_height,
            m_setting_->camera_intrinsic.image_width};
    }

    template<typename Dtype>
    bool
    DepthFrame3D<Dtype>::PointIsInFrame(const Vector3 &xyz_frame) const {
        if (xyz_frame[2] < 0) { return false; }  // behind the camera
        return this->CoordsIsInFrame(ComputeFrameCoords(xyz_frame));
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
        MatrixX depth) {

        const long image_height = m_setting_->camera_intrinsic.image_height;
        const long image_width = m_setting_->camera_intrinsic.image_width;
        ERL_ASSERTM(
            depth.rows() == image_height,
            "depth image height ({}) does not match setting ({}).",
            depth.rows(),
            image_height);
        ERL_ASSERTM(
            depth.cols() == image_width,
            "depth image width ({}) does not match setting ({}).",
            depth.cols(),
            image_width);

        this->m_rotation_ << rotation;
        this->m_translation_ << translation;
        this->m_ranges_ = std::move(depth);

        this->m_kd_tree_->Clear();

        this->m_dirs_world_.resize(image_height, image_width);
        this->m_end_pts_frame_.resize(image_height, image_width);
        this->m_end_pts_world_.resize(image_height, image_width);

        this->m_mask_hit_.setConstant(image_height, image_width, false);
        this->m_hit_ray_indices_.clear();
        this->m_hit_ray_indices_.reserve(image_height * image_width);
        this->m_hit_points_frame_.clear();
        this->m_hit_points_frame_.reserve(image_height * image_width);
        this->m_hit_points_world_.clear();
        this->m_hit_points_world_.reserve(image_height * image_width);

        const Dtype valid_range_min = m_setting_->valid_range_min;
        const Dtype valid_range_max = m_setting_->valid_range_max;

        // compute directions and end points
#pragma omp parallel for default(none) \
    shared(image_height, image_width, valid_range_min, valid_range_max, Eigen::Dynamic)
        for (long u = 0; u < image_width; ++u) {
            for (long v = 0; v < image_height; ++v) {
                // directions and end_points in the frame
                const Vector3 &dir_frame = this->m_dirs_frame_(v, u);
                Vector3 &end_pt_frame = this->m_end_pts_frame_(v, u);
                // directions and end_points in the world
                Vector3 &dir_world = this->m_dirs_world_(v, u);
                Vector3 &end_pt_world = this->m_end_pts_world_(v, u);

                Dtype &range = this->m_ranges_(v, u);
                if (range <= 0 || !std::isfinite(range)) {
                    end_pt_frame.setZero();
                    dir_world.setZero();
                    end_pt_world.setZero();
                    continue;
                }

                // u <--> image coordinate x
                // v <--> image coordinate y
                end_pt_frame << dir_frame * (range / dir_frame[2]);  // range is depth currently
                range = end_pt_frame.norm();  // range is now the actual range

                // transform directions and end_points to the world
                dir_world << this->m_rotation_ * dir_frame;
                end_pt_world << this->m_rotation_ * end_pt_frame + this->m_translation_;

                // max valid range
                if (range < valid_range_min || range > valid_range_max) { continue; }
                this->m_mask_hit_(v, u) = true;
            }
        }

        this->m_max_valid_range_ = 0.0f;
        for (long u = 0; u < image_width; ++u) {
            const bool *mask_hit_ptr = this->m_mask_hit_.col(u).data();
            const Dtype *ranges_ptr = this->m_ranges_.col(u).data();
            const Vector3 *end_pts_frame_ptr = this->m_end_pts_frame_.col(u).data();
            const Vector3 *end_pts_world_ptr = this->m_end_pts_world_.col(u).data();
            for (long v = 0; v < image_height; ++v) {
                if (!mask_hit_ptr[v]) { continue; }
                if (const Dtype range = ranges_ptr[v]; range > this->m_max_valid_range_) {
                    this->m_max_valid_range_ = range;
                }
                this->m_hit_ray_indices_.emplace_back(v, u);
                this->m_hit_points_frame_.emplace_back(end_pts_frame_ptr[v]);
                this->m_hit_points_world_.emplace_back(end_pts_world_ptr[v]);
            }
        }
    }

    template<typename Dtype>
    void
    DepthFrame3D<Dtype>::UpdateRanges(
        const Eigen::Ref<const Matrix3> &rotation,
        const Eigen::Ref<const Vector3> &translation,
        const std::string &depth_file,
        const double depth_scale) {
        cv::Mat depth_img = cv::imread(depth_file, cv::IMREAD_UNCHANGED);
        // convert to float or double
        depth_img.convertTo(depth_img, sizeof(Dtype) == 4 ? CV_32FC1 : CV_64FC1);
        MatrixX depth;
        cv::cv2eigen(depth_img, depth);
        UpdateRanges(rotation, translation, DepthImageToDepth(depth, depth_scale));
    }

    template<typename Dtype>
    typename DepthFrame3D<Dtype>::MatrixX
    DepthFrame3D<Dtype>::PointCloudToRanges(
        const Matrix3 &rotation,
        const Vector3 &translation,
        const Eigen::Ref<const Matrix3X> &points,
        const bool are_local) const {
        Eigen::Matrix2Xi frame_coords(2, points.cols());  // [row, col]
        VectorX depths(points.cols());
        const Dtype fx = m_setting_->camera_intrinsic.camera_fx;
        const Dtype fy = m_setting_->camera_intrinsic.camera_fy;
        const Dtype cx = m_setting_->camera_intrinsic.camera_cx;
        const Dtype cy = m_setting_->camera_intrinsic.camera_cy;
        // compute directions, distances and frame coordinates
#pragma omp parallel for default(none) \
    shared(rotation, translation, points, are_local, frame_coords, depths, fx, fy, cx, cy)
        for (long i = 0; i < points.cols(); ++i) {
            Vector3 p = are_local ? Vector3(points.col(i))
                                  : Vector3(rotation.transpose() * (points.col(i) - translation));
            // clang-format off
            frame_coords.col(i) << static_cast<int>(std::floor(fy * p[1] / p[2] + cy)),  // row
                                   static_cast<int>(std::floor(fx * p[0] / p[2] + cx));  // col
            // clang-format on
            depths[i] = p[2];
        }
        const long rows = m_setting_->camera_intrinsic.image_height;
        const long cols = m_setting_->camera_intrinsic.image_width;
        MatrixX ranges(rows, cols);
        ranges.setConstant(-1.0f);  // -1.0f means invalid range
        for (long i = 0; i < points.cols(); ++i) {
            auto coords = frame_coords.col(i);
            if (coords[0] < 0 || coords[0] >= rows || coords[1] < 0 || coords[1] >= cols) {
                continue;  // out of range
            }
            Dtype &range = ranges(coords[0], coords[1]);
            if (range > 0.0f) { continue; }  // already has a range
            range = depths[i];
        }
        return ranges;
    }

    template<typename Dtype>
    bool
    DepthFrame3D<Dtype>::operator==(const Super &other) const {
        if (!Super::operator==(other)) { return false; }
        const auto *other_ptr = dynamic_cast<const DepthFrame3D *>(&other);
        if (other_ptr == nullptr) { return false; }
        if (m_setting_ == nullptr && other_ptr->m_setting_ != nullptr) { return false; }
        if (m_setting_ != nullptr &&
            (other_ptr->m_setting_ == nullptr || *m_setting_ != *other_ptr->m_setting_)) {
            return false;
        }
        return true;
    }

    template<typename Dtype>
    bool
    DepthFrame3D<Dtype>::Write(std::ostream &s) const {
        if (!Super::Write(s)) {
            ERL_WARN("Failed to write parent class {}.", type_name<Super>());
            return false;
        }
        static const common::TokenWriteFunctionPairs<DepthFrame3D> token_function_pairs = {
            {
                "setting",
                [](const DepthFrame3D *self, std::ostream &stream) {
                    return self->m_setting_->Write(stream) && stream.good();
                },
            },
        };
        return common::WriteTokens(s, this, token_function_pairs);
    }

    template<typename Dtype>
    bool
    DepthFrame3D<Dtype>::Read(std::istream &s) {
        if (!Super::Read(s)) {
            ERL_WARN("Failed to read parent class {}.", type_name<Super>());
            return false;
        }
        static const common::TokenReadFunctionPairs<DepthFrame3D> token_function_pairs = {
            {
                "setting",
                [](DepthFrame3D *self, std::istream &stream) {
                    return self->m_setting_->Read(stream) && stream.good();
                },
            },
        };
        return common::ReadTokens(s, this, token_function_pairs);
    }

    template<typename Dtype>
    void
    DepthFrame3D<Dtype>::UpdateFrameCoords() {
        m_setting_->camera_intrinsic.ComputeFrameDirections(
            this->m_frame_coords_,
            this->m_dirs_frame_);
    }
}  // namespace erl::geometry
