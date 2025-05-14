#pragma once

namespace erl::geometry {
    template<typename Dtype>
    YAML::Node
    LidarFrame3D<Dtype>::Setting::YamlConvertImpl::encode(const Setting &setting) {
        YAML::Node node = Super::Setting::YamlConvertImpl::encode(setting);
        ERL_YAML_SAVE_ATTR(node, setting, azimuth_min);
        ERL_YAML_SAVE_ATTR(node, setting, azimuth_max);
        ERL_YAML_SAVE_ATTR(node, setting, elevation_min);
        ERL_YAML_SAVE_ATTR(node, setting, elevation_max);
        ERL_YAML_SAVE_ATTR(node, setting, num_azimuth_lines);
        ERL_YAML_SAVE_ATTR(node, setting, num_elevation_lines);
        return node;
    }

    template<typename Dtype>
    bool
    LidarFrame3D<Dtype>::Setting::YamlConvertImpl::decode(
        const YAML::Node &node,
        Setting &setting) {
        if (!Super::Setting::YamlConvertImpl::decode(node, setting)) { return false; }
        ERL_YAML_LOAD_ATTR(node, setting, azimuth_min);
        ERL_YAML_LOAD_ATTR(node, setting, azimuth_max);
        ERL_YAML_LOAD_ATTR(node, setting, elevation_min);
        ERL_YAML_LOAD_ATTR(node, setting, elevation_max);
        ERL_YAML_LOAD_ATTR(node, setting, num_azimuth_lines);
        ERL_YAML_LOAD_ATTR(node, setting, num_elevation_lines);
        return true;
    }

    template<typename Dtype>
    LidarFrame3D<Dtype>::LidarFrame3D(std::shared_ptr<Setting> setting)
        : Super(setting),
          m_setting_(std::move(setting)) {
        ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");

        VectorX azimuths = VectorX::LinSpaced(
            m_setting_->num_azimuth_lines,
            m_setting_->azimuth_min,
            m_setting_->azimuth_max);
        VectorX elevations = VectorX::LinSpaced(
            m_setting_->num_elevation_lines,
            m_setting_->elevation_min,
            m_setting_->elevation_max);
        const long num_azimuths = azimuths.size();
        const long num_elevations = elevations.size();
        this->m_frame_coords_.resize(num_azimuths, num_elevations);
        ERL_ASSERTM(num_azimuths > 0, "no azimuth angle.");
        ERL_ASSERTM(num_elevations > 0, "no elevation angle.");
        this->m_dirs_frame_.resize(num_azimuths, num_elevations);

#pragma omp parallel for collapse(2) default(none) \
    shared(num_azimuths, num_elevations, azimuths, elevations, Eigen::Dynamic)
        for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
            for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
                Vector2 &frame_coord = this->m_frame_coords_(azimuth_idx, elevation_idx);
                frame_coord << azimuths[azimuth_idx], elevations[elevation_idx];
                this->m_dirs_frame_(azimuth_idx, elevation_idx)
                    << common::AzimuthElevationToDirection(frame_coord[0], frame_coord[1]);
            }
        }
    }

    template<typename Dtype>
    bool
    LidarFrame3D<Dtype>::PointIsInFrame(const Vector3 &xyz_frame) const {
        if (const Dtype range = xyz_frame.norm();
            range < m_setting_->valid_range_min || range > m_setting_->valid_range_max) {
            return false;
        }
        return Super::CoordsIsInFrame(ComputeFrameCoords(xyz_frame.normalized()));
    }

    template<typename Dtype>
    typename LidarFrame3D<Dtype>::Vector2
    LidarFrame3D<Dtype>::ComputeFrameCoords(const Vector3 &dir_frame) const {
        Vector2 frame_coords;
        common::DirectionToAzimuthElevation<Dtype>(dir_frame, frame_coords[0], frame_coords[1]);
        return frame_coords;
    }

    template<typename Dtype>
    void
    LidarFrame3D<Dtype>::UpdateRanges(
        const Eigen::Ref<const Matrix3> &rotation,
        const Eigen::Ref<const Vector3> &translation,
        MatrixX ranges) {

        this->m_rotation_ << rotation;
        this->m_translation_ << translation;
        this->m_ranges_ = std::move(ranges);
        this->m_kd_tree_->Clear();

        const long num_azimuths = GetNumAzimuthLines();
        const long num_elevations = GetNumElevationLines();
        ERL_ASSERTM(
            num_azimuths == this->m_ranges_.rows(),
            "num of azimuths ({}) does not match rows of ranges ({}).",
            num_azimuths,
            this->m_ranges_.rows());
        ERL_ASSERTM(
            num_elevations == this->m_ranges_.cols(),
            "num of elevations ({}) does not match cols of ranges ({}).",
            num_elevations,
            this->m_ranges_.cols());

        this->m_dirs_world_.resize(num_azimuths, num_elevations);
        this->m_end_pts_frame_.resize(num_azimuths, num_elevations);
        this->m_end_pts_world_.resize(num_azimuths, num_elevations);

        this->m_mask_hit_.setConstant(num_azimuths, num_elevations, false);

        this->m_hit_ray_indices_.clear();
        this->m_hit_ray_indices_.reserve(num_azimuths * num_elevations);
        this->m_hit_points_frame_.clear();
        this->m_hit_points_frame_.reserve(num_azimuths * num_elevations);
        this->m_hit_points_world_.clear();
        this->m_hit_points_world_.reserve(num_azimuths * num_elevations);

        const Dtype valid_range_min = m_setting_->valid_range_min;
        const Dtype valid_range_max = m_setting_->valid_range_max;

#pragma omp parallel for default(none) \
    shared(num_azimuths, num_elevations, valid_range_min, valid_range_max, Eigen::Dynamic)
        for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
            for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
                // directions and end points in the frame
                const Vector3 &dir_frame = this->m_dirs_frame_(azimuth_idx, elevation_idx);
                Vector3 &end_pt_frame = this->m_end_pts_frame_(azimuth_idx, elevation_idx);
                // directions and end points in the world
                Vector3 &dir_world = this->m_dirs_world_(azimuth_idx, elevation_idx);
                Vector3 &end_pt_world = this->m_end_pts_world_(azimuth_idx, elevation_idx);

                Dtype &range = this->m_ranges_(azimuth_idx, elevation_idx);
                if (range <= 0 || !std::isfinite(range)) {
                    end_pt_frame.setZero();
                    dir_world.setZero();
                    end_pt_world.setZero();
                    continue;
                }

                dir_world << this->m_rotation_ * dir_frame;

                // end points
                end_pt_frame << range * dir_frame;
                end_pt_world << this->m_rotation_ * end_pt_frame + this->m_translation_;

                // max valid range
                if (range < valid_range_min || range > valid_range_max) { continue; }
                this->m_mask_hit_(azimuth_idx, elevation_idx) = true;
            }
        }

        this->m_max_valid_range_ = 0.0;
        for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
            const bool *mask_hit_ptr = this->m_mask_hit_.col(elevation_idx).data();
            const Dtype *ranges_ptr = this->m_ranges_.col(elevation_idx).data();
            const Vector3 *end_pts_frame_ptr = this->m_end_pts_frame_.col(elevation_idx).data();
            const Vector3 *end_pts_world_ptr = this->m_end_pts_world_.col(elevation_idx).data();
            for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
                if (!mask_hit_ptr[azimuth_idx]) { continue; }
                if (const Dtype range = ranges_ptr[azimuth_idx]; range > this->m_max_valid_range_) {
                    this->m_max_valid_range_ = range;
                }
                this->m_hit_ray_indices_.emplace_back(azimuth_idx, elevation_idx);
                this->m_hit_points_frame_.emplace_back(end_pts_frame_ptr[azimuth_idx]);
                this->m_hit_points_world_.emplace_back(end_pts_world_ptr[azimuth_idx]);
            }
        }
    }

    template<typename Dtype>
    bool
    LidarFrame3D<Dtype>::operator==(const Super &other) const {
        if (!Super::operator==(other)) { return false; }
        const auto *other_ptr = dynamic_cast<const LidarFrame3D *>(&other);
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
    LidarFrame3D<Dtype>::Write(std::ostream &s) const {
        if (!Super::Write(s)) {
            ERL_WARN("Failed to write parent class {}.", type_name<Super>());
            return false;
        }
        static const common::TokenWriteFunctionPairs<LidarFrame3D> token_function_pairs = {
            {
                "setting",
                [](const LidarFrame3D *self, std::ostream &stream) {
                    return self->m_setting_->Write(stream) && stream.good();
                },
            },
        };
        return common::WriteTokens(s, this, token_function_pairs);
    }

    template<typename Dtype>
    bool
    LidarFrame3D<Dtype>::Read(std::istream &s) {
        if (!Super::Read(s)) {
            ERL_WARN("Failed to read parent class {}.", type_name<Super>());
            return false;
        }
        static const common::TokenReadFunctionPairs<LidarFrame3D> token_function_pairs = {
            {
                "setting",
                [](LidarFrame3D *self, std::istream &stream) {
                    return self->m_setting_->Read(stream) && stream.good();
                },
            },
        };
        return common::ReadTokens(s, this, token_function_pairs);
    }
}  // namespace erl::geometry
