#pragma once

#include "erl_common/yaml.hpp"
#include "lidar_frame_3d.hpp"

namespace erl::geometry {

    class RgbdFrame3D : protected LidarFrame3D {
    public:
        struct Setting : public common::OverrideYamlable<LidarFrame3D::Setting, Setting> {
            Eigen::Matrix4d camera_to_optical = Eigen::Matrix4d::Identity();  // used when camera frame is not aligned with optical frame
            // defaults are from https://github.com/cvg/nice-slam/blob/master/configs/Replica/replica.yaml
            int image_height = 680;
            int image_width = 1200;
            double camera_fx = 600.0;
            double camera_fy = 600.0;
            double camera_cx = 599.5;
            double camera_cy = 339.5;
            double depth_scale = 6553.5;
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;

    public:
        explicit RgbdFrame3D(std::shared_ptr<Setting> setting)
            : LidarFrame3D(setting),
              m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        /**
         * @brief Resize the frame by a factor. Need to call Update() after this.
         * @param factor the factor to resize the frame. (0, 1) to shrink, (1, +inf) to enlarge.
         * @return the new image size (height, width).
         */
        std::pair<int, int>
        Resize(double factor) {
            Reset();
            int old_image_height = m_setting_->image_height;
            int old_image_width = m_setting_->image_width;
            m_setting_->image_height = static_cast<int>(m_setting_->image_height * factor);
            m_setting_->image_width = static_cast<int>(m_setting_->image_width * factor);
            factor = (double(m_setting_->image_height) / double(old_image_height) + double(m_setting_->image_width) / double(old_image_width)) / 2.0;
            m_setting_->camera_fx *= factor;
            m_setting_->camera_fy *= factor;
            m_setting_->camera_cx *= factor;
            m_setting_->camera_cy *= factor;
            return {m_setting_->image_height, m_setting_->image_width};
        }

        void
        Update(
            const Eigen::Ref<const Eigen::Matrix3d> &rotation,
            const Eigen::Ref<const Eigen::Vector3d> &translation,
            Eigen::MatrixXd depth,
            bool depth_scaled,
            bool partition_rays = false
        );

        inline void
        Update(
            const Eigen::Ref<const Eigen::Matrix3d> &rotation,
            const Eigen::Ref<const Eigen::Vector3d> &translation,
            const std::string &depth_file,
            bool partition_rays = false
        ) {
            cv::Mat depth_img = cv::imread(depth_file, cv::IMREAD_UNCHANGED);
            depth_img.convertTo(depth_img, CV_64FC1);  // convert to double
            Eigen::MatrixXd depth;
            cv::cv2eigen(depth_img, depth);
            Update(rotation, translation, depth, false, partition_rays);
        }

        [[nodiscard]] inline std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] inline Eigen::Matrix4d
        GetCameraExtrinsicMatrix() const {
            Eigen::Matrix4d extrinsic = m_setting_->camera_to_optical * GetPoseMatrix();
            return extrinsic;
        }

        [[nodiscard]] inline Eigen::Matrix3d
        GetCameraIntrinsicMatrix() const {
            Eigen::Matrix3d intrinsic;
            // clang-format off
            intrinsic << m_setting_->camera_fx, 0, m_setting_->camera_cx,
                         0, m_setting_->camera_fy, m_setting_->camera_cy,
                         0, 0, 1;
            // clang-format on
            return intrinsic;
        }

        // expose LidarFrame3D's public methods that are not overridden
        using LidarFrame3D::ComputeClosestEndPoint;
        using LidarFrame3D::ComputeRaysAt;
        using LidarFrame3D::GetAzimuthAnglesInFrame;
        using LidarFrame3D::GetElevationAnglesInFrame;
        using LidarFrame3D::GetEndPointsInFrame;
        using LidarFrame3D::GetEndPointsInWorld;
        using LidarFrame3D::GetHitMask;
        using LidarFrame3D::GetMaxValidRange;
        using LidarFrame3D::GetNumAzimuthLines;
        using LidarFrame3D::GetNumElevationLines;
        using LidarFrame3D::GetNumHitRays;
        using LidarFrame3D::GetNumRays;
        using LidarFrame3D::GetPartitions;
        using LidarFrame3D::GetPoseMatrix;
        using LidarFrame3D::GetRanges;
        using LidarFrame3D::GetRayDirectionsInFrame;
        using LidarFrame3D::GetRayDirectionsInWorld;
        using LidarFrame3D::GetRotationMatrix;
        using LidarFrame3D::GetTranslationVector;
        using LidarFrame3D::IsPartitioned;
        using LidarFrame3D::IsValid;
        using LidarFrame3D::Reset;
        using LidarFrame3D::SampleAlongRays;
        using LidarFrame3D::SampleInRegion;
        using LidarFrame3D::SampleNearSurface;
    };
}  // namespace erl::geometry

namespace YAML {
    template<>
    struct convert<erl::geometry::RgbdFrame3D::Setting> {
        static inline Node
        encode(const erl::geometry::RgbdFrame3D::Setting &rhs) {
            Node node = convert<erl::geometry::LidarFrame3D::Setting>::encode(rhs);
            node["camera_to_optical"] = rhs.camera_to_optical;
            node["image_height"] = rhs.image_height;
            node["image_width"] = rhs.image_width;
            node["camera_fx"] = rhs.camera_fx;
            node["camera_fy"] = rhs.camera_fy;
            node["camera_cx"] = rhs.camera_cx;
            node["camera_cy"] = rhs.camera_cy;
            node["depth_scale"] = rhs.depth_scale;
            return node;
        }

        static inline bool
        decode(const Node &node, erl::geometry::RgbdFrame3D::Setting &rhs) {
            if (!node.IsMap()) { return false; }
            convert<erl::geometry::LidarFrame3D::Setting>::decode(node, rhs);
            rhs.camera_to_optical = node["camera_to_optical"].as<Eigen::Matrix4d>();
            rhs.image_height = node["image_height"].as<int>();
            rhs.image_width = node["image_width"].as<int>();
            rhs.camera_fx = node["camera_fx"].as<double>();
            rhs.camera_fy = node["camera_fy"].as<double>();
            rhs.camera_cx = node["camera_cx"].as<double>();
            rhs.camera_cy = node["camera_cy"].as<double>();
            rhs.depth_scale = node["depth_scale"].as<double>();
            return true;
        }
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::RgbdFrame3D::Setting &rhs) {
        out << BeginMap;
        out << Key << "valid_range_min" << Value << rhs.valid_range_min;
        out << Key << "valid_range_max" << Value << rhs.valid_range_max;
        out << Key << "valid_azimuth_min" << Value << rhs.valid_azimuth_min;
        out << Key << "valid_azimuth_max" << Value << rhs.valid_azimuth_max;
        out << Key << "valid_elevation_min" << Value << rhs.valid_elevation_min;
        out << Key << "valid_elevation_max" << Value << rhs.valid_elevation_max;
        out << Key << "discontinuity_factor" << Value << rhs.discontinuity_factor;
        out << Key << "rolling_diff_discount" << Value << rhs.rolling_diff_discount;
        out << Key << "min_partition_size" << Value << rhs.min_partition_size;
        out << Key << "camera_to_optical" << Value << rhs.camera_to_optical;
        out << Key << "image_height" << Value << rhs.image_height;
        out << Key << "image_width" << Value << rhs.image_width;
        out << Key << "camera_fx" << Value << rhs.camera_fx;
        out << Key << "camera_fy" << Value << rhs.camera_fy;
        out << Key << "camera_cx" << Value << rhs.camera_cx;
        out << Key << "camera_cy" << Value << rhs.camera_cy;
        out << Key << "depth_scale" << Value << rhs.depth_scale;
        out << EndMap;
        return out;
    }
}  // namespace YAML
