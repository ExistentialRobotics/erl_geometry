#pragma once

#include "lidar_frame_3d.hpp"

#include "erl_common/yaml.hpp"

namespace erl::geometry {

    class RgbdFrame3D : public LidarFrame3D {
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

    public:
        explicit RgbdFrame3D(std::shared_ptr<Setting> setting)
            : LidarFrame3D(std::move(setting)) {}

        /**
         * @brief Resize the frame by a factor. Need to call Update() after this.
         * @param factor the factor to resize the frame. (0, 1) to shrink, (1, +inf) to enlarge.
         * @return the new image size (height, width).
         */
        std::pair<int, int>
        Resize(double factor) {
            Reset();
            auto *setting = reinterpret_cast<Setting *>(m_setting_.get());
            const int old_image_height = setting->image_height;
            const int old_image_width = setting->image_width;
            setting->image_height = static_cast<int>(setting->image_height * factor);
            setting->image_width = static_cast<int>(setting->image_width * factor);
            factor = (static_cast<double>(setting->image_height) / static_cast<double>(old_image_height) +
                      static_cast<double>(setting->image_width) / static_cast<double>(old_image_width)) /
                     2.0;
            setting->camera_fx *= factor;
            setting->camera_fy *= factor;
            setting->camera_cx *= factor;
            setting->camera_cy *= factor;
            return {setting->image_height, setting->image_width};
        }

        void
        Update(
            const Eigen::Ref<const Eigen::Matrix3d> &rotation,
            const Eigen::Ref<const Eigen::Vector3d> &translation,
            Eigen::MatrixXd depth,
            bool depth_scaled,
            bool partition_rays = false);

        void
        Update(
            const Eigen::Ref<const Eigen::Matrix3d> &rotation,
            const Eigen::Ref<const Eigen::Vector3d> &translation,
            const std::string &depth_file,
            const bool partition_rays = false) {
            cv::Mat depth_img = cv::imread(depth_file, cv::IMREAD_UNCHANGED);
            depth_img.convertTo(depth_img, CV_64FC1);  // convert to double
            Eigen::MatrixXd depth;
            cv::cv2eigen(depth_img, depth);
            Update(rotation, translation, depth, false, partition_rays);
        }

        [[nodiscard]] Eigen::Matrix4d
        GetCameraExtrinsicMatrix() const {
            Eigen::Matrix4d extrinsic = reinterpret_cast<Setting *>(m_setting_.get())->camera_to_optical * GetPoseMatrix();
            return extrinsic;
        }

        [[nodiscard]] Eigen::Matrix3d
        GetCameraIntrinsicMatrix() const {
            Eigen::Matrix3d intrinsic;
            const auto *setting = reinterpret_cast<Setting *>(m_setting_.get());
            // clang-format off
            intrinsic << setting->camera_fx, 0, setting->camera_cx,
                         0, setting->camera_fy, setting->camera_cy,
                         0, 0, 1;
            // clang-format on
            return intrinsic;
        }
    };
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::RgbdFrame3D::Setting> {
    static Node
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

    static bool
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
};  // namespace YAML
