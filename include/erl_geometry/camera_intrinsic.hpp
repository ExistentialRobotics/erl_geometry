#pragma once

#include "erl_common/yaml.hpp"

namespace erl::geometry {

    struct CameraIntrinsic : common::Yamlable<CameraIntrinsic> {
        // defaults are from https://github.com/cvg/nice-slam/blob/master/configs/Replica/replica.yaml
        long image_height = 680;
        long image_width = 1200;
        double camera_fx = 600.0;
        double camera_fy = 600.0;
        double camera_cx = 599.5;
        double camera_cy = 339.5;

        [[nodiscard]] Eigen::Matrix3d
        GetIntrinsicMatrix() const {
            Eigen::Matrix3d intrinsic;
            // clang-format off
            intrinsic << camera_fx, 0, camera_cx,
                         0, camera_fy, camera_cy,
                         0, 0, 1;
            // clang-format on
            return intrinsic;
        }

        std::pair<long, long>
        Resize(double factor) {
            const long old_image_height = image_height;
            const long old_image_width = image_width;
            image_height = static_cast<int>(static_cast<double>(image_height) * factor);
            image_width = static_cast<int>(static_cast<double>(image_width) * factor);
            factor = (static_cast<double>(image_height) / static_cast<double>(old_image_height) +
                      static_cast<double>(image_width) / static_cast<double>(old_image_width)) /
                     2.0;
            camera_fx *= factor;
            camera_cx *= factor;
            camera_cy *= factor;
            return {image_height, image_width};
        }

        void
        ComputeFrameDirection(const long u, const long v, double &dir_x, double &dir_y, double &dir_z) const {
            dir_x = (static_cast<double>(u) - camera_cx) / camera_fx;
            dir_y = (static_cast<double>(v) - camera_cy) / camera_fy;
            const double norm = std::sqrt(dir_x * dir_x + dir_y * dir_y + 1.0);
            dir_x /= norm;
            dir_y /= norm;
            dir_z = 1.0 / norm;
        }

        void
        ComputeFrameDirection(const long u, const long v, Eigen::Vector3d &dir) const {
            ComputeFrameDirection(u, v, dir.x(), dir.y(), dir.z());
        }

        void
        ComputeFrameDirections(Eigen::MatrixX<Eigen::Vector3d> &dirs) const;

        void
        ComputeFrameDirections(Eigen::MatrixX<Eigen::Vector2d> &coords, Eigen::MatrixX<Eigen::Vector3d> &dirs) const;

        void
        ConvertDepthToDistance(const long u, const long v, const double depth, double &distance) const {
            const double xu = (static_cast<double>(u) - camera_cx) / camera_fx;
            const double yv = (static_cast<double>(v) - camera_cy) / camera_fy;
            distance = depth * std::sqrt(xu * xu + yv * yv + 1.0);
        }

        void
        ConvertDepthToDistance(const Eigen::MatrixXd &depth, Eigen::MatrixXd &distance) const;

        void
        ConvertDistanceToDepth(const Eigen::MatrixXd &distance, Eigen::MatrixXd &depth) const;

        void
        ConvertRgbdToPointCloud(
            const Eigen::MatrixXd &depth,
            const cv::Mat &rgb,
            const std::optional<Eigen::Matrix4d> &optical_pose,
            std::vector<Eigen::Vector3d> &points,
            std::vector<Eigen::Vector3d> &colors) const;
    };

}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::CameraIntrinsic> {
    static Node
    encode(const erl::geometry::CameraIntrinsic &rhs) {
        Node node;
        node["image_height"] = rhs.image_height;
        node["image_width"] = rhs.image_width;
        node["camera_fx"] = rhs.camera_fx;
        node["camera_fy"] = rhs.camera_fy;
        node["camera_cx"] = rhs.camera_cx;
        node["camera_cy"] = rhs.camera_cy;
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::CameraIntrinsic &rhs) {
        if (!node.IsMap()) { return false; }
        rhs.image_height = node["image_height"].as<long>();
        rhs.image_width = node["image_width"].as<long>();
        rhs.camera_fx = node["camera_fx"].as<double>();
        rhs.camera_fy = node["camera_fy"].as<double>();
        rhs.camera_cx = node["camera_cx"].as<double>();
        rhs.camera_cy = node["camera_cy"].as<double>();
        return true;
    }
};  // namespace YAML
