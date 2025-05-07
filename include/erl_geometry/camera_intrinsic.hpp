#pragma once

#include "erl_common/yaml.hpp"

namespace erl::geometry {

    template<typename Dtype>
    struct CameraIntrinsic : common::Yamlable<CameraIntrinsic<Dtype>> {
        // the defaults are from
        // https://github.com/cvg/nice-slam/blob/master/configs/Replica/replica.yaml
        long image_height = 680;
        long image_width = 1200;
        Dtype camera_fx = 600.0f;
        Dtype camera_fy = 600.0f;
        Dtype camera_cx = 599.5f;
        Dtype camera_cy = 339.5f;

        struct YamlConvertImpl {
            static YAML::Node
            encode(const CameraIntrinsic &intrinsic);

            static bool
            decode(const YAML::Node &node, CameraIntrinsic &intrinsic);
        };

        using MatrixX = Eigen::MatrixX<Dtype>;
        using Matrix4 = Eigen::Matrix4<Dtype>;
        using Matrix3 = Eigen::Matrix3<Dtype>;
        using Vector3 = Eigen::Vector3<Dtype>;
        using Vector2 = Eigen::Vector2<Dtype>;

        [[nodiscard]] Matrix3
        GetIntrinsicMatrix() const {
            Matrix3 intrinsic;
            // clang-format off
            intrinsic << camera_fx, 0, camera_cx,
                         0, camera_fy, camera_cy,
                         0, 0, 1;
            // clang-format on
            return intrinsic;
        }

        std::pair<long, long>
        Resize(Dtype factor) {
            const auto old_image_height = static_cast<Dtype>(image_height);
            const auto old_image_width = static_cast<Dtype>(image_width);
            image_height = static_cast<int>(image_height * factor);
            image_width = static_cast<int>(image_width * factor);
            factor = (static_cast<Dtype>(image_height) / old_image_height +
                      static_cast<Dtype>(image_width) / old_image_width) *
                     0.5f;
            camera_fx *= factor;
            camera_cx *= factor;
            camera_cy *= factor;
            return {image_height, image_width};
        }

        void
        ComputeFrameDirection(const long u, const long v, Dtype &dir_x, Dtype &dir_y, Dtype &dir_z)
            const {
            dir_x = (static_cast<Dtype>(u) - camera_cx) / camera_fx;
            dir_y = (static_cast<Dtype>(v) - camera_cy) / camera_fy;
            const Dtype norm = std::sqrt(dir_x * dir_x + dir_y * dir_y + 1.0f);
            dir_x /= norm;
            dir_y /= norm;
            dir_z = 1.0f / norm;
        }

        void
        ComputeFrameDirection(const long u, const long v, Vector3 &dir) const {
            ComputeFrameDirection(u, v, dir.x(), dir.y(), dir.z());
        }

        void
        ComputeFrameDirections(Eigen::MatrixX<Vector3> &dirs) const;

        void
        ComputeFrameDirections(Eigen::MatrixX<Vector2> &coords, Eigen::MatrixX<Vector3> &dirs)
            const;

        void
        ConvertDepthToDistance(const long u, const long v, const Dtype depth, Dtype &distance)
            const {
            const Dtype xu = (static_cast<Dtype>(u) - camera_cx) / camera_fx;
            const Dtype yv = (static_cast<Dtype>(v) - camera_cy) / camera_fy;
            distance = depth * std::sqrt(xu * xu + yv * yv + 1.0f);
        }

        void
        ConvertDepthToDistance(const MatrixX &depth, MatrixX &distance) const;

        void
        ConvertDistanceToDepth(const MatrixX &distance, MatrixX &depth) const;

        void
        ConvertRgbdToPointCloud(
            const MatrixX &depth,
            const cv::Mat &rgb,
            const std::optional<Matrix4> &optical_pose,
            std::vector<Vector3> &points,
            std::vector<Vector3> &colors) const;
    };

#include "camera_intrinsic.tpp"

    using CameraIntrinsicD = CameraIntrinsic<double>;
    using CameraIntrinsicF = CameraIntrinsic<float>;
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::CameraIntrinsic<double>>
    : erl::geometry::CameraIntrinsic<double>::YamlConvertImpl {};

template<>
struct YAML::convert<erl::geometry::CameraIntrinsic<float>>
    : erl::geometry::CameraIntrinsic<float>::YamlConvertImpl {};
