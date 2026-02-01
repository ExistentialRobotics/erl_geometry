#pragma once

#include "lidar_frame_3d.hpp"

#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>

#include <filesystem>

namespace erl::geometry {

    class NewerCollege {
    public:
        static constexpr long kNumFrames = 1988;
        static constexpr long kNumAzimuthLines = 1024;
        static constexpr long kNumElevationLines = 128;
        static constexpr double kVerticalFov = M_PI_2;
        inline static const Eigen::Matrix3d kOrientedBoundingBoxRotation = []() -> Eigen::Matrix3d {
            Eigen::Matrix3d pose;
            // clang-format off
            pose << 0.774249792098999, 0.6328740119934082, 0.0027840007096528534,
                    -0.6328780055046082, 0.7742511034011841, 0.0008005643612705558,
                    -0.00164885923732066, -0.002381769474595785, 0.9999958276748657;
            // clang-format on
            return pose;
        }();
        inline static const Eigen::Quaterniond kOrientedBoundingBoxQuaternion = {
            0.9418726988259943,      // qw
            -0.0008446825775481628,  // qx
            0.0011766080364413608,   // qy
            -0.3359668506889849,     // qz
        };
        inline static const Eigen::Vector3d kOrientedBoundingBoxTranslation = {
            23.242795944213867,
            -34.984092712402344,
            11.9350006103515625};
        inline static const Eigen::Vector3d kOrientedBoundingBoxSize = {
            39.444294829572186,
            55.454335589784975,
            27.358959347669018};
        inline static const Eigen::Vector3d kMinBound = {
            -5.039750099182129,
            -67.9994888305664,
            -1.3748812675476074};
        inline static const Eigen::Vector3d kMaxBound = {
            55.44883346557617,
            -1.8061834573745728,
            26.16172218322754};
        static constexpr double kValidRangeMin = 0.6;
        static constexpr double kValidRangeMax = 50.0;

    private:
        std::filesystem::path m_directory_;
        Eigen::Matrix<double, 7, Eigen::Dynamic> m_poses_;  // (x, y, z, qx, qy, qz, qw)

    public:
        struct Frame {
            Eigen::Matrix3d rotation;
            Eigen::Vector3d translation;
            Eigen::Matrix3Xd points;

            [[nodiscard]] Eigen::MatrixXd
            GetRangeMatrix() const;

            [[nodiscard]] Eigen::Matrix3Xd
            GetPointsInWorldFrame() const;
        };

        explicit NewerCollege(std::filesystem::path directory);

        [[nodiscard]] std::shared_ptr<open3d::geometry::PointCloud>
        GetGroundTruthPointCloud() const;

        [[nodiscard]] std::shared_ptr<open3d::geometry::TriangleMesh>
        GetGroundTruthMesh() const;

        [[nodiscard]] static long
        Size() {
            return kNumFrames;
        }

        [[nodiscard]] static Eigen::Vector3d
        GetMapMin() {
            return kMinBound;
        }

        [[nodiscard]] static Eigen::Vector3d
        GetMapMax() {
            return kMaxBound;
        }

        [[nodiscard]] Frame
        operator[](long index) const;
    };
}  // namespace erl::geometry
