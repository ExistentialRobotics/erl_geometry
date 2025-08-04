#include "lidar_frame_3d.hpp"

#include "erl_common/eigen.hpp"

#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>

#include <filesystem>

namespace erl::geometry {

    class NewerCollege {

        inline static const long kNumFrames = 1988;
        inline static const long kWidth = 1024;
        inline static const long kHeight = 128;
        inline static const double kVerticalFov = M_PI_2;
        inline static const Eigen::Matrix4d kOrientedBoundingBoxPose = []() -> Eigen::Matrix4d {
            Eigen::Matrix4d pose;
            // clang-format off
            pose << 0.555697221,  -0.831384704, 0.000270056355,   22.399,
                    0.830864006,   0.555360623,   0.0352076864, -28.2124,
                  -0.0294211106, -0.0193404334,    0.999379981,  10.3735,
                              0,             0,              0,        1;
            // clang-format on
            return pose;
        }();
        inline static const Eigen::Vector3d kOrientedBoundingBoxSize = {97.2007, 78.4891, 28.4871};
        inline static const Eigen::Vector3d kMinBound = {-14.63119532, -71.84256392, -2.95865968};
        inline static const Eigen::Vector3d kMaxBound = {62.0303405, 9.5561026, 23.65667085};
        inline static const double kValidRangeMin = 0.6;
        inline static const double kValidRangeMax = 50.0;

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

        [[nodiscard]] long
        Size() const {
            return kNumFrames;
        }

        [[nodiscard]] Eigen::Vector3d
        GetMapMin() const {
            return kMinBound;
        }

        [[nodiscard]] Eigen::Vector3d
        GetMapMax() const {
            return kMaxBound;
        }

        [[nodiscard]] Frame
        operator[](long index) const;
    };
}  // namespace erl::geometry
