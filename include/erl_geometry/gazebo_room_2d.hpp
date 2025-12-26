#pragma once

#include "erl_common/binary_file.hpp"

#include <open3d/geometry/TriangleMesh.h>

namespace erl::geometry {

    class GazeboRoom2D {

    public:
        static constexpr double kSensorOffsetX = 0.08;  // the sensor x-offset in the robot frame.
        static constexpr double kSensorOffsetY = 0.;    // the sensor y-offset in the robot frame.
        static constexpr long kNumLines = 270;          // number of laser scan lines per frame.
        static constexpr long kNumFrames = 2879;        // number of frames in the dataset.
        static constexpr double kFov = M_PI_2 * 3.0;    // field of view of the 2D lidar (270 deg).
        static constexpr double kRangeMin = 1.0;        // minimum range of the data.
        static constexpr double kRangeMax = 20.5;       // maximum range of the data.
        inline static const Eigen::Vector2d kMapMin = {-3.5, -14.5};
        inline static const Eigen::Vector2d kMapMax = {18.5, 4.0};
        inline static const std::vector<Eigen::Vector2d> kWallCorners = {
            // wall corners, extracted by GazeboRoom2D.ExtractCorners from test_gazebo_room_2d.cpp
            {16.71, 3.49},
            {7.79, 2.72},
            {6.69, 2.60},
            {-3.15, 1.86},
            {7.02, -1.27},
            {4.06, -1.52},
            {8.20, -2.23},
            {4.12, -2.53},
            {13.52, -5.78},
            {10.47, -6.01},
            {3.63, -7.59},
            {-2.34, -8.08},
            {13.78, -8.80},
            {10.76, -9.05},
            {3.77, -9.57},
            {-2.16, -10.08},
            {18.04, -12.32},
            {-1.78, -14.09},
        };
        inline static const std::vector<Eigen::Vector2i> kWallSegments = {
            // outer wall: 1, 17, 18, 16, 15, 11, 12, 4, 3, 5, 6, 8, 7, 2 (clockwise)
            {0, 16},
            {16, 17},
            {17, 15},
            {15, 14},
            {14, 10},
            {10, 11},
            {11, 3},
            {3, 2},
            {2, 4},
            {4, 5},
            {5, 7},
            {7, 6},
            {6, 1},
            {1, 0},
            // inner wall: 10, 14, 13, 9 (counter-clockwise)
            {9, 13},
            {13, 12},
            {12, 8},
            {8, 9},
        };
        inline static const Eigen::Matrix2d kOrientedBoundingBoxRotation = []() -> Eigen::Matrix2d {
            // obtained from GazeboRoom2D.ComputeOrientedBoundingBox in test_gazebo_room_2d.cpp
            Eigen::Matrix2d rot;
            // clang-format off
            rot << 0.9962470, -0.0865519,
                   0.0865519, 0.99624700;
            // clang-format on
            return rot;
        }();
        static constexpr double kOrientedBoundingBoxRotationAngle = 0.08666035900008191;  // rad
        inline static const Eigen::Vector2d kOrientedBoundingBoxCenter = {7.4550, -5.265};
        inline static const Eigen::Vector2d kOrientedBoundingBoxSize = {20.1422, 16.2087};

        struct TrainDataFrame {
            Eigen::Matrix2d rotation;     // 2D rotation
            Eigen::Vector2d translation;  // 2D position
            Eigen::VectorXd angles;
            Eigen::VectorXd ranges;
        };

        class TrainDataLoader {
            std::vector<TrainDataFrame> m_data_frames_;

        public:
            explicit TrainDataLoader(const std::string &path);

            TrainDataFrame &
            operator[](const size_t i) {
                return m_data_frames_[i];
            }

            const TrainDataFrame &
            operator[](const size_t i) const {
                return m_data_frames_[i];
            }

            [[nodiscard]] long
            size() const {
                return static_cast<long>(m_data_frames_.size());
            }

            auto
            begin() {
                return m_data_frames_.begin();
            }

            auto
            end() {
                return m_data_frames_.end();
            }
        };

        struct TestDataFrame {
            Eigen::Matrix2Xd positions;
            std::vector<double> out_buf;  // for GPisMap
            int dim = 0;
            int num_queries = 0;

            explicit TestDataFrame(const std::string &path);

            void
            Extract(
                Eigen::VectorXd &distance,
                Eigen::Matrix2Xd &gradient,
                Eigen::VectorXd &distance_variance,
                Eigen::Matrix2Xd &gradient_variance);
        };

        static std::shared_ptr<open3d::geometry::TriangleMesh>
        ExtrudeTo3D(double room_height, bool add_ceiling);

        static Eigen::VectorXd
        ComputeSdf(const Eigen::Ref<const Eigen::Matrix2Xd> &positions);
    };
}  // namespace erl::geometry
