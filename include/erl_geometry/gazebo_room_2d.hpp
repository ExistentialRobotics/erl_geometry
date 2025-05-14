#pragma once

#include "erl_common/binary_file.hpp"
#include "erl_common/eigen.hpp"

namespace erl::geometry {

    class GazeboRoom2D {

    public:
        static constexpr double kSensorOffsetX = 0.08;  // the sensor x-offset in the robot frame.
        static constexpr double kSensorOffsetY = 0.;    // the sensor y-offset in the robot frame.
        inline static const Eigen::Vector2d kMapMin = {-4.0, -15.0};
        inline static const Eigen::Vector2d kMapMax = {19.0, 4.0};

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
    };
}  // namespace erl::geometry
