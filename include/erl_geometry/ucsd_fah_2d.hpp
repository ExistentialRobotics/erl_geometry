#pragma once
#include "erl_common/eigen.hpp"

namespace erl::geometry {

    class UcsdFah2D {
        Eigen::MatrixXd m_data_;
        long m_num_rays_ = 0;

    public:
        inline static const Eigen::Vector2d kMapMin = {-13.04, -24.70};
        inline static const Eigen::Vector2d kMapMax = {15.46, 21.63};

        struct Frame {
            long sequence_number = 0;
            long time_stamp = 0;
            long header_time_stamp = 0;
            Eigen::Matrix2d rotation;     // 2D rotation
            Eigen::Vector2d translation;  // 2D position
            Eigen::VectorXd angles;
            Eigen::VectorXd ranges;
        };

        explicit UcsdFah2D(const std::string &filename);

        [[nodiscard]] long
        Size() const {
            return m_data_.cols();
        }

        [[nodiscard]] double
        GetTimeStep() const {
            return (m_data_(1, 1) - m_data_(1, 0)) * 1e-9;
        }

        [[nodiscard]] long
        GetNumRays() const {
            return m_num_rays_;
        }

        [[nodiscard]] Frame
        operator[](long index) const;
    };
}  // namespace erl::geometry
