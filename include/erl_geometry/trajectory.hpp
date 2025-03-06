#pragma once
#include "erl_common/eigen.hpp"

#include <string>
#include <vector>

namespace erl::geometry {

    template<typename Dtype>
    struct Trajectory {
        using Vector2 = Eigen::Vector2<Dtype>;
        using Vector3 = Eigen::Vector3<Dtype>;
        using Matrix2 = Eigen::Matrix2<Dtype>;
        using Matrix3 = Eigen::Matrix3<Dtype>;

        static std::vector<Vector2>
        Load2D(const std::string &filename, bool binary);

        static std::vector<Vector3>
        Load3D(const std::string &filename, bool binary);

        static std::vector<std::pair<Matrix2, Vector2>>
        LoadSe2(const std::string &filename, bool binary);

        static std::vector<std::pair<Matrix3, Vector3>>
        LoadSe3(const std::string &filename, bool binary);
    };

    using TrajectoryD = Trajectory<double>;
    using TrajectoryF = Trajectory<float>;
}  // namespace erl::geometry

#include "trajectory.tpp"
