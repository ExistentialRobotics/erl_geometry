#pragma once
#include "erl_common/eigen.hpp"

#include <string>
#include <vector>

namespace erl::geometry {

    struct Trajectory {

        static std::vector<Eigen::Vector2d>
        Load2D(const std::string &filename, bool binary);

        static std::vector<Eigen::Vector3d>
        Load3D(const std::string &filename, bool binary);

        static std::vector<std::pair<Eigen::Matrix2d, Eigen::Vector2d>>
        LoadSe2(const std::string &filename, bool binary);

        static std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>
        LoadSe3(const std::string &filename, bool binary);
    };
}  // namespace erl::geometry
