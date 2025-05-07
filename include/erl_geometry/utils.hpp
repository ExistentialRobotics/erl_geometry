#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    template<typename Dtype>
    std::vector<Eigen::Matrix4<Dtype>>
    ConvertPath2dTo3d(const Eigen::Ref<const Eigen::Matrix3X<Dtype>> &path_2d, const Dtype &z) {
        std::vector<Eigen::Matrix4<Dtype>> path_3d(
            path_2d.cols(),
            Eigen::Matrix4<Dtype>::Identity());
        for (long i = 0; i < path_2d.cols(); ++i) {
            Eigen::Matrix4<Dtype> &mat = path_3d[i];
            mat.coeffRef(0, 0) = std::cos(path_2d(2, i));
            mat.coeffRef(0, 1) = -std::sin(path_2d(2, i));
            mat.coeffRef(1, 0) = -mat(0, 1);
            mat.coeffRef(1, 1) = mat(0, 0);
            mat.coeffRef(0, 3) = path_2d(0, i);
            mat.coeffRef(1, 3) = path_2d(1, i);
            mat.coeffRef(2, 3) = z;
        }
        return path_3d;
    }

}  // namespace erl::geometry
