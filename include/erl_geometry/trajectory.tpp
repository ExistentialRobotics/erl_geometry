#pragma once

#include <Eigen/Geometry>

namespace erl::geometry {

    template<typename Dtype>
    std::vector<typename Trajectory<Dtype>::Vector2>
    Trajectory<Dtype>::Load2D(const std::string &filename, const bool binary) {
        Eigen::Matrix2X<Dtype> data;
        if (binary) {
            data = common::LoadEigenMatrixFromBinaryFile<Dtype, 2, Eigen::Dynamic>(filename);
        } else {
            data = common::LoadEigenMatrixFromTextFile<Dtype, Eigen::Dynamic, 2>(filename, common::EigenTextFormat::kCsvFmt, true);
        }
        std::vector<Vector2> trajectory;
        trajectory.reserve(data.cols());
        for (long i = 0; i < data.cols(); ++i) { trajectory.emplace_back(data.col(i)); }
        return trajectory;
    }

    template<typename Dtype>
    std::vector<typename Trajectory<Dtype>::Vector3>
    Trajectory<Dtype>::Load3D(const std::string &filename, const bool binary) {
        Eigen::Matrix3X<Dtype> data;
        if (binary) {
            data = common::LoadEigenMatrixFromBinaryFile<Dtype, 3, Eigen::Dynamic>(filename);
        } else {
            data = common::LoadEigenMatrixFromTextFile<Dtype, Eigen::Dynamic, 3>(filename, common::EigenTextFormat::kCsvFmt, true);
        }
        std::vector<Vector3> trajectory;
        trajectory.reserve(data.cols());
        for (long i = 0; i < data.cols(); ++i) { trajectory.emplace_back(data.col(i)); }
        return trajectory;
    }

    template<typename Dtype>
    std::vector<std::pair<typename Trajectory<Dtype>::Matrix2, typename Trajectory<Dtype>::Vector2>>
    Trajectory<Dtype>::LoadSe2(const std::string &filename, const bool binary) {
        Eigen::Matrix3X<Dtype> data;
        if (binary) {
            data = common::LoadEigenMatrixFromBinaryFile<Dtype, 3, Eigen::Dynamic>(filename);
        } else {
            data = common::LoadEigenMatrixFromTextFile<Dtype, Eigen::Dynamic, 3>(filename, common::EigenTextFormat::kCsvFmt, true);
        }
        std::vector<std::pair<Matrix2, Vector2>> trajectory;
        trajectory.reserve(data.cols());
        for (long i = 0; i < data.cols(); ++i) {
            Vector2 translation = data.col(i).template head<2>();
            Matrix2 rotation = Eigen::Rotation2D<Dtype>(data(2, i)).toRotationMatrix();
            trajectory.emplace_back(std::move(rotation), std::move(translation));
        }
        return trajectory;
    }

    template<typename Dtype>
    std::vector<std::pair<typename Trajectory<Dtype>::Matrix3, typename Trajectory<Dtype>::Vector3>>
    Trajectory<Dtype>::LoadSe3(const std::string &filename, const bool binary) {
        Eigen::Matrix<Dtype, 7, Eigen::Dynamic> data;
        if (binary) {
            data = common::LoadEigenMatrixFromBinaryFile<Dtype, 7, Eigen::Dynamic>(filename);
        } else {
            data = common::LoadEigenMatrixFromTextFile<Dtype>(filename, common::EigenTextFormat::kCsvFmt, true);
        }
        std::vector<std::pair<Matrix3, Vector3>> trajectory;
        trajectory.reserve(data.cols());
        for (long i = 0; i < data.cols(); ++i) {
            Vector3 translation = data.col(i).template head<3>();
            Matrix3 rotation = Eigen::Quaternion<Dtype>(data.col(i).template tail<4>()).toRotationMatrix();
            trajectory.emplace_back(std::move(rotation), std::move(translation));
        }
        return trajectory;
    }

}  // namespace erl::geometry
