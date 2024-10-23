#include "erl_geometry/trajectory.hpp"

#include <Eigen/Geometry>

namespace erl::geometry {

    std::vector<Eigen::Vector2d>
    Trajectory::Load2D(const std::string &filename, const bool binary) {
        Eigen::Matrix2Xd data;
        if (binary) {
            data = common::LoadEigenMatrixFromBinaryFile<double, 2, Eigen::Dynamic>(filename);
        } else {
            data = common::LoadEigenMatrixFromTextFile<double, Eigen::Dynamic, 2>(filename, common::EigenTextFormat::kCsvFmt, true);
        }
        std::vector<Eigen::Vector2d> trajectory;
        trajectory.reserve(data.cols());
        for (long i = 0; i < data.cols(); ++i) { trajectory.emplace_back(data.col(i)); }
        return trajectory;
    }

    std::vector<Eigen::Vector3d>
    Trajectory::Load3D(const std::string &filename, const bool binary) {
        Eigen::Matrix3Xd data;
        if (binary) {
            data = common::LoadEigenMatrixFromBinaryFile<double, 3, Eigen::Dynamic>(filename);
        } else {
            data = common::LoadEigenMatrixFromTextFile<double, Eigen::Dynamic, 3>(filename, common::EigenTextFormat::kCsvFmt, true);
        }
        std::vector<Eigen::Vector3d> trajectory;
        trajectory.reserve(data.cols());
        for (long i = 0; i < data.cols(); ++i) { trajectory.emplace_back(data.col(i)); }
        return trajectory;
    }

    std::vector<std::pair<Eigen::Matrix2d, Eigen::Vector2d>>
    Trajectory::LoadSe2(const std::string &filename, const bool binary) {
        Eigen::Matrix3Xd data;
        if (binary) {
            data = common::LoadEigenMatrixFromBinaryFile<double, 3, Eigen::Dynamic>(filename);
        } else {
            data = common::LoadEigenMatrixFromTextFile<double, Eigen::Dynamic, 3>(filename, common::EigenTextFormat::kCsvFmt, true);
        }
        std::vector<std::pair<Eigen::Matrix2d, Eigen::Vector2d>> trajectory;
        trajectory.reserve(data.cols());
        for (long i = 0; i < data.cols(); ++i) {
            Eigen::Vector2d translation = data.col(i).head<2>();
            Eigen::Matrix2d rotation = Eigen::Rotation2Dd(data(2, i)).toRotationMatrix();
            trajectory.emplace_back(std::move(rotation), std::move(translation));
        }
        return trajectory;
    }

    std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>
    Trajectory::LoadSe3(const std::string &filename, const bool binary) {
        Eigen::Matrix<double, 7, Eigen::Dynamic> data;
        if (binary) {
            data = common::LoadEigenMatrixFromBinaryFile<double, 7, Eigen::Dynamic>(filename);
        } else {
            data = common::LoadEigenMatrixFromTextFile<double>(filename, common::EigenTextFormat::kCsvFmt, true);
        }
        std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> trajectory;
        trajectory.reserve(data.cols());
        for (long i = 0; i < data.cols(); ++i) {
            Eigen::Vector3d translation = data.col(i).head<3>();
            Eigen::Matrix3d rotation = Eigen::Quaterniond(data.col(i).tail<4>()).toRotationMatrix();
            trajectory.emplace_back(std::move(rotation), std::move(translation));
        }
        return trajectory;
    }

}  // namespace erl::geometry
