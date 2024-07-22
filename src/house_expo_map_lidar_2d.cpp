#include "erl_geometry/house_expo_map_lidar_2d.hpp"

#include "erl_common/eigen.hpp"
#include "erl_common/random.hpp"

namespace erl::geometry {
    HouseExpoMapLidar2D::HouseExpoMapLidar2D(
        const std::string &map_file,
        const std::string &traj_file,
        const double wall_thickness,
        const std::shared_ptr<Lidar2D::Setting> &lidar_setting,
        const bool add_noise,
        const double noise_std)
        : m_map_(map_file, wall_thickness),
          m_lidar_(lidar_setting, m_map_.GetMeterSpace()),
          m_trajectory_(common::LoadEigenMatrixFromTextFile<double>(traj_file, common::EigenTextFormat::kCsvFmt, true)),
          m_add_noise_(add_noise),
          m_noise_std_(noise_std) {}

    std::tuple<Eigen::Matrix2d, Eigen::Vector2d, Eigen::VectorXd, Eigen::VectorXd>
    HouseExpoMapLidar2D::operator[](const long index) const {
        const double *data = m_trajectory_.col(index).data();
        Eigen::Vector2d translation(data[0], data[1]);
        Eigen::Matrix2d rotation = Eigen::Rotation2Dd(data[2]).toRotationMatrix();
        Eigen::VectorXd ranges = m_lidar_.Scan(rotation, translation, true);
        Eigen::VectorXd angles = m_lidar_.GetAngles();
        if (m_add_noise_) { ranges += common::GenerateGaussianNoise(ranges.size(), 0.0, m_noise_std_); }
        return {std::move(rotation), std::move(translation), std::move(angles), std::move(ranges)};
    }

}  // namespace erl::geometry
