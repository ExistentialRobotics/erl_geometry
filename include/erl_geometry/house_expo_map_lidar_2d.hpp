#pragma once

#include "house_expo_map.hpp"
#include "lidar_2d.hpp"

namespace erl::geometry {

    /**
     * 2D LiDAR dataset based on the HouseExpoMap dataset.
     */
    class HouseExpoMapLidar2D {
        HouseExpoMap m_map_;
        Lidar2D m_lidar_;
        Eigen::Matrix3Xd m_trajectory_;
        bool m_add_noise_ = false;
        double m_noise_std_ = 0.01;

    public:
        HouseExpoMapLidar2D(
            const std::string &map_file,
            const std::string &traj_file,
            double wall_thickness,
            const std::shared_ptr<Lidar2D::Setting> &lidar_setting,
            bool add_noise = false,
            double noise_std = 0.01);

        [[nodiscard]] long
        Size() const {
            return m_trajectory_.cols();
        }

        [[nodiscard]] Eigen::Vector2d
        GetMapMin() const {
            return m_map_.GetMeterSpace()->GetSurface()->vertices.rowwise().minCoeff();
        }

        [[nodiscard]] Eigen::Vector2d
        GetMapMax() const {
            return m_map_.GetMeterSpace()->GetSurface()->vertices.rowwise().maxCoeff();
        }

        /**
         *
         * @param index frame index.
         * @return a tuple of rotation, translation, angles, and ranges.
         */
        [[nodiscard]] std::tuple<Eigen::Matrix2d, Eigen::Vector2d, Eigen::VectorXd, Eigen::VectorXd>
        operator[](long index) const;
    };
}  // namespace erl::geometry
