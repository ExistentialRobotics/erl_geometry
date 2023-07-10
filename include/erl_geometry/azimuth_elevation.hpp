#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    /**
     * Convert azimuth-elevation to 3d direction vector
     * @param azimuth: [0, 2pi), angle between x-axis and the projection of the direction vector on x-y plane
     * @param elevation: [-pi/2, pi/2], angle between x-y plane and the direction vector
     * @return
     */
    inline Eigen::Vector3d
    AzimuthElevationToDirection(double azimuth, double elevation) {
        double sin_azimuth = std::sin(azimuth);
        double cos_azimuth = std::cos(azimuth);
        double sin_elevation = std::sin(elevation);
        double cos_elevation = std::cos(elevation);

        return {cos_azimuth * cos_elevation, sin_azimuth * cos_elevation, sin_elevation};
    }

    inline Eigen::Vector2d DirectionToAzimuthElevation(const Eigen::Ref<const Eigen::Vector3d> &direction) {
        Eigen::Vector2d azimuth_elevation;

        double cos_elevation = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1]);
        double sin_elevation = direction[2];
        azimuth_elevation[1] = std::atan2(sin_elevation, cos_elevation);
        azimuth_elevation[0] = std::atan2(direction[1] / cos_elevation, direction[0] / cos_elevation);

        return azimuth_elevation;
    }

}  // namespace erl::geometry
