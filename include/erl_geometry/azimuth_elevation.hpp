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
    AzimuthElevationToDirection(const double azimuth, const double elevation) {
        const double sin_azimuth = std::sin(azimuth);
        const double cos_azimuth = std::cos(azimuth);
        const double sin_elevation = std::sin(elevation);
        const double cos_elevation = std::cos(elevation);
        return {cos_azimuth * cos_elevation, sin_azimuth * cos_elevation, sin_elevation};
    }

    inline void
    DirectionToAzimuthElevation(const Eigen::Ref<const Eigen::Vector3d> &direction, double &azimuth, double &elevation) {
        elevation = std::asin(direction[2]);               // [-pi/2, pi/2]
        azimuth = std::atan2(direction[1], direction[0]);  // [-pi, pi)
    }

}  // namespace erl::geometry
