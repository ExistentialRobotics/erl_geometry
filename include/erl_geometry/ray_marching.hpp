#pragma once

#include <Eigen/Core>

#include <functional>

namespace erl::geometry {

    /**
     * Compute the distance from the origin to the surface along the direction using signed distance field.
     * @param sdf Signed distance field.
     * @param origin Origin of the ray.
     * @param direction Direction of the ray.
     * @param threshold Accuracy threshold.
     * @param max_marching_steps Maximum number of marching steps.
     * @param max_distance Maximum distance to march.
     * @return
     * @refitem https://jamie-wong.com/2016/07/15/ray-marching-signed-distance-functions/
     */
    template<int Dim>
    double
    RayMarching(
        const std::function<double(const Eigen::Ref<const Eigen::Vector<double, Dim>> &)> &sdf,
        const Eigen::Ref<const Eigen::Vector<double, Dim>> &origin,
        const Eigen::Ref<const Eigen::Vector<double, Dim>> &direction,
        const double threshold,
        const double max_distance,
        const int max_marching_steps) {

        double distance = 0;
        int num_marching_steps = 0;
        while (max_marching_steps < 0 || num_marching_steps < max_marching_steps) {
            Eigen::Vector<double, Dim> pos = origin + distance * direction;
            const double sdf_value = sdf(pos);
            distance += sdf_value;
            num_marching_steps++;

            if (std::abs(sdf_value) < threshold || (max_distance > 0 && std::abs(distance) > max_distance)) { break; }
        }

        return distance;
    }

}  // namespace erl::geometry
