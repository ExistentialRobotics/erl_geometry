#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    Eigen::Matrix3Xd
    VisibleRaySynthesis(
        long point_index,
        const Eigen::Ref<const Eigen::Matrix3Xd> &hit_points,
        const Eigen::Ref<const Eigen::Vector3d> &sensor_position,
        long num_azimuth_segments,
        long num_samples_per_azimuth_segment,
        uint64_t seed);
}
