#include "erl_geometry/lidar_frame_3d.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/random.hpp"

#include <absl/container/flat_hash_map.h>

#include <utility>

namespace erl::geometry {

    LidarFrame3D::LidarFrame3D(std::shared_ptr<Setting> setting)
        : RangeSensorFrame3D(setting),
          m_setting_(std::move(setting)) {
        ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");

        Eigen::VectorXd azimuths = Eigen::VectorXd::LinSpaced(m_setting_->num_azimuth_lines, m_setting_->azimuth_min, m_setting_->azimuth_max);
        Eigen::VectorXd elevations = Eigen::VectorXd::LinSpaced(m_setting_->num_elevation_lines, m_setting_->elevation_min, m_setting_->elevation_max);
        const long num_azimuths = azimuths.size();
        const long num_elevations = elevations.size();
        m_frame_coords_.resize(num_azimuths, num_elevations);
        ERL_ASSERTM(num_azimuths > 0, "no azimuth angle.");
        ERL_ASSERTM(num_elevations > 0, "no elevation angle.");
        m_dirs_frame_.resize(num_azimuths, num_elevations);

#pragma omp parallel for collapse(2) default(none) shared(num_azimuths, num_elevations, azimuths, elevations, Eigen::Dynamic)
        for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
            for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
                Eigen::Vector2d &frame_coord = m_frame_coords_(azimuth_idx, elevation_idx);
                frame_coord << azimuths[azimuth_idx], elevations[elevation_idx];
                m_dirs_frame_(azimuth_idx, elevation_idx) << common::AzimuthElevationToDirection(frame_coord[0], frame_coord[1]);
            }
        }
    }

    void
    LidarFrame3D::UpdateRanges(
        const Eigen::Ref<const Eigen::Matrix3d> &rotation,
        const Eigen::Ref<const Eigen::Vector3d> &translation,
        Eigen::MatrixXd ranges,
        const bool partition_rays) {

        m_rotation_ << rotation;
        m_translation_ << translation;
        m_ranges_ = std::move(ranges);
        m_partitions_.clear();
        m_partitioned_ = false;
        m_kd_tree_->Clear();

        const long num_azimuths = GetNumAzimuthLines();
        const long num_elevations = GetNumElevationLines();
        ERL_ASSERTM(num_azimuths == m_ranges_.rows(), "num of azimuths ({}) does not match rows of ranges ({}).", num_azimuths, m_ranges_.rows());
        ERL_ASSERTM(num_elevations == m_ranges_.cols(), "num of elevations ({}) does not match cols of ranges ({}).", num_elevations, m_ranges_.cols());

        m_dirs_world_.resize(num_azimuths, num_elevations);
        m_end_pts_frame_.resize(num_azimuths, num_elevations);
        m_end_pts_world_.resize(num_azimuths, num_elevations);

        m_mask_hit_.setConstant(num_azimuths, num_elevations, false);

        m_hit_ray_indices_.clear();
        m_hit_ray_indices_.reserve(num_azimuths * num_elevations);
        m_hit_points_world_.clear();
        m_hit_points_world_.reserve(num_azimuths * num_elevations);

#pragma omp parallel for default(none) shared(num_azimuths, num_elevations, Eigen::Dynamic)
        for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
            for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
                double &range = m_ranges_(azimuth_idx, elevation_idx);
                if (range == 0 || std::isnan(range)) { continue; }  // zero or nan depth! Not allowed.

                // directions
                const Eigen::Vector3d &dir_frame = m_dirs_frame_(azimuth_idx, elevation_idx);
                m_dirs_world_(azimuth_idx, elevation_idx) << m_rotation_ * dir_frame;

                // end points
                Eigen::Vector3d &end_pt_frame = m_end_pts_frame_(azimuth_idx, elevation_idx);
                end_pt_frame << range * dir_frame;
                m_end_pts_world_(azimuth_idx, elevation_idx) << m_rotation_ * end_pt_frame + m_translation_;

                // max valid range
                if (range < m_setting_->valid_range_min || range > m_setting_->valid_range_max) { continue; }
                m_mask_hit_(azimuth_idx, elevation_idx) = true;
            }
        }

        m_max_valid_range_ = 0.0;
        for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
            for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
                if (!m_mask_hit_(azimuth_idx, elevation_idx)) { continue; }
                if (const double range = m_ranges_(azimuth_idx, elevation_idx); range > m_max_valid_range_) { m_max_valid_range_ = range; }
                m_hit_ray_indices_.emplace_back(azimuth_idx, elevation_idx);
                m_hit_points_world_.emplace_back(m_end_pts_world_(azimuth_idx, elevation_idx));
            }
        }

        if (!partition_rays) { return; }  // do not partition rays
        PartitionRays();
    }

    void
    LidarFrame3D::PartitionRays() {
        const long num_azimuths = GetNumAzimuthLines();
        const long num_elevations = GetNumElevationLines();
        // detect discontinuities, out-of-max-range measurements
        m_mask_continuous_.setConstant(num_azimuths, num_elevations, true);
        m_mask_continuous_.row(0).setConstant(false);
        m_mask_continuous_.col(0).setConstant(false);
        m_mask_continuous_.bottomRows<1>().setConstant(false);
        m_mask_continuous_.rightCols<1>().setConstant(false);
        double rolling_range_diff = 0.0;
        const double gamma1 = m_setting_->rolling_diff_discount;
        const double gamma2 = 1 - gamma1;
        for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
            for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
                if (!m_mask_hit_(azimuth_idx, elevation_idx)) { continue; }
                const Eigen::Vector2d &frame_coord = m_frame_coords_(azimuth_idx, elevation_idx);
                const double azimuth = frame_coord[0];
                const double elevation = frame_coord[1];
                const double range = m_ranges_(azimuth_idx, elevation_idx);
                if (azimuth_idx == 0) {
                    if (elevation_idx == 0) { continue; }
                    const double range_diff = std::abs(  //
                        (range - m_ranges_(azimuth_idx, elevation_idx - 1)) / (elevation - m_frame_coords_(azimuth_idx, elevation_idx - 1)[1]));
                    if (range_diff > m_setting_->discontinuity_factor * rolling_range_diff) { m_mask_continuous_(azimuth_idx, elevation_idx - 1) = false; }
                    if (rolling_range_diff == 0.0) { rolling_range_diff = range_diff; }
                    rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * range_diff;
                } else {
                    double range_diff = std::abs(  //
                        (range - m_ranges_(azimuth_idx - 1, elevation_idx)) / (azimuth - m_frame_coords_(azimuth_idx - 1, elevation_idx)[0]));
                    if (range_diff > m_setting_->discontinuity_factor * rolling_range_diff) { m_mask_continuous_(azimuth_idx - 1, elevation_idx) = false; }
                    rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * range_diff;
                    if (elevation_idx == 0) { continue; }
                    range_diff = std::abs(  //
                        (range - m_ranges_(azimuth_idx, elevation_idx - 1)) / (elevation - m_frame_coords_(azimuth_idx, elevation_idx - 1)[1]));
                    if (range_diff > m_setting_->discontinuity_factor * rolling_range_diff) { m_mask_continuous_(azimuth_idx, elevation_idx - 1) = false; }
                    rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * range_diff;
                }
            }
        }

        throw std::runtime_error("Implementation for partitioning rays is not complete.");
    }
}  // namespace erl::geometry
