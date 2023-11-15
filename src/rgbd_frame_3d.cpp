#include "erl_geometry/rgbd_frame_3d.hpp"
#include "erl_common/opencv.hpp"
#include "erl_geometry/azimuth_elevation.hpp"

namespace erl::geometry {

    void
    RgbdFrame3D::Update(
        const Eigen::Ref<const Eigen::Matrix3d> &rotation,
        const Eigen::Ref<const Eigen::Vector3d> &translation,
        Eigen::MatrixXd depth,
        bool depth_scaled,
        bool partition_rays
    ) {
        ERL_ASSERTM(depth.rows() == m_setting_->image_height, "depth image height (%ld) does not match setting (%d).", depth.rows(), m_setting_->image_height);
        ERL_ASSERTM(depth.cols() == m_setting_->image_width, "depth image width (%ld) does not match setting (%d).", depth.cols(), m_setting_->image_width);
        m_rotation_ << rotation;
        m_translation_ << translation;
        m_azimuth_frame_.resize(depth.rows(), depth.cols());
        m_elevation_frame_.resize(depth.rows(), depth.cols());
        m_ranges_ = std::move(depth);
        if (!depth_scaled) { m_ranges_ /= m_setting_->depth_scale; }
        m_partitions_.clear();
        m_partitioned_ = false;
        m_kd_tree_->Clear();

        long num_azimuths = GetNumAzimuthLines();
        long num_elevations = GetNumElevationLines();
        ERL_ASSERTM(num_azimuths == m_ranges_.rows(), "num of azimuths (%ld) does not match rows of ranges (%ld).", num_azimuths, m_ranges_.rows());
        ERL_ASSERTM(num_elevations == m_ranges_.cols(), "num of elevations (%ld) does not match cols of ranges (%ld).", num_elevations, m_ranges_.cols());
        ERL_ASSERTM(num_azimuths > 0, "no azimuth angle.");
        ERL_ASSERTM(num_elevations > 0, "no elevation angle.");

        m_dirs_frame_.resize(num_azimuths, num_elevations);
        m_dirs_world_.resize(num_azimuths, num_elevations);
        m_end_pts_frame_.resize(num_azimuths, num_elevations);
        m_end_pts_world_.resize(num_azimuths, num_elevations);

        m_mask_hit_.setConstant(num_azimuths, num_elevations, false);
        m_hit_ray_indices_.resize(2, num_azimuths * num_elevations);
        m_hit_points_world_.resize(3, num_azimuths * num_elevations);

        // compute directions and end points
#pragma omp parallel for collapse(2) default(none) shared(num_azimuths, num_elevations, Eigen::Dynamic)
        for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
            for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
                double &range = m_ranges_(azimuth_idx, elevation_idx);
                if (range == 0 || std::isnan(range)) { continue; }  // zero or nan depth! Not allowed.
                double &azimuth = m_azimuth_frame_(azimuth_idx, elevation_idx);
                double &elevation = m_elevation_frame_(azimuth_idx, elevation_idx);

                // directions and end_points in frame
                Eigen::Vector3d &dir_frame = m_dirs_frame_(azimuth_idx, elevation_idx);
                Eigen::Vector3d &end_pt_frame = m_end_pts_frame_(azimuth_idx, elevation_idx);
                // elevation_idx <--> image coordinate x
                // azimuth_idx <--> image coordinate y
                // clang-format off
                // camera normalized coordinates, i.e. depth = 1.0
                dir_frame << (double(elevation_idx) - m_setting_->camera_cx) / m_setting_->camera_fx,
                             (double(azimuth_idx) - m_setting_->camera_cy) / m_setting_->camera_fy,
                             1.0;
                // clang-format on
                end_pt_frame = dir_frame * range;   // range is depth currently
                range = end_pt_frame.norm();        // range is now the actual range
                dir_frame << end_pt_frame / range;  // normalize direction
                DirectionToAzimuthElevation(dir_frame, azimuth, elevation);

                // transform directions and end_points to world
                m_dirs_world_(azimuth_idx, elevation_idx) << m_rotation_ * dir_frame;
                m_end_pts_world_(azimuth_idx, elevation_idx) << m_rotation_ * end_pt_frame + m_translation_;

                // max valid range
                // cannot move this line to the outer loop, because directions are computed in the inner loop
                if ((azimuth < m_setting_->valid_azimuth_min) || (azimuth > m_setting_->valid_azimuth_max)) { continue; }
                if (std::isnan(range) || (range < m_setting_->valid_range_min) || (range > m_setting_->valid_range_max)) { continue; }
                m_mask_hit_(azimuth_idx, elevation_idx) = true;
            }
        }

        m_max_valid_range_ = 0.0;
        long num_hit_points = 0;
        for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
            for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
                if (!m_mask_hit_(azimuth_idx, elevation_idx)) { continue; }
                double &range = m_ranges_(azimuth_idx, elevation_idx);
                if (range > m_max_valid_range_) { m_max_valid_range_ = range; }
                m_hit_ray_indices_.col(num_hit_points) << azimuth_idx, elevation_idx;
                m_hit_points_world_.col(num_hit_points++) << m_end_pts_world_(azimuth_idx, elevation_idx);
            }
        }
        m_hit_ray_indices_.conservativeResize(2, num_hit_points);
        m_hit_points_world_.conservativeResize(3, num_hit_points);

        if (!partition_rays) { return; }  // do not partition rays
        PartitionRays();
    }

}  // namespace erl::geometry
