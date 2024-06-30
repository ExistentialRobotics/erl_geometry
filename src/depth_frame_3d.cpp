#include "erl_geometry/depth_frame_3d.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/exception.hpp"
#include "erl_common/opencv.hpp"

namespace erl::geometry {

    DepthFrame3D::DepthFrame3D(std::shared_ptr<Setting> setting)
        : RangeSensorFrame3D(setting),
          m_setting_(std::move(setting)) {
        ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        ERL_ASSERTM(
            std::abs(m_setting_->camera_to_optical.topLeftCorner<3, 3>().determinant() - 1.0) < 1e-6,
            "camera_to_optical does not have a valid rotation matrix.");
        ERL_ASSERTM((m_setting_->camera_to_optical.bottomLeftCorner<1, 3>().isZero(1e-6)), "camera_to_optical bottom row should be [0, 0, 0, 1].");
        ERL_ASSERTM(m_setting_->camera_to_optical(3, 3) == 1.0, "camera_to_optical(3, 3) should be 1.0.");
        UpdateFrameCoords();
    }

    std::pair<long, long>
    DepthFrame3D::Setting::Resize(double factor) {
        const long old_image_height = image_height;
        const long old_image_width = image_width;
        image_height = static_cast<int>(image_height * factor);
        image_width = static_cast<int>(image_width * factor);
        factor = (static_cast<double>(image_height) / static_cast<double>(old_image_height) +
                  static_cast<double>(image_width) / static_cast<double>(old_image_width)) /
                 2.0;
        camera_fx *= factor;
        camera_cx *= factor;
        camera_cy *= factor;
        return {image_height, image_width};
    }

    void
    DepthFrame3D::UpdateRanges(
        const Eigen::Ref<const Eigen::Matrix3d> &rotation,
        const Eigen::Ref<const Eigen::Vector3d> &translation,
        Eigen::MatrixXd depth,
        const bool partition_rays) {

        const long image_height = m_setting_->image_height;
        const long image_width = m_setting_->image_width;
        ERL_ASSERTM(depth.rows() == image_height, "depth image height ({}) does not match setting ({}).", depth.rows(), image_height);
        ERL_ASSERTM(depth.cols() == image_width, "depth image width ({}) does not match setting ({}).", depth.cols(), image_width);

        m_rotation_ << rotation;
        m_translation_ << translation;
        m_camera_extrinsic_ = GetPoseMatrix() * m_setting_->camera_to_optical;
        m_ranges_ = std::move(depth);

        m_partitions_.clear();
        m_partitioned_ = false;
        m_kd_tree_->Clear();

        m_dirs_world_.resize(image_height, image_width);
        m_end_pts_frame_.resize(image_height, image_width);
        m_end_pts_world_.resize(image_height, image_width);

        m_mask_hit_.setConstant(image_height, image_width, false);
        m_hit_ray_indices_.clear();
        m_hit_ray_indices_.reserve(image_height * image_width);
        m_hit_points_world_.clear();
        m_hit_points_world_.reserve(image_height * image_width);

        // compute directions and end points
#pragma omp parallel for default(none) shared(image_height, image_width, Eigen::Dynamic)
        for (long u = 0; u < image_width; ++u) {
            for (long v = 0; v < image_height; ++v) {
                double &range = m_ranges_(v, u);
                if (range == 0 || std::isnan(range)) { continue; }  // zero or nan depth! Not allowed.

                // directions and end_points in frame
                Eigen::Vector3d &dir_frame = m_dirs_frame_(v, u);
                Eigen::Vector3d &end_pt_frame = m_end_pts_frame_(v, u);
                // u <--> image coordinate x
                // v <--> image coordinate y
                end_pt_frame << dir_frame * (range / dir_frame[2]);  // range is depth currently
                range = end_pt_frame.norm();                         // range is now the actual range

                // transform directions and end_points to world
                auto rotation_ref = m_camera_extrinsic_.topLeftCorner<3, 3>();
                auto translation_ref = m_camera_extrinsic_.topRightCorner<3, 1>();
                m_dirs_world_(v, u) << rotation_ref * dir_frame;
                m_end_pts_world_(v, u) << rotation_ref * end_pt_frame + translation_ref;

                // max valid range
                if (range < m_setting_->valid_range_min || range > m_setting_->valid_range_max) { continue; }
                m_mask_hit_(v, u) = true;
            }
        }

        m_max_valid_range_ = 0.0;
        for (long u = 0; u < image_width; ++u) {
            for (long v = 0; v < image_height; ++v) {
                if (!m_mask_hit_(v, u)) { continue; }
                if (const double range = m_ranges_(v, u); range > m_max_valid_range_) { m_max_valid_range_ = range; }
                m_hit_ray_indices_.emplace_back(v, u);
                m_hit_points_world_.emplace_back(m_end_pts_world_(v, u));
            }
        }

        if (!partition_rays) { return; }  // do not partition rays
        PartitionRays();
    }

    void
    DepthFrame3D::PartitionRays() {
        throw NotImplemented(__PRETTY_FUNCTION__);
    }

    void
    DepthFrame3D::UpdateFrameCoords() {
        const long image_height = m_setting_->image_height;
        const long image_width = m_setting_->image_width;
        m_frame_coords_.resize(image_height, image_width);
        m_dirs_frame_.resize(image_height, image_width);

#pragma omp parallel for default(none) shared(image_height, image_width, Eigen::Dynamic)
        for (long u = 0; u < image_width; ++u) {
            const double xu = (static_cast<double>(u) - m_setting_->camera_cx) / m_setting_->camera_fx;
            for (long v = 0; v < image_height; ++v) {
                const double yv = (static_cast<double>(v) - m_setting_->camera_cy) / m_setting_->camera_fy;
                Eigen::Vector2d &frame_coord = m_frame_coords_(v, u);
                // normalized image coordinates
                // v <--> image coordinate y, u <--> image coordinate x
                frame_coord << yv, xu;
                Eigen::Vector3d &dir_frame = m_dirs_frame_(v, u);
                dir_frame << xu, yv, 1.0;
                dir_frame.normalize();
            }
        }
    }

}  // namespace erl::geometry
