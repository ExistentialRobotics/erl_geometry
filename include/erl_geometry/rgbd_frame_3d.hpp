#pragma once

#include "depth_frame_3d.hpp"

namespace erl::geometry {

    class RgbdFrame3D : public DepthFrame3D {
    protected:
        cv::Mat m_rgb_;

    public:
        explicit RgbdFrame3D(std::shared_ptr<Setting> setting)
            : DepthFrame3D(std::move(setting)) {}

        void
        UpdateRgbd(
            const Eigen::Ref<const Eigen::Matrix3d> &rotation,
            const Eigen::Ref<const Eigen::Vector3d> &translation,
            const Eigen::MatrixXd &depth,
            const cv::Mat &rgb,
            const bool partition_rays) {
            DepthFrame3D::UpdateRanges(rotation, translation, depth, partition_rays);
            rgb.convertTo(m_rgb_, CV_8UC3);
        }

        void
        ConvertToPointCloud(bool in_world_frame, std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) const;
    };

}  // namespace erl::geometry
