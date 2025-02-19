#pragma once

#include "depth_frame_3d.hpp"

namespace erl::geometry {

    template<typename Dtype>
    class RgbdFrame3D : public DepthFrame3D<Dtype> {
    protected:
        cv::Mat m_rgb_;

    public:
        using Super = DepthFrame3D<Dtype>;
        using Setting = typename Super::Setting;
        using Matrix = typename Super::Matrix;
        using Matrix3 = typename Super::Matrix3;
        using Vector3 = typename Super::Vector3;

        explicit RgbdFrame3D(std::shared_ptr<Setting> setting)
            : Super(std::move(setting)) {}

        void
        UpdateRgbd(
            const Eigen::Ref<const Matrix3> &rotation,
            const Eigen::Ref<const Vector3> &translation,
            const Matrix &depth,
            const cv::Mat &rgb,
            const bool partition_rays) {
            Super::UpdateRanges(rotation, translation, depth, partition_rays);
            rgb.convertTo(m_rgb_, CV_8UC3);
        }

        void
        ConvertToPointCloud(bool in_world_frame, std::vector<Vector3> &points, std::vector<Vector3> &colors) const;
    };

#include "rgbd_frame_3d.tpp"
}  // namespace erl::geometry
