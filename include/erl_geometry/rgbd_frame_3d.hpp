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
        using MatrixX = typename Super::MatrixX;
        using Matrix3 = typename Super::Matrix3;
        using Vector3 = typename Super::Vector3;

        explicit RgbdFrame3D(std::shared_ptr<Setting> setting);

        void
        UpdateRgbd(
            const Eigen::Ref<const Matrix3> &rotation,
            const Eigen::Ref<const Vector3> &translation,
            const MatrixX &depth,
            const cv::Mat &rgb,
            bool partition_rays);

        void
        ConvertToPointCloud(bool in_world_frame, std::vector<Vector3> &points, std::vector<Vector3> &colors) const;
    };

    using RgbdFrame3Dd = RgbdFrame3D<double>;
    using RgbdFrame3Df = RgbdFrame3D<float>;

}  // namespace erl::geometry

#include "rgbd_frame_3d.tpp"
