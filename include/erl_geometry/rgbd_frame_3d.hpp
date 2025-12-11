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

        /**
         *
         * @param rotation orientation of the optical frame.
         * @param translation translation of the optical frame.
         * @param depth depth image.
         * @param rgb color image in BGR format if is_rgb is false, otherwise in RGB format.
         * @param is_rgb if true, the input color image is in RGB format; if false, in BGR format.
         */
        void
        UpdateRgbd(
            const Eigen::Ref<const Matrix3> &rotation,
            const Eigen::Ref<const Vector3> &translation,
            const MatrixX &depth,
            const cv::Mat &rgb,
            bool is_rgb = true);

        void
        ConvertToPointCloud(
            bool in_world_frame,
            std::vector<Vector3> &points,
            std::vector<Vector3> &colors) const;
    };

    using RgbdFrame3Dd = RgbdFrame3D<double>;
    using RgbdFrame3Df = RgbdFrame3D<float>;

    extern template class RgbdFrame3D<double>;
    extern template class RgbdFrame3D<float>;

}  // namespace erl::geometry
