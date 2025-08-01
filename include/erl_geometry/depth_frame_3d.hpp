#pragma once

#include "camera_intrinsic.hpp"
#include "range_sensor_frame_3d.hpp"

#include "erl_common/yaml.hpp"

namespace erl::geometry {

    class DepthFramePartition3D;

    template<typename Dtype>
    class DepthFrame3D : public RangeSensorFrame3D<Dtype> {
    public:
        using Super = RangeSensorFrame3D<Dtype>;
        using MatrixX = Eigen::MatrixX<Dtype>;
        using Matrix3 = Eigen::Matrix3<Dtype>;
        using Matrix3X = Eigen::Matrix3X<Dtype>;
        using Vector3 = Eigen::Vector3<Dtype>;
        using Vector2 = Eigen::Vector2<Dtype>;
        using VectorX = Eigen::VectorX<Dtype>;

        struct Setting : public common::Yamlable<Setting, typename Super::Setting> {
            CameraIntrinsic<Dtype> camera_intrinsic = {};

            struct YamlConvertImpl {
                static YAML::Node
                encode(const Setting &setting);

                static bool
                decode(const YAML::Node &node, Setting &setting);
            };

            std::pair<long, long>
            Resize(Dtype factor) {
                return camera_intrinsic.Resize(factor);
            }
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;

    public:
        explicit DepthFrame3D(std::shared_ptr<Setting> setting);

        [[nodiscard]] std::shared_ptr<const Setting>
        GetSetting() const;

        void
        Reset();

        [[nodiscard]] long
        GetImageHeight() const;

        [[nodiscard]] long
        GetImageWidth() const;

        [[nodiscard]] std::pair<long, long>
        GetFrameShape() const override;

        [[nodiscard]] bool
        PointIsInFrame(const Vector3 &xyz_frame) const override;

        [[nodiscard]] Vector2
        ComputeFrameCoords(const Vector3 &dir_frame) const override;

        [[nodiscard]] static MatrixX
        DepthImageToDepth(const MatrixX &depth_img, double depth_scale);

        [[nodiscard]] static MatrixX
        DepthToDepthImage(const MatrixX &depth, double depth_scale);

        /**
         * @brief Update the frame with new depth measurements.
         * @param rotation orientation of the optical frame in the world frame.
         * @param translation translation of the optical frame in the world frame.
         * @param depth depth measurements (not depth image) in the camera frame.
         */
        void
        UpdateRanges(
            const Eigen::Ref<const Matrix3> &rotation,
            const Eigen::Ref<const Vector3> &translation,
            MatrixX depth) override;

        void
        UpdateRanges(
            const Eigen::Ref<const Matrix3> &rotation,
            const Eigen::Ref<const Vector3> &translation,
            const std::string &depth_file,
            double depth_scale);

        [[nodiscard]] MatrixX
        PointCloudToRanges(
            const Matrix3 &rotation,
            const Vector3 &translation,
            const Eigen::Ref<const Matrix3X> &points,
            bool are_local) const override;

        [[nodiscard]] bool
        operator==(const Super &other) const override;

        [[nodiscard]] bool
        Write(std::ostream &s) const override;

        [[nodiscard]] bool
        Read(std::istream &s) override;

    protected:
        void
        UpdateFrameCoords();
    };

    using DepthFrame3Dd = DepthFrame3D<double>;
    using DepthFrame3Df = DepthFrame3D<float>;

    extern template class DepthFrame3D<double>;
    extern template class DepthFrame3D<float>;

}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::DepthFrame3D<double>::Setting>
    : erl::geometry::DepthFrame3D<double>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::geometry::DepthFrame3D<float>::Setting>
    : erl::geometry::DepthFrame3D<float>::Setting::YamlConvertImpl {};
