#pragma once

#include "camera_base_3d.hpp"
#include "camera_intrinsic.hpp"
#include "range_sensor_3d.hpp"

#include "erl_common/logging.hpp"
#include "erl_common/yaml.hpp"

namespace erl::geometry {

    template<typename Dtype>
    class DepthCamera3D : public RangeSensor3D<Dtype>, public CameraBase3D<Dtype> {
    public:
        using Setting = CameraIntrinsic<Dtype>;
        using Matrix3 = Eigen::Matrix3<Dtype>;
        using Vector3 = Eigen::Vector3<Dtype>;

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;

    public:
        explicit DepthCamera3D(std::shared_ptr<Setting> setting)
            : m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        DepthCamera3D(
            std::shared_ptr<Setting> setting,
            const std::shared_ptr<open3d::t::geometry::RaycastingScene> &o3d_scene)
            : RangeSensor3D<Dtype>(o3d_scene),
              m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        [[nodiscard]] std::shared_ptr<const Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] Eigen::MatrixX<Vector3>
        GetRayDirectionsInFrame() const override {
            return CameraBase3D<Dtype>::ComputeRayDirectionsInFrame(
                m_setting_->image_height,
                m_setting_->image_width,
                static_cast<Dtype>(m_setting_->camera_fx),
                static_cast<Dtype>(m_setting_->camera_fy),
                static_cast<Dtype>(m_setting_->camera_cx),
                static_cast<Dtype>(m_setting_->camera_cy));
        }

        [[nodiscard]] std::tuple<Matrix3, Vector3>
        GetOpticalPose(
            const Eigen::Ref<const Matrix3> &orientation,
            const Eigen::Ref<const Vector3> &translation) const override {
            return CameraBase3D<Dtype>::ComputeOpticalPose(orientation, translation);
        }
    };

    using DepthCamera3Dd = DepthCamera3D<double>;
    using DepthCamera3Df = DepthCamera3D<float>;

}  // namespace erl::geometry
