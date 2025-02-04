#pragma once

#include "camera_base_3d.hpp"
#include "camera_intrinsic.hpp"
#include "range_sensor_3d.hpp"

#include "erl_common/yaml.hpp"

namespace erl::geometry {

    class DepthCamera3D : public RangeSensor3D, public CameraBase3D {
    public:
        using Setting = CameraIntrinsic;

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;

    public:
        explicit DepthCamera3D(std::shared_ptr<Setting> setting)
            : m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        DepthCamera3D(std::shared_ptr<Setting> setting, const std::shared_ptr<open3d::t::geometry::RaycastingScene> &o3d_scene)
            : RangeSensor3D(o3d_scene),
              m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        [[nodiscard]] std::shared_ptr<const Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] Eigen::MatrixX<Eigen::Vector3d>
        GetRayDirectionsInFrame() const override {
            return ComputeRayDirectionsInFrame(
                m_setting_->image_height,
                m_setting_->image_width,
                m_setting_->camera_fx,
                m_setting_->camera_fy,
                m_setting_->camera_cx,
                m_setting_->camera_cy);
        }

        [[nodiscard]] std::tuple<Eigen::Matrix3d, Eigen::Vector3d>
        GetOpticalPose(const Eigen::Ref<const Eigen::Matrix3d> &orientation, const Eigen::Ref<const Eigen::Vector3d> &translation) const override {
            return ComputeCameraPose(orientation, translation);
        }
    };

}  // namespace erl::geometry
