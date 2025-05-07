#pragma once

#include "camera_base_3d.hpp"
#include "camera_intrinsic.hpp"

#include "erl_common/opencv.hpp"

#include <open3d/visualization/visualizer/Visualizer.h>

#include <memory>

namespace erl::geometry {

    template<typename Dtype>
    class RgbdCamera3D : public CameraBase3D<Dtype> {
    public:
        using Setting = CameraIntrinsic<Dtype>;
        using Matrix3 = Eigen::Matrix3<Dtype>;
        using Vector3 = Eigen::Vector3<Dtype>;

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<open3d::visualization::Visualizer> m_visualizer_ = nullptr;

    public:
        explicit RgbdCamera3D(std::shared_ptr<Setting> setting)
            : m_setting_(std::move(setting)),
              m_visualizer_(std::make_shared<open3d::visualization::Visualizer>()) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
            m_visualizer_->CreateVisualizerWindow(
                "RGBD",
                static_cast<int>(m_setting_->image_width),
                static_cast<int>(m_setting_->image_height),
                50,
                50,
                false);
            m_visualizer_->GetRenderOption().mesh_show_back_face_ = true;
        }

        [[nodiscard]] std::shared_ptr<const Setting>
        GetSetting() const {
            return m_setting_;
        }

        void
        AddMesh(const std::string &mesh_path) const;

        /**
         * @brief Scan the scene with the camera.
         * @param orientation The orientation of the camera.
         * @param translation The translation of the camera.
         */
        [[nodiscard]] std::pair<cv::Mat, cv::Mat>
        Scan(
            const Eigen::Ref<const Matrix3> &orientation,
            const Eigen::Ref<const Vector3> &translation) const;

        [[nodiscard]] Eigen::MatrixX<Vector3>
        GetRayDirectionsInFrame() const {
            return CameraBase3D<Dtype>::ComputeRayDirectionsInFrame(
                m_setting_->image_height,
                m_setting_->image_width,
                m_setting_->camera_fx,
                m_setting_->camera_fy,
                m_setting_->camera_cx,
                m_setting_->camera_cy);
        }
    };

    using RgbdCamera3Dd = RgbdCamera3D<double>;
    using RgbdCamera3Df = RgbdCamera3D<float>;
}  // namespace erl::geometry

#include "rgbd_camera_3d.tpp"
