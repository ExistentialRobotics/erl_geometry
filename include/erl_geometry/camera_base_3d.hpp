#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    /**
     * CameraBase3D implements the common functionalities used by DepthCamera3D and RgbdCamera3D.
     */
    struct CameraBase3D {

        inline static const Eigen::Matrix4d cTo = []() -> Eigen::Matrix4d {  // cTo
            Eigen::Matrix4d mat;
            // clang-format off
            mat << 0,  0, 1, 0,
                  -1,  0, 0, 0,
                   0, -1, 0, 0,
                   0,  0, 0, 1;
            // clang-format on
            return mat;
        }();

        inline static const Eigen::Matrix4d oTc = []() -> Eigen::Matrix4d {  // oTc
            Eigen::Matrix4d mat;
            // clang-format off
            mat << 0, -1,  0, 0,
                   0,  0, -1, 0,
                   1,  0,  0, 0,
                   0,  0,  0, 1;
            // clang-format on
            return mat;
        }();

        /**
         * @brief Get the optical pose wTo from the camera pose wTc.
         * @param orientation The orientation of the camera.
         * @param translation The translation of the camera.
         */
        [[nodiscard]] static std::tuple<Eigen::Matrix3d, Eigen::Vector3d>
        ComputeOpticalPose(const Eigen::Ref<const Eigen::Matrix3d> &orientation, const Eigen::Ref<const Eigen::Vector3d> &translation) {
            // extrinsic = oTc @ cTw
            // cTw = wTc.inverse(), wTc = [orientation, translation; 0, 1]
            // optical_pose: wTo = wTc @ cTo
            Eigen::Matrix3d r = orientation * cTo.topLeftCorner<3, 3>();
            return {std::move(r), translation};
        }

        [[nodiscard]] static std::tuple<Eigen::Matrix3d, Eigen::Vector3d>
        ComputeCameraPose(const Eigen::Ref<const Eigen::Matrix3d> &orientation, const Eigen::Ref<const Eigen::Vector3d> &translation) {
            // camera_pose: wTc = wTo @ oTc
            Eigen::Matrix3d r = orientation * oTc.topLeftCorner<3, 3>();
            return {std::move(r), translation};
        }

        [[nodiscard]] static Eigen::Matrix4d
        ComputeExtrinsic(const Eigen::Ref<const Eigen::Matrix3d> &camera_orientation, const Eigen::Ref<const Eigen::Vector3d> &camera_translation) {
            // extrinsic = oTc @ cTw
            // cTw = wTc.inverse(), wTc = [orientation, translation; 0, 1]
            Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();
            auto r = extrinsic.topLeftCorner<3, 3>();
            r << oTc.topLeftCorner<3, 3>() * camera_orientation.transpose();
            extrinsic.topRightCorner<3, 1>() = -r * camera_translation;
            return extrinsic;
        }

        [[nodiscard]] static Eigen::MatrixX<Eigen::Vector3d>
        ComputeRayDirectionsInFrame(long image_height, long image_width, double camera_fx, double camera_fy, double camera_cx, double camera_cy);
    };

}  // namespace erl::geometry
