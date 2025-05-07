#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    /**
     * CameraBase3D implements the common functionalities used by DepthCamera3D and RgbdCamera3D.
     */
    template<typename Dtype>
    struct CameraBase3D {

        using Matrix4 = Eigen::Matrix4<Dtype>;
        using Matrix3 = Eigen::Matrix3<Dtype>;
        using Vector3 = Eigen::Vector3<Dtype>;

        inline static const Matrix4 cTo = []() -> Matrix4 {  // cTo
            Matrix4 mat;
            // clang-format off
            mat << 0,  0, 1, 0,
                  -1,  0, 0, 0,
                   0, -1, 0, 0,
                   0,  0, 0, 1;
            // clang-format on
            return mat;
        }();

        inline static const Matrix4 oTc = []() -> Matrix4 {  // oTc
            Matrix4 mat;
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
        [[nodiscard]] static std::tuple<Matrix3, Vector3>
        ComputeOpticalPose(
            const Eigen::Ref<const Matrix3> &orientation,
            const Eigen::Ref<const Vector3> &translation) {
            // extrinsic = oTc @ cTw
            // cTw = wTc.inverse(), wTc = [orientation, translation; 0, 1]
            // optical_pose: wTo = wTc @ cTo
            Matrix3 r = orientation * cTo.template topLeftCorner<3, 3>();
            return {std::move(r), translation};
        }

        [[nodiscard]] static std::tuple<Matrix3, Vector3>
        ComputeCameraPose(
            const Eigen::Ref<const Matrix3> &orientation,
            const Eigen::Ref<const Vector3> &translation) {
            // camera_pose: wTc = wTo @ oTc
            Matrix3 r = orientation * oTc.template topLeftCorner<3, 3>();
            return {std::move(r), translation};
        }

        [[nodiscard]] static Matrix4
        ComputeExtrinsic(
            const Eigen::Ref<const Matrix3> &camera_orientation,
            const Eigen::Ref<const Vector3> &camera_translation) {
            // extrinsic = oTc @ cTw
            // cTw = wTc.inverse(), wTc = [orientation, translation; 0, 1]
            Matrix4 extrinsic = Matrix4::Identity();
            auto r = extrinsic.template topLeftCorner<3, 3>();
            r << oTc.template topLeftCorner<3, 3>() * camera_orientation.transpose();
            extrinsic.template topRightCorner<3, 1>() = -r * camera_translation;
            return extrinsic;
        }

        [[nodiscard]] static Eigen::MatrixX<Vector3>
        ComputeRayDirectionsInFrame(
            long image_height,
            long image_width,
            Dtype camera_fx,
            Dtype camera_fy,
            Dtype camera_cx,
            Dtype camera_cy) {
            Eigen::MatrixX<Vector3> directions(image_height, image_width);

#pragma omp parallel for default(none) \
    shared(image_height,               \
               image_width,            \
               camera_fx,              \
               camera_fy,              \
               camera_cx,              \
               camera_cy,              \
               directions,             \
               Eigen::Dynamic)
            for (int u = 0; u < image_width; ++u) {
                const Dtype xu = -(static_cast<Dtype>(u) - camera_cx) / camera_fx;
                for (int v = 0; v < image_height; ++v) {
                    Vector3 &dir_frame = directions(v, u);
                    // camera normalized coordinates, i.e. depth = 1.0
                    // dir_frame << (static_cast<Dtype>(u) - camera_cx) / camera_fx,
                    //              (static_cast<Dtype>(v) - camera_cy) / camera_fy,
                    //              1.0;
                    // transform from optical frame to camera frame
                    // cRo = [0, 0, 1; -1, 0, 0; 0, -1, 0];
                    // clang-format off
                    dir_frame << 1.0,
                                 xu,
                                 -(static_cast<Dtype>(v) - camera_cy) / camera_fy;
                    // clang-format on
                }
            }

            return directions;
        }
    };

}  // namespace erl::geometry
