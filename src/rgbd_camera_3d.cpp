#include "erl_geometry/rgbd_camera_3d.hpp"

#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/TriangleMeshIO.h>

namespace erl::geometry {

    void
    RgbdCamera3D::AddMesh(const std::string &mesh_path) const {
        const auto mesh = open3d::io::CreateMeshFromFile(mesh_path);
        m_visualizer_->AddGeometry(mesh);
    }

    std::pair<cv::Mat, cv::Mat>
    RgbdCamera3D::Scan(const Eigen::Ref<const Eigen::Matrix3d> &orientation, const Eigen::Ref<const Eigen::Vector3d> &translation) const {
        const auto image_height = static_cast<int>(m_setting_->image_height);
        const auto image_width = static_cast<int>(m_setting_->image_width);
        open3d::camera::PinholeCameraParameters camera_parameters;
        camera_parameters.extrinsic_ = ComputeExtrinsic(orientation, translation);
        camera_parameters.intrinsic_.intrinsic_matrix_ = m_setting_->GetIntrinsicMatrix();
        camera_parameters.intrinsic_.height_ = image_height;
        camera_parameters.intrinsic_.width_ = image_width;
        if (!m_visualizer_->GetViewControl().ConvertFromPinholeCameraParameters(camera_parameters)) {
            ERL_WARN(
                "Open3D failed to set camera parameters:\n{}\nwindow size: {}(width) x {}(height)",
                m_setting_->AsYamlString(),
                m_visualizer_->GetViewControl().GetWindowWidth(),
                m_visualizer_->GetViewControl().GetWindowHeight());
        }

        const auto rgb_image = m_visualizer_->CaptureScreenFloatBuffer(true);
        const auto depth_image = m_visualizer_->CaptureDepthFloatBuffer(true);              // depth in view space
        cv::Mat rgb_mat(image_height, image_width, CV_32FC3, rgb_image->data_.data());      // reference only
        cv::Mat depth_mat(image_height, image_width, CV_32FC1, depth_image->data_.data());  // reference only
        rgb_mat = rgb_mat.clone();                                                          // perform copy
        depth_mat = depth_mat.clone();                                                      // perform copy
        return {rgb_mat, depth_mat};
    }

}  // namespace erl::geometry
