#include "erl_common/opencv.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/camera_intrinsic.hpp"
#include "erl_geometry/open3d_visualizer_wrapper.hpp"

#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/visualization/utility/DrawGeometry.h>
#include <open3d/visualization/visualizer/Visualizer.h>
#include <open3d/visualization/visualizer/VisualizerWithKeyCallback.h>

#include <memory>

const std::filesystem::path kProjectRootDir = ERL_GEOMETRY_ROOT_DIR;

TEST(ERL_GEOMETRY, Open3dOffScreen) {
    GTEST_PREPARE_OUTPUT_DIR();
    using namespace erl::common;
    using namespace erl::geometry;

    const std::filesystem::path data_dir = kProjectRootDir / "data";
    const std::filesystem::path ply_path = data_dir / "replica-office-0.ply";

    const auto mesh = open3d::io::CreateMeshFromFile(ply_path);
    const auto visualizer = std::make_shared<open3d::visualization::Visualizer>();

    Eigen::Matrix4d cTo;
    // clang-format off
    cTo <<
        0, 0, 1, 0,
        -1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 0, 1;
    // clang-format on
    Eigen::Matrix4d wTc = Eigen::Matrix4d::Identity();
    wTc.block<3, 1>(0, 3) = mesh->GetCenter();  // place the camera at the center of the scene
    Eigen::Matrix4d wTo = wTc * cTo;

    // const Eigen::Matrix4d wTc = wTo * cTo.inverse();
    const Eigen::Matrix4d extrinsic = wTo.inverse();
    // const Eigen::Matrix3d rotation = wTc.topLeftCorner<3, 3>();
    // const Eigen::Vector3d translation = wTc.topRightCorner<3, 1>();
    // const Eigen::Vector3d right = -rotation.col(1);
    // const Eigen::Vector3d up = rotation.col(2);
    // const Eigen::Vector3d front = -rotation.col(0);
    CameraIntrinsic intrinsic;

    visualizer->CreateVisualizerWindow(test_info->name(), static_cast<int>(intrinsic.image_width), static_cast<int>(intrinsic.image_height), 50, 50, false);
    visualizer->AddGeometry(mesh);
    // auto axis_mesh = CreateAxisMesh(Eigen::Matrix4d::Identity(), 0.1);
    // axis_mesh->Translate(wTo.block<3, 1>(0, 3));
    // visualizer->AddGeometry(axis_mesh);

    auto &view_control = visualizer->GetViewControl();
    open3d::camera::PinholeCameraParameters camera_parameters;
    camera_parameters.extrinsic_ = extrinsic;
    camera_parameters.intrinsic_.intrinsic_matrix_ = intrinsic.GetIntrinsicMatrix();
    camera_parameters.intrinsic_.height_ = static_cast<int>(intrinsic.image_height);
    camera_parameters.intrinsic_.width_ = static_cast<int>(intrinsic.image_width);
    view_control.ConvertFromPinholeCameraParameters(camera_parameters);

    const auto rgb_image = visualizer->CaptureScreenFloatBuffer();
    const auto depth_image = visualizer->CaptureDepthFloatBuffer();  // depth in view space

    cv::Mat rgb_mat(static_cast<int>(intrinsic.image_height), static_cast<int>(intrinsic.image_width), CV_32FC3, rgb_image->data_.data());
    cv::Mat depth_mat(static_cast<int>(intrinsic.image_height), static_cast<int>(intrinsic.image_width), CV_32FC1, depth_image->data_.data());
    rgb_mat.convertTo(rgb_mat, CV_8UC3, 255);
    cv::cvtColor(rgb_mat, rgb_mat, cv::COLOR_RGB2BGR);
    cv::Mat depth_mat_jet;
    cv::normalize(depth_mat, depth_mat_jet, 0, 1, cv::NORM_MINMAX);  // normalize depth to [0, 1]
    depth_mat_jet.convertTo(depth_mat_jet, CV_8UC1, 255);
    cv::applyColorMap(depth_mat_jet, depth_mat_jet, cv::COLORMAP_JET);

    cv::imshow("rgb", rgb_mat);
    cv::imshow("depth", depth_mat_jet);
    cv::waitKey(0);

    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    Eigen::MatrixXd depth_eigen;
    cv::cv2eigen(depth_mat, depth_eigen);
    Eigen::Matrix4d optical_pose = wTo;
    cv::cvtColor(rgb_mat, rgb_mat, cv::COLOR_BGR2RGB);
    intrinsic.ConvertRgbdToPointCloud(depth_eigen, rgb_mat, optical_pose, pcd->points_, pcd->colors_);
    pcd->PaintUniformColor({1.0, 0.0, 0.0});
    open3d::visualization::DrawGeometries({mesh, pcd});
    // visualizer->Run();
}
