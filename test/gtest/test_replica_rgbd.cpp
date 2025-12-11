#include "erl_common/test_helper.hpp"
#include "erl_geometry/camera_base_3d.hpp"
#include "erl_geometry/open3d_helper.hpp"
#include "erl_geometry/open3d_visualizer_wrapper.hpp"
#include "erl_geometry/replica_rgbd.hpp"
#include "erl_geometry/rgbd_frame_3d.hpp"

#include <open3d/geometry/PointCloud.h>
#include <open3d/io/TriangleMeshIO.h>

TEST(ReplicaRgbd, Load) {
    using namespace erl::common;
    using namespace erl::geometry;
    constexpr bool bgr = true;

    ReplicaRgbd dataset("/home/daizhirui/DataArchive/Replica-SDF-aug2", "room0", bgr);
    ASSERT_EQ(dataset.Size(), 2600);

    const auto frame_setting = std::make_shared<RgbdFrame3Dd::Setting>();
    frame_setting->camera_intrinsic.image_width = ReplicaRgbd::kImageWidth;
    frame_setting->camera_intrinsic.image_height = ReplicaRgbd::kImageHeight;
    frame_setting->camera_intrinsic.camera_fx = ReplicaRgbd::kCameraFx;
    frame_setting->camera_intrinsic.camera_fy = ReplicaRgbd::kCameraFy;
    frame_setting->camera_intrinsic.camera_cx = ReplicaRgbd::kCameraCx;
    frame_setting->camera_intrinsic.camera_cy = ReplicaRgbd::kCameraCy;
    RgbdFrame3Dd rgbd_frame(frame_setting);

    const auto vis_setting = std::make_shared<Open3dVisualizerWrapper::Setting>();
    Open3dVisualizerWrapper visualizer(vis_setting);
    long frame_idx = 0;
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    auto mesh_scene = open3d::io::CreateMeshFromFile(dataset.GetMeshPath().string());
    auto mesh_cam_org = CreateCameraMesh();
    auto mesh_cam = std::make_shared<open3d::geometry::TriangleMesh>();
    auto lines_traj = std::make_shared<open3d::geometry::LineSet>();
    visualizer.AddGeometries({mesh_scene, mesh_cam, pcd, lines_traj});
    visualizer.SetAnimationCallback([&](auto wrapper, auto o3d_vis) -> bool {
        ReplicaRgbd::Frame frame = dataset[frame_idx];
        ERL_ASSERT_EQ(frame.sequence_number, frame_idx);
        rgbd_frame.UpdateRgbd(frame.rotation, frame.translation, frame.depth, frame.color, !bgr);
        rgbd_frame.ConvertToPointCloud(true, pcd->points_, pcd->colors_);

        mesh_cam->vertices_ = mesh_cam_org->vertices_;
        mesh_cam->triangles_ = mesh_cam_org->triangles_;
        mesh_cam->vertex_colors_ = mesh_cam_org->vertex_colors_;
        mesh_cam->Rotate(frame.rotation, {0, 0, 0}).Translate(frame.translation);

        lines_traj->points_.push_back(frame.translation);
        if (const auto n = static_cast<int>(lines_traj->points_.size()); n >= 2) {
            lines_traj->lines_.push_back(Eigen::Vector2i(n - 1, n - 2));
            lines_traj->colors_.push_back(Eigen::Vector3d(0, 1.0, 0));
        }

        o3d_vis->UpdateGeometry(pcd);
        o3d_vis->UpdateGeometry(mesh_cam);
        o3d_vis->UpdateGeometry(lines_traj);

        if constexpr (!bgr) { cv::cvtColor(frame.color, frame.color, cv::COLOR_RGB2BGR); }
        cv::imshow("color", frame.color);
        cv::imshow("depth", frame.depth_jet);
        cv::waitKey(1);

        ++frame_idx;
        if (frame_idx >= dataset.Size()) {
            wrapper->SetAnimationCallback(nullptr);
            o3d_vis->Close();
            return false;
        }
        return true;
    });
    visualizer.Show();
}
