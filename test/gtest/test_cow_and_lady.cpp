#include "erl_common/angle_utils.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/cow_and_lady.hpp"
#include "erl_geometry/open3d_visualizer_wrapper.hpp"

#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/visualization/utility/DrawGeometry.h>

struct Options : erl::common::Yamlable<Options> {
    std::string directory = fmt::format("{}/data/cow_and_lady", ERL_GEOMETRY_ROOT_DIR);
    double valid_range_min = 0.0;
    double valid_range_max = 1000;
    int frame_depth = 0;
    int frame_rgb = 0;
    long max_wp_idx = erl::geometry::CowAndLady::kEndIdx;
    bool show_gt = false;
    bool use_icp = false;
    bool hold = false;

    ERL_REFLECT_SCHEMA(
        Options,
        ERL_REFLECT_MEMBER(Options, directory),
        ERL_REFLECT_MEMBER(Options, valid_range_min),
        ERL_REFLECT_MEMBER(Options, valid_range_max),
        ERL_REFLECT_MEMBER(Options, frame_depth),
        ERL_REFLECT_MEMBER(Options, frame_rgb),
        ERL_REFLECT_MEMBER(Options, max_wp_idx),
        ERL_REFLECT_MEMBER(Options, show_gt),
        ERL_REFLECT_MEMBER(Options, use_icp),
        ERL_REFLECT_MEMBER(Options, hold));
};

Options g_options;

TEST(CowAndLady, Load) {
    GTEST_PREPARE_OUTPUT_DIR();

    std::cout << "Transform: vicon -> camera" << std::endl;
    Eigen::Quaterniond q(erl::geometry::CowAndLady::sk_Transform_.topLeftCorner<3, 3>());
    std::cout << "rotation: " << q.coeffs().transpose() << std::endl;
    std::cout << "translation: "
              << erl::geometry::CowAndLady::sk_Transform_.topRightCorner<3, 1>().transpose()
              << std::endl;

    const auto depth_frame_setting = std::make_shared<erl::geometry::DepthFrame3Dd::Setting>();
    depth_frame_setting->camera_intrinsic.image_height = erl::geometry::CowAndLady::kImageHeight;
    depth_frame_setting->camera_intrinsic.image_width = erl::geometry::CowAndLady::kImageWidth;
    depth_frame_setting->camera_intrinsic.camera_fx = erl::geometry::CowAndLady::kCameraFx;
    depth_frame_setting->camera_intrinsic.camera_fy = erl::geometry::CowAndLady::kCameraFy;
    depth_frame_setting->camera_intrinsic.camera_cx = erl::geometry::CowAndLady::kCameraCx;
    depth_frame_setting->camera_intrinsic.camera_cy = erl::geometry::CowAndLady::kCameraCy;
    depth_frame_setting->valid_range_min = g_options.valid_range_min;
    depth_frame_setting->valid_range_max = g_options.valid_range_max;
    erl::geometry::DepthFrame3Dd depth_frame(depth_frame_setting);

    erl::geometry::CowAndLady cow_and_lady(g_options.directory, g_options.use_icp);
    auto pcd_gt = cow_and_lady.GetGroundTruthPointCloud();
    ASSERT_TRUE(!pcd_gt->points_.empty()) << "Failed to load G.T. point cloud";

    auto traj = std::make_shared<open3d::geometry::LineSet>();
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();

    long wp_idx = 0;
    auto callback = [&](erl::geometry::Open3dVisualizerWrapper *wrapper,
                        open3d::visualization::Visualizer *vis) -> bool {
        if (wp_idx >= g_options.max_wp_idx) {
            // compute chamfer distance
            wrapper->SetAnimationCallback(nullptr);
            if (!g_options.hold) { vis->Close(); }
            return false;
        }

        auto
            [valid_frame,
             sequence_number,
             time_stamp,
             header_time_stamp,
             rotation,
             translation,
             depth,
             color,
             depth_jet] = cow_and_lady[wp_idx];
        if (!valid_frame) {  // invalid frame, stop the callback
            vis->Close();
            return false;
        }
        depth_frame.UpdateRanges(rotation, translation, depth);
        auto &points = depth_frame.GetHitPointsWorld();
        auto &hit_indices = depth_frame.GetHitRayIndices();
        for (std::size_t i = 0; i < points.size(); i += 100) {
            pcd->points_.push_back(points[i]);
            const auto &[row, col] = hit_indices[i];
            const auto pixel = color.at<cv::Vec3b>(static_cast<int>(row), static_cast<int>(col));
            pcd->colors_.emplace_back(pixel[2] / 255.0, pixel[1] / 255.0, pixel[1] / 255.0);
        }
        traj->points_.push_back(depth_frame.GetTranslationVector());
        if (traj->points_.size() > 1) {
            traj->lines_.emplace_back(wp_idx - 1, wp_idx);
            traj->colors_.emplace_back(1, 0, 0);
        }

        cv::imshow("depth_jet", depth_jet);
        cv::imshow("color", color);
        cv::waitKey(1);
        ++wp_idx;

        return true;
    };

    const auto visualizer_setting =
        std::make_shared<erl::geometry::Open3dVisualizerWrapper::Setting>();
    visualizer_setting->window_name = test_info->name();
    visualizer_setting->mesh_show_back_face = false;
    erl::geometry::Open3dVisualizerWrapper visualizer(visualizer_setting);
    if (g_options.show_gt) {
        visualizer.AddGeometries({pcd_gt, pcd});
    } else {
        visualizer.AddGeometries({pcd});
    }
    visualizer.SetViewStatus(ERL_GEOMETRY_ROOT_DIR "/data/cow_and_lady/open3d_view_status.json");
    visualizer.SetAnimationCallback(callback);
    visualizer.Show();

    // compute global ICP
    const auto result = open3d::pipelines::registration::RegistrationICP(*pcd, *pcd_gt, 0.1);
    std::cout << "Global ICP result: " << std::endl << result.transformation_ << std::endl;
}

TEST(CowAndLady, Align) {
    GTEST_PREPARE_OUTPUT_DIR();
    const erl::geometry::CowAndLady cow_and_lady(g_options.directory, false);

    auto depth = cow_and_lady[g_options.frame_depth].depth_jet;
    auto color = cow_and_lady[g_options.frame_rgb].color;

    cv::Mat overlap;
    cv::addWeighted(depth, 0.5, color, 0.5, 0, overlap);
    cv::imshow("overlap", overlap);
    cv::waitKey(0);

    for (long i = 0; i < g_options.max_wp_idx; ++i) {
        depth = cow_and_lady[i].depth_jet;
        color = cow_and_lady[i].color;
        cv::addWeighted(depth, 0.5, color, 0.5, 0, overlap);
        cv::imshow("overlap", overlap);

        cv::Mat depth_edge;
        cv::Mat color_edge;
        cv::Canny(depth, depth_edge, 100, 200);  // use as the red channel
        cv::Canny(color, color_edge, 100, 200);  // use as the green channel

        cv::Mat overlap_edge;
        cv::merge(
            std::vector<cv::Mat>{
                cv::Mat::zeros(depth_edge.size(), CV_8UC1),
                color_edge,
                depth_edge},
            overlap_edge);
        cv::imshow("overlap_edge", overlap_edge);

        cv::waitKey(10);
    }
}

TEST(CowAndLady, ComputePcdPointNormals) {
    GTEST_PREPARE_OUTPUT_DIR();
    const erl::geometry::CowAndLady cow_and_lady(g_options.directory, false);
    cow_and_lady.ComputePcdPointNormals(
        ERL_GEOMETRY_ROOT_DIR "/data/cow_and_lady/cow_and_lady_gt_with_normals.ply");
}

TEST(CowAndLady, CropCowAndLady) {
    const auto mesh0 = open3d::io::CreateMeshFromFile(
        ERL_GEOMETRY_ROOT_DIR "/data/cow_and_lady/cow_and_lady_gt_mesh0.ply");

    using namespace erl::geometry;
    using namespace open3d::geometry;
    const AxisAlignedBoundingBox cow_bounding_box(
        CowAndLady::kCowBoundingBoxMin,
        CowAndLady::kCowBoundingBoxMax);
    const AxisAlignedBoundingBox lady_bounding_box(
        CowAndLady::kLadyBoundingBoxMin,
        CowAndLady::kLadyBoundingBoxMax);

    const auto mesh_cow = mesh0->Crop(cow_bounding_box);
    const auto mesh_lady = mesh0->Crop(lady_bounding_box);
    const auto axes = TriangleMesh::CreateCoordinateFrame(0.5);
    const auto cow_bbox_lines = LineSet::CreateFromAxisAlignedBoundingBox(cow_bounding_box);
    const auto lady_bbox_lines = LineSet::CreateFromAxisAlignedBoundingBox(lady_bounding_box);
    cow_bbox_lines->PaintUniformColor({1, 0, 0});
    lady_bbox_lines->PaintUniformColor({0, 1, 0});

    open3d::visualization::DrawGeometries(
        {mesh_cow, mesh_lady, axes, cow_bbox_lines, lady_bbox_lines},
        "Cropped Cow and Lady Meshes");

    const AxisAlignedBoundingBox scene_bounding_box(
        CowAndLady::kSceneBoundingBox1Min,
        CowAndLady::kSceneBoundingBox1Max);
    const auto mesh_scene = mesh0->Crop(scene_bounding_box);
    open3d::visualization::DrawGeometries({mesh_scene, axes}, "Cropped Scene Mesh");
}

int
main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    g_options.FromCommandLine(argc, argv);
    return RUN_ALL_TESTS();
}
