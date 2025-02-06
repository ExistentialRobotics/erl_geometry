#include "erl_common/angle_utils.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/cow_and_lady.hpp"
#include "erl_geometry/open3d_visualizer_wrapper.hpp"

#include <boost/program_options.hpp>
#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/visualization/utility/DrawGeometry.h>

struct Options {
    std::string directory;
    bool use_icp = false;
    bool hold = false;
};

Options g_options;

TEST(CowAndLady, Load) {
    GTEST_PREPARE_OUTPUT_DIR();

    const auto depth_frame_setting = std::make_shared<erl::geometry::DepthFrame3D::Setting>();
    depth_frame_setting->camera_intrinsic.image_height = erl::geometry::CowAndLady::kImageHeight;
    depth_frame_setting->camera_intrinsic.image_width = erl::geometry::CowAndLady::kImageWidth;
    depth_frame_setting->camera_intrinsic.camera_fx = erl::geometry::CowAndLady::kCameraFx;
    depth_frame_setting->camera_intrinsic.camera_fy = erl::geometry::CowAndLady::kCameraFy;
    depth_frame_setting->camera_intrinsic.camera_cx = erl::geometry::CowAndLady::kCameraCx;
    depth_frame_setting->camera_intrinsic.camera_cy = erl::geometry::CowAndLady::kCameraCy;
    erl::geometry::DepthFrame3D depth_frame(depth_frame_setting);

    erl::geometry::CowAndLady cow_and_lady(g_options.directory, g_options.use_icp);
    auto pcd_gt = cow_and_lady.GetGroundTruthPointCloud();
    ASSERT_TRUE(!pcd_gt->points_.empty()) << "Failed to load G.T. point cloud";

    auto traj = std::make_shared<open3d::geometry::LineSet>();
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();

    long wp_idx = 0;
    auto callback = [&](erl::geometry::Open3dVisualizerWrapper *wrapper, open3d::visualization::Visualizer *vis) -> bool {
        if (wp_idx >= cow_and_lady.Size()) {
            // compute chamfer distance
            wrapper->SetAnimationCallback(nullptr);
            if (!g_options.hold) { vis->Close(); }
            return false;
        }

        auto [sequence_number, time_stamp, header_time_stamp, rotation, translation, depth, color, depth_jet] = cow_and_lady[wp_idx];
        depth_frame.UpdateRanges(rotation, translation, depth, false);
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
        wp_idx++;

        return true;
    };

    const auto visualizer_setting = std::make_shared<erl::geometry::Open3dVisualizerWrapper::Setting>();
    visualizer_setting->window_name = test_info->name();
    visualizer_setting->mesh_show_back_face = false;
    erl::geometry::Open3dVisualizerWrapper visualizer(visualizer_setting);
    visualizer.AddGeometries({pcd_gt, pcd});
    visualizer.SetAnimationCallback(callback);
    visualizer.Show();

    // compute global ICP
    const auto result = open3d::pipelines::registration::RegistrationICP(*pcd, *pcd_gt, 0.1);
    std::cout << "Global ICP result: " << std::endl << result.transformation_ << std::endl;
}

int
main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    try {
        namespace po = boost::program_options;
        po::options_description desc;
        // clang-format off
        desc.add_options()
            ("help", "produce help message")
            ("directory", po::value<std::string>(&g_options.directory), "Cow and lady dataset directory")
            ("use-icp", po::bool_switch(&g_options.use_icp)->default_value(false), "Use ICP for registration")
            ("hold", po::bool_switch(&g_options.hold)->default_value(false), "Hold the visualization window");
        // clang-format on
        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
        if (vm.count("help")) {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl << desc << std::endl;
            return 0;
        }
        po::notify(vm);
    } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return RUN_ALL_TESTS();
}
