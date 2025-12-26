#include "erl_common/angle_utils.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/newer_college.hpp"
#include "erl_geometry/open3d_visualizer_wrapper.hpp"

#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/PointCloud.h>

struct Options : erl::common::Yamlable<Options> {
    std::string newer_college_directory = ERL_GEOMETRY_ROOT_DIR "/data/newer_college";
    long stride = 1;
    long max_wp_idx = erl::geometry::NewerCollege::Size() - 1;

    ERL_REFLECT_SCHEMA(
        Options,
        ERL_REFLECT_MEMBER(Options, newer_college_directory),
        ERL_REFLECT_MEMBER(Options, stride),
        ERL_REFLECT_MEMBER(Options, max_wp_idx));
};

static Options options;

TEST(NewerCollege, Load) {
    GTEST_PREPARE_OUTPUT_DIR();

    using namespace erl::geometry;
    NewerCollege newer_college(options.newer_college_directory);
    cv::Mat range_img;
    cv::Mat range_img_jet;
    auto gt_mesh = newer_college.GetGroundTruthMesh();
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    auto line_set = std::make_shared<open3d::geometry::LineSet>();

    long idx = 0;
    auto callback = [&](Open3dVisualizerWrapper *wrapper,
                        open3d::visualization::Visualizer *vis) -> bool {
        if (idx >= options.max_wp_idx) {
            wrapper->SetAnimationCallback(nullptr);
            return false;
        }

        auto frame = newer_college[idx];
        cv::eigen2cv(frame.GetRangeMatrix(), range_img);
        cv::normalize(range_img, range_img_jet, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::applyColorMap(range_img_jet, range_img_jet, cv::COLORMAP_JET);
        cv::transpose(range_img_jet, range_img_jet);
        cv::flip(range_img_jet, range_img_jet, 0);
        cv::resize(range_img_jet, range_img_jet, cv::Size(), 2.0, 2.0, cv::INTER_LINEAR);
        cv::imshow("Range Image", range_img_jet);
        cv::waitKey(100);

        pcd->points_.clear();
        pcd->points_.reserve(frame.points.cols());

        Eigen::Matrix3Xd points_in_world_frame = frame.GetPointsInWorldFrame();
        for (int i = 0; i < points_in_world_frame.cols(); ++i) {
            pcd->points_.emplace_back(points_in_world_frame.col(i));
        }
        pcd->PaintUniformColor({1.0, 0.0, 0.0});
        vis->UpdateGeometry(pcd);

        line_set->points_.push_back(frame.translation);
        if (line_set->points_.size() > 1) {
            line_set->lines_.emplace_back(
                line_set->points_.size() - 2,
                line_set->points_.size() - 1);
            line_set->colors_.emplace_back(0.0, 1.0, 0.0);
            vis->UpdateGeometry(line_set);
        }

        idx += options.stride;

        return true;
    };

    const auto visualizer_setting =
        std::make_shared<erl::geometry::Open3dVisualizerWrapper::Setting>();
    visualizer_setting->window_name = test_info->name();
    visualizer_setting->mesh_show_back_face = false;
    erl::geometry::Open3dVisualizerWrapper visualizer(visualizer_setting);
    visualizer.SetAnimationCallback(callback);
    visualizer.AddGeometries({gt_mesh, pcd, line_set});
    visualizer.Show();
}

int
main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    if (!options.FromCommandLine(argc, argv)) {
        ERL_ERROR("Failed to parse command line arguments");
        return -1;
    }
    return RUN_ALL_TESTS();
}
