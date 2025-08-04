#include "erl_common/angle_utils.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/newer_college.hpp"
#include "erl_geometry/open3d_visualizer_wrapper.hpp"

#include <boost/program_options.hpp>
#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/PointCloud.h>

std::string newer_college_directory;
long stride = 1;

TEST(NewerCollege, Load) {
    GTEST_PREPARE_OUTPUT_DIR();

    using namespace erl::geometry;
    NewerCollege newer_college(newer_college_directory);
    cv::Mat range_img;
    cv::Mat range_img_jet;
    auto gt_mesh = newer_college.GetGroundTruthMesh();
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    auto line_set = std::make_shared<open3d::geometry::LineSet>();

    long idx = 0;
    auto callback = [&](Open3dVisualizerWrapper *wrapper,
                        open3d::visualization::Visualizer *vis) -> bool {
        if (idx >= newer_college.Size()) {
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
        // erl::common::SaveEigenMatrixToTextFile<double>(
        //     fmt::format("points.txt"),
        //     points_in_world_frame.transpose());
        // TODO: compute the azimuthe and elevation to check the layout of points.
        for (int i = 0; i < points_in_world_frame.cols(); ++i) {
            pcd->points_.emplace_back(points_in_world_frame.col(i));
        }
        pcd->PaintUniformColor({1.0, 0.0, 0.0});
        vis->UpdateGeometry(pcd);

        line_set->points_.push_back(frame.translation);
        if (line_set->points_.size() > 1) {
            line_set->lines_.emplace_back(
                Eigen::Vector2i(line_set->points_.size() - 2, line_set->points_.size() - 1));
            line_set->colors_.emplace_back(0.0, 1.0, 0.0);
            vis->UpdateGeometry(line_set);
        }

        idx += stride;

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
    try {
        namespace po = boost::program_options;
        po::options_description desc;
        desc.add_options()("help,h", "Show help message")(
            "directory",
            po::value<std::string>(&newer_college_directory),
            "Directory containing the Newer College dataset")(
            "stride",
            po::value<long>(&stride)->default_value(1),
            "Stride for loading frames (default: 1, load every frame)");

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return RUN_ALL_TESTS();
}
