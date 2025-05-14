#include "erl_common/grid_map_info.hpp"
#include "erl_common/opencv.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_covariance/radial_bias_function.hpp"
#include "erl_geometry/bayesian_hilbert_map.hpp"
#include "erl_geometry/lidar_2d.hpp"
#include "erl_geometry/space_2d.hpp"

#include <boost/program_options.hpp>

using namespace erl::common;
using namespace erl::geometry;
using namespace erl::covariance;

template<typename Dtype>
std::vector<Eigen::Vector3<Dtype>>
GenerateTrajectory(const int n = 50, const int repeats = 1) {
    const int total = n * repeats;
    std::vector<Eigen::Vector3<Dtype>> trajectory;
    trajectory.reserve(total);
    const Dtype angle_step = 2 * M_PI / n;
    constexpr Dtype a = 1.6;
    constexpr Dtype b = 1.2;
    Dtype angle = 0;
    for (int i = 0; i < n; ++i) {
        Dtype pose_angle = 0;
        if (i > 0) {
            pose_angle = std::atan2(
                trajectory[i].y() - trajectory[i - 1].y(),
                trajectory[i].x() - trajectory[i - 1].x());
        }
        trajectory.emplace_back(a * std::cos(angle), b * std::sin(angle), pose_angle);
        angle += angle_step;
    }
    for (int r = 1; r < repeats; ++r) {
        trajectory.insert(trajectory.end(), trajectory.begin(), trajectory.begin() + n);
    }
    return trajectory;
}

std::shared_ptr<Space2D>
GenerateSpace() {

    std::vector<Eigen::Vector2d> ordered_object_vertices;

    int n = 50;
    Eigen::Matrix2Xd pts_circle1(2, n);
    double angle_step = 2 * M_PI / static_cast<double>(n);
    double angle = 0;
    for (int i = 0; i < n; ++i) {
        constexpr double r1 = 0.3;
        constexpr double x1 = -1.0;
        constexpr double y1 = 0.2;
        pts_circle1.col(i) << r1 * std::cos(angle) + x1, r1 * std::sin(angle) + y1;
        angle += angle_step;
    }

    n = 100;
    Eigen::Matrix2Xd pts_circle2(2, n);
    angle_step = 2 * M_PI / static_cast<double>(n);
    angle = 0;
    for (int i = 0; i < n; ++i) {
        constexpr double r2 = 0.8;
        constexpr double x2 = 0.3;
        constexpr double y2 = 0.0;
        pts_circle2.col(i) << r2 * std::cos(angle) + x2, r2 * std::sin(angle) + y2;
        angle += angle_step;
    }

    n = 40;
    Eigen::Matrix2Xd pts_box(2, n * 4);
    constexpr double half_size = 2.0;
    const double step = 2 * half_size / static_cast<double>(n);
    double v = -half_size;
    for (int i = 0; i < n; ++i) {
        int j = i;
        pts_box.col(j) << -half_size, v;
        j += n;
        pts_box.col(j) << v, half_size;
        j += n;
        pts_box.col(j) << half_size, -v;
        j += n;
        pts_box.col(j) << -v, -half_size;
        v += step;
    }

    std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> pts;
    pts.reserve(3);
    pts.emplace_back(pts_circle1);
    pts.emplace_back(pts_circle2);
    pts.emplace_back(pts_box);

    Eigen::VectorXb outside_flags(3);
    outside_flags << true, true, false;

    return std::make_shared<Space2D>(pts, outside_flags);
}

template<typename Dtype>
std::pair<GridMapInfo2D<Dtype>, Eigen::Matrix2X<Dtype>>
GenerateGridPoints(const int grid_size, const Aabb<Dtype, 2> &map_boundary) {
    GridMapInfo2D<Dtype> grid_map(
        Eigen::Vector2i(grid_size, grid_size),
        map_boundary.min(),
        map_boundary.max());
    Eigen::Matrix2X<Dtype> points = grid_map.GenerateMeterCoordinates(true);
    return {grid_map, points};
}

template<typename Dtype>
void
VisualizeResult(
    const GridMapInfo2D<Dtype> &grid_map,
    const std::vector<Eigen::Vector2<Dtype>> &waypoints,
    const std::vector<Eigen::Matrix2X<Dtype>> &scanned_points,
    const Eigen::VectorX<Dtype> &prob_occupied,
    const Eigen::Matrix2X<Dtype> &gradient_grid,
    const Eigen::Matrix2X<Dtype> &surface_points,
    const Eigen::Matrix2X<Dtype> &gradient_surf) {

    const long img_size = grid_map.Shape(0);
    using MatrixX = Eigen::MatrixX<Dtype>;
    MatrixX eigen_img = Eigen::Map<const MatrixX>(prob_occupied.data(), img_size, img_size);
    cv::Mat img_prob_occupied, img_prob_occupied_rgb;
    cv::eigen2cv(eigen_img, img_prob_occupied);
    cv::normalize(img_prob_occupied, img_prob_occupied, 0, 255, cv::NORM_MINMAX);
    img_prob_occupied.convertTo(img_prob_occupied, CV_8UC1);
    cv::applyColorMap(img_prob_occupied, img_prob_occupied_rgb, cv::COLORMAP_JET);

    Eigen::VectorX<Dtype> gradient_norm = gradient_grid.colwise().norm();
    eigen_img = Eigen::Map<MatrixX>(gradient_norm.data(), img_size, img_size);
    cv::Mat img_gradient_norm, img_gradient_norm_rgb;
    cv::eigen2cv(eigen_img, img_gradient_norm);
    cv::normalize(img_gradient_norm, img_gradient_norm, 0, 255, cv::NORM_MINMAX);
    img_gradient_norm.convertTo(img_gradient_norm, CV_8UC1);
    cv::applyColorMap(img_gradient_norm, img_gradient_norm_rgb, cv::COLORMAP_JET);

    cv::flip(img_prob_occupied_rgb, img_prob_occupied_rgb, 0);
    cv::flip(img_gradient_norm_rgb, img_gradient_norm_rgb, 0);

    constexpr int s = 9;
    cv::resize(
        img_prob_occupied_rgb,
        img_prob_occupied_rgb,
        cv::Size(),
        s,
        s,
        cv::INTER_LINEAR_EXACT);
    cv::resize(
        img_gradient_norm_rgb,
        img_gradient_norm_rgb,
        cv::Size(),
        s,
        s,
        cv::INTER_LINEAR_EXACT);

    GridMapInfo2D<Dtype> grid_map_scaled(grid_map.Shape() * s, grid_map.Min(), grid_map.Max());

    // visualize scanned points
    std::vector<std::vector<cv::Point>> pts(1);
    for (std::size_t idx = 0; idx < waypoints.size(); ++idx) {
        const Eigen::Vector2<Dtype> &waypoint = waypoints[idx];
        Eigen::Vector2i px = grid_map_scaled.MeterToPixelForPoints(waypoint);
        pts[0].emplace_back(px[0], px[1]);
        // cv::circle(img_prob_occupied_rgb, cv::Point(px[0], px[1]), 2, cv::Scalar(255, 255, 255),
        // -1);

        const Eigen::Matrix2X<Dtype> &points = scanned_points[idx];
        for (long i = 0; i < points.cols(); ++i) {
            px = grid_map_scaled.MeterToPixelForPoints(points.col(i));
            cv::circle(
                img_prob_occupied_rgb,
                cv::Point(px[0], px[1]),
                1,
                cv::Scalar(0, 255, 0),
                -1);
        }
    }
    cv::polylines(img_prob_occupied_rgb, pts, false, cv::Scalar(255, 255, 255), 1, cv::LINE_AA, 0);

    // visualize gradient
    for (long i = 0; i < surface_points.cols(); ++i) {
        Eigen::Vector2i px = grid_map_scaled.MeterToPixelForPoints(surface_points.col(i));
        const cv::Point pt1(px[0], px[1]);
        Eigen::Vector2<Dtype> gradient = gradient_surf.col(i);
        gradient.normalize();
        px += grid_map_scaled.MeterToPixelForVectors(-gradient * 0.25);
        const cv::Point pt2(px[0], px[1]);
        cv::arrowedLine(img_gradient_norm_rgb, pt1, pt2, cv::Scalar(255, 255, 255), 2);
        cv::circle(img_gradient_norm_rgb, pt1, 1, cv::Scalar(0, 0, 255), -1);
    }

    cv::imshow("Probability Occupied", img_prob_occupied_rgb);
    cv::imshow("Gradient Norm", img_gradient_norm_rgb);
    cv::waitKey(100);
}

template<typename Dtype>
void
TestIo(
    const erl::geometry::BayesianHilbertMap<Dtype, 2> &bhm,
    const Eigen::Matrix2X<Dtype> &hinged_points,
    const Aabb<Dtype, 2> &map_boundary) {
    GTEST_PREPARE_OUTPUT_DIR();
    std::string filename = fmt::format("test_bhm_2d_{}.bin", type_name<Dtype>());
    filename = test_output_dir / filename;
    using Serializer = Serialization<BayesianHilbertMap<Dtype, 2>>;
    ASSERT_TRUE(Serializer::Write(filename, &bhm));
    BayesianHilbertMap<Dtype, 2> bhm_read(
        std::make_shared<BayesianHilbertMapSetting>(),
        std::make_shared<RadialBiasFunction<Dtype, 2>>(
            std::make_shared<typename Covariance<Dtype>::Setting>()),
        hinged_points,
        map_boundary,
        1);
    ASSERT_TRUE(Serializer::Read(filename, &bhm_read));
    ASSERT_TRUE(bhm == bhm_read);
}

template<typename Dtype>
void
TestImpl2D(
    const int hinged_grid_size,
    const int test_grid_size,
    const Dtype rbf_gamma,
    const bool diagonal_sigma,
    const bool faster,
    const bool use_sparse) {

    auto bhm_setting = std::make_shared<BayesianHilbertMapSetting>();
    bhm_setting->diagonal_sigma = diagonal_sigma;
    bhm_setting->use_sparse = use_sparse;

    auto kernel_setting = std::make_shared<typename Covariance<Dtype>::Setting>();
    kernel_setting->scale = std::sqrt(0.5 / rbf_gamma);
    std::shared_ptr<Covariance<Dtype>> kernel =
        std::make_shared<RadialBiasFunction<Dtype, 2>>(kernel_setting);

    Aabb<Dtype, 2> map_boundary(Eigen::Vector2<Dtype>(-5.0, -5.0), Eigen::Vector2<Dtype>(5.0, 5.0));
    Eigen::Matrix2X<Dtype> hinged_points =
        GenerateGridPoints(hinged_grid_size, map_boundary).second;
    BayesianHilbertMap<Dtype, 2> bhm(bhm_setting, kernel, hinged_points, map_boundary, 0);

    TestIo<Dtype>(bhm, hinged_points, map_boundary);

    std::vector<Eigen::Vector3<Dtype>> trajectory = GenerateTrajectory<Dtype>(50, 1);

    const auto lidar_setting = std::make_shared<Lidar2D::Setting>();
    lidar_setting->max_angle = 135.0 / 180.0 * M_PI;   // 135 degrees
    lidar_setting->min_angle = -135.0 / 180.0 * M_PI;  // -135 degrees
    lidar_setting->num_lines = 135;
    const std::shared_ptr<Space2D> space = GenerateSpace();
    const Lidar2D lidar(lidar_setting, space);
    const Eigen::Matrix2Xd ray_dirs_frame = lidar.GetRayDirectionsInFrame();

    const auto [test_grid, test_points] = GenerateGridPoints(test_grid_size, map_boundary);
    const Eigen::Matrix2X<Dtype> surf_points = space->GetSurface()->vertices.cast<Dtype>();
    Eigen::VectorX<Dtype> predicted_prob_occupied(test_points.cols());
    Eigen::Matrix2X<Dtype> predicted_gradient(2, test_points.cols());
    Eigen::Matrix2X<Dtype> predicted_gradient_surf(2, surf_points.cols());

    std::vector<Eigen::Vector2<Dtype>> waypoints;
    std::vector<Eigen::Matrix2X<Dtype>> scanned_points;

    for (const auto &pose: trajectory) {
        auto scan = lidar.Scan(
            static_cast<double>(pose[2]),
            pose.template head<2>().template cast<double>(),
            true);
        const Eigen::Matrix2<Dtype> rotation = Eigen::Rotation2D<Dtype>(pose[2]).toRotationMatrix();
        Eigen::Matrix2X<Dtype> points(2, scan.size());
        for (long i = 0; i < scan.size(); ++i) {
            points.col(i) << scan[i] * (rotation * ray_dirs_frame.col(i).cast<Dtype>()) +
                                 pose.template head<2>();
        }
        scanned_points.push_back(points);
        waypoints.push_back(pose.template head<2>());

        long num_points = 0;
        Eigen::Matrix2X<Dtype> dataset_points;
        Eigen::VectorX<Dtype> dataset_labels;
        std::vector<long> hit_indices;

        bhm.Update(
            pose.template head<2>(),
            points,
            -1l,
            num_points,
            dataset_points,
            dataset_labels,
            hit_indices);

        // constexpr bool with_sigmoid = false;
        // constexpr bool parallel = true;
        // bhm.Predict(
        //     test_points,
        //     false /*logodd*/,
        //     faster,
        //     true /*compute_gradient*/,
        //     with_sigmoid,
        //     parallel,
        //     predicted_prob_occupied,
        //     predicted_gradient);
        // bhm.PredictGradient(surf_points, faster, with_sigmoid, parallel,
        // predicted_gradient_surf); VisualizeResult(
        //     test_grid,
        //     waypoints,
        //     scanned_points,
        //     predicted_prob_occupied,
        //     predicted_gradient,
        //     surf_points,
        //     predicted_gradient_surf);

        bhm.GenerateDataset(
            pose.template head<2>(),
            points,
            -1l,
            num_points,
            dataset_points,
            dataset_labels,
            hit_indices);

        if (num_points == 0) {
            ERL_WARN("No valid points generated for update. Skipping update.");
            continue;
        }

        bhm.PrepareExpectationMaximization(dataset_points, dataset_labels, num_points);
        for (int itr = 0; itr < bhm_setting->num_em_iterations; ++itr) {
            if (bhm_setting->use_sparse) {
                bhm.RunExpectationMaximizationIterationSparse(num_points);
            } else {
                bhm.RunExpectationMaximizationIteration(num_points);
            }
            // predict and visualize
            constexpr bool with_sigmoid = false;
            constexpr bool parallel = true;
            bhm.Predict(
                test_points,
                false /*logodd*/,
                faster,
                true /*compute_gradient*/,
                with_sigmoid,
                parallel,
                predicted_prob_occupied,
                predicted_gradient);
            bhm.PredictGradient(
                surf_points,
                faster,
                with_sigmoid,
                parallel,
                predicted_gradient_surf);
            VisualizeResult(
                test_grid,
                waypoints,
                scanned_points,
                predicted_prob_occupied,
                predicted_gradient,
                surf_points,
                predicted_gradient_surf);
        }
    }

    cv::waitKey(2000);

    TestIo<Dtype>(bhm, hinged_points, map_boundary);
}

struct Options {
    int hinged_grid_size = 31;
    int test_grid_size = 100;
    float rbf_gamma = 20;
    bool diagonal_sigma = true;
    bool faster = true;
    bool use_sparse = false;
};

Options g_options;

TEST(BayesianHilbertMap, 2Dd) {
    TestImpl2D<double>(
        g_options.hinged_grid_size,
        g_options.test_grid_size,
        g_options.rbf_gamma,
        g_options.diagonal_sigma,
        g_options.faster,
        g_options.use_sparse);
}

TEST(BayesianHilbertMap, 2Df) {
    TestImpl2D<float>(
        g_options.hinged_grid_size,
        g_options.test_grid_size,
        g_options.rbf_gamma,
        g_options.diagonal_sigma,
        g_options.faster,
        g_options.use_sparse);
}

int
main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);

    try {
        namespace po = boost::program_options;
        po::options_description desc;
        // clang-format off
        desc.add_options()
            ("help,h", "Show help message")
            ("hinged-grid-size", po::value<int>(&g_options.hinged_grid_size)->default_value(g_options.hinged_grid_size), "Size of the hinged grid")
            ("test-grid-size", po::value<int>(&g_options.test_grid_size)->default_value(g_options.test_grid_size), "Size of the test grid")
            ("rbf-gamma", po::value<float>(&g_options.rbf_gamma)->default_value(g_options.rbf_gamma), "RBF gamma value")
            ("diagonal-sigma", po::value<bool>(&g_options.diagonal_sigma)->default_value(g_options.diagonal_sigma), "Use diagonal sigma")
            ("faster", po::value<bool>(&g_options.faster)->default_value(g_options.faster), "Use faster prediction")
            ("use-sparse", po::value<bool>(&g_options.use_sparse)->default_value(g_options.use_sparse), "Use sparse matrix");
        // clang-format on
        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
        if (vm.count("help")) {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl << desc << std::endl;
            return 0;
        }
        po::notify(vm);
    } catch (const std::exception &e) {
        ERL_ERROR("Error parsing command line arguments: {}", e.what());
        return -1;
    }

    return RUN_ALL_TESTS();
}
