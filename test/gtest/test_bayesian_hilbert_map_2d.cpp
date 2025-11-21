#include "erl_common/block_timer.hpp"
#include "erl_common/grid_map_info.hpp"
#include "erl_common/opencv.hpp"
#include "erl_common/serialization.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_covariance/radial_bias_function.hpp"
#include "erl_geometry/bayesian_hilbert_map.hpp"
#include "erl_geometry/lidar_2d.hpp"
#include "erl_geometry/space_2d.hpp"

using namespace erl::common;
using namespace erl::geometry;
using namespace erl::covariance;

struct Options : Yamlable<Options> {
    bool hold = true;
    int hinged_grid_size = 31;
    int max_dataset_size = 2000;
    int test_grid_size = 100;
    float kernel_scale = 0.02f;
    bool faster_prediction = true;
    std::vector<float> iso_values = {-20.f, 0.f, 20.f};
    float iso_value = 0.0f;
    std::shared_ptr<BayesianHilbertMapSetting> bhm = std::make_shared<BayesianHilbertMapSetting>();

    ERL_REFLECT_SCHEMA(
        Options,
        ERL_REFLECT_MEMBER(Options, hold),
        ERL_REFLECT_MEMBER(Options, hinged_grid_size),
        ERL_REFLECT_MEMBER(Options, max_dataset_size),
        ERL_REFLECT_MEMBER(Options, test_grid_size),
        ERL_REFLECT_MEMBER(Options, kernel_scale),
        ERL_REFLECT_MEMBER(Options, faster_prediction),
        ERL_REFLECT_MEMBER(Options, iso_values),
        ERL_REFLECT_MEMBER(Options, iso_value),
        ERL_REFLECT_MEMBER(Options, bhm));
};

Options g_options;

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
std::tuple<cv::Mat, cv::Mat, cv::Mat>
VisualizeResult(
    const BayesianHilbertMap<Dtype, 2> &bhm,
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
    std::cout << "Prob occupied min: " << eigen_img.minCoeff() << ", max: " << eigen_img.maxCoeff()
              << std::endl;
    cv::Mat img_prob_occupied, img_prob_occupied_rgb;
    cv::eigen2cv(eigen_img, img_prob_occupied);
    cv::normalize(img_prob_occupied, img_prob_occupied, 0, 255, cv::NORM_MINMAX);
    img_prob_occupied.convertTo(img_prob_occupied, CV_8UC1);
    cv::applyColorMap(img_prob_occupied, img_prob_occupied_rgb, cv::COLORMAP_JET);

    // visualize contours
    for (float iso_value: g_options.iso_values) {
        cv::Mat img_contour;
        cv::eigen2cv(eigen_img, img_contour);
        img_contour = img_contour > iso_value;

        cv::imshow(fmt::format("img_contour: {:.3f}", iso_value), img_contour);

        cv::Scalar color;
        if (iso_value > 0.f) {
            color = cv::Scalar(0, 0, 255);
        } else if (iso_value < 0.f) {
            color = cv::Scalar(0, 255, 0);
        } else {
            color = cv::Scalar(255, 255, 255);
        }

        // cv::waitKey(0);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(img_contour, contours, cv::RETR_LIST, cv::CHAIN_APPROX_TC89_KCOS);
        cv::polylines(img_prob_occupied_rgb, contours, false, color, 1, cv::LINE_AA, 0);
        // cv::polylines(img_gradient_norm_rgb, contours, false, cv::Scalar(255, 255, 255), 1,
        // cv::LINE_AA, 0);
    }

    Eigen::VectorX<Dtype> gradient_norm = gradient_grid.colwise().norm();
    eigen_img = Eigen::Map<MatrixX>(gradient_norm.data(), img_size, img_size);
    cv::Mat img_gradient_norm, img_gradient_norm_rgb;
    cv::eigen2cv(eigen_img, img_gradient_norm);
    cv::normalize(img_gradient_norm, img_gradient_norm, 0, 255, cv::NORM_MINMAX);
    img_gradient_norm.convertTo(img_gradient_norm, CV_8UC1);
    cv::applyColorMap(img_gradient_norm, img_gradient_norm_rgb, cv::COLORMAP_JET);

    cv::flip(img_prob_occupied_rgb, img_prob_occupied_rgb, 0);
    cv::flip(img_gradient_norm_rgb, img_gradient_norm_rgb, 0);

    const int s = 800 / img_prob_occupied_rgb.rows;
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
    cv::polylines(img_prob_occupied_rgb, pts, false, cv::Scalar(0, 255, 0), 1, cv::LINE_AA, 0);

    // visualize gradient
    for (long i = 0; i < surface_points.cols(); ++i) {
        Eigen::Vector2i px = grid_map_scaled.MeterToPixelForPoints(surface_points.col(i));
        const cv::Point pt1(px[0], px[1]);
        Eigen::Vector2<Dtype> gradient = gradient_surf.col(i);
        gradient.normalize();
        px += grid_map_scaled.MeterToPixelForVectors(-gradient * 0.125);
        const cv::Point pt2(px[0], px[1]);
        // cv::arrowedLine(img_prob_occupied_rgb, pt1, pt2, cv::Scalar(255, 255, 255), 2);
        cv::arrowedLine(img_gradient_norm_rgb, pt1, pt2, cv::Scalar(255, 255, 255), 2);
        cv::circle(img_gradient_norm_rgb, pt1, 1, cv::Scalar(0, 0, 255), -1);
    }
    cv::polylines(img_gradient_norm_rgb, pts, false, cv::Scalar(0, 255, 0), 1, cv::LINE_AA, 0);

    // visualize bhm weights
    Eigen::VectorX<Dtype> mu = bhm.GetWeights();
    // Apply x^0.25 to the weights for visualization
    mu = mu.unaryExpr([](Dtype v) { return v > 0 ? std::pow(v, 0.25f) : -std::pow(-v, 0.25f); });
    cv::Mat mu_color;
    cv::eigen2cv(mu, mu_color);
    cv::normalize(mu_color, mu_color, 0, 255, cv::NORM_MINMAX);
    mu_color.convertTo(mu_color, CV_8UC1);
    cv::applyColorMap(mu_color, mu_color, cv::COLORMAP_JET);
    const Eigen::Matrix2X<Dtype> &hinged_points = bhm.GetHingedPoints();
    cv::Mat img_bhm_weights(img_prob_occupied_rgb.rows, img_prob_occupied_rgb.cols, CV_8UC3);
    img_bhm_weights.setTo(0);
    for (long i = 0; i < hinged_points.cols(); ++i) {
        Eigen::Vector2i px = grid_map_scaled.MeterToPixelForPoints(hinged_points.col(i));
        cv::circle(
            img_bhm_weights,
            cv::Point(px[0], px[1]),
            8,
            mu_color.at<cv::Vec3b>(static_cast<int>(i), 0),
            -1);
    }
    cv::polylines(img_bhm_weights, pts, false, cv::Scalar(0, 255, 0), 1, cv::LINE_AA, 0);

    cv::imshow("Probability Occupied", img_prob_occupied_rgb);
    cv::imshow("Gradient Norm", img_gradient_norm_rgb);
    cv::imshow("BHM Weights", img_bhm_weights);

    static bool windows_created = false;
    if (!windows_created) {
        cv::moveWindow("Gradient Norm", img_prob_occupied_rgb.cols, 0);
        cv::moveWindow("BHM Weights", 2 * img_prob_occupied_rgb.cols, 0);
        windows_created = true;
    }

    cv::waitKey(10);

    return {img_prob_occupied_rgb, img_gradient_norm_rgb, img_bhm_weights};
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
    using Serializer = erl::common::serialization::Serialization<BayesianHilbertMap<Dtype, 2>>;
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
TestImpl2D() {

    GTEST_PREPARE_OUTPUT_DIR();

    auto kernel_setting = std::make_shared<typename Covariance<Dtype>::Setting>();
    kernel_setting->scale = g_options.kernel_scale;
    std::shared_ptr<Covariance<Dtype>> kernel =
        std::make_shared<RadialBiasFunction<Dtype, 2>>(kernel_setting);

    Aabb<Dtype, 2> map_boundary(Eigen::Vector2<Dtype>(-3.0, -3.0), Eigen::Vector2<Dtype>(3.0, 3.0));
    Eigen::Matrix2X<Dtype> hinged_points =
        GenerateGridPoints(g_options.hinged_grid_size, map_boundary).second;
    BayesianHilbertMap<Dtype, 2> bhm(g_options.bhm, kernel, hinged_points, map_boundary, 0);

    TestIo<Dtype>(bhm, hinged_points, map_boundary);

    std::vector<Eigen::Vector3<Dtype>> trajectory = GenerateTrajectory<Dtype>(50, 2);

    const auto lidar_setting = std::make_shared<Lidar2D::Setting>();
    lidar_setting->max_angle = 135.0 / 180.0 * M_PI;   // 135 degrees
    lidar_setting->min_angle = -135.0 / 180.0 * M_PI;  // -135 degrees
    lidar_setting->num_lines = 135;
    const std::shared_ptr<Space2D> space = GenerateSpace();
    const Lidar2D lidar(lidar_setting, space);
    const Eigen::Matrix2Xd ray_dirs_frame = lidar.GetRayDirectionsInFrame();

    auto [test_grid, test_points] = GenerateGridPoints(g_options.test_grid_size, map_boundary);
    const Eigen::Matrix2X<Dtype> surf_points = space->GetSurface()->vertices.cast<Dtype>();
    Eigen::VectorX<Dtype> predicted_prob_occupied(test_points.cols());
    Eigen::Matrix2X<Dtype> predicted_gradient(2, test_points.cols());
    Eigen::Matrix2X<Dtype> predicted_gradient_surf(2, surf_points.cols());

    auto sdf = space->ComputeSdf(
        test_points.template cast<double>(),
        Space2D::SignMethod::kLineNormal,
        true,
        true);

    std::vector<Eigen::Vector2<Dtype>> waypoints;
    std::vector<Eigen::Matrix2X<Dtype>> scanned_points;
    long cnt = 0;
    std::filesystem::path prob_occupied_dir = test_output_dir / "prob_occupied";
    std::filesystem::create_directories(prob_occupied_dir);
    std::filesystem::path gradient_norms_dir = test_output_dir / "gradient_norms";
    std::filesystem::create_directories(gradient_norms_dir);
    std::filesystem::path bhm_weights_dir = test_output_dir / "bhm_weights";
    std::filesystem::create_directories(bhm_weights_dir);
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

        {
            ERL_BLOCK_TIMER_MSG("dataset generation");
            std::vector<long> point_indices;
            std::vector<long> hit_indices;
            bhm.GenerateDataset(
                pose.template head<2>(),
                points,
                point_indices,
                0,
                g_options.max_dataset_size,
                num_points,
                dataset_points,
                dataset_labels,
                hit_indices);
        }

        if (num_points == 0) {
            ERL_WARN("No valid points generated for update. Skipping update.");
            continue;
        }

        bhm.PrepareExpectationMaximization(dataset_points, dataset_labels, num_points);
        for (int itr = 0; itr < g_options.bhm->num_em_iterations; ++itr) {
            {
                ERL_BLOCK_TIMER_MSG("bhm");
                if (g_options.bhm->use_sparse) {
                    bhm.RunExpectationMaximizationIterationSparse(num_points);
                } else {
                    bhm.RunExpectationMaximizationIteration(num_points);
                }
            }
            // predict and visualize
            constexpr bool gradient_with_sigmoid = false;
            constexpr bool parallel = true;
            bhm.Predict(
                test_points,
                true /*logodd*/,
                g_options.faster_prediction,
                true /*compute_gradient*/,
                gradient_with_sigmoid,
                parallel,
                predicted_prob_occupied,
                predicted_gradient);
            bhm.PredictGradient(
                surf_points,
                g_options.faster_prediction,
                gradient_with_sigmoid,
                parallel,
                predicted_gradient_surf);
            auto [img_prob_occupied_rgb, img_gradient_norm_rgb, img_bhm_weights] = VisualizeResult(
                bhm,
                test_grid,
                waypoints,
                scanned_points,
                predicted_prob_occupied,
                predicted_gradient,
                surf_points,
                predicted_gradient_surf);

            long n_correct = 0;
            for (long i = 0; i < predicted_prob_occupied.size(); ++i) {
                if ((sdf[i] < 0) == (predicted_prob_occupied[i] >= g_options.iso_value)) {
                    n_correct++;
                }
            }
            const Dtype accuracy =
                static_cast<Dtype>(n_correct) / static_cast<Dtype>(predicted_prob_occupied.size());
            ERL_INFO("EM Iteration {}: Prediction accuracy: {:.2f}%", itr, accuracy * 100.0f);

            std::string filename = fmt::format("{:04d}.png", cnt++);
            cv::imwrite(prob_occupied_dir / filename, img_prob_occupied_rgb);
            cv::imwrite(gradient_norms_dir / filename, img_gradient_norm_rgb);
            cv::imwrite(bhm_weights_dir / filename, img_bhm_weights);
        }
    }

    TestIo<Dtype>(bhm, hinged_points, map_boundary);

    if (g_options.hold) {
        cv::waitKey();
    } else {
        cv::waitKey(2000);
    }
}

TEST(BayesianHilbertMap, 2Dd) { TestImpl2D<double>(); }

TEST(BayesianHilbertMap, 2Df) { TestImpl2D<float>(); }

int
main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    g_options.FromCommandLine(argc, argv);
    std::cout << "Options:\n" << g_options << std::endl;
    return RUN_ALL_TESTS();
}
