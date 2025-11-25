#include "erl_common/test_helper.hpp"
#include "erl_geometry/hidden_point_removal.hpp"
#include "erl_geometry/lidar_2d.hpp"
#include "erl_geometry/space_2d.hpp"

TEST(HiddenPointRemoval2D, Basic) {
    GTEST_PREPARE_OUTPUT_DIR();

    using namespace erl::common;
    using namespace erl::geometry;

    // create a star-shaped room in 2D
    Eigen::Matrix2Xd layout_vertices(2, 10);
    Eigen::Matrix2Xd layout_normals(2, 10);
    constexpr double radius1 = 3.5;
    constexpr double radius2 = 1.5;
    for (int i = 0; i < 10; ++i) {
        const double angle = i * 2.0 * M_PI / 10.0;
        const double radius = (i % 2 == 0) ? radius1 : radius2;
        const double nx = std::cos(angle);
        const double ny = std::sin(angle);
        layout_vertices.col(i) = Eigen::Vector2d(radius * nx, radius * ny);
        layout_normals.col(i) = Eigen::Vector2d(-nx, -ny);
    }
    std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> ordered_object_vertices{layout_vertices};
    std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> ordered_object_normals{layout_normals};
    auto room = std::make_shared<Space2D>(ordered_object_vertices, ordered_object_normals);

    // create a lidar
    auto lidar_setting = std::make_shared<Lidar2D::Setting>();
    lidar_setting->num_lines = 270;
    lidar_setting->min_angle = -M_PI * 3.0 / 4.0;
    lidar_setting->max_angle = M_PI * 3.0 / 4.0;
    Lidar2D lidar(lidar_setting, room);

    // generate scan points along a circle
    constexpr double radius3 = 1.0;
    constexpr int n_wp = 30;
    Eigen::Matrix2Xd scan_points(2, lidar_setting->num_lines * n_wp);
    Eigen::Matrix2Xd ray_dirs = lidar.GetRayDirectionsInFrame();

    for (int i = 0, j = 0; i < n_wp; ++i) {
        const double angle = i * 2.0 * M_PI / n_wp;
        const double x = radius3 * std::cos(angle);
        const double y = radius3 * std::sin(angle);
        const double rotation_angle = angle + M_PI_2;
        Eigen::Matrix2d rotation = Eigen::Rotation2Dd(rotation_angle).toRotationMatrix();
        const Eigen::Vector2d translation(x, y);
        Eigen::VectorXd scan = lidar.Scan(rotation, translation, true);
        for (int k = 0; k < lidar_setting->num_lines; ++k) {
            scan_points.col(j++) = translation + scan[k] * (rotation * ray_dirs.col(k));
        }
    }
    ERL_INFO("{} scan points generated.", scan_points.cols());

    // run hidden point removal
    constexpr double scale = 4.0;
    std::vector<long> visible_point_indices;
    Eigen::Vector2d view_position(2.5, 0.0);
    HiddenPointRemoval<double, 2>(
        scan_points,
        view_position,
        400 * scale,
        visible_point_indices,
        false,
        true);
    ERL_INFO("{} visible points found.", visible_point_indices.size());

    // visualization
    constexpr int nx = 801;
    constexpr int ny = 801;
    GridMapInfo2Dd grid_map_info(
        Eigen::Vector2i(nx, ny),
        Eigen::Vector2d(-scale, -scale),
        Eigen::Vector2d(scale, scale));
    Eigen::MatrixX8U map_img_eigen = room->GenerateMapImage(grid_map_info, true);
    cv::Mat map_img;
    cv::eigen2cv(map_img_eigen, map_img);
    cv::cvtColor(map_img, map_img, cv::COLOR_GRAY2BGR);
    cv::Mat img_copy = map_img.clone();
    Eigen::Vector2i uv_trans_t0 = grid_map_info.MeterToPixelForPoint(Eigen::Vector2d(radius3, 0.0));
    // draw scan points
    for (int i = 0; i < scan_points.cols(); ++i) {
        const Eigen::Vector2i uv = grid_map_info.MeterToPixelForPoint(scan_points.col(i));
        cv::circle(map_img, cv::Point(uv.x(), uv.y()), 4, cv::Scalar(0, 0, 255), -1);
        if (i < lidar_setting->num_lines) {
            // draw the rays for the first scan
            cv::line(
                img_copy,
                cv::Point(uv_trans_t0.x(), uv_trans_t0.y()),
                cv::Point(uv.x(), uv.y()),
                cv::Scalar(0, 128, 255),
                1);
            cv::circle(img_copy, cv::Point(uv.x(), uv.y()), 4, cv::Scalar(0, 0, 255), -1);
        }
    }
    // draw visible points
    const Eigen::Vector2i uv_view_pos = grid_map_info.MeterToPixelForPoint(view_position);
    for (const auto &idx: visible_point_indices) {
        const Eigen::Vector2i uv = grid_map_info.MeterToPixelForPoint(scan_points.col(idx));
        cv::line(
            map_img,
            cv::Point(uv_view_pos.x(), uv_view_pos.y()),
            cv::Point(uv.x(), uv.y()),
            cv::Scalar(255, 128, 0),
            1);
        cv::circle(map_img, cv::Point(uv.x(), uv.y()), 4, cv::Scalar(0, 255, 0), -1);
    }
    // draw view position
    cv::circle(map_img, cv::Point(uv_view_pos.x(), uv_view_pos.y()), 8, cv::Scalar(255, 0, 0), -1);
    cv::imwrite(test_output_dir / "hidden_point_removal_2d.png", map_img);
    cv::imwrite(test_output_dir / "hidden_point_removal_2d_with_scan.png", img_copy);
    cv::imshow("map", map_img);
    cv::imshow("map with scan", img_copy);
    cv::waitKey(0);
}