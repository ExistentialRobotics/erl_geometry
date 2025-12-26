#include "erl_common/test_helper.hpp"
#include "erl_geometry/convex_hull.hpp"
#include "erl_geometry/gazebo_room_2d.hpp"

#include <open3d/io/TriangleMeshIO.h>
#include <open3d/visualization/utility/DrawGeometry.h>

TEST(GazeboRoom2D, ExtrudeTo3D) {
    GTEST_PREPARE_OUTPUT_DIR();
    using namespace erl::geometry;

    // Extrude to 3D with a height of 3.0 and add ceiling
    auto room_mesh = GazeboRoom2D::ExtrudeTo3D(3.0, false);

    // Check if the mesh is not null
    ASSERT_NE(room_mesh, nullptr);

    // Check if the mesh has vertices and triangles
    ASSERT_GT(room_mesh->vertices_.size(), 0);
    ASSERT_GT(room_mesh->triangles_.size(), 0);

    open3d::visualization::DrawGeometries({room_mesh});
    open3d::io::WriteTriangleMesh(
        test_output_dir / "gazebo_room_3d_no_ceiling.ply",
        *room_mesh,
        true);

    room_mesh = GazeboRoom2D::ExtrudeTo3D(3.0, true);
    open3d::visualization::DrawGeometries({room_mesh});
    open3d::io::WriteTriangleMesh(
        test_output_dir / "gazebo_room_3d_with_ceiling.ply",
        *room_mesh,
        true);
}

TEST(GazeboRoom2D, ExtractCorners) {
    erl::geometry::GazeboRoom2D::TrainDataLoader gazebo_rom_2d(
        ERL_GEOMETRY_ROOT_DIR "/data/gazebo");
    Eigen::Matrix2Xd points(2, 270 * gazebo_rom_2d.size());
    Eigen::Matrix2Xd frame_positions(2, gazebo_rom_2d.size());
    double min_range = std::numeric_limits<double>::max();
    double max_range = std::numeric_limits<double>::lowest();
    for (long frame_idx = 0; frame_idx < gazebo_rom_2d.size(); ++frame_idx) {
        const auto &frame = gazebo_rom_2d[frame_idx];
        const long base = frame_idx * 270;
        for (long i = 0; i < frame.angles.size(); ++i) {
            auto p = points.col(base + i);
            p[0] = frame.ranges[i] * std::cos(frame.angles[i]);
            p[1] = frame.ranges[i] * std::sin(frame.angles[i]);
            min_range = std::min(min_range, frame.ranges[i]);
            max_range = std::max(max_range, frame.ranges[i]);
        }
        auto frame_points = points.middleCols(base, 270);
        frame_points << (frame.rotation * frame_points).colwise() + frame.translation;
        frame_positions.col(frame_idx) = frame.translation;
    }
    const Eigen::Vector2d min = points.rowwise().minCoeff();
    const Eigen::Vector2d max = points.rowwise().maxCoeff();
    constexpr double resolution = 0.03;
    const erl::common::GridMapInfo2Dd grid_map_info(
        min,
        max,
        Eigen::Vector2d::Constant(resolution),
        Eigen::Vector2i::Constant(30));
    ERL_INFO("Number of frames: {}", gazebo_rom_2d.size());
    ERL_INFO("Range, Min: {:.2f}, Max: {:.2f}", min_range, max_range);
    ERL_INFO("Points, Min: [{}], Max: [{}]", min.transpose(), max.transpose());

    cv::Mat image(grid_map_info.Height(), grid_map_info.Width(), CV_8UC1);
    image.setTo(cv::Scalar(0));
    for (long i = 0; i < frame_positions.cols(); ++i) {
        auto frame_pos = frame_positions.col(i);
        const long base = i * 270;
        for (long j = 0; j < 270; ++j) {
            const Eigen::Matrix2Xi grids =
                grid_map_info.RayCasting(frame_pos, points.col(base + j));
            const Eigen::Matrix2Xi pixels = grid_map_info.GridToPixelForPoints(grids);
            for (long k = 0; k < pixels.cols(); ++k) {
                const auto p = pixels.col(k);
                image.at<uint8_t>(p.y(), p.x()) = 255;
            }
        }
    }

    constexpr int n_iters = 2;
    cv::erode(
        image,
        image,
        cv::getStructuringElement(cv::MORPH_ERODE, cv::Size(3, 3)),
        cv::Point(-1, -1),
        n_iters);
    cv::dilate(
        image,
        image,
        cv::getStructuringElement(cv::MORPH_DILATE, cv::Size(3, 3)),
        cv::Point(-1, -1),
        n_iters);
    // cv::imshow("gazebo_room_2d_corners", image);
    // cv::waitKey();

    // extract corners using OpenCV's corner detection
    constexpr int block_size = 11;
    constexpr int aperture_size = 3;
    constexpr double k = 0.05;
    cv::Mat dst;
    cv::Mat dst_norm;
    cv::Mat dst_norm_scaled;
    cv::cornerHarris(image, dst, block_size, aperture_size, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);
    dst_norm_scaled.convertTo(dst_norm_scaled, CV_8UC1);

    // non-maximum suppression of dst_norm, and collect corner points
    std::vector<cv::Point2f> corner_points;
    corner_points.reserve(18);
    for (int i = 0; i < dst_norm.rows; i++) {
        for (int j = 0; j < dst_norm.cols; j++) {
            constexpr int nms_size = 3;
            const float val = dst_norm.at<float>(i, j);
            bool is_max = true;
            for (int m = -nms_size / 2; m <= nms_size / 2; m++) {
                for (int n = -nms_size / 2; n <= nms_size / 2; n++) {
                    const int y = i + m;
                    const int x = j + n;
                    if (y < 0 || y >= dst_norm.rows || x < 0 || x >= dst_norm.cols) { continue; }
                    if (dst_norm.at<float>(y, x) > val) {
                        is_max = false;
                        break;
                    }
                }
                if (!is_max) { break; }
            }
            if (is_max) {
                constexpr int corner_threshold = 140;
                if (static_cast<int>(val) > corner_threshold) { corner_points.emplace_back(j, i); }
            }
        }
    }

    // refine corner locations using cv::cornerSubPix
    if (!corner_points.empty()) {
        cv::cornerSubPix(
            image,
            corner_points,
            cv::Size(7, 7),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 20, 0.01));
    }

    // draw corners on the image
    cv::Mat image_with_corners;
    image.convertTo(image_with_corners, CV_8UC3);
    cv::cvtColor(image_with_corners, image_with_corners, cv::COLOR_GRAY2BGR);
    // compute corner positions in world coordinates
    std::vector<Eigen::Vector2d> corner_positions;
    corner_positions.reserve(corner_points.size());
    for (const cv::Point2f &pt: corner_points) {
        corner_positions.emplace_back(
            grid_map_info.SubPixelToMeterForPoint(Eigen::Vector2d(pt.x, pt.y)));
        cv::circle(
            image_with_corners,
            cv::Point(static_cast<int>(std::round(pt.x)), static_cast<int>(std::round(pt.y))),
            5,
            cv::Scalar(0, 0, 255),
            2);
        cv::putText(
            image_with_corners,
            fmt::format("{}", corner_positions.size()),
            cv::Point(
                static_cast<int>(std::round(pt.x)) + 5,
                static_cast<int>(std::round(pt.y)) - 5),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 0),
            2);
    }
    // print corner positions
    ERL_INFO("Number of corners detected: {}", corner_positions.size());
    for (std::size_t i = 0; i < corner_positions.size(); ++i) {
        const auto &pos = corner_positions[i];
        ERL_INFO("Corner {:02d} at ({:.2f}, {:.2f})", i + 1, pos.x(), pos.y());
    }

    cv::imshow("gazebo_room_2d_corners", image_with_corners);
    cv::Mat dst_norm_scaled_color;
    cv::applyColorMap(dst_norm_scaled, dst_norm_scaled_color, cv::COLORMAP_JET);
    cv::imshow("Harris Corners Response", dst_norm_scaled_color);
    cv::waitKey();
}

TEST(GazeboRoom2D, ComputeOrientedBoundingBox) {
    using namespace erl::geometry;

    Eigen::Matrix2Xd hull_points(2, 4);
    for (long i = 0; i < 4; ++i) {
        constexpr std::array<long, 4> indices = {0, 3, 16, 17};
        const long idx = indices.at(i);
        hull_points.col(i) = GazeboRoom2D::kWallCorners[idx];
    }

    const Eigen::Vector2d box_center = hull_points.rowwise().mean();
    hull_points.colwise() -= box_center;
    const Eigen::JacobiSVD<Eigen::Matrix2d> svd(
        hull_points * hull_points.transpose(),
        Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Eigen::Matrix2d rotation = svd.matrixU();
    const Eigen::Matrix2Xd rotated_points = rotation.transpose() * hull_points;
    const Eigen::Vector2d min = rotated_points.rowwise().minCoeff().array() - 0.1;  // add margin
    const Eigen::Vector2d max = rotated_points.rowwise().maxCoeff().array() + 0.1;  // add margin
    const Eigen::Vector2d box_size = max - min;

    std::cout << "Oriented Bounding Box:\nCenter: [" << box_center.transpose() << "]\nRotation:\n"
              << rotation << "\nSize: [" << box_size.transpose() << "]\n"
              << std::flush;

    const erl::common::GridMapInfo2Dd grid_map_info(
        min,
        max,
        Eigen::Vector2d::Constant(0.01),
        Eigen::Vector2i::Constant(10));
    cv::Mat image(grid_map_info.Height(), grid_map_info.Width(), CV_8UC3);
    image.setTo(cv::Scalar(0));

    std::vector<cv::Point2i> contour_points;
    // draw the scene
    contour_points.reserve(14);
    for (long i = 0; i < 14; ++i) {
        const long j = GazeboRoom2D::kWallSegments[i][0];
        const Eigen::Vector2d p =
            rotation.transpose() * (GazeboRoom2D::kWallCorners[j] - box_center);
        const Eigen::Vector2i pixel = grid_map_info.MeterToPixelForPoint(p);
        contour_points.emplace_back(pixel.x(), pixel.y());
    }
    cv::drawContours(
        image,
        std::vector<std::vector<cv::Point2i>>{contour_points},
        0,
        cv::Scalar(255, 255, 255),
        cv::FILLED);
    contour_points.clear();
    for (long i = 14; i < 18; ++i) {
        const long j = GazeboRoom2D::kWallSegments[i][0];
        const Eigen::Vector2d p =
            rotation.transpose() * (GazeboRoom2D::kWallCorners[j] - box_center);
        const Eigen::Vector2i pixel = grid_map_info.MeterToPixelForPoint(p);
        contour_points.emplace_back(pixel.x(), pixel.y());
    }
    cv::drawContours(
        image,
        std::vector<std::vector<cv::Point2i>>{contour_points},
        0,
        cv::Scalar(0, 0, 0),
        cv::FILLED);

    // draw the rotated points
    for (long i = 0; i < rotated_points.cols(); ++i) {
        const Eigen::Vector2d p = rotated_points.col(i);
        const Eigen::Vector2i pixel = grid_map_info.MeterToPixelForPoint(p);
        cv::circle(image, cv::Point2i(pixel.x(), pixel.y()), 8, cv::Scalar(0, 0, 255), -1);
    }

    // draw the oriented bounding box
    Eigen::Vector2i pt1 = grid_map_info.MeterToPixelForPoint(min);
    Eigen::Vector2i pt2 = grid_map_info.MeterToPixelForPoint(max);
    cv::rectangle(
        image,
        cv::Point2i(pt1.x(), pt1.y()),
        cv::Point2i(pt2.x(), pt2.y()),
        cv::Scalar(0, 255, 0),
        2);

    cv::imshow("oriented_bounding_box", image);
    cv::waitKey();
}

TEST(GazeboRoom2D, ComputeSdf) {
    using namespace erl::geometry;

    // Compute SDF over the entire map
    erl::common::GridMapInfo2Dd grid_map_info(
        GazeboRoom2D::kMapMin,
        GazeboRoom2D::kMapMax,
        Eigen::Vector2d::Constant(0.02),
        Eigen::Vector2i::Constant(10));
    Eigen::VectorXd sdf = GazeboRoom2D::ComputeSdf(grid_map_info.GenerateMeterCoordinates(false));
    ERL_INFO("SDF stats: Min: {:.3f}, Max: {:.3f}", sdf.minCoeff(), sdf.maxCoeff());

    cv::Mat image(grid_map_info.Height(), grid_map_info.Width(), CV_64FC1, sdf.data());
    cv::flip(image, image, 0);
    cv::normalize(image, image, 0, 255, cv::NORM_MINMAX);
    image.convertTo(image, CV_8UC1);
    cv::applyColorMap(image, image, cv::COLORMAP_JET);
    cv::imshow("gazebo_room_2d_sdf", image);
    cv::waitKey();

    // Compute SDF over the oriented bounding box
    grid_map_info = erl::common::GridMapInfo2Dd(
        -GazeboRoom2D::kOrientedBoundingBoxSize / 2,
        GazeboRoom2D::kOrientedBoundingBoxSize / 2,
        Eigen::Vector2d::Constant(0.02),
        Eigen::Vector2i::Constant(10));
    Eigen::Isometry2d pose;
    pose.linear() = GazeboRoom2D::kOrientedBoundingBoxRotation;
    pose.translation() = GazeboRoom2D::kOrientedBoundingBoxCenter;
    const Eigen::Matrix2Xd positions = pose * grid_map_info.GenerateMeterCoordinates(false);
    sdf = GazeboRoom2D::ComputeSdf(positions);
    ERL_INFO("SDF stats (OBB): Min: {:.3f}, Max: {:.3f}", sdf.minCoeff(), sdf.maxCoeff());

    image = cv::Mat(grid_map_info.Height(), grid_map_info.Width(), CV_64FC1, sdf.data());
    cv::flip(image, image, 0);
    cv::normalize(image, image, 0, 255, cv::NORM_MINMAX);
    image.convertTo(image, CV_8UC1);
    cv::applyColorMap(image, image, cv::COLORMAP_JET);
    cv::imshow("gazebo_room_2d_sdf_obb", image);
    cv::waitKey();
}
