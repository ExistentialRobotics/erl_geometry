#include "erl_common/test_helper.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"

using Dtype = double;
using OccupancyQuadtree = erl::geometry::OccupancyQuadtree<Dtype>;
using OccupancyQuadtreeDrawer = erl::geometry::OccupancyQuadtreeDrawer<OccupancyQuadtree>;
using QuadtreeKey = erl::geometry::QuadtreeKey;
using Vector2 = Eigen::Vector2<Dtype>;
using Matrix2X = Eigen::Matrix2X<Dtype>;
using VectorX = Eigen::VectorX<Dtype>;

/// Build a tree with a circular scan: free interior, occupied ring, unknown exterior.
static std::shared_ptr<OccupancyQuadtree>
BuildCircleTree(const Dtype resolution = 0.04) {
    auto setting = std::make_shared<OccupancyQuadtree::Setting>();
    setting->resolution = static_cast<float>(resolution);
    auto tree = std::make_shared<OccupancyQuadtree>(setting);

    constexpr long n = 90;
    VectorX angles = VectorX::LinSpaced(n, -M_PI, M_PI);
    Matrix2X points(2, n);
    const Vector2 sensor_origin(0., 0.);

    for (long i = 0; i < n; ++i) {
        constexpr Dtype radius = 1.0;
        points.col(i) << std::cos(angles[i]) * radius, std::sin(angles[i]) * radius;
    }

    tree->InsertPointCloud(
        points,
        sensor_origin,
        /*min_range=*/0.0,
        /*max_range=*/-1.0,
        /*with_count=*/false,
        /*parallel=*/false,
        /*lazy_eval=*/false,
        /*discrete=*/false);

    return tree;
}

/// Compute the total edge length of a frontier polyline.
static Dtype
FrontierLength(const Matrix2X &frontier) {
    Dtype len = 0;
    for (Eigen::Index i = 1; i < frontier.cols(); ++i) {
        len += (frontier.col(i) - frontier.col(i - 1)).norm();
    }
    return len;
}

TEST(OccupancyQuadtree, ExtractFrontiers_EmptyTree) {
    auto setting = std::make_shared<OccupancyQuadtree::Setting>();
    setting->resolution = 0.1;
    const OccupancyQuadtree tree(setting);

    auto frontiers = tree.ExtractFrontiers();
    EXPECT_TRUE(frontiers.empty());
}

TEST(OccupancyQuadtree, ExtractFrontiers_AllOccupied) {
    auto setting = std::make_shared<OccupancyQuadtree::Setting>();
    setting->resolution = 0.5;
    auto tree = std::make_shared<OccupancyQuadtree>(setting);

    // Mark a dense block as occupied — no free cells, so no frontiers.
    for (double x = -2.0; x <= 2.0; x += 0.5) {
        for (double y = -2.0; y <= 2.0; y += 0.5) {
            tree->UpdateNode(x, y, /*occupied=*/true, /*lazy_eval=*/false);
        }
    }

    auto frontiers = tree->ExtractFrontiers();
    EXPECT_TRUE(frontiers.empty());
}

TEST(OccupancyQuadtree, ExtractFrontiers_SingleFreeCell) {
    auto setting = std::make_shared<OccupancyQuadtree::Setting>();
    setting->resolution = 1.0;
    auto tree = std::make_shared<OccupancyQuadtree>(setting);

    // A single free cell surrounded by unknown space produces frontier edges
    // on all 4 faces. The 4 edge segments share corners and should chain into
    // a single polyline with 5 vertices (a closed square).
    tree->UpdateNode(0.0, 0.0, /*occupied=*/false, /*lazy_eval=*/false);

    auto frontiers = tree->ExtractFrontiers();
    EXPECT_GT(frontiers.size(), 0);

    std::size_t total_vertices = 0;
    for (const auto &f: frontiers) { total_vertices += f.cols(); }
    // 4 edges around a cell: at least 4 unique corner vertices
    EXPECT_GE(total_vertices, 4);
}

TEST(OccupancyQuadtree, ExtractFrontiers_CircleHasFrontiers) {
    auto tree = BuildCircleTree(0.04);
    auto frontiers = tree->ExtractFrontiers();

    ASSERT_GT(frontiers.size(), 0);
    // The largest frontier (from the occupied ring boundary) should be substantial
    EXPECT_GT(frontiers[0].cols(), 2);
}

TEST(OccupancyQuadtree, ExtractFrontiers_Visualization) {
    GTEST_PREPARE_OUTPUT_DIR();
    auto tree = BuildCircleTree(0.04);
    auto frontiers = tree->ExtractFrontiers();
    ERL_INFO("Extracted {} frontiers", frontiers.size());
    ASSERT_GT(frontiers.size(), 0);

    // Draw the quadtree leaves as the background
    auto drawer_setting = std::make_shared<OccupancyQuadtreeDrawer::Setting>();
    drawer_setting->area_min = tree->GetMetricMin().template cast<float>();
    drawer_setting->area_max = tree->GetMetricMax().template cast<float>();
    drawer_setting->resolution = 0.001f;
    drawer_setting->padding = 50;
    drawer_setting->border_color = cv::Scalar(128, 128, 128, 255);
    drawer_setting->border_thickness = 0;
    const OccupancyQuadtreeDrawer drawer(drawer_setting, tree);
    cv::Mat img;
    drawer.DrawLeaves(img);

    // Draw each frontier polyline in a distinct color
    const cv::Scalar colors[] = {
        {255, 0, 0, 255},    // blue
        {0, 0, 255, 255},    // red
        {0, 200, 0, 255},    // green
        {255, 0, 255, 255},  // magenta
        {0, 200, 200, 255},  // yellow
        {255, 128, 0, 255},  // cyan
    };
    constexpr std::size_t num_colors = std::size(colors);

    for (std::size_t fi = 0; fi < frontiers.size(); ++fi) {
        const auto &frontier = frontiers[fi];
        Eigen::Matrix2Xi pixels = drawer.GetPixelCoordsForPositions<Dtype>(frontier, false);
        const cv::Scalar &color = colors[fi % num_colors];
        for (Eigen::Index i = 1; i < pixels.cols(); ++i) {
            cv::line(
                img,
                cv::Point(pixels(0, i - 1), pixels(1, i - 1)),
                cv::Point(pixels(0, i), pixels(1, i)),
                color,
                1);
        }
        // Mark vertices as small circles
        for (Eigen::Index i = 0; i < pixels.cols(); ++i) {
            cv::circle(img, cv::Point(pixels(0, i), pixels(1, i)), 3, color, cv::FILLED);
        }
    }

    cv::imwrite((test_output_dir / "frontiers.png").string(), img);
}

TEST(OccupancyQuadtree, ExtractFrontiers_SortedByLength) {
    auto tree = BuildCircleTree(0.04);
    auto frontiers = tree->ExtractFrontiers(/*min_num_vertices=*/1, /*sort_by_length=*/true);

    for (std::size_t i = 1; i < frontiers.size(); ++i) {
        EXPECT_GE(FrontierLength(frontiers[i - 1]), FrontierLength(frontiers[i]))
            << "Frontiers not sorted by length at index " << i;
    }
}

TEST(OccupancyQuadtree, ExtractFrontiers_UnsortedSameContent) {
    auto tree = BuildCircleTree(0.04);
    auto sorted = tree->ExtractFrontiers(/*min_num_vertices=*/1, /*sort_by_length=*/true);
    auto unsorted = tree->ExtractFrontiers(/*min_num_vertices=*/1, /*sort_by_length=*/false);

    // Both should return the same number of frontiers
    ASSERT_EQ(sorted.size(), unsorted.size());

    // Collect lengths from both, sort them, and verify they match
    std::vector<Dtype> sorted_lengths;
    std::vector<Dtype> unsorted_lengths;
    sorted_lengths.reserve(sorted.size());
    unsorted_lengths.reserve(unsorted.size());
    for (const auto &f: sorted) { sorted_lengths.push_back(FrontierLength(f)); }
    for (const auto &f: unsorted) { unsorted_lengths.push_back(FrontierLength(f)); }
    std::sort(sorted_lengths.begin(), sorted_lengths.end());
    std::sort(unsorted_lengths.begin(), unsorted_lengths.end());
    for (std::size_t i = 0; i < sorted_lengths.size(); ++i) {
        EXPECT_NEAR(sorted_lengths[i], unsorted_lengths[i], 1e-12)
            << "Frontier length mismatch at index " << i;
    }
}

TEST(OccupancyQuadtree, ExtractFrontiers_MinVerticesFiltering) {
    auto tree = BuildCircleTree(0.04);

    auto frontiers_all = tree->ExtractFrontiers(1);
    auto frontiers_large = tree->ExtractFrontiers(100);

    EXPECT_GE(frontiers_all.size(), frontiers_large.size());

    // Every frontier in the filtered set must meet the minimum vertex count
    for (const auto &f: frontiers_large) { EXPECT_GE(static_cast<std::size_t>(f.cols()), 100u); }
}

TEST(OccupancyQuadtree, ExtractFrontiers_InAabb) {
    GTEST_PREPARE_OUTPUT_DIR();
    auto tree = BuildCircleTree(0.04);

    // Extract frontiers only within the first quadrant
    constexpr Dtype aabb_min_x = 0.0;
    constexpr Dtype aabb_min_y = 0.0;
    constexpr Dtype aabb_max_x = 3.0;
    constexpr Dtype aabb_max_y = 3.0;
    auto frontiers = tree->ExtractFrontiers(aabb_min_x, aabb_min_y, aabb_max_x, aabb_max_y);
    auto frontiers_all = tree->ExtractFrontiers();

    // AABB-restricted extraction should return fewer or equal frontiers
    EXPECT_LE(frontiers.size(), frontiers_all.size());

    // Draw visualization
    auto drawer_setting = std::make_shared<OccupancyQuadtreeDrawer::Setting>();
    drawer_setting->area_min = tree->GetMetricMin().template cast<float>();
    drawer_setting->area_max = tree->GetMetricMax().template cast<float>();
    drawer_setting->resolution = 0.01f;
    drawer_setting->border_color = cv::Scalar(200, 200, 200);
    const OccupancyQuadtreeDrawer drawer(drawer_setting, tree);
    cv::Mat img;
    drawer.DrawLeaves(img);

    // Draw AABB rectangle
    Eigen::Matrix2X<Dtype> aabb_corners(2, 4);
    aabb_corners.col(0) << aabb_min_x, aabb_min_y;
    aabb_corners.col(1) << aabb_max_x, aabb_min_y;
    aabb_corners.col(2) << aabb_max_x, aabb_max_y;
    aabb_corners.col(3) << aabb_min_x, aabb_max_y;
    Eigen::Matrix2Xi aabb_px = drawer.GetPixelCoordsForPositions<Dtype>(aabb_corners, false);
    for (int i = 0; i < 4; ++i) {
        cv::line(
            img,
            cv::Point(aabb_px(0, i), aabb_px(1, i)),
            cv::Point(aabb_px(0, (i + 1) % 4), aabb_px(1, (i + 1) % 4)),
            cv::Scalar(0, 255, 255),
            2);
    }

    // Draw frontiers
    const cv::Scalar colors[] = {
        {255, 0, 0},
        {0, 0, 255},
        {0, 200, 0},
        {255, 0, 255},
        {0, 200, 200},
        {255, 128, 0},
    };
    constexpr std::size_t num_colors = std::size(colors);
    for (std::size_t fi = 0; fi < frontiers.size(); ++fi) {
        const auto &frontier = frontiers[fi];
        Eigen::Matrix2Xi pixels = drawer.GetPixelCoordsForPositions<Dtype>(frontier, false);
        const cv::Scalar &color = colors[fi % num_colors];
        for (Eigen::Index i = 1; i < pixels.cols(); ++i) {
            cv::line(
                img,
                cv::Point(pixels(0, i - 1), pixels(1, i - 1)),
                cv::Point(pixels(0, i), pixels(1, i)),
                color,
                2);
        }
        for (Eigen::Index i = 0; i < pixels.cols(); ++i) {
            cv::circle(img, cv::Point(pixels(0, i), pixels(1, i)), 3, color, cv::FILLED);
        }
    }

    cv::imwrite((test_output_dir / "frontiers_in_aabb.png").string(), img);
}
