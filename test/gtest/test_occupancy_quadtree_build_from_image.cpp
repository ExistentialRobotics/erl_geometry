#include "erl_common/test_helper.hpp"
#include "erl_geometry/city_street_map.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"

TEST(OccupancyQuadtree, BuildFromImage) {
    GTEST_PREPARE_OUTPUT_DIR();

    const cv::Mat map = erl::geometry::CityStreetMap::LoadMap(gtest_src_dir / "Berlin_0_1024.map");
    std::shared_ptr<erl::common::GridMapInfo2D> map_info = std::make_shared<erl::common::GridMapInfo2D>(
        /*map_shape*/ Eigen::Vector2i(1024, 1024),
        /*map_min*/ Eigen::Vector2d(-10.0, -10.0),
        /*map_max*/ Eigen::Vector2d(92.4, 92.4));
    const auto quadtree = std::make_shared<erl::geometry::OccupancyQuadtree>(map_info, map, /* occupied_threshold */ 0, /* padding */ 1);
    const auto drawer_setting = std::make_shared<erl::geometry::OccupancyQuadtree::Drawer::Setting>();
    drawer_setting->resolution = 0.075;
    drawer_setting->area_min = map_info->Min().array() - 0.25;
    drawer_setting->area_max = map_info->Max().array() + 0.25;
    drawer_setting->border_color = cv::Scalar(255, 0, 0);
    const auto drawer = std::make_shared<erl::geometry::OccupancyQuadtree::Drawer>(drawer_setting, quadtree);
    cv::Mat img;
    drawer->DrawTree(img);

    auto mouse_callback = [](const int event, const int mouse_x, const int mouse_y, const int flags, void *userdata) {
        (void) flags;
        static bool mouse_fixed = false;

        if (event == cv::EVENT_LBUTTONDOWN) {
            std::cout << "Left button of the mouse is clicked - position (" << mouse_x << ", " << mouse_y << ")" << std::endl;
            mouse_fixed = !mouse_fixed;
            return;
        }

        if (event == cv::EVENT_MOUSEMOVE) {
            if (mouse_fixed) { return; }

            auto &[window_name, img, quadtree, drawer] = *static_cast<
                std::tuple<std::string, cv::Mat, std::shared_ptr<erl::geometry::OccupancyQuadtree>, std::shared_ptr<erl::geometry::OccupancyQuadtree::Drawer>>
                    *>(userdata);
            const auto grid_map_info = drawer->GetGridMapInfo();
            cv::Mat img_new = img.clone();

            // drawer coordinates: origin is at the bottom left corner, x is right, y is up
            // openCV coordinates: origin is at the top left corner, x is right, y is down

            Eigen::Vector2d position;
            position[0] = grid_map_info->GridToMeterForValue(mouse_x, 0);
            position[1] = grid_map_info->GridToMeterForValue(grid_map_info->Shape(1) - mouse_y, 1);
            const Eigen::Matrix2d rotation = Eigen::Matrix2d::Identity();
            const Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(361, 0, 2 * M_PI).head(360);

            std::vector<Eigen::Vector2d> hit_positions;

            {
                std::vector<const erl::geometry::OccupancyQuadtreeNode *> hit_nodes;
                constexpr bool ignore_unknown = false;  // if true, unknown cells are treated as free cells
                constexpr double max_range = -1;
                constexpr bool prune_rays = false;
                constexpr bool parallel = true;
                std::vector<long> hit_ray_indices;

                const erl::common::BlockTimer<std::chrono::milliseconds> timer("CastRays");
                (void) timer;
                quadtree->CastRays(position, rotation, angles, ignore_unknown, max_range, prune_rays, parallel, hit_ray_indices, hit_positions, hit_nodes);
            }

            // long num_hits = 0;
            for (const Eigen::Vector2d &hit_position: hit_positions) {
                cv::line(
                    img_new,
                    cv::Point(mouse_x, mouse_y),
                    cv::Point(
                        grid_map_info->MeterToGridForValue(hit_position[0], 0),
                        grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(hit_position[1], 1)),
                    cv::Scalar(0, 0, 255, 255),
                    1);
            }

            cv::imshow(window_name, img_new);
        }
    };

    std::string window_name = "quadtree";
    std::tuple userdata{window_name, img, quadtree, drawer};
    cv::imshow(window_name, img);
    cv::setMouseCallback(window_name, mouse_callback, &userdata);
    cv::waitKey(0);
}
