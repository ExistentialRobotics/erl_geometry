#include "erl_common/angle_utils.hpp"
#include "erl_common/opencv.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"

struct UserData {
    inline static const char *window_name = "quadtree ray casting";
    std::shared_ptr<erl::geometry::OccupancyQuadtree::Setting> tree_setting = std::make_shared<erl::geometry::OccupancyQuadtree::Setting>();
    std::shared_ptr<erl::geometry::OccupancyQuadtree> tree;
    std::shared_ptr<erl::geometry::OccupancyQuadtree::Drawer> drawer;
    cv::Mat img;
};

void
MouseCallback(const int event, const int mouse_x, const int mouse_y, const int flags, void *userdata) {
    static bool mouse_fixed = false;

    (void) flags;
    const auto data = static_cast<UserData *>(userdata);

    if (event == cv::EVENT_LBUTTONDOWN) {
        std::cout << "Left button of the mouse is clicked - position (" << mouse_x << ", " << mouse_y << ")" << std::endl;
        mouse_fixed = !mouse_fixed;
        return;
    }

    if (event == cv::EVENT_MOUSEMOVE) {
        if (mouse_fixed) { return; }
        if (data->tree == nullptr) { return; }

        const auto grid_map_info = data->drawer->GetGridMapInfo();
        cv::Mat img = data->img.clone();
        const double x = grid_map_info->GridToMeterForValue(mouse_x, 0);
        const double y = grid_map_info->GridToMeterForValue(grid_map_info->Shape(1) - mouse_y, 1);

        constexpr long n = 721;
        Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(n, 0, 2 * M_PI);
        const auto t0 = std::chrono::high_resolution_clock::now();
        Eigen::Matrix2Xd ray_origins(2, n);
        Eigen::Matrix2Xd ray_directions(2, n);
        for (long i = 0; i < n; ++i) {
            ray_origins.col(i) << x, y;
            ray_directions.col(i) << std::cos(angles[i]), std::sin(angles[i]);
        }
        Eigen::VectorXd default_max_ranges, default_node_paddings;
        Eigen::VectorXb default_bidirectional_flags;
        Eigen::VectorXb leaf_only_flags = Eigen::VectorXb::Constant(n, true);
        Eigen::VectorXi default_min_node_depths, default_max_node_depths;
        auto batch_ray_caster = data->tree->GetBatchRayCaster(
            ray_origins,
            ray_directions,
            default_max_ranges,
            default_node_paddings,
            default_bidirectional_flags,
            leaf_only_flags,
            default_min_node_depths,
            default_max_node_depths);
        Eigen::VectorXd hit_distances = Eigen::VectorXd::Zero(n);
        Eigen::VectorXi thicknesses = Eigen::VectorXi::Ones(n);
        while (!batch_ray_caster.GetFrontierKeys().empty()) {
            Eigen::Matrix2Xd start_pts(2, n);
            Eigen::Matrix2Xd end_pts(2, n);
            Eigen::VectorXd new_hit_distances = batch_ray_caster.GetHitDistances();

            std::uniform_int_distribution color_gen(0, 255);
            const double color_r = color_gen(erl::common::g_random_engine);
            const double color_g = color_gen(erl::common::g_random_engine);
            const double color_b = color_gen(erl::common::g_random_engine);

            for (long i = 0; i < n; ++i) {
                const Eigen::Vector2d start_pt = ray_origins.col(i) + ray_directions.col(i) * hit_distances[i];
                const Eigen::Vector2d end_pt = ray_origins.col(i) + ray_directions.col(i) * new_hit_distances[i];
                const int start_px = grid_map_info->MeterToGridForValue(start_pt[0], 0);
                const int start_py = grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(start_pt[1], 1);
                const int end_px = grid_map_info->MeterToGridForValue(end_pt[0], 0);
                const int end_py = grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(end_pt[1], 1);

                cv::line(img, {start_px, start_py}, {end_px, end_py}, {color_b, color_g, color_r, 255}, thicknesses[i]);
            }
            hit_distances = new_hit_distances;
            thicknesses += batch_ray_caster.GetHitFlags().cast<int>();
            ++batch_ray_caster;
            // batch_ray_caster.Step(Eigen::VectorXb::Random(n));
        }

        const auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "Time: " << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms." << std::endl;
        cv::addWeighted(data->img, 0.5, img, 0.5, 0, img);
        cv::imshow(UserData::window_name, img);
    }
}

const std::filesystem::path kProjectRootDir = ERL_GEOMETRY_ROOT_DIR;

TEST(OccupancyQuadtree, BatchRayCasting) {
    UserData data;
    data.tree_setting->resolution = 0.1;
    data.tree = std::make_shared<erl::geometry::OccupancyQuadtree>(data.tree_setting);
    const std::string file = (kProjectRootDir / "data" / "house_expo_room_1451_2d.bt").string();
    ASSERT_TRUE(data.tree->ReadBinary(file)) << "Fail to load the tree.";
    auto setting = std::make_shared<erl::geometry::OccupancyQuadtree::Drawer::Setting>();
    // setting->resolution = 0.0025;
    setting->resolution = 0.01;
    setting->border_color = cv::Scalar(255, 0, 0);
    data.tree->GetMetricMin(setting->area_min[0], setting->area_min[1]);
    data.tree->GetMetricMax(setting->area_max[0], setting->area_max[1]);
    data.drawer = std::make_shared<erl::geometry::OccupancyQuadtree::Drawer>(setting, data.tree);
    data.drawer->DrawLeaves(data.img);

    cv::imshow(UserData::window_name, data.img);
    cv::setMouseCallback(UserData::window_name, MouseCallback, &data);
    cv::waitKey(0);
}
