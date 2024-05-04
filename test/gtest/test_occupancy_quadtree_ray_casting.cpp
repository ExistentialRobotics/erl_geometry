#include "erl_common/opencv.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_common/angle_utils.hpp"
#include "erl_common/test_helper.hpp"

struct UserData {
    inline static const char *window_name = "quadtree ray casting";
    std::shared_ptr<erl::geometry::OccupancyQuadtree::Setting> tree_setting = std::make_shared<erl::geometry::OccupancyQuadtree::Setting>();
    std::shared_ptr<erl::geometry::OccupancyQuadtree> tree;
    std::shared_ptr<erl::geometry::OccupancyQuadtree::Drawer> drawer;
    cv::Mat img;
};

void
MouseCallback(int event, int mouse_x, int mouse_y, int flags, void *userdata) {
    static bool mouse_fixed = false;

    (void) flags;
    auto data = static_cast<UserData *>(userdata);

    if (event == cv::EVENT_LBUTTONDOWN) {
        std::cout << "Left button of the mouse is clicked - position (" << mouse_x << ", " << mouse_y << ")" << std::endl;
        mouse_fixed = !mouse_fixed;
        return;
    }

    if (event == cv::EVENT_MOUSEMOVE) {
        if (mouse_fixed) { return; }
        if (data->tree == nullptr) { return; }

        auto grid_map_info = data->drawer->GetGridMapInfo();
        cv::Mat img = data->img.clone();
        double x = grid_map_info->GridToMeterForValue(mouse_x, 0);
        double y = grid_map_info->GridToMeterForValue(grid_map_info->Shape(1) - mouse_y, 1);

        long n = 721;
        Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(n, 0, 2 * M_PI);
        auto t0 = std::chrono::high_resolution_clock::now();
        for (long i = 0; i < n; ++i) {
            double ex = 0, ey = 0;
            double vx = std::cos(angles[i]);
            double vy = std::sin(angles[i]);
            constexpr bool kIgnoreUnknown = false;
            double max_range = -1;
            if (!data->tree->CastRay(x, y, vx, vy, kIgnoreUnknown, max_range, ex, ey)) {
                if (!data->tree->CastRay(x, y, vx, vy, !kIgnoreUnknown, max_range, ex, ey)) {
                    std::cout << "Fail to cast ray for angle " << erl::common::RadianToDegree(angles[i]) << std::endl;
                }
            }
            cv::line(
                img,
                cv::Point(mouse_x, mouse_y),
                cv::Point(grid_map_info->MeterToGridForValue(ex, 0), grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(ey, 1)),
                cv::Scalar(0, 0, 255, 255),
                1);
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "Time: " << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms." << std::endl;
        cv::addWeighted(data->img, 0.5, img, 0.5, 0, img);
        cv::imshow(UserData::window_name, img);
    }
}

static std::filesystem::path g_test_data_dir = std::filesystem::path(__FILE__).parent_path();

TEST(OccupancyQuadtree, RayCasting) {
    UserData data;
    data.tree_setting->resolution = 0.1;
    data.tree = std::make_shared<erl::geometry::OccupancyQuadtree>(data.tree_setting);
    std::string file = (g_test_data_dir / "house_expo_room_1451_2d.bt").string();
    ASSERT_TRUE(data.tree->ReadBinary(file)) << "Fail to load the tree.";
    auto setting = std::make_shared<erl::geometry::OccupancyQuadtree::Drawer::Setting>();
    setting->resolution = 0.0025;
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
