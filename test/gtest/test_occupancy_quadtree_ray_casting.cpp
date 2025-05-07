#include "erl_common/angle_utils.hpp"
#include "erl_common/opencv.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"

using namespace erl::common;
using namespace erl::geometry;
using QuadtreeDrawer = OccupancyQuadtreeDrawer<OccupancyQuadtreeD>;

struct UserData {
    inline static const char *window_name = "quadtree ray casting";
    std::shared_ptr<OccupancyQuadtreeD::Setting> tree_setting =
        std::make_shared<OccupancyQuadtreeD::Setting>();
    std::shared_ptr<OccupancyQuadtreeD> tree;
    std::shared_ptr<QuadtreeDrawer> drawer;
    cv::Mat img;
};

void
MouseCallback(
    const int event,
    const int mouse_x,
    const int mouse_y,
    const int flags,
    void *userdata) {
    static bool mouse_fixed = false;

    (void) flags;
    const auto data = static_cast<UserData *>(userdata);

    if (event == cv::EVENT_LBUTTONDOWN) {
        std::cout << "Left button of the mouse is clicked - position (" << mouse_x << ", "
                  << mouse_y << ")" << std::endl;
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
        Eigen::Matrix2Xd end_points(2, n);
#pragma omp parallel for default(none) \
    shared(n, data, angles, img, x, y, grid_map_info, mouse_x, mouse_y, end_points)
        for (long i = 0; i < n; ++i) {
            double &ex = end_points(0, i);
            double &ey = end_points(1, i);

            ex = x;
            ey = y;

            // // (ex, ey) is node center position
            // if (const double vx = std::cos(angles[i]), vy = std::sin(angles[i]);
            //     !data->tree->CastRay(x, y, vx, vy, /*ignore_unknown*/ false, /*max_range*/ -1,
            //     ex, ey) || !data->tree->CastRay(x, y, vx, vy, /*ignore_unknown*/ true,
            //     /*max_range*/ -1, ex, ey)) { ERL_DEBUG("Fail to cast ray for angle {}.",
            //     erl::common::RadianToDegree(angles[i]));
            // }

            // similar to CastRay, but using BeginNodeOnRay, (ex, ey) is the hit point on the node
            // boundary
            const double vx = std::cos(angles[i]), vy = std::sin(angles[i]);
            auto itr = data->tree->BeginNodeOnRay(x, y, vx, vy, -1, 0, false, true);
            auto end = data->tree->EndNodeOnRay();
            while (itr != end) {
                if (data->tree->IsNodeOccupied(*itr)) {
                    ex = x + vx * itr.GetDistance();  // point on the node boundary
                    ey = y + vy * itr.GetDistance();
                    break;
                }
                ++itr;
            }
        }
        for (long i = 0; i < n; ++i) {
            const double &ex = end_points(0, i);
            const double &ey = end_points(1, i);
            cv::line(
                img,
                cv::Point(mouse_x, mouse_y),
                cv::Point(
                    grid_map_info->MeterToGridForValue(ex, 0),
                    grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(ey, 1)),
                cv::Scalar(0, 0, 255, 255),
                1);
        }
        const auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "Time: " << std::chrono::duration<double, std::milli>(t1 - t0).count()
                  << " ms." << std::endl;
        cv::addWeighted(data->img, 0.5, img, 0.5, 0, img);
        cv::imshow(UserData::window_name, img);
    }
}

const std::filesystem::path kProjectRootDir = ERL_GEOMETRY_ROOT_DIR;

TEST(OccupancyQuadtree, RayCasting) {
    UserData data;
    data.tree_setting->resolution = 0.1;
    data.tree = std::make_shared<OccupancyQuadtreeD>(data.tree_setting);
    const std::string file = (kProjectRootDir / "data" / "house_expo_room_1451_2d.bt").string();
    ASSERT_TRUE(data.tree->ReadBinary(file)) << "Fail to load the tree.";
    auto setting = std::make_shared<QuadtreeDrawer::Setting>();
    // setting->resolution = 0.0025;
    setting->resolution = 0.01;
    setting->border_color = cv::Scalar(255, 0, 0);
    setting->padding = 10;
    data.tree->GetMetricMin(setting->area_min[0], setting->area_min[1]);
    data.tree->GetMetricMax(setting->area_max[0], setting->area_max[1]);
    data.drawer = std::make_shared<QuadtreeDrawer>(setting, data.tree);
    data.drawer->DrawLeaves(data.img);

    cv::imshow(UserData::window_name, data.img);
    cv::setMouseCallback(UserData::window_name, MouseCallback, &data);
    cv::waitKey(0);
}
