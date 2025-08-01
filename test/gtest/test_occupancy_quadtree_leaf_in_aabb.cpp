#include "erl_common/opencv.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"

using namespace erl::common;
using namespace erl::geometry;
using QuadtreeDrawer = OccupancyQuadtreeDrawer<OccupancyQuadtreeD>;

struct UserData {
    inline static const char *window_name = "quadtree leaf of node at level 6";
    std::shared_ptr<OccupancyQuadtreeD::Setting> tree_setting =
        std::make_shared<OccupancyQuadtreeD::Setting>();
    std::shared_ptr<OccupancyQuadtreeD> tree;
    std::shared_ptr<QuadtreeDrawer> drawer;
    cv::Mat img;
};

void
MouseCallback(int event, int mouse_x, int mouse_y, int flags, void *userdata) {
    static bool mouse_fixed = false;

    (void) flags;
    auto data = static_cast<UserData *>(userdata);

    if (event == cv::EVENT_LBUTTONDOWN) {
        std::cout << "Left button of the mouse is clicked - position (" << mouse_x << ", "
                  << mouse_y << ")" << std::endl;
        mouse_fixed = !mouse_fixed;
        return;
    }

    if (event == cv::EVENT_MOUSEMOVE) {
        if (mouse_fixed) { return; }
        if (data->tree == nullptr) { return; }

        auto grid_map_info = data->drawer->GetGridMapInfo();
        cv::Mat img = data->img.clone();
        double x = grid_map_info->GridToMeterAtDim(mouse_x, 0);
        double y = grid_map_info->GridToMeterAtDim(grid_map_info->Shape(1) - mouse_y, 1);
        double aabb_mix_x = x - 2.0;
        double aabb_mix_y = y - 2.0;
        double aabb_max_x = x + 2.0;
        double aabb_max_y = y + 2.0;
        auto it = data->tree->BeginLeafInAabb(aabb_mix_x, aabb_mix_y, aabb_max_x, aabb_max_y);
        auto end = data->tree->EndLeafOfNode();
        auto t0 = std::chrono::high_resolution_clock::now();
        for (; it != end; ++it) {
            double leaf_x = it.GetX();
            double leaf_y = it.GetY();
            double leaf_size = it.GetNodeSize();
            double half_size = leaf_size / 2;
            Eigen::Vector2d min_meter(leaf_x - half_size, leaf_y - half_size);
            Eigen::Vector2d max_meter(leaf_x + half_size, leaf_y + half_size);
            Eigen::Vector2i min = grid_map_info->MeterToPixelForPoints(min_meter);
            Eigen::Vector2i max = grid_map_info->MeterToPixelForPoints(max_meter);
            cv::Point pt1(min[0], min[1]);
            cv::Point pt2(max[0], max[1]);
            cv::rectangle(
                img,
                pt1,
                pt2,
                data->tree->IsNodeOccupied(*it) ? cv::Scalar(0, 0, 255, 255)
                                                : cv::Scalar(0, 128, 255, 255),
                cv::FILLED);
            cv::Point pt_key(
                grid_map_info->MeterToGridAtDim(leaf_x, 0),
                grid_map_info->Shape(1) - grid_map_info->MeterToGridAtDim(leaf_y, 1));
            cv::circle(img, pt_key, 3, cv::Scalar(0, 0, 0, 255), cv::FILLED);
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "Time: " << std::chrono::duration<double, std::milli>(t1 - t0).count()
                  << " ms." << std::endl;
        cv::addWeighted(data->img, 0.4, img, 0.6, 0, img);
        cv::imshow(UserData::window_name, img);
    }
}

TEST(OccupancyQuadtree, IterateLeafInAABB) {
    GTEST_PREPARE_OUTPUT_DIR();
    UserData data;
    data.tree_setting->resolution = 0.1;
    data.tree = std::make_shared<OccupancyQuadtreeD>(data.tree_setting);
    std::filesystem::path data_dir = ERL_GEOMETRY_ROOT_DIR;
    data_dir /= "data";
    const std::filesystem::path path = data_dir / "house_expo_room_1451_2d_double.bt";
    ASSERT_TRUE(Serialization<OccupancyQuadtreeD>::Read(path, [&](std::istream &s) -> bool {
        return data.tree->ReadBinary(s);
    }));
    auto setting = std::make_shared<QuadtreeDrawer::Setting>();
    setting->resolution = 0.01;
    setting->border_color = cv::Scalar(255, 0, 0);
    data.tree->GetMetricMin(setting->area_min[0], setting->area_min[1]);
    data.tree->GetMetricMax(setting->area_max[0], setting->area_max[1]);
    data.drawer = std::make_shared<QuadtreeDrawer>(setting, data.tree);
    data.drawer->DrawLeaves(data.img);

    cv::imshow(UserData::window_name, data.img);
    cv::setMouseCallback(UserData::window_name, MouseCallback, &data);
    cv::waitKey(0);  // 5 sec.
}
