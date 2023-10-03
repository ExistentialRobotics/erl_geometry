#include "erl_common/opencv.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"
#include <gtest/gtest.h>

using OccupancyQuadtreeDrawer = erl::geometry::OccupancyQuadtreeDrawer<erl::geometry::OccupancyQuadtree>;

struct UserData {
    inline static const char *window_name = "quadtree leaf of node at level 6";
    std::shared_ptr<erl::geometry::OccupancyQuadtree> tree;
    std::shared_ptr<OccupancyQuadtreeDrawer> drawer;
    cv::Mat img;
};

void
MouseCallback(int event, int mouse_x, int mouse_y, int flags, void *userdata) {
    static bool mouse_fixed = false;

    (void) flags;
    auto data = reinterpret_cast<UserData *>(userdata);

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

        unsigned int node_depth = data->tree->GetTreeDepth() - 6;
        erl::geometry::QuadtreeKey key = data->tree->CoordToKey(x, y, node_depth);
        double kx, ky;
        {
            data->tree->KeyToCoord(key, node_depth, kx, ky);
            double size = data->tree->GetNodeSize(node_depth);
            double half_size = size / 2;
            Eigen::Vector2i min = grid_map_info->MeterToPixelForPoints(Eigen::Vector2d(kx - half_size, ky - half_size));
            Eigen::Vector2i max = grid_map_info->MeterToPixelForPoints(Eigen::Vector2d(kx + half_size, ky + half_size));
            cv::Point pt1(min[0], min[1]);
            cv::Point pt2(max[0], max[1]);
            cv::rectangle(img, pt1, pt2, cv::Scalar(0, 255, 255, 255), cv::FILLED);
        }
        auto it = data->tree->BeginLeafOfNode(key, node_depth);
        auto end = data->tree->EndLeafOfNode();
        auto t0 = std::chrono::high_resolution_clock::now();
        for (; it != end; ++it) {
            double leaf_x = it.GetX();
            double leaf_y = it.GetY();
            double leaf_size = it.GetNodeSize();
            double half_size = leaf_size / 2;
            Eigen::Vector2i min = grid_map_info->MeterToPixelForPoints(Eigen::Vector2d(leaf_x - half_size, leaf_y - half_size));
            Eigen::Vector2i max = grid_map_info->MeterToPixelForPoints(Eigen::Vector2d(leaf_x + half_size, leaf_y + half_size));
            cv::Point pt1(min[0], min[1]);
            cv::Point pt2(max[0], max[1]);
            cv::rectangle(img, pt1, pt2, data->tree->IsNodeOccupied(*it) ? cv::Scalar(0, 0, 255, 255) : cv::Scalar(0, 128, 255, 255), cv::FILLED);
            cv::Point pt_key(grid_map_info->MeterToGridForValue(kx, 0), grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(ky, 1));
            cv::circle(img, pt_key, 5, cv::Scalar(0, 0, 0, 255), cv::FILLED);
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "Time: " << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms." << std::endl;
        cv::addWeighted(data->img, 0.4, img, 0.6, 0, img);
        cv::imshow(UserData::window_name, img);
    }
}

TEST(ERL_GEOMETRY, OccupancyQuadtree_LeafOfNode) {
    UserData data;
    data.tree = std::make_shared<erl::geometry::OccupancyQuadtree>(0.1);
    ERL_ASSERTM(data.tree->ReadBinary("house_expo_room_1451.bt"), "Fail to load the tree.");
    auto setting = std::make_shared<OccupancyQuadtreeDrawer::Setting>();
    setting->resolution = 0.0025;
    setting->resolution = 0.01;
    setting->border_color = cv::Scalar(255, 0, 0);
    data.tree->GetMetricMin(setting->area_min[0], setting->area_min[1]);
    data.tree->GetMetricMax(setting->area_max[0], setting->area_max[1]);
    data.drawer = std::make_shared<OccupancyQuadtreeDrawer>(setting, data.tree);
    data.drawer->DrawLeaves(data.img);

    cv::imshow(UserData::window_name, data.img);
    cv::setMouseCallback(UserData::window_name, MouseCallback, &data);
    cv::waitKey(5000);  // 5 sec.
}
