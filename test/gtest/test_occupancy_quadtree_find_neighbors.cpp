#include "erl_common/opencv.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"

using namespace erl::common;
using namespace erl::geometry;

using QuadtreeDrawer = OccupancyQuadtreeDrawer<OccupancyQuadtreeD>;

struct UserData {
    inline static const char *window_name = "quadtree find neighbors";
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
        double x = grid_map_info->GridToMeterForValue(mouse_x, 0);
        double y = grid_map_info->GridToMeterForValue(grid_map_info->Shape(1) - mouse_y, 1);
        QuadtreeKey key;
        if (!data->tree->CoordToKeyChecked(x, y, key)) { return; }
        unsigned int key_depth = 0;
        auto node = data->tree->Search(key, key_depth);
        if (node == nullptr) { return; }
        // draw the selected node
        key_depth = node->GetDepth();
        data->tree->KeyToCoord(key, key_depth, x, y);
        double half_size = data->tree->GetNodeSize(key_depth) / 2.0;
        Eigen::Vector2d min_meter(x - half_size, y - half_size);
        Eigen::Vector2d max_meter(x + half_size, y + half_size);
        Eigen::Vector2i min = grid_map_info->MeterToPixelForPoints(min_meter);
        Eigen::Vector2i max = grid_map_info->MeterToPixelForPoints(max_meter);
        cv::rectangle(img, {min[0], min[1]}, {max[0], max[1]}, {255, 0, 0, 100}, cv::FILLED);
        // draw neighbors on west
        {
            for (auto it = data->tree->BeginWestLeafNeighbor(x, y),
                      end = data->tree->EndWestLeafNeighbor();
                 it != end;
                 ++it) {
                double node_x = it.GetX();
                double node_y = it.GetY();
                half_size = it.GetNodeSize() / 2.;
                min = grid_map_info->MeterToPixelForPoints(
                    Eigen::Vector2d(node_x - half_size, node_y - half_size));
                max = grid_map_info->MeterToPixelForPoints(
                    Eigen::Vector2d(node_x + half_size, node_y + half_size));
                cv::rectangle(
                    img,
                    {min[0], min[1]},
                    {max[0], max[1]},
                    {255, 128, 0, 128},
                    cv::FILLED);
            }
        }
        // draw neighbors on east
        {
            for (auto it = data->tree->BeginEastLeafNeighbor(x, y),
                      end = data->tree->EndEastLeafNeighbor();
                 it != end;
                 ++it) {
                double node_x = it.GetX();
                double node_y = it.GetY();
                half_size = it.GetNodeSize() / 2.;
                min = grid_map_info->MeterToPixelForPoints(
                    Eigen::Vector2d(node_x - half_size, node_y - half_size));
                max = grid_map_info->MeterToPixelForPoints(
                    Eigen::Vector2d(node_x + half_size, node_y + half_size));
                cv::rectangle(
                    img,
                    {min[0], min[1]},
                    {max[0], max[1]},
                    {128, 255, 0, 128},
                    cv::FILLED);
            }
        }
        // draw neighbors on north
        {
            for (auto it = data->tree->BeginNorthLeafNeighbor(x, y),
                      end = data->tree->EndNorthLeafNeighbor();
                 it != end;
                 ++it) {
                double node_x = it.GetX();
                double node_y = it.GetY();
                half_size = it.GetNodeSize() / 2.;
                min = grid_map_info->MeterToPixelForPoints(
                    Eigen::Vector2d(node_x - half_size, node_y - half_size));
                max = grid_map_info->MeterToPixelForPoints(
                    Eigen::Vector2d(node_x + half_size, node_y + half_size));
                cv::rectangle(
                    img,
                    {min[0], min[1]},
                    {max[0], max[1]},
                    {0, 255, 128, 128},
                    cv::FILLED);
            }
        }
        // draw neighbors on south
        {
            for (auto it = data->tree->BeginSouthLeafNeighbor(x, y),
                      end = data->tree->EndSouthLeafNeighbor();
                 it != end;
                 ++it) {
                double node_x = it.GetX();
                double node_y = it.GetY();
                half_size = it.GetNodeSize() / 2.;
                min = grid_map_info->MeterToPixelForPoints(
                    Eigen::Vector2d(node_x - half_size, node_y - half_size));
                max = grid_map_info->MeterToPixelForPoints(
                    Eigen::Vector2d(node_x + half_size, node_y + half_size));
                cv::rectangle(
                    img,
                    {min[0], min[1]},
                    {max[0], max[1]},
                    {0, 128, 255, 128},
                    cv::FILLED);
            }
        }
        cv::imshow(UserData::window_name, img);
    }
}

TEST(OccupancyQuadtree, FindNeighbors) {
    UserData data;
    data.tree_setting->resolution = 0.1;
    data.tree = std::make_shared<OccupancyQuadtreeD>(data.tree_setting);
    std::filesystem::path file = ERL_GEOMETRY_ROOT_DIR;
    file /= "data";
    file /= "square.bt";
    EXPECT_TRUE(Serialization<OccupancyQuadtreeD>::Read(file, [&](std::istream &s) -> bool {
        return data.tree->ReadBinary(s);
    }));

    auto setting = std::make_shared<QuadtreeDrawer::Setting>();
    setting->resolution = 0.0025;
    setting->border_color = cv::Scalar(255, 0, 0);
    data.tree->GetMetricMin(setting->area_min[0], setting->area_min[1]);
    data.tree->GetMetricMax(setting->area_max[0], setting->area_max[1]);
    data.drawer = std::make_shared<QuadtreeDrawer>(setting, data.tree);
    data.drawer->DrawLeaves(data.img);

    cv::imshow(UserData::window_name, data.img);
    cv::setMouseCallback(UserData::window_name, MouseCallback, &data);
    cv::waitKey(0);
}
