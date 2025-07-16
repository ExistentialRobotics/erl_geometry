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

        const uint32_t tree_depth = data->tree->GetTreeDepth();
        const QuadtreeKey key = data->tree->CoordToKey(x, y, tree_depth);
        const OccupancyQuadtreeNode *node = data->tree->Search(key);
        if (node == nullptr) { return; }
        double kx, ky;
        data->tree->KeyToCoord(key, node->GetDepth(), kx, ky);
        const double half_size = data->tree->GetNodeSize(node->GetDepth()) / 2;
        Eigen::Vector2i min =
            grid_map_info->MeterToPixelForPoints(Eigen::Vector2d(kx - half_size, ky - half_size));
        Eigen::Vector2i max =
            grid_map_info->MeterToPixelForPoints(Eigen::Vector2d(kx + half_size, ky + half_size));
        cv::rectangle(
            img,
            cv::Point(min[0], min[1]),
            cv::Point(max[0], max[1]),
            cv::Scalar(0, 255, 255, 255),
            cv::FILLED);

        const uint32_t voxel_size = 1 << (tree_depth - node->GetDepth());
        const QuadtreeKey voxel_key = data->tree->AdjustKeyToDepth(key, node->GetDepth());
        QuadtreeKey vertex_key;
        for (uint32_t i = 0; i < 4; ++i) {
            QuadtreeKey::ComputeVertexKey(i, voxel_size, voxel_key, vertex_key);
            Eigen::Vector2d coord = data->tree->KeyToVertexCoord(vertex_key, node->GetDepth());
            Eigen::Vector2i pixel = grid_map_info->MeterToPixelForPoints(coord);
            cv::circle(
                img,
                cv::Point(pixel[0], pixel[1]),
                4,
                cv::Scalar(0, 0, 255, 255),
                cv::FILLED);
        }

        cv::addWeighted(data->img, 0.3, img, 0.7, 0, img);
        cv::imshow(UserData::window_name, img);
    }
}

TEST(OccupancyQuadtree, LeafVertices) {
    GTEST_PREPARE_OUTPUT_DIR();
    UserData data;
    data.tree_setting->resolution = 0.1;
    data.tree = std::make_shared<OccupancyQuadtreeD>(data.tree_setting);
    std::filesystem::path data_dir = ERL_GEOMETRY_ROOT_DIR;
    data_dir /= "data";
    ASSERT_TRUE(
        Serialization<OccupancyQuadtreeD>::Read(
            data_dir /= "house_expo_room_1451_2d_double.bt",
            [&](std::istream &s) -> bool { return data.tree->ReadBinary(s); }));
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
