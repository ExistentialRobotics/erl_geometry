#include "erl_common/angle_utils.hpp"
#include "erl_common/opencv.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"

#include <boost/program_options.hpp>

using namespace erl::common;
using namespace erl::geometry;
using QuadtreeDrawer = OccupancyQuadtreeDrawer<OccupancyQuadtreeD>;

struct UserData {
    inline static const char *window_name = "quadtree ray tracing";
    std::shared_ptr<OccupancyQuadtreeD> tree;
    std::shared_ptr<QuadtreeDrawer> drawer;
    bool occupied_only = true;
    bool bidirectional = true;
    cv::Mat img;
    double angle = 0;
    bool mouse_fixed = false;
    int mouse_x = 0;
    int mouse_y = 0;
};

void
Draw(UserData *data) {
    if (data->mouse_fixed) { return; }
    if (data->tree == nullptr) { return; }

    auto grid_map_info = data->drawer->GetGridMapInfo();
    cv::Mat img = data->img.clone();
    double x = grid_map_info->GridToMeterForValue(data->mouse_x, 0);
    double y = grid_map_info->GridToMeterForValue(grid_map_info->Shape(1) - data->mouse_y, 1);

    auto t0 = std::chrono::high_resolution_clock::now();
    double vx = std::cos(data->angle);
    double vy = std::sin(data->angle);
    double max_range = 10;
    int ex1 = 0, ey1 = 0, ex2 = 0, ey2 = 0;
    for (auto it = data->tree->BeginNodeOnRay(
                  x,
                  y,
                  vx,
                  vy,
                  max_range,
                  data->bidirectional,
                  /*leaf_only*/ true),
              end = data->tree->EndNodeOnRay();
         it != end;
         ++it) {
        if (data->occupied_only && !data->tree->IsNodeOccupied(*it)) { continue; }
        double node_x = it.GetX();
        double node_y = it.GetY();
        double half_size = it.GetNodeSize() / 2.;
        Eigen::Vector2i min = grid_map_info->MeterToPixelForPoints(
            Eigen::Vector2d(node_x - half_size, node_y - half_size));
        Eigen::Vector2i max = grid_map_info->MeterToPixelForPoints(
            Eigen::Vector2d(node_x + half_size, node_y + half_size));
        cv::rectangle(img, {min[0], min[1]}, {max[0], max[1]}, {0, 0, 255, 255}, cv::FILLED);
        if (it.GetDistance() > 0) {
            ex1 = (max[0] + min[0]) / 2;
            ey1 = (max[1] + min[1]) / 2;
        } else {
            ex2 = (max[0] + min[0]) / 2;
            ey2 = (max[1] + min[1]) / 2;
        }
        // std::cout << "x: " << node_x << ", y: " << node_y << ", dist: " << it.GetDistance() << ",
        // size: " << it.GetNodeSize() << std::endl;
    }
    cv::line(img, {data->mouse_x, data->mouse_y}, {ex1, ey1}, {0, 255, 0, 255}, 2);
    cv::line(img, {data->mouse_x, data->mouse_y}, {ex2, ey2}, {255, 0, 0, 255}, 2);
    auto t1 = std::chrono::high_resolution_clock::now();
    std::cout << "Time: " << std::chrono::duration<double, std::micro>(t1 - t0).count() << " us."
              << std::endl;
    cv::addWeighted(data->img, 0.5, img, 0.5, 0, img);
    cv::imshow(UserData::window_name, img);
}

void
MouseCallback(
    const int event,
    const int mouse_x,
    const int mouse_y,
    const int flags,
    void *userdata) {
    (void) flags;
    const auto data = static_cast<UserData *>(userdata);

    if (event == cv::EVENT_LBUTTONDOWN) {
        std::cout << "Left button of the mouse is clicked - position (" << mouse_x << ", "
                  << mouse_y << ")" << std::endl;
        data->mouse_fixed = !data->mouse_fixed;
        return;
    }

    if (event == cv::EVENT_MOUSEMOVE) {
        data->mouse_x = mouse_x;
        data->mouse_y = mouse_y;
        Draw(data);
    }
}

static std::filesystem::path g_test_data_dir = std::filesystem::path(__FILE__).parent_path();

struct Options {
    std::string tree_bt_file = (g_test_data_dir / "house_expo_room_1451_2d.bt").string();
    double resolution = 0.01;
    int padding = 10;
    bool occupied_only = false;
    bool bidirectional = true;
};

Options g_options;

TEST(OccupancyQuadtree, IterateLeafOnRay) {
    UserData data;
    data.occupied_only = g_options.occupied_only;
    auto tree_setting = std::make_shared<OccupancyQuadtreeD::Setting>();
    tree_setting->resolution = 0.1;
    data.tree = std::make_shared<OccupancyQuadtreeD>(tree_setting);
    ERL_ASSERTM(data.tree->ReadBinary(g_options.tree_bt_file), "Fail to load the tree.");
    auto setting = std::make_shared<QuadtreeDrawer::Setting>();
    setting->resolution = g_options.resolution;
    setting->padding = g_options.padding;
    setting->border_color = cv::Scalar(255, 0, 0);
    data.tree->GetMetricMin(setting->area_min[0], setting->area_min[1]);
    data.tree->GetMetricMax(setting->area_max[0], setting->area_max[1]);
    data.drawer = std::make_shared<QuadtreeDrawer>(setting, data.tree);
    data.drawer->DrawLeaves(data.img);

    cv::imshow(UserData::window_name, data.img);
    cv::setMouseCallback(UserData::window_name, MouseCallback, &data);

    while (true) {
        const auto key = cv::waitKey(0);
        if (key == 27) { break; }
        if (key == 'a') {
            data.angle += DegreeToRadian(1);
        } else if (key == 'd') {
            data.angle -= DegreeToRadian(1);
        }
        data.angle = WrapAnglePi(data.angle);
        Draw(&data);
        std::cout << "Angle: " << RadianToDegree(data.angle) << "\n";
    }
}

int
main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    try {
        namespace po = boost::program_options;
        po::options_description desc;
        po::positional_options_description positional_options;
        // clang-format off
        desc.add_options()
            ("help", "produce help message")
            ("resolution", po::value<double>(&g_options.resolution)->default_value(g_options.resolution)->value_name("res"), "resolution of the display image")
            ("padding", po::value<int>(&g_options.padding)->default_value(g_options.padding), "padding of the display image")
            ("occupied-only", po::bool_switch(&g_options.occupied_only)->default_value(g_options.occupied_only), "show occupied only")
            ("single-direction", po::bool_switch(&g_options.bidirectional)->default_value(g_options.bidirectional), "single direction")
            ("tree-bt-file", po::value<std::string>(&g_options.tree_bt_file)->value_name("tree_bt_file")->default_value(g_options.tree_bt_file), "occupancy tree binary format file");
        positional_options.add("tree-bt-file", 1);
        // clang-format on

        po::variables_map vm;
        po::store(
            po::command_line_parser(argc, argv).options(desc).positional(positional_options).run(),
            vm);

        if (vm.count("help")) {
            std::cout << "Usage: " << argv[0] << " [options] tree_bt_file" << std::endl
                      << desc << std::endl;
            return 0;
        }
        po::notify(vm);
    } catch (std::exception &e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    return RUN_ALL_TESTS();
}
