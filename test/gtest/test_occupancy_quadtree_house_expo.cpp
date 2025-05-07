#include "erl_common/csv.hpp"
#include "erl_common/random.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/house_expo_map.hpp"
#include "erl_geometry/lidar_2d.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"

using namespace erl::common;
using namespace erl::geometry;

TEST(OccupancyQuadtree, BuildWithHouseExpo) {
    std::filesystem::path test_data_dir = std::filesystem::path(__FILE__).parent_path();
    int map_index = 1451;
    std::filesystem::path map_file_path =
        test_data_dir / ("house_expo_room_" + std::to_string(map_index) + ".json");
    HouseExpoMap house_expo_map(map_file_path.string().c_str(), 0.2);
    Eigen::Vector2d map_min =
        house_expo_map.GetMeterSpace()->GetSurface()->vertices.rowwise().minCoeff();
    Eigen::Vector2d map_max =
        house_expo_map.GetMeterSpace()->GetSurface()->vertices.rowwise().maxCoeff();
    Eigen::Vector2d map_resolution(0.01, 0.01);
    Eigen::Vector2i map_padding(100, 100);
    auto lidar_setting = std::make_shared<Lidar2D::Setting>();
    lidar_setting->num_lines = 720;
    Lidar2D lidar(lidar_setting, house_expo_map.GetMeterSpace());

    std::filesystem::path traj_file_path =
        test_data_dir / ("house_expo_room_" + std::to_string(map_index) + ".csv");
    std::vector<std::vector<double>> trajectory = LoadAndCastCsvFile<double>(
        traj_file_path.string().c_str(),
        [](const std::string &str) -> double { return std::stod(str); });
    long max_update_cnt = static_cast<long>(trajectory.size());

    auto tree_setting = std::make_shared<OccupancyQuadtreeD::Setting>();
    tree_setting->resolution = 0.05;
    auto tree = std::make_shared<OccupancyQuadtreeD>(tree_setting);
    using QuadtreeDrawer = OccupancyQuadtreeDrawer<OccupancyQuadtreeD>;
    auto drawer_setting = std::make_shared<QuadtreeDrawer::Setting>();
    drawer_setting->area_min = map_min;
    drawer_setting->area_max = map_max;
    drawer_setting->resolution = map_resolution[0];
    drawer_setting->padding = map_padding[0];
    drawer_setting->border_color = cv::Scalar(255, 0, 0, 255);
    auto drawer = std::make_shared<QuadtreeDrawer>(drawer_setting, tree);
    auto grid_map_info = drawer->GetGridMapInfo();

    long stride = 5;
    cv::Scalar trajectory_color(0, 0, 255, 255);
    Eigen::VectorXd lidar_angles = lidar.GetAngles();
    Eigen::Matrix2Xd line_directions(2, lidar_setting->num_lines);
    Eigen::Matrix2Xd points(2, lidar_setting->num_lines);
    cv::Mat img;
    const char *window_name = "quadtree house expo";
    for (long i = 0; i < lidar_setting->num_lines; ++i) {
        line_directions.col(i) << std::cos(lidar_angles[i]), std::sin(lidar_angles[i]);
    }
    std::cout << "Press any key to start" << std::endl;
    drawer->DrawLeaves(img);
    cv::imshow(window_name, img);
    cv::waitKey(0);
    Eigen::Matrix2Xd cur_traj(2, max_update_cnt);
    for (long i = 0; i < max_update_cnt; ++i) {
        cur_traj.col(i) << trajectory[i][0], trajectory[i][1];
    }
    for (long i = 0; i < max_update_cnt; i += stride) {
        constexpr bool pixel_based = true;
        constexpr bool discrete = false;
        constexpr bool lazy_eval = false;
        constexpr bool parallel = true;
        constexpr double max_range = 30;
        constexpr bool scan_in_parallel = true;
        std::vector<double> &waypoint = trajectory[i];

        Eigen::Matrix2d rotation = Eigen::Rotation2Dd(waypoint[2]).toRotationMatrix();
        Eigen::Vector2d translation(waypoint[0], waypoint[1]);
        auto lidar_ranges = lidar.Scan(rotation, translation, scan_in_parallel);
        // lidar_ranges += GenerateGaussianNoise(lidar_ranges.size(), 0.0, 0.01);

        for (long j = 0; j < lidar_setting->num_lines; ++j) {
            points.col(j) << rotation * (lidar_ranges[j] * line_directions.col(j)) + translation;
        }
        auto t0 = std::chrono::high_resolution_clock::now();
        tree->InsertPointCloud(points, translation, max_range, parallel, lazy_eval, discrete);
        auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "update time: " << std::chrono::duration<double, std::milli>(t1 - t0).count()
                  << " ms" << std::endl;

        drawer->DrawLeaves(img);
        DrawTrajectoryInplace(
            img,
            cur_traj.block(0, 0, 2, i),
            grid_map_info,
            trajectory_color,
            2,
            pixel_based);
        cv::imshow(window_name, img);
        cv::waitKey(10);
    }

    tree->WriteBinary("house_expo_room_" + std::to_string(map_index) + ".bt");
    ERL_ASSERT(tree->Write("house_expo_room_" + std::to_string(map_index) + ".ot"));
    std::cout << "Press any key to exit immediately. Test will exist in 10 seconds." << std::endl;
    cv::waitKey(10000);  // 10 seconds
}
