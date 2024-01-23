#include "erl_common/test_helper.hpp"
#include "erl_common/csv.hpp"
#include "erl_common/binary_file.hpp"
#include "erl_common/progress_bar.hpp"
#include "erl_common/random.hpp"
#include "erl_geometry/gazebo_room.hpp"
#include "erl_geometry/house_expo_map.hpp"
#include "erl_geometry/lidar_2d.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include <boost/program_options.hpp>

static std::filesystem::path g_test_data_dir = std::filesystem::path(__FILE__).parent_path();
static std::string g_window_name = "ERL_GEOMETRY";

struct Options {
    std::string gazebo_train_file = (g_test_data_dir / "gazebo_train.dat").string();
    std::string gazebo_test_file = (g_test_data_dir / "gazebo_test.dat").string();
    std::string house_expo_map_file = (g_test_data_dir / "house_expo_room_1451.json").string();
    std::string house_expo_traj_file = (g_test_data_dir / "house_expo_room_1451.csv").string();
    std::string ros_bag_dat_file = (g_test_data_dir / "ros_bag.dat").string();
    bool use_gazebo_data = false;
    bool use_house_expo_data = false;
    bool use_ros_bag_data = true;
    bool visualize = false;
    bool hold = false;
    int stride = 1;
    double quadtree_resolution = 0.05;
    bool quadtree_lazy_eval = false;  // bad, slower
    double map_resolution = 0.025;
};

static Options g_options;

TEST(OccupancyQuadtree, Build) {

    long max_update_cnt;
    Eigen::Vector2d map_min(0, 0);
    Eigen::Vector2d map_max(0, 0);
    Eigen::Vector2d map_resolution(g_options.map_resolution, g_options.map_resolution);
    Eigen::Vector2i map_padding(10, 10);
    std::vector<Eigen::Matrix2Xd> buf_points;
    Eigen::Matrix2Xd trajectory;

    auto load_scan = [&](const Eigen::Matrix2d &rotation, const Eigen::Vector2d &translation, const Eigen::VectorXd &angles, const Eigen::VectorXd &ranges) {
        long n = ranges.size();
        Eigen::Matrix2Xd points(2, n);
        long cnt = 0;
        for (long k = 0; k < n; ++k) {
            const double &angle = angles[k];
            const double &range = ranges[k];
            if (std::isinf(range) || std::isnan(range)) { continue; }
            Eigen::Vector2d local_pt(std::cos(angle) * range, std::sin(angle) * range);
            Eigen::Vector2d global_pt = rotation * local_pt + translation;
            points.col(cnt) = global_pt;
            cnt++;

            double &x = global_pt[0];
            double &y = global_pt[1];
            if (x < map_min[0]) { map_min[0] = x; }
            if (x > map_max[0]) { map_max[0] = x; }
            if (y < map_min[1]) { map_min[1] = y; }
            if (y > map_max[1]) { map_max[1] = y; }
        }
        points.conservativeResize(2, cnt);
        return points;
    };

    if (g_options.use_gazebo_data) {
        // load raw data
        auto train_data_loader = erl::geometry::GazeboRoom::TrainDataLoader(g_options.gazebo_train_file.c_str());
        // prepare buffer
        max_update_cnt = long(train_data_loader.size()) / g_options.stride + 1;
        buf_points.reserve(max_update_cnt);
        trajectory.resize(2, max_update_cnt);
        // load data into buffer
        long j = 0;
        erl::common::ProgressBar bar(int(max_update_cnt), true, std::cout);
        for (std::size_t i = 0; i < train_data_loader.size(); i += g_options.stride) {
            auto &df = train_data_loader[i];
            trajectory.col(j) << df.pose_numpy(0, 2), df.pose_numpy(1, 2);
            buf_points.push_back(load_scan(df.pose_numpy.block<2, 2>(0, 0), trajectory.col(j), df.angles, df.distances));
            j++;
            bar.Update();
        }
        trajectory.conservativeResize(2, j);
    } else if (g_options.use_house_expo_data) {
        // load raw data
        erl::geometry::HouseExpoMap house_expo_map(g_options.house_expo_map_file.c_str(), 0.2);
        auto caster = [](const std::string &str) -> double { return std::stod(str); };
        auto csv_trajectory = erl::common::LoadAndCastCsvFile<double>(g_options.house_expo_traj_file.c_str(), caster);
        // prepare buffer
        max_update_cnt = long(csv_trajectory.size()) / g_options.stride + 1;
        buf_points.reserve(max_update_cnt);
        trajectory.resize(2, max_update_cnt);
        // setup lidar
        auto lidar_setting = std::make_shared<erl::geometry::Lidar2D::Setting>();
        lidar_setting->num_lines = 720;
        erl::geometry::Lidar2D lidar(lidar_setting, house_expo_map.GetMeterSpace());
        bool scan_in_parallel = true;
        Eigen::VectorXd lidar_angles = lidar.GetAngles();
        // load data into buffer
        long j = 0;
        erl::common::ProgressBar bar(int(max_update_cnt), true, std::cout);
        for (std::size_t i = 0; i < csv_trajectory.size(); i += g_options.stride) {
            trajectory.col(j) << csv_trajectory[i][0], csv_trajectory[i][1];
            Eigen::Matrix2d rotation = Eigen::Rotation2Dd(csv_trajectory[i][2]).toRotationMatrix();
            Eigen::VectorXd lidar_ranges = lidar.Scan(rotation, trajectory.col(j), scan_in_parallel);
            lidar_ranges += erl::common::GenerateGaussianNoise(lidar_ranges.size(), 0.0, 0.01);
            buf_points.push_back(load_scan(rotation, trajectory.col(j), lidar_angles, lidar_ranges));
            j++;
            bar.Update();
        }
        trajectory.conservativeResize(2, j);
    } else {
        Eigen::MatrixXd ros_bag_data = erl::common::LoadEigenMatrixFromBinaryFile<double, Eigen::Dynamic, Eigen::Dynamic>(g_options.ros_bag_dat_file);
        // prepare buffer
        max_update_cnt = long(ros_bag_data.rows()) / g_options.stride + 1;
        buf_points.reserve(max_update_cnt);
        trajectory.resize(2, max_update_cnt);
        // load data into buffer
        long num_rays = (ros_bag_data.cols() - 7) / 2;
        long j = 0;
        erl::common::ProgressBar bar(int(max_update_cnt), true, std::cout);
        for (long i = 0; i < ros_bag_data.rows(); i += g_options.stride, j++) {
            Eigen::Matrix23d pose = ros_bag_data.row(i).segment(1, 6).reshaped(3, 2).transpose();
            trajectory.col(j) << pose(0, 2), pose(1, 2);
            Eigen::VectorXd angles = ros_bag_data.row(i).segment(7, num_rays);
            Eigen::VectorXd ranges = ros_bag_data.row(i).segment(7 + num_rays, num_rays);
            buf_points.push_back(load_scan(pose.block<2, 2>(0, 0), trajectory.col(j), angles, ranges));
            bar.Update();
        }
        trajectory.conservativeResize(2, j);
    }

    // initialize occupancy quadtree
    auto tree = std::make_shared<erl::geometry::OccupancyQuadtree>(g_options.quadtree_resolution);
    std::cout << "OccupancyQuadtree Setting:" << std::endl << tree->GetSetting() << std::endl;

    // setup visualization
    auto drawer_setting = std::make_shared<erl::geometry::OccupancyQuadtree::Drawer::Setting>();
    drawer_setting->area_min = map_min;
    drawer_setting->area_max = map_max;
    drawer_setting->resolution = map_resolution[0];
    drawer_setting->padding = map_padding[0];
    drawer_setting->border_color = cv::Scalar(255, 0, 0, 255);
    std::cout << "OccupancyQuadtree::Drawer Setting:" << std::endl << *drawer_setting << std::endl;
    auto drawer = std::make_shared<erl::geometry::OccupancyQuadtree::Drawer>(drawer_setting, tree);
    auto grid_map_info = drawer->GetGridMapInfo();
    cv::Mat img;
    cv::Scalar trajectory_color(0, 0, 255, 255);
    drawer->DrawLeaves(img);
    cv::imshow(g_window_name, img);
    std::cout << "Press any key to start" << std::endl;
    cv::waitKey(0);

    // build occupancy quadtree
    double max_range = 100;
    bool parallel = false;  // bad, use more CPU cores but not faster
    bool discrete = false;
    bool pixel_based = true;
    max_update_cnt = long(buf_points.size());
    for (long i = 0; i < max_update_cnt; ++i) {
        auto t0 = std::chrono::high_resolution_clock::now();
        tree->InsertPointCloud(buf_points[i], trajectory.col(i), max_range, parallel, g_options.quadtree_lazy_eval, discrete);
        if (g_options.quadtree_lazy_eval) {
            tree->UpdateInnerOccupancy();
            tree->Prune();
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "update time: " << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms" << std::endl;

        drawer->DrawLeaves(img);
        erl::common::DrawTrajectoryInplace(img, trajectory.block(0, 0, 2, i), grid_map_info, trajectory_color, 2, pixel_based);
        cv::imshow(g_window_name, img);
        cv::waitKey(10);
    }
    std::cout << "Press any key to exit immediately. Test will exist in 10 seconds." << std::endl;
    cv::waitKey(10000);  // 10 seconds
}

int
main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    try {
        namespace po = boost::program_options;
        po::options_description desc;
        // clang-format off
        desc.add_options()
            ("help", "produce help message")
            ("use-gazebo-data", po::bool_switch(&g_options.use_gazebo_data)->default_value(g_options.use_gazebo_data), "Use Gazebo data")
            ("use-house-expo-data", po::bool_switch(&g_options.use_house_expo_data)->default_value(g_options.use_house_expo_data), "Use HouseExpo data")
            ("use-ros-bag-data", po::bool_switch(&g_options.use_ros_bag_data)->default_value(g_options.use_ros_bag_data), "Use ROS bag data")
            ("stride", po::value<int>(&g_options.stride)->default_value(g_options.stride), "stride for running the sequence")
            ("quadtree-resolution", po::value<double>(&g_options.quadtree_resolution)->default_value(g_options.quadtree_resolution), "Quadtree resolution")
            ("quadtree-lazy-eval", po::bool_switch(&g_options.quadtree_lazy_eval)->default_value(g_options.quadtree_lazy_eval), "Quadtree lazy evaluation")
            ("map-resolution", po::value<double>(&g_options.map_resolution)->default_value(g_options.map_resolution), "Map resolution")
            ("visualize", po::bool_switch(&g_options.visualize)->default_value(g_options.visualize), "Visualize the mapping")
            ("hold", po::bool_switch(&g_options.hold)->default_value(g_options.hold), "Hold the test until a key is pressed")
            (
                "house-expo-map-file",
                po::value<std::string>(&g_options.house_expo_map_file)->default_value(g_options.house_expo_map_file)->value_name("file"),
                "HouseExpo map file"
            )(
                "house-expo-traj-file",
                po::value<std::string>(&g_options.house_expo_traj_file)->default_value(g_options.house_expo_traj_file)->value_name("file"),
                "HouseExpo trajectory file"
            )(
                "gazebo-train-file",
                po::value<std::string>(&g_options.gazebo_train_file)->default_value(g_options.gazebo_train_file)->value_name("file"),
                "Gazebo train data file"
            )(
                "gazebo-test-file",
                po::value<std::string>(&g_options.gazebo_test_file)->default_value(g_options.gazebo_test_file)->value_name("file"),
                "Gazebo test data file"
            )(
                "ros-bag-csv-file",
                po::value<std::string>(&g_options.ros_bag_dat_file)->default_value(g_options.ros_bag_dat_file)->value_name("file"),
                "ROS bag csv file"
            );
        // clang-format on

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
        if (vm.count("help")) {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl << desc << std::endl;
            return 0;
        }
        po::notify(vm);
        if (g_options.use_gazebo_data + g_options.use_house_expo_data + g_options.use_ros_bag_data != 1) {
            std::cerr << "Please specify one of --use-gazebo-data, --use-house-expo-data, --use-ros-bag-data." << std::endl;
            return 1;
        }
    } catch (std::exception &e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    g_window_name = argv[0];
    return RUN_ALL_TESTS();
}
