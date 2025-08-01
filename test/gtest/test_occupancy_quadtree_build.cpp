#include "erl_common/csv.hpp"
#include "erl_common/progress_bar.hpp"
#include "erl_common/random.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/gazebo_room_2d.hpp"
#include "erl_geometry/house_expo_map.hpp"
#include "erl_geometry/lidar_2d.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"

#include <boost/program_options.hpp>

using Dtype = double;
using OccupancyQuadtree = erl::geometry::OccupancyQuadtree<Dtype>;
using OccupancyQuadtreeDrawer = erl::geometry::OccupancyQuadtreeDrawer<OccupancyQuadtree>;
using Vector2 = Eigen::Vector2<Dtype>;
using VectorX = Eigen::VectorX<Dtype>;
using Matrix2X = Eigen::Matrix2X<Dtype>;
using MatrixX = Eigen::MatrixX<Dtype>;
using Matrix2 = Eigen::Matrix2<Dtype>;

const std::filesystem::path kProjectRootDir = ERL_GEOMETRY_ROOT_DIR;
const std::filesystem::path kDataDir = kProjectRootDir / "data";
const std::filesystem::path kConfigDir = kProjectRootDir / "config";
static std::string g_window_name = "OccupancyQuadtree_Build";

struct Options {
    std::string gazebo_train_folder = kDataDir / "gazebo";
    std::string gazebo_test_file = kDataDir / "gazebo_test.dat";
    std::string house_expo_map_file = kDataDir / "house_expo_room_1451.json";
    std::string house_expo_traj_file = kDataDir / "house_expo_room_1451.csv";
    std::string ucsd_fah_2d_file = kDataDir / "ucsd_fah_2d.dat";
    bool use_gazebo_room_2d = true;
    bool use_house_expo_lidar_2d = false;
    bool use_ucsd_fah_2d = false;
    bool hold = false;
    int stride = 1;
    Dtype quadtree_resolution = 0.05;
    bool quadtree_lazy_eval = false;  // bad, slower
    Dtype map_resolution = 0.025;
};

static Options g_options;

TEST(OccupancyQuadtree, Build) {
    GTEST_PREPARE_OUTPUT_DIR();

    long max_update_cnt;
    Vector2 map_min(0, 0);
    Vector2 map_max(0, 0);
    Vector2 map_resolution(g_options.map_resolution, g_options.map_resolution);
    Eigen::Vector2i map_padding(10, 10);
    std::vector<Matrix2X> buf_points;
    Eigen::Matrix2Xd trajectory;

    auto load_scan = [&](const Matrix2 &rotation,
                         const Vector2 &translation,
                         const VectorX &angles,
                         const VectorX &ranges) {
        const long n = ranges.size();
        Matrix2X points(2, n);
        long cnt = 0;
        for (long k = 0; k < n; ++k) {
            const Dtype &angle = angles[k];
            const Dtype &range = ranges[k];
            if (!std::isfinite(range)) { continue; }
            Vector2 local_pt(std::cos(angle) * range, std::sin(angle) * range);
            Vector2 global_pt = rotation * local_pt + translation;
            points.col(cnt) = global_pt;
            cnt++;

            const Dtype &x = global_pt[0];
            const Dtype &y = global_pt[1];
            if (x < map_min[0]) { map_min[0] = x; }
            if (x > map_max[0]) { map_max[0] = x; }
            if (y < map_min[1]) { map_min[1] = y; }
            if (y > map_max[1]) { map_max[1] = y; }
        }
        points.conservativeResize(2, cnt);
        return points;
    };

    std::string tree_name;
    if (g_options.use_gazebo_room_2d) {
        tree_name = "gazebo";
        // load raw data
        auto train_data_loader =
            erl::geometry::GazeboRoom2D::TrainDataLoader(g_options.gazebo_train_folder);
        // prepare buffer
        max_update_cnt = static_cast<long>(train_data_loader.size()) / g_options.stride + 1;
        buf_points.reserve(max_update_cnt);
        trajectory.resize(2, max_update_cnt);
        // load data into buffer
        long j = 0;
        std::shared_ptr<erl::common::ProgressBar> bar = erl::common::ProgressBar::Open();
        bar->SetTotal(max_update_cnt);
        for (long i = 0; i < train_data_loader.size(); i += g_options.stride) {
            auto &df = train_data_loader[i];
            trajectory.col(j) << df.translation;
            buf_points.push_back(load_scan(
                df.rotation.cast<Dtype>(),
                trajectory.col(j).cast<Dtype>(),
                df.angles.cast<Dtype>(),
                df.ranges.cast<Dtype>()));
            j++;
            bar->Update();
        }
        bar->Close();
        trajectory.conservativeResize(2, j);
    } else if (g_options.use_house_expo_lidar_2d) {
        tree_name =
            std::filesystem::path(g_options.house_expo_map_file).stem().filename().string() + "_2d";
        // load raw data
        erl::geometry::HouseExpoMap house_expo_map(g_options.house_expo_map_file, 0.2);
        auto caster = [](const std::string &str) { return static_cast<Dtype>(std::stod(str)); };
        auto csv_trajectory =
            erl::common::LoadAndCastCsvFile<Dtype>(g_options.house_expo_traj_file.c_str(), caster);
        // prepare buffer
        max_update_cnt = static_cast<long>(csv_trajectory.size()) / g_options.stride + 1;
        buf_points.reserve(max_update_cnt);
        trajectory.resize(2, max_update_cnt);
        // setup lidar
        auto lidar_setting = std::make_shared<erl::geometry::Lidar2D::Setting>();
        lidar_setting->num_lines = 720;
        erl::geometry::Lidar2D lidar(lidar_setting, house_expo_map.GetMeterSpace());
        VectorX lidar_angles = lidar.GetAngles().cast<Dtype>();
        // load data into buffer
        long j = 0;
        std::shared_ptr<erl::common::ProgressBar> bar = erl::common::ProgressBar::Open();
        bar->SetTotal(max_update_cnt);
        for (std::size_t i = 0; i < csv_trajectory.size(); i += g_options.stride) {
            trajectory.col(j) << csv_trajectory[i][0], csv_trajectory[i][1];
            Eigen::Matrix2d rotation = Eigen::Rotation2Dd(csv_trajectory[i][2]).toRotationMatrix();
            VectorX lidar_ranges =
                lidar.Scan(rotation, trajectory.col(j), /*scan_in_parallel*/ true).cast<Dtype>();
            lidar_ranges +=
                erl::common::GenerateGaussianNoise<Dtype>(lidar_ranges.size(), 0.0, 0.01);
            buf_points.push_back(load_scan(
                rotation.cast<Dtype>(),
                trajectory.col(j).cast<Dtype>(),
                lidar_angles,
                lidar_ranges));
            j++;
            bar->Update();
        }
        bar->Close();
        trajectory.conservativeResize(2, j);
    } else {
        tree_name = "ros_bag";
        Eigen::MatrixXd ros_bag_data =
            erl::common::LoadEigenMatrixFromBinaryFile<double, Eigen::Dynamic, Eigen::Dynamic>(
                g_options.ucsd_fah_2d_file);
        // prepare buffer
        max_update_cnt = ros_bag_data.cols() / g_options.stride + 1;
        buf_points.reserve(max_update_cnt);
        trajectory.resize(2, max_update_cnt);
        // load data into buffer
        long num_rays = (ros_bag_data.rows() - 7) / 2;
        long j = 0;
        std::shared_ptr<erl::common::ProgressBar> bar = erl::common::ProgressBar::Open();
        bar->SetTotal(max_update_cnt);
        for (long i = 0; i < ros_bag_data.cols(); i += g_options.stride, j++) {
            auto pose =
                Eigen::Map<Eigen::Matrix23d>(ros_bag_data.col(i).segment(1, 6).data(), 2, 3);
            trajectory.col(j) << pose(0, 2), pose(1, 2);
            Eigen::VectorXd angles = ros_bag_data.col(i).segment(7, num_rays);
            Eigen::VectorXd ranges = ros_bag_data.col(i).segment(7 + num_rays, num_rays);
            buf_points.push_back(load_scan(
                pose.block<2, 2>(0, 0).cast<Dtype>(),
                trajectory.col(j).cast<Dtype>(),
                angles.cast<Dtype>(),
                ranges.cast<Dtype>()));
            bar->Update();
        }
        bar->Close();
        trajectory.conservativeResize(2, j);
    }

    // initialize occupancy quadtree
    auto tree_setting = std::make_shared<OccupancyQuadtree::Setting>();
    tree_setting->resolution = static_cast<float>(g_options.quadtree_resolution);
    auto tree = std::make_shared<OccupancyQuadtree>(tree_setting);
    std::cout << "OccupancyQuadtree Setting:" << std::endl
              << tree->GetSetting<OccupancyQuadtree::Setting>() << std::endl;

    // setup visualization
    auto drawer_setting = std::make_shared<OccupancyQuadtreeDrawer::Setting>();
    drawer_setting->area_min = map_min;
    drawer_setting->area_max = map_max;
    drawer_setting->resolution = map_resolution[0];
    drawer_setting->padding = map_padding[0];
    drawer_setting->border_color = cv::Scalar(255, 0, 0, 255);
    std::cout << "OccupancyQuadtree::Drawer Setting:" << std::endl << *drawer_setting << std::endl;
    auto drawer = std::make_shared<OccupancyQuadtreeDrawer>(drawer_setting, tree);
    auto grid_map_info = drawer->GetGridMapInfo();
    cv::Mat img;
    cv::Scalar trajectory_color(0, 0, 255, 255);
    drawer->DrawLeaves(img);
    cv::imshow(g_window_name, img);
    if (g_options.hold) {
        std::cout << "Press any key to start" << std::endl;
        cv::waitKey(0);
    }

    // build occupancy quadtree
    max_update_cnt = static_cast<long>(buf_points.size());
    for (long i = 0; i < max_update_cnt; ++i) {
        auto t0 = std::chrono::high_resolution_clock::now();
        tree->InsertPointCloud(
            buf_points[i],
            trajectory.col(i).cast<Dtype>(),
            /*max_range*/ 100,
            /*parallel*/ false,
            g_options.quadtree_lazy_eval,
            /*discrete*/ false);
        if (g_options.quadtree_lazy_eval) {
            tree->UpdateInnerOccupancy();
            tree->Prune();
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "update time: " << std::chrono::duration<Dtype, std::milli>(t1 - t0).count()
                  << " ms" << std::endl;

        drawer->DrawLeaves(img);
        erl::common::DrawTrajectoryInplace<Dtype>(
            img,
            trajectory.block(0, 0, 2, i).cast<Dtype>(),
            grid_map_info->CastSharedPtr<Dtype>(),
            trajectory_color,
            2,
            /*pixel_based*/ true);
        cv::imshow(g_window_name, img);
        cv::waitKey(10);
    }
    EXPECT_TRUE(erl::common::Serialization<OccupancyQuadtree>::Write(
        test_output_dir / fmt::format("{}_{}.ot", tree_name, type_name<Dtype>()),
        tree));
    EXPECT_TRUE(erl::common::Serialization<OccupancyQuadtree>::Write(
        test_output_dir / fmt::format("{}_{}.bt", tree_name, type_name<Dtype>()),
        [&](std::ostream &s) -> bool { return tree->WriteBinary(s, true); }));
    if (g_options.hold) {
        std::cout << "Press any key to exit." << std::endl;
        cv::waitKey(0);
    } else {
        std::cout << "Press any key to exit now. Or test will exist in 10 seconds." << std::endl;
        cv::waitKey(10000);  // 10 seconds
    }
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
            ("use-gazebo-data", po::bool_switch(&g_options.use_gazebo_room_2d)->default_value(g_options.use_gazebo_room_2d), "Use Gazebo data")
            ("use-house-expo-data", po::bool_switch(&g_options.use_house_expo_lidar_2d)->default_value(g_options.use_house_expo_lidar_2d), "Use HouseExpo data")
            ("use-ros-bag-data", po::bool_switch(&g_options.use_ucsd_fah_2d)->default_value(g_options.use_ucsd_fah_2d), "Use ROS bag data")
            ("stride", po::value<int>(&g_options.stride)->default_value(g_options.stride), "stride for running the sequence")
            ("quadtree-resolution", po::value<Dtype>(&g_options.quadtree_resolution)->default_value(g_options.quadtree_resolution), "Quadtree resolution")
            ("quadtree-lazy-eval", po::bool_switch(&g_options.quadtree_lazy_eval)->default_value(g_options.quadtree_lazy_eval), "Quadtree lazy evaluation")
            ("map-resolution", po::value<Dtype>(&g_options.map_resolution)->default_value(g_options.map_resolution), "Map resolution")
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
                po::value<std::string>(&g_options.gazebo_train_folder)->default_value(g_options.gazebo_train_folder)->value_name("file"),
                "Gazebo train data file"
            )(
                "gazebo-test-file",
                po::value<std::string>(&g_options.gazebo_test_file)->default_value(g_options.gazebo_test_file)->value_name("file"),
                "Gazebo test data file"
            )(
                "ros-bag-csv-file",
                po::value<std::string>(&g_options.ucsd_fah_2d_file)->default_value(g_options.ucsd_fah_2d_file)->value_name("file"),
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
        if (g_options.use_gazebo_room_2d + g_options.use_house_expo_lidar_2d +
                g_options.use_ucsd_fah_2d !=
            1) {
            std::cerr << "Please specify one of --use-gazebo-data, --use-house-expo-data, "
                         "--use-ros-bag-data."
                      << std::endl;
            return 1;
        }
    } catch (std::exception &e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    g_window_name = argv[0];
    return RUN_ALL_TESTS();
}
