#include "erl_common/angle_utils.hpp"
#include "erl_common/serialization.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/lidar_3d.hpp"
#include "erl_geometry/lidar_frame_3d.hpp"

#include <boost/program_options.hpp>

std::filesystem::path project_dir = ERL_GEOMETRY_ROOT_DIR;

struct Options {
    std::string window_name = "LidarFrame3D";
    std::string ply_file = (project_dir / "data/house_expo_room_1451.ply").string();
    double lidar_elevation_min = -30;
    double lidar_elevation_max = 30;
    int lidar_num_elevation_lines = 61;
};

Options g_user_data;

TEST(LidarFrame3D, Serialization) {
    using namespace erl::common;
    using namespace erl::geometry;
    std::cout << "ply_file: " << g_user_data.ply_file << std::endl
              << "lidar_elevation_min: " << g_user_data.lidar_elevation_min << std::endl
              << "lidar_elevation_max: " << g_user_data.lidar_elevation_max << std::endl
              << "lidar_num_elevation_lines: " << g_user_data.lidar_num_elevation_lines
              << std::endl;

    auto room_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    {
        open3d::io::ReadTriangleMeshOptions options;
        options.enable_post_processing = true;
        open3d::io::ReadTriangleMesh(g_user_data.ply_file, *room_mesh, options);
        room_mesh->ComputeTriangleNormals();
    }
    auto o3d_scene = std::make_shared<open3d::t::geometry::RaycastingScene>();
    o3d_scene->AddTriangles(open3d::t::geometry::TriangleMesh::FromLegacy(*room_mesh));

    auto lidar_3d_setting = std::make_shared<Lidar3Dd::Setting>();
    lidar_3d_setting->elevation_min = DegreeToRadian(g_user_data.lidar_elevation_min);
    lidar_3d_setting->elevation_max = DegreeToRadian(g_user_data.lidar_elevation_max);
    lidar_3d_setting->num_elevation_lines = g_user_data.lidar_num_elevation_lines;
    auto lidar_3d = std::make_shared<Lidar3Dd>(lidar_3d_setting);
    lidar_3d->AddMesh(room_mesh->vertices_, room_mesh->triangles_);

    auto lidar_frame_3d_setting = std::make_shared<LidarFrame3Dd::Setting>();
    lidar_frame_3d_setting->azimuth_min = lidar_3d_setting->azimuth_min;
    lidar_frame_3d_setting->azimuth_max = lidar_3d_setting->azimuth_max;
    lidar_frame_3d_setting->elevation_min = lidar_3d_setting->elevation_min;
    lidar_frame_3d_setting->elevation_max = lidar_3d_setting->elevation_max;
    lidar_frame_3d_setting->num_azimuth_lines = lidar_3d_setting->num_azimuth_lines;
    lidar_frame_3d_setting->num_elevation_lines = lidar_3d_setting->num_elevation_lines;
    auto lidar_frame_3d = std::make_shared<LidarFrame3Dd>(lidar_frame_3d_setting);

    {
        ASSERT_TRUE(Serialization<LidarFrame3Dd>::Write("lidar_frame_3d.bin", lidar_frame_3d));
        LidarFrame3Dd lidar_frame_3d_read(std::make_shared<LidarFrame3Dd::Setting>());
        ASSERT_TRUE(Serialization<LidarFrame3Dd>::Read("lidar_frame_3d.bin", &lidar_frame_3d_read));
        EXPECT_TRUE(*lidar_frame_3d == lidar_frame_3d_read);
    }

    const Eigen::Vector3d lidar_position = room_mesh->GetCenter();
    const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    const Eigen::MatrixXd ranges = lidar_3d->Scan(rotation, lidar_position);
    lidar_frame_3d->UpdateRanges(rotation, lidar_position, ranges);

    {
        ASSERT_TRUE(Serialization<LidarFrame3Dd>::Write("lidar_frame_3d.bin", lidar_frame_3d));
        LidarFrame3Dd lidar_frame_3d_read(std::make_shared<LidarFrame3Dd::Setting>());
        ASSERT_TRUE(Serialization<LidarFrame3Dd>::Read("lidar_frame_3d.bin", &lidar_frame_3d_read));
        EXPECT_TRUE(*lidar_frame_3d == lidar_frame_3d_read);
    }
}

int
main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    try {
        namespace po = boost::program_options;
        po::options_description desc;
        // clang-format off
        desc.add_options()
            ("help", "produce help message")
            ("ply-file", po::value<std::string>(&g_user_data.ply_file)->default_value(g_user_data.ply_file), "ply file path")
            ("lidar-elevation-min", po::value<double>(&g_user_data.lidar_elevation_min)->default_value(g_user_data.lidar_elevation_min), "lidar elevation min")
            ("lidar-elevation-max", po::value<double>(&g_user_data.lidar_elevation_max)->default_value(g_user_data.lidar_elevation_max), "lidar elevation max")
            ("lidar-num-elevation-lines", po::value<int>(&g_user_data.lidar_num_elevation_lines)->default_value(g_user_data.lidar_num_elevation_lines), "lidar num elevation lines");
        // clang-format on

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);

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
