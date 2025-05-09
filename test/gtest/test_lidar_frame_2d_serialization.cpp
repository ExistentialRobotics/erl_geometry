#include "erl_common/grid_map_info.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/house_expo_map.hpp"
#include "erl_geometry/lidar_2d.hpp"
#include "erl_geometry/lidar_frame_2d.hpp"

#include <filesystem>

TEST(LidarFrame2D, Serialization) {

    using namespace erl::common;
    using namespace erl::geometry;

    std::filesystem::path data_dir = ERL_GEOMETRY_ROOT_DIR;
    data_dir /= "data";

    constexpr double fov = 3 * M_PI / 2;  // 270 deg

    std::shared_ptr<erl::geometry::HouseExpoMap> map = std::make_shared<HouseExpoMap>(
        (data_dir / "house_expo_room_1451.json").string().c_str(),
        0.2);
    auto grid_map_info = std::make_shared<GridMapInfo2Dd>(
        map->GetMeterSpace()->GetSurface()->vertices.rowwise().minCoeff(),
        map->GetMeterSpace()->GetSurface()->vertices.rowwise().maxCoeff(),
        Eigen::Vector2d(0.01, 0.01),
        Eigen::Vector2i(10, 10));
    auto lidar_setting = std::make_shared<Lidar2D::Setting>();
    lidar_setting->min_angle = -fov / 2;
    lidar_setting->max_angle = fov / 2;
    lidar_setting->num_lines = static_cast<int>(std::round(fov / M_PI * 180)) + 1;
    auto lidar = std::make_shared<Lidar2D>(lidar_setting, map->GetMeterSpace());
    auto setting = std::make_shared<LidarFrame2Dd::Setting>();
    setting->angle_min = -fov / 2;
    setting->angle_max = fov / 2;
    setting->valid_range_min = 0.0;
    setting->valid_range_max = 30.0;
    setting->num_rays = lidar_setting->num_lines;
    auto lidar_frame = std::make_shared<LidarFrame2Dd>(setting);

    {
        ASSERT_TRUE(Serialization<LidarFrame2Dd>::Write("lidar_frame_2d.bin", *lidar_frame));
        LidarFrame2Dd lidar_frame_read(std::make_shared<LidarFrame2Dd::Setting>());
        ASSERT_TRUE(Serialization<LidarFrame2Dd>::Read("lidar_frame_2d.bin", lidar_frame_read));
        EXPECT_TRUE(*lidar_frame == lidar_frame_read);
    }

    const Eigen::Matrix2d rotation = Eigen::Matrix2d::Identity();
    const Eigen::Vector2d position = grid_map_info->Center();
    lidar_frame->UpdateRanges(rotation, position, lidar->Scan(rotation, position, true));

    {
        ASSERT_TRUE(Serialization<LidarFrame2Dd>::Write("lidar_frame_2d.bin", *lidar_frame));
        LidarFrame2Dd lidar_frame_read(std::make_shared<LidarFrame2Dd::Setting>());
        ASSERT_TRUE(Serialization<LidarFrame2Dd>::Read("lidar_frame_2d.bin", lidar_frame_read));
        EXPECT_TRUE(*lidar_frame == lidar_frame_read);
    }

    lidar_frame->PartitionRays();
    {
        ASSERT_TRUE(Serialization<LidarFrame2Dd>::Write("lidar_frame_2d.bin", *lidar_frame));
        LidarFrame2Dd lidar_frame_read(std::make_shared<LidarFrame2Dd::Setting>());
        ASSERT_TRUE(Serialization<LidarFrame2Dd>::Read("lidar_frame_2d.bin", lidar_frame_read));
        EXPECT_TRUE(*lidar_frame == lidar_frame_read);
    }
}
