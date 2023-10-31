#include <filesystem>

#include "erl_common/grid_map_info.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/house_expo.hpp"
#include "erl_geometry/lidar_2d.hpp"
#include "erl_geometry/lidar_frame_2d.hpp"

struct UserData {
    inline static const char *window_name1 = "test lidar frame 2d: map";
    inline static const char *window_name2 = "test lidar frame 2d: lidar frame";
    std::shared_ptr<erl::geometry::HouseExpoMap> map = nullptr;
    std::shared_ptr<erl::common::GridMapInfo2D> grid_map_info = nullptr;
    std::shared_ptr<erl::geometry::Lidar2D> lidar = nullptr;
    std::shared_ptr<erl::geometry::LidarFrame2D> lidar_frame = nullptr;
    double fov = 3 * M_PI / 2;  // 270 deg
    cv::Mat map_image = {};
    cv::Mat image1 = {};
    cv::Mat image2 = {};
    cv::Mat image2_bk = {};
    bool quit = false;
};

void
MouseCallback1(int event, int mouse_x, int mouse_y, int flags, void *userdata) {
    (void) flags;
    auto data = reinterpret_cast<UserData *>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN) {
        data->lidar->SetTranslation(data->grid_map_info->PixelToMeterForPoints(Eigen::Vector2i(mouse_x, mouse_y)));
        data->lidar_frame->Update(data->lidar->GetRotation(), data->lidar->GetTranslation(), data->lidar->GetAngles(), data->lidar->Scan(true));
        data->image1 = data->map_image.clone();
        std::vector<cv::Point2i> points;
        points.reserve(data->lidar_frame->GetNumRays() * 2);
        for (int i = 0; i < data->lidar_frame->GetNumRays(); ++i) {
            points.emplace_back(mouse_x, mouse_y);
            Eigen::Vector2i end_pt = data->grid_map_info->MeterToPixelForPoints(data->lidar_frame->GetEndPointsInWorld().col(i));
            points.emplace_back(end_pt[0], end_pt[1]);
        }
        cv::polylines(data->image1, points, false, cv::Scalar(0, 255, 0), 1);
        cv::circle(data->image1, cv::Point(mouse_x, mouse_y), 2, cv::Scalar(0, 0, 255), cv::FILLED);
        cv::imshow(UserData::window_name1, data->image1);

        data->image2 = cv::Mat(data->map_image.rows, data->map_image.cols, CV_8UC3, cv::Scalar(255, 255, 255));
        points.clear();
        points.reserve(data->lidar_frame->GetNumRays() + 2);
        points.emplace_back(mouse_x, mouse_y);
        for (int i = 0; i < data->lidar_frame->GetNumRays(); ++i) {
            Eigen::Vector2i end_pt = data->grid_map_info->MeterToPixelForPoints(data->lidar_frame->GetEndPointsInWorld().col(i));
            points.emplace_back(end_pt[0], end_pt[1]);
        }
        points.emplace_back(mouse_x, mouse_y);
        cv::polylines(data->image2, points, false, cv::Scalar(0, 0, 0), 2);
        cv::circle(data->image2, cv::Point(mouse_x, mouse_y), 2, cv::Scalar(0, 0, 255), cv::FILLED);
        data->image2_bk = data->image2.clone();
        cv::imshow(UserData::window_name2, data->image2);
        return;
    }
}

void
MouseCallback2(int event, int mouse_x, int mouse_y, int flags, void *userdata) {
    (void) flags;
    auto data = reinterpret_cast<UserData *>(userdata);
    if (!data->lidar_frame->IsValid()) { return; }
    static bool mouse_fixed = false;

    if (event == cv::EVENT_LBUTTONDOWN) {
        mouse_fixed = !mouse_fixed;
        return;
    }

    if (event == cv::EVENT_MOUSEMOVE) {
        if (mouse_fixed) { return; }
        data->image2 = data->image2_bk.clone();
        Eigen::Vector2d position = data->grid_map_info->PixelToMeterForPoints(Eigen::Vector2i(mouse_x, mouse_y));
        Eigen::Matrix2Xd directions;
        Eigen::VectorXd distances;
        data->lidar_frame->ComputeRaysAt(position, directions, distances);
        std::vector<cv::Point2i> points;
        points.reserve(distances.size() * 2);
        points.emplace_back(mouse_x, mouse_y);
        for (int i = 0; i < distances.size(); ++i) {
            points.emplace_back(mouse_x, mouse_y);
            Eigen::Vector2i end_pt = data->grid_map_info->MeterToPixelForPoints(position + distances[i] * directions.col(i));
            points.emplace_back(end_pt[0], end_pt[1]);
        }
        cv::polylines(data->image2, points, false, cv::Scalar(255, 0, 0), 1);
        cv::imshow(UserData::window_name2, data->image2);
    }
}

TEST(ERL_GEOMETRY, LidarFrame2D) {

    using namespace erl::common;
    using namespace erl::geometry;

    std::filesystem::path data_dir = __FILE__;
    data_dir = data_dir.parent_path();

    UserData data;
    data.map = std::make_shared<HouseExpoMap>((data_dir / "house_expo_room_1451.json").string().c_str(), 0.2);
    data.grid_map_info = std::make_shared<GridMapInfo2D>(
        data.map->GetMeterSpace()->GetSurface()->vertices.rowwise().minCoeff(),
        data.map->GetMeterSpace()->GetSurface()->vertices.rowwise().maxCoeff(),
        Eigen::Vector2d(0.01, 0.01),
        Eigen::Vector2i(10, 10));
    data.lidar = std::make_shared<Lidar2D>(data.map->GetMeterSpace());
    data.lidar->SetMinAngle(-data.fov / 2);
    data.lidar->SetMaxAngle(data.fov / 2);
    data.lidar->SetNumLines(int(std::round(data.fov / M_PI * 180)));
    auto setting = std::make_shared<LidarFrame2D::Setting>();
    setting->valid_angle_min = -data.fov / 2;
    setting->valid_angle_max = data.fov / 2;
    setting->valid_range_min = 0.0;
    setting->valid_range_max = 30.0;
    data.lidar_frame = std::make_shared<LidarFrame2D>(setting);
    auto map_image = data.map->GetMeterSpace()->GenerateMapImage(*data.grid_map_info);
    cv::eigen2cv(map_image, data.map_image);
    cv::cvtColor(data.map_image, data.map_image, cv::COLOR_GRAY2BGR);
    cv::imshow(UserData::window_name1, data.map_image);
    data.image2 = cv::Mat(data.map_image.rows, data.map_image.cols, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::imshow(UserData::window_name2, data.image2);
    cv::setMouseCallback(UserData::window_name1, MouseCallback1, &data);
    cv::setMouseCallback(UserData::window_name2, MouseCallback2, &data);
    while (!data.quit) {
        int key = cv::waitKey(0);
        if (key == 'q') { data.quit = true; }
    }
}
