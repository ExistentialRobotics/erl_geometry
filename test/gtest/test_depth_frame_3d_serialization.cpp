#include "erl_common/serialization.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/depth_frame_3d.hpp"

TEST(DepthFrame3D, Serialization) {
    using namespace erl::common;
    using namespace erl::geometry;

    std::filesystem::path depth_png = ERL_GEOMETRY_ROOT_DIR;
    depth_png /= "data/replica_office_0_depth/depth000000.png";

    auto depth_frame_setting = std::make_shared<DepthFrame3Dd::Setting>();
    depth_frame_setting->camera_intrinsic.image_height = 680;
    depth_frame_setting->camera_intrinsic.image_width = 1200;
    depth_frame_setting->camera_intrinsic.camera_fx = 600.0f;
    depth_frame_setting->camera_intrinsic.camera_fy = 600.0f;
    depth_frame_setting->camera_intrinsic.camera_cx = 599.5f;
    depth_frame_setting->camera_intrinsic.camera_cy = 339.5f;
    depth_frame_setting->row_margin = 0;
    depth_frame_setting->col_margin = 0;
    DepthFrame3Dd depth_frame(depth_frame_setting);

    cv::Mat depth_img = cv::imread(depth_png.string(), cv::IMREAD_UNCHANGED);
    depth_img.convertTo(depth_img, CV_64FC1);  // convert to double
    Eigen::MatrixXd depth;
    cv::cv2eigen(depth_img, depth);
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    depth_frame.UpdateRanges(rotation, translation, depth);

    EXPECT_TRUE(Serialization<DepthFrame3Dd>::Write("depth_frame.bin", depth_frame));

    auto depth_frame_setting_new = std::make_shared<DepthFrame3Dd::Setting>();
    DepthFrame3Dd depth_frame_new(depth_frame_setting_new);
    EXPECT_TRUE(Serialization<DepthFrame3Dd>::Read("depth_frame.bin", depth_frame_new));

    EXPECT_TRUE(depth_frame == depth_frame_new);
}
