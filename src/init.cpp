#include "erl_geometry/init.hpp"

#include "erl_geometry/depth_camera_3d.hpp"
#include "erl_geometry/depth_frame_3d.hpp"
#include "erl_geometry/lidar_3d.hpp"
#include "erl_geometry/lidar_frame_3d.hpp"
#include "erl_geometry/rgbd_camera_3d.hpp"

namespace erl::geometry {

#define REGISTER(x) (void) x::Register<x>()

    bool initialized = false;

    bool
    Init() {
        if (initialized) { return true; }
        REGISTER(Lidar3Dd::Setting);
        REGISTER(Lidar3Df::Setting);
        REGISTER(LidarFrame3Dd);
        REGISTER(LidarFrame3Df);
        REGISTER(LidarFrame3Dd::Setting);
        REGISTER(LidarFrame3Df::Setting);

        REGISTER(CameraIntrinsic_d);
        REGISTER(CameraIntrinsic_f);
        // REGISTER(DepthCamera3Dd::Setting);  // CameraIntrinsic_d
        // REGISTER(DepthCamera3Df::Setting);  // CameraIntrinsic_f
        REGISTER(DepthFrame3Dd);
        REGISTER(DepthFrame3Df);
        REGISTER(DepthFrame3Dd::Setting);
        REGISTER(DepthFrame3Df::Setting);
        // REGISTER(RgbdCamera3Dd::Setting);  // CameraIntrinsic_d
        // REGISTER(RgbdCamera3Df::Setting);  // CameraIntrinsic_f
        ERL_INFO("erl_geometry initialized");
        initialized = true;
        return true;
    }
}  // namespace erl::geometry
