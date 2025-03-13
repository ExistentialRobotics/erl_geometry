#include "erl_geometry/init.hpp"

#include "erl_geometry/abstract_octree_drawer.hpp"
#include "erl_geometry/abstract_quadtree_drawer.hpp"
#include "erl_geometry/depth_camera_3d.hpp"
#include "erl_geometry/depth_frame_3d.hpp"
#include "erl_geometry/lidar_3d.hpp"
#include "erl_geometry/lidar_frame_3d.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/occupancy_octree_base.hpp"
#include "erl_geometry/occupancy_octree_drawer.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/occupancy_quadtree_base.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"
#include "erl_geometry/range_sensor_frame_3d.hpp"
#include "erl_geometry/rgbd_camera_3d.hpp"
#include "erl_geometry/rgbd_frame_3d.hpp"

namespace erl::geometry {

#define REGISTER(x) (void) x::Register<x>()

    bool initialized = Init();

    bool
    Init() {
        static bool initialized_ = false;

        if (initialized_) { return true; }

        REGISTER(Lidar3Dd::Setting);
        REGISTER(Lidar3Df::Setting);
        REGISTER(LidarFrame3Dd);
        REGISTER(LidarFrame3Df);
        REGISTER(LidarFrame3Dd::Setting);
        REGISTER(LidarFrame3Df::Setting);

        REGISTER(CameraIntrinsicD);
        REGISTER(CameraIntrinsicF);
        // REGISTER(DepthCamera3Dd::Setting);  // CameraIntrinsicD
        // REGISTER(DepthCamera3Df::Setting);  // CameraIntrinsicF
        // REGISTER(RangeSensorFrame3Dd);  // abstract class
        // REGISTER(RangeSensorFrame3Df);  // abstract class
        REGISTER(RangeSensorFrame3Dd::Setting);
        REGISTER(RangeSensorFrame3Df::Setting);
        REGISTER(DepthFrame3Dd);
        REGISTER(DepthFrame3Df);
        REGISTER(DepthFrame3Dd::Setting);
        REGISTER(DepthFrame3Df::Setting);
        // REGISTER(RgbdCamera3Dd::Setting);  // CameraIntrinsicD
        // REGISTER(RgbdCamera3Df::Setting);  // CameraIntrinsicF
        REGISTER(RgbdFrame3Dd);
        REGISTER(RgbdFrame3Df);
        // REGISTER(RgbdFrame3Dd::Setting);  // DepthFrame3Dd::Setting
        // REGISTER(RgbdFrame3Df::Setting);  // DepthFrame3Df::Setting

        REGISTER(NdTreeSetting);
        REGISTER(OccupancyQuadtreeBaseSetting);
        REGISTER(OccupancyOctreeBaseSetting);
        REGISTER(OccupancyQuadtreeNode);
        REGISTER(OccupancyOctreeNode);
        REGISTER(OccupancyOctreeD);
        REGISTER(OccupancyOctreeF);
        REGISTER(OccupancyQuadtreeD);
        REGISTER(OccupancyQuadtreeF);

        REGISTER(AbstractQuadtreeDrawer::Setting);
        REGISTER(AbstractOctreeDrawer::Setting);
        REGISTER(OccupancyQuadtreeDrawerSettingD);
        REGISTER(OccupancyQuadtreeDrawerSettingF);
        REGISTER(OccupancyOctreeDrawerSetting);

        ERL_INFO("erl_geometry initialized");
        initialized_ = true;
        return true;
    }
}  // namespace erl::geometry
