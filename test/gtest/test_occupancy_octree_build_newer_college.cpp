#include "erl_common/block_timer.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/newer_college.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/occupancy_octree_drawer.hpp"
#include "erl_geometry/open3d_visualizer_wrapper.hpp"
#include "erl_geometry/utils.hpp"

#include <open3d/geometry/VoxelGrid.h>
#include <open3d/io/TriangleMeshIO.h>

#include <filesystem>

// parameters
#define WINDOW_NAME          "OccupancyOctree_Build"
#define OCTREE_RESOLUTION    0.1
#define ANIMATION_INTERVAL   2
#define MAX_POINT_CLOUD_SIZE 1000000
#define STRIDE               1

using Dtype = float;
using AbstractOctree = erl::geometry::AbstractOctree<Dtype>;
using OccupancyOctree = erl::geometry::OccupancyOctree<Dtype>;
using OccupancyOctreeNode = erl::geometry::OccupancyOctreeNode;
using OccupancyOctreeDrawer = erl::geometry::OccupancyOctreeDrawer<OccupancyOctree>;
using Open3dVisualizerWrapper = erl::geometry::Open3dVisualizerWrapper;
using VectorX = Eigen::VectorX<Dtype>;
using Vector3 = Eigen::Vector3<Dtype>;
using MatrixX = Eigen::MatrixX<Dtype>;
using Matrix3 = Eigen::Matrix3<Dtype>;
using Matrix3X = Eigen::Matrix3X<Dtype>;
using Matrix4 = Eigen::Matrix4<Dtype>;

TEST(OccupancyOctree, BuildNewerCollege) {
    GTEST_PREPARE_OUTPUT_DIR();
    using namespace erl::common;

    erl::geometry::NewerCollege dataset("/home/daizhirui/Data/NewerCollege");

    auto octree_setting = std::make_shared<OccupancyOctree::Setting>();
    ASSERT_TRUE(
        octree_setting->FromYamlFile(ERL_GEOMETRY_ROOT_DIR "/data/octree_newer_college.yaml"));
    octree_setting->use_change_detection = true;

    auto octree = std::make_shared<OccupancyOctree>(octree_setting);
    const auto visualizer_setting = std::make_shared<Open3dVisualizerWrapper::Setting>();
    visualizer_setting->window_name = WINDOW_NAME;
    visualizer_setting->mesh_show_back_face = false;
    Open3dVisualizerWrapper visualizer(visualizer_setting);
    auto point_cloud = std::make_shared<open3d::geometry::PointCloud>();
    auto line_set_traj = std::make_shared<open3d::geometry::LineSet>();
    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries =
        OccupancyOctreeDrawer::GetBlankGeometries();
    geometries.push_back(point_cloud);
    geometries.push_back(line_set_traj);
    visualizer.AddGeometries(geometries);

    auto drawer_setting = std::make_shared<OccupancyOctreeDrawer::Setting>();
    drawer_setting->area_min = dataset.GetMapMin();
    drawer_setting->area_max = dataset.GetMapMax();
    drawer_setting->occupied_only = true;
    OccupancyOctreeDrawer drawer(drawer_setting);
    drawer.SetOctree(octree);

    std::size_t idx = 0;
    bool octree_saved = false;
    int animation_cnt = 0;
    double mean_insert_time = 0;
    auto callback = [&](Open3dVisualizerWrapper *wrapper, open3d::visualization::Visualizer *vis) {
        if (idx >= static_cast<std::size_t>(dataset.Size())) {
            if (octree_saved) {
                ERL_WARN_ONCE("callback is still called after octree is saved.");
                return false;
            }
            EXPECT_TRUE(
                Serialization<OccupancyOctree>::Write(
                    test_output_dir / "newer_college.ot",
                    octree));
            EXPECT_TRUE(
                Serialization<OccupancyOctree>::Write(
                    test_output_dir / "newer_college.bt",
                    [&](std::ostream &s) -> bool { return octree->WriteBinary(s); }));
            octree_saved = true;
            wrapper->ClearGeometries();

            drawer.DrawLeaves(geometries);
            geometries.push_back(point_cloud);
            geometries.push_back(line_set_traj);
            wrapper->AddGeometries(geometries);
            vis->UpdateGeometry();
            wrapper->SetAnimationCallback(nullptr);  // stop calling this callback
            return false;
        }

        const auto t_start = std::chrono::high_resolution_clock::now();
        auto frame = dataset[idx];
        idx += STRIDE;

        std::cout << "==== " << idx << " ====" << std::endl;
        double dt = 0;
        line_set_traj->points_.emplace_back(frame.translation.cast<double>());
        if (line_set_traj->points_.size() > 1) {
            line_set_traj->lines_.emplace_back(
                line_set_traj->points_.size() - 2,
                line_set_traj->points_.size() - 1);
        }

        Eigen::Matrix3Xd points_in_world = frame.GetPointsInWorldFrame();
        point_cloud->points_.clear();
        point_cloud->points_.reserve(points_in_world.cols());
        for (int i = 0; i < points_in_world.cols(); ++i) {
            point_cloud->points_.emplace_back(points_in_world.col(i));
        }

        octree->ClearChangedKeys();
        {
            ERL_BLOCK_TIMER_MSG_TIME("Insert time", dt);
            octree->InsertPointCloud(
                points_in_world.cast<Dtype>(),
                frame.translation.cast<Dtype>(),
                0.6,
                50.0,
                false,
                true,
                true);
            octree->UpdateInnerOccupancy();
            octree->Prune();
        }
        mean_insert_time = (mean_insert_time * animation_cnt + dt) / (animation_cnt + 1);
        std::cout << "Number of points: " << frame.points.cols() << std::endl;
        std::cout << "Mean insert time: " << mean_insert_time << " ms." << std::endl;

        drawer.DrawLeaves(geometries);

        line_set_traj->PaintUniformColor({1, 0, 0});
        if (line_set_traj->lines_.empty()) { vis->ResetViewPoint(true); }
        if (point_cloud->points_.size() > MAX_POINT_CLOUD_SIZE) {
            point_cloud->points_.swap(point_cloud->RandomDownSample(0.5)->points_);
        }

        const auto t_end = std::chrono::high_resolution_clock::now();
        const auto duration_total =
            std::chrono::duration<double, std::milli>(t_end - t_start).count();
        std::cout << "Callback time: " << duration_total << " ms." << std::endl;

        return animation_cnt++ % ANIMATION_INTERVAL == 0;
    };

    visualizer.SetAnimationCallback(callback);
    visualizer.Show();
}
