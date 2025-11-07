#include "erl_common/block_timer.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/cow_and_lady.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/occupancy_octree_drawer.hpp"
#include "erl_geometry/open3d_visualizer_wrapper.hpp"

#include <open3d/geometry/VoxelGrid.h>
#include <open3d/io/TriangleMeshIO.h>

#include <filesystem>

// parameters
#define WINDOW_NAME "OccupancyOctree_Build"

struct Options : public erl::common::Yamlable<Options> {
    int animation_interval = 2;
    std::size_t max_point_cloud_size = 1000000;
    int stride = 1;
    float scaling = 1.0f;
    float min_range = 0.6f;
    float max_range = 35.0f;
    bool draw_tree_grid = false;

    ERL_REFLECT_SCHEMA(
        Options,
        ERL_REFLECT_MEMBER(Options, animation_interval),
        ERL_REFLECT_MEMBER(Options, max_point_cloud_size),
        ERL_REFLECT_MEMBER(Options, stride),
        ERL_REFLECT_MEMBER(Options, scaling),
        ERL_REFLECT_MEMBER(Options, min_range),
        ERL_REFLECT_MEMBER(Options, max_range),
        ERL_REFLECT_MEMBER(Options, draw_tree_grid));
};

Options options;

using Dtype = float;
using AbstractOctree = erl::geometry::AbstractOctree<Dtype>;
using OccupancyOctree = erl::geometry::OccupancyOctree<Dtype>;
using OccupancyOctreeNode = erl::geometry::OccupancyOctreeNode;
using OccupancyOctreeDrawer = erl::geometry::OccupancyOctreeDrawer<OccupancyOctree>;
using Open3dVisualizerWrapper = erl::geometry::Open3dVisualizerWrapper;
using CowAndLady = erl::geometry::CowAndLady;
using DepthFrame3D = erl::geometry::DepthFrame3D<Dtype>;
using VectorX = Eigen::VectorX<Dtype>;
using Vector3 = Eigen::Vector3<Dtype>;
using MatrixX = Eigen::MatrixX<Dtype>;
using Matrix3 = Eigen::Matrix3<Dtype>;
using Matrix3X = Eigen::Matrix3X<Dtype>;
using Matrix4 = Eigen::Matrix4<Dtype>;

TEST(OccupancyOctree, BuildCowAndLady) {
    GTEST_PREPARE_OUTPUT_DIR();
    using namespace erl::common;
    using namespace erl::common::serialization;

    CowAndLady dataset("/home/daizhirui/Data/CowAndLady");

    const auto depth_frame_setting = std::make_shared<DepthFrame3D::Setting>();
    depth_frame_setting->camera_intrinsic.image_height = CowAndLady::kImageHeight;
    depth_frame_setting->camera_intrinsic.image_width = CowAndLady::kImageWidth;
    depth_frame_setting->camera_intrinsic.camera_fx = CowAndLady::kCameraFx;
    depth_frame_setting->camera_intrinsic.camera_fy = CowAndLady::kCameraFy;
    depth_frame_setting->camera_intrinsic.camera_cx = CowAndLady::kCameraCx;
    depth_frame_setting->camera_intrinsic.camera_cy = CowAndLady::kCameraCy;
    auto range_sensor_frame = std::make_shared<DepthFrame3D>(depth_frame_setting);

    auto octree_setting = std::make_shared<OccupancyOctree::Setting>();
    ASSERT_TRUE(
        octree_setting->FromYamlFile(ERL_GEOMETRY_ROOT_DIR "/config/octree_cow_and_lady.yaml"));
    octree_setting->use_change_detection = true;
    octree_setting->resolution *= options.scaling;

    auto octree = std::make_shared<OccupancyOctree>(octree_setting);
    const auto visualizer_setting = std::make_shared<Open3dVisualizerWrapper::Setting>();
    visualizer_setting->window_name = WINDOW_NAME;
    visualizer_setting->mesh_show_back_face = false;
    Open3dVisualizerWrapper visualizer(visualizer_setting);
    auto point_cloud = std::make_shared<open3d::geometry::PointCloud>();
    auto line_set_traj = std::make_shared<open3d::geometry::LineSet>();
    auto obb = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(
        dataset.GetGroundTruthPointCloud()->GetAxisAlignedBoundingBox());
    obb->color_ = {1, 0, 0};
    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries =
        OccupancyOctreeDrawer::GetBlankGeometries();
    geometries.push_back(point_cloud);
    geometries.push_back(line_set_traj);
    geometries.push_back(obb);
    visualizer.AddGeometries(geometries);

    auto drawer_setting = std::make_shared<OccupancyOctreeDrawer::Setting>();
    drawer_setting->scaling = 1.0f / options.scaling;
    drawer_setting->area_min = dataset.GetMapMin().array() * options.scaling;
    drawer_setting->area_max = dataset.GetMapMax().array() * options.scaling;
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
                Serialization<OccupancyOctree>::Write(test_output_dir / "cow_and_lady.ot", octree));
            EXPECT_TRUE(
                Serialization<OccupancyOctree>::Write(
                    test_output_dir / "cow_and_lady.bt",
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
        idx += options.stride;

        std::cout << "==== " << idx << " ====" << std::endl;
        range_sensor_frame->UpdateRanges(
            frame.rotation.cast<Dtype>(),
            frame.translation.cast<Dtype>(),
            frame.depth.cast<Dtype>());
        Matrix3X points_in_world = Eigen::Map<const Matrix3X>(
            range_sensor_frame->GetEndPointsInWorld().data()->data(),
            3,
            range_sensor_frame->GetNumHitRays());
        const Dtype min_range = 0.1f * options.scaling;
        const Dtype max_range = 4.0f * options.scaling;

        double dt = 0;
        line_set_traj->points_.emplace_back(frame.translation);
        if (line_set_traj->points_.size() > 1) {
            line_set_traj->lines_.emplace_back(
                line_set_traj->points_.size() - 2,
                line_set_traj->points_.size() - 1);
        }

        point_cloud->points_.clear();
        point_cloud->points_.reserve(points_in_world.cols());
        for (int i = 0; i < points_in_world.cols(); ++i) {
            point_cloud->points_.emplace_back(points_in_world.col(i).cast<double>());
        }

        octree->ClearChangedKeys();
        {
            ERL_BLOCK_TIMER_MSG_TIME("Insert time", dt);
            constexpr bool with_count = false;
            constexpr bool parallel = true;
            constexpr bool lazy_eval = true;
            constexpr bool discrete = true;
            octree->InsertPointCloud(
                points_in_world.cast<Dtype>() * options.scaling,
                frame.translation.cast<Dtype>() * options.scaling,
                min_range,
                max_range,
                with_count,
                parallel,
                lazy_eval,
                discrete);
            octree->UpdateInnerOccupancy();
            octree->Prune();
        }
        mean_insert_time = (mean_insert_time * animation_cnt + dt) / (animation_cnt + 1);
        std::cout << "Number of points: " << points_in_world.cols() << std::endl;
        std::cout << "Mean insert time: " << mean_insert_time << " ms." << std::endl;

        if (options.draw_tree_grid) {
            drawer.DrawTree(geometries);
        } else {
            drawer.DrawLeaves(geometries);
        }

        line_set_traj->PaintUniformColor({1, 0, 0});
        if (line_set_traj->lines_.empty()) { vis->ResetViewPoint(true); }
        if (point_cloud->points_.size() > options.max_point_cloud_size) {
            point_cloud->points_.swap(point_cloud->RandomDownSample(0.5)->points_);
        }

        const auto t_end = std::chrono::high_resolution_clock::now();
        const auto duration_total =
            std::chrono::duration<double, std::milli>(t_end - t_start).count();
        std::cout << "Callback time: " << duration_total << " ms." << std::endl;

        return animation_cnt++ % options.animation_interval == 0;
    };

    visualizer.SetAnimationCallback(callback);
    visualizer.Show();
}

int
main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    options.FromCommandLine(argc, argv);
    return RUN_ALL_TESTS();
}
