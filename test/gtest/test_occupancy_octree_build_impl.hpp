#pragma once

#include "erl_common/block_timer.hpp"
#include "erl_common/serialization.hpp"
#include "erl_common/yaml.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/occupancy_octree_drawer.hpp"
#include "erl_geometry/open3d_visualizer_wrapper.hpp"

#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/VoxelGrid.h>

#include <cstddef>
#include <string>

template<typename Dtype>
struct TestOccupancyOctreeBuildImpl {

    using OccupancyOctree = erl::geometry::OccupancyOctree<Dtype>;
    using OctreeSetting = typename OccupancyOctree::Setting;
    using OccupancyOctreeDrawer = erl::geometry::OccupancyOctreeDrawer<OccupancyOctree>;
    using OctreeDrawerSetting = typename OccupancyOctreeDrawer::Setting;
    using Open3dVisualizerWrapper = erl::geometry::Open3dVisualizerWrapper;
    using O3dWrapperSetting = Open3dVisualizerWrapper::Setting;

    struct Options : public erl::common::Yamlable<Options> {
        std::string octree_config_file;
        std::string open3d_view_status_file;
        int animation_interval = 2;
        std::size_t max_point_cloud_size = 1000000;
        int stride = 1;
        long max_wp_idx = -1;
        float scaling = 1.0f;
        float min_range = 0.0f;
        float max_range = 1000.0f;
        bool draw_tree_grid = false;
        bool hold = false;

        ERL_REFLECT_SCHEMA(
            Options,
            ERL_REFLECT_MEMBER(Options, octree_config_file),
            ERL_REFLECT_MEMBER(Options, open3d_view_status_file),
            ERL_REFLECT_MEMBER(Options, animation_interval),
            ERL_REFLECT_MEMBER(Options, max_point_cloud_size),
            ERL_REFLECT_MEMBER(Options, stride),
            ERL_REFLECT_MEMBER(Options, max_wp_idx),
            ERL_REFLECT_MEMBER(Options, scaling),
            ERL_REFLECT_MEMBER(Options, min_range),
            ERL_REFLECT_MEMBER(Options, max_range),
            ERL_REFLECT_MEMBER(Options, draw_tree_grid),
            ERL_REFLECT_MEMBER(Options, hold));

        bool
        PostDeserialization() override {
            ERL_ASSERTM(
                !octree_config_file.empty(),
                "Please provide the octree config file via --octree_config_file");
            return true;
        }
    };

private:
    std::shared_ptr<Options> options = nullptr;

public:
    std::shared_ptr<OctreeSetting> octree_setting = std::make_shared<OctreeSetting>();
    std::shared_ptr<OccupancyOctree> octree = nullptr;
    std::shared_ptr<OctreeDrawerSetting> drawer_setting = std::make_shared<OctreeDrawerSetting>();
    std::shared_ptr<OccupancyOctreeDrawer> drawer = nullptr;
    std::shared_ptr<O3dWrapperSetting> visualizer_setting = std::make_shared<O3dWrapperSetting>();
    std::shared_ptr<open3d::geometry::PointCloud> point_cloud =
        std::make_shared<open3d::geometry::PointCloud>();
    std::shared_ptr<open3d::geometry::LineSet> line_set_traj =
        std::make_shared<open3d::geometry::LineSet>();
    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries =
        OccupancyOctreeDrawer::GetBlankGeometries();

    std::filesystem::path test_output_dir = ".";
    long idx = 0;
    bool octree_saved = false;
    int animation_cnt = 0;
    double mean_insert_time = 0;

    explicit TestOccupancyOctreeBuildImpl(std::shared_ptr<Options> options_in)
        : options(std::move(options_in)) {

        drawer_setting->occupied_only = true;
        visualizer_setting->mesh_show_back_face = false;

        geometries.push_back(point_cloud);
        geometries.push_back(line_set_traj);
    }

    TestOccupancyOctreeBuildImpl(const TestOccupancyOctreeBuildImpl &) = delete;
    TestOccupancyOctreeBuildImpl &
    operator=(const TestOccupancyOctreeBuildImpl &) = delete;
    TestOccupancyOctreeBuildImpl(TestOccupancyOctreeBuildImpl &&) = delete;
    TestOccupancyOctreeBuildImpl &
    operator=(TestOccupancyOctreeBuildImpl &&) = delete;

    virtual ~TestOccupancyOctreeBuildImpl() = default;

    virtual void
    Run() {
        ERL_ASSERT(octree_setting->FromYamlFile(options->octree_config_file));
        octree_setting->use_change_detection = true;
        octree_setting->resolution *= options->scaling;
        octree = std::make_shared<OccupancyOctree>(octree_setting);

        drawer_setting->scaling = 1.0 / options->scaling;
        drawer_setting->area_min = GetMapMin().array() * options->scaling;
        drawer_setting->area_max = GetMapMax().array() * options->scaling;

        Open3dVisualizerWrapper visualizer(visualizer_setting);
        visualizer.AddGeometries(geometries);

        drawer = std::make_shared<OccupancyOctreeDrawer>(drawer_setting);
        drawer->SetOctree(octree);

        auto callback =
            [this](Open3dVisualizerWrapper *wrapper, open3d::visualization::Visualizer *vis) {
                return Callback(wrapper, vis);
            };

        visualizer.SetAnimationCallback(callback);
        visualizer.Show();
    }

    [[nodiscard]] virtual Eigen::Vector3d
    GetMapMin() const = 0;

    [[nodiscard]] virtual Eigen::Vector3d
    GetMapMax() const = 0;

    virtual Eigen::Matrix3Xd
    LoadPcdInWorldFrame() = 0;

    virtual Eigen::Vector3d
    LoadSensorTranslation() = 0;

    virtual bool
    Callback(Open3dVisualizerWrapper *wrapper, open3d::visualization::Visualizer *vis) {
        if (idx >= options->max_wp_idx) {
            if (octree_saved) {
                ERL_WARN_ONCE("callback is still called after octree is saved.");
                return false;
            }
            using namespace erl::common::serialization;
            ERL_ASSERT(
                Serialization<OccupancyOctree>::Write(test_output_dir / "octree.ot", octree));
            ERL_ASSERT(
                Serialization<OccupancyOctree>::Write(
                    test_output_dir / "octree.bt",
                    [&](std::ostream &s) -> bool { return octree->WriteBinary(s); }));
            octree_saved = true;
            wrapper->ClearGeometries();

            drawer->DrawLeaves(geometries);
            geometries.push_back(point_cloud);
            geometries.push_back(line_set_traj);
            wrapper->AddGeometries(geometries);
            vis->UpdateGeometry();
            wrapper->SetAnimationCallback(nullptr);  // stop calling this callback
            if (!options->hold) { vis->Close(); }
            return false;
        }

        const auto t_start = std::chrono::high_resolution_clock::now();

        std::cout << "==== " << idx << " ====" << std::endl;
        Eigen::Matrix3Xd points_in_world = LoadPcdInWorldFrame();
        Eigen::Vector3d sensor_translation = LoadSensorTranslation();
        idx += options->stride;

        const Dtype min_range = options->min_range * options->scaling;
        const Dtype max_range = options->max_range * options->scaling;

        double dt = 0;
        line_set_traj->points_.emplace_back(sensor_translation);
        if (line_set_traj->points_.size() > 1) {
            line_set_traj->lines_.emplace_back(
                line_set_traj->points_.size() - 2,
                line_set_traj->points_.size() - 1);
        }

        point_cloud->points_.clear();
        point_cloud->points_.reserve(points_in_world.cols());
        for (int i = 0; i < points_in_world.cols(); ++i) {
            point_cloud->points_.emplace_back(points_in_world.col(i));
        }

        octree->ClearChangedKeys();
        {
            const ERL_BLOCK_TIMER_MSG_TIME("Insert time", dt);
            constexpr bool with_count = false;
            constexpr bool parallel = true;
            constexpr bool lazy_eval = true;
            constexpr bool discrete = true;
            octree->InsertPointCloud(
                points_in_world.cast<Dtype>() * options->scaling,
                sensor_translation.cast<Dtype>() * options->scaling,
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

        if (options->draw_tree_grid) {
            drawer->DrawTree(geometries);
        } else {
            drawer->DrawLeaves(geometries);
        }

        line_set_traj->PaintUniformColor({1, 0, 0});
        if (line_set_traj->lines_.empty()) { vis->ResetViewPoint(true); }
        if (point_cloud->points_.size() > options->max_point_cloud_size) {
            point_cloud->points_.swap(point_cloud->RandomDownSample(0.5)->points_);
        }

        const auto t_end = std::chrono::high_resolution_clock::now();
        const auto duration_total =
            std::chrono::duration<double, std::milli>(t_end - t_start).count();
        std::cout << "Callback time: " << duration_total << " ms." << std::endl;

        if (!options->open3d_view_status_file.empty()) {
            wrapper->SetViewStatus(options->open3d_view_status_file);
            vis->UpdateRender();
            options->open3d_view_status_file.clear();  // only set once
        }

        return animation_cnt++ % options->animation_interval == 0;
    }
};
