#include "test_occupancy_octree_build_impl.hpp"

#include "erl_common/test_helper.hpp"
#include "erl_geometry/depth_frame_3d.hpp"
#include "erl_geometry/replica_rgbd.hpp"

template<typename Dtype>
struct TestOccupancyOctreeBuildWithReplicaRgbd : TestOccupancyOctreeBuildImpl<Dtype> {
    using Super = TestOccupancyOctreeBuildImpl<Dtype>;
    using ReplicaRgbd = erl::geometry::ReplicaRgbd;
    using DepthFrame = erl::geometry::DepthFrame3Dd;

    struct Options : public erl::common::Yamlable<Options, typename Super::Options> {
        std::string data_dir;
        std::string scene_name;

        ERL_REFLECT_SCHEMA(
            Options,
            ERL_REFLECT_MEMBER(Options, data_dir),
            ERL_REFLECT_MEMBER(Options, scene_name));

        bool
        PostDeserialization() override {
            if (!Super::Options::PostDeserialization()) { return false; }
            ERL_ASSERTM(
                !data_dir.empty(),
                "Please provide the Newer College dataset directory via --data_dir");
            ERL_ASSERTM(!scene_name.empty(), "Please provide the scene name via --scene_name");
            return true;
        }
    };

private:
    std::shared_ptr<Options> options = nullptr;
    ReplicaRgbd dataset;
    ReplicaRgbd::Frame frame;
    DepthFrame depth_frame;

public:
    explicit TestOccupancyOctreeBuildWithReplicaRgbd(std::shared_ptr<Options> options_in)
        : Super(options_in),
          options(options_in),
          dataset(options->data_dir, options->scene_name),
          depth_frame([] {
              auto depth_frame_setting = std::make_shared<DepthFrame::Setting>();
              depth_frame_setting->camera_intrinsic.image_height = ReplicaRgbd::kImageHeight;
              depth_frame_setting->camera_intrinsic.image_width = ReplicaRgbd::kImageWidth;
              depth_frame_setting->camera_intrinsic.camera_fx = ReplicaRgbd::kCameraFx;
              depth_frame_setting->camera_intrinsic.camera_fy = ReplicaRgbd::kCameraFy;
              depth_frame_setting->camera_intrinsic.camera_cx = ReplicaRgbd::kCameraCx;
              depth_frame_setting->camera_intrinsic.camera_cy = ReplicaRgbd::kCameraCy;
              return depth_frame_setting;
          }()) {

        options->min_range = 0.1f;
        options->max_range = 4.0f;

        if (options->max_wp_idx < 0) {
            options->max_wp_idx = dataset.Size();
        } else {
            options->max_wp_idx = std::min(options->max_wp_idx, dataset.Size());
        }
    }

    TestOccupancyOctreeBuildWithReplicaRgbd(const TestOccupancyOctreeBuildWithReplicaRgbd &) =
        delete;
    TestOccupancyOctreeBuildWithReplicaRgbd &
    operator=(const TestOccupancyOctreeBuildWithReplicaRgbd &) = delete;
    TestOccupancyOctreeBuildWithReplicaRgbd(TestOccupancyOctreeBuildWithReplicaRgbd &&) = delete;
    TestOccupancyOctreeBuildWithReplicaRgbd &
    operator=(const TestOccupancyOctreeBuildWithReplicaRgbd &&) = delete;

    ~TestOccupancyOctreeBuildWithReplicaRgbd() override = default;

    Eigen::Vector3d
    GetMapMin() const override {
        return dataset.GetMapMin();
    }

    Eigen::Vector3d
    GetMapMax() const override {
        return dataset.GetMapMax();
    }

    Eigen::Matrix3Xd
    LoadPcdInWorldFrame() override {
        frame = dataset[this->idx];
        depth_frame.UpdateRanges(frame.rotation, frame.translation, frame.depth);
        return Eigen::Map<const Eigen::Matrix3Xd>(
            depth_frame.GetEndPointsInWorld().data()->data(),
            3,
            depth_frame.GetNumHitRays());
    }

    Eigen::Vector3d
    LoadSensorTranslation() override {
        return frame.translation;
    }
};

static int g_argc = 0;
static char **g_argv = nullptr;

TEST(OccupancyOctree, BuildReplicaRgbd) {
    GTEST_PREPARE_OUTPUT_DIR();
    using Test = TestOccupancyOctreeBuildWithReplicaRgbd<float>;
    const auto options = std::make_shared<Test::Options>();
    options->FromCommandLine(g_argc, g_argv);
    ERL_INFO("Options:\n{}", options->AsYamlString());
    Test test(options);
    test.visualizer_setting->window_name = test_output_dir.string();
    test.test_output_dir = test_output_dir;
    test.Run();
}

int
main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    g_argc = argc;
    g_argv = argv;
    return RUN_ALL_TESTS();
}
