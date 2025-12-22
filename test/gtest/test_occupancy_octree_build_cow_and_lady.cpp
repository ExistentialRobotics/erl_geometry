#include "test_occupancy_octree_build_impl.hpp"

#include "erl_common/test_helper.hpp"
#include "erl_geometry/cow_and_lady.hpp"

template<typename Dtype>
struct TestOccupancyOctreeBuildWithCowAndLady : TestOccupancyOctreeBuildImpl<Dtype> {
    using Super = TestOccupancyOctreeBuildImpl<Dtype>;
    using CowAndLady = erl::geometry::CowAndLady;
    using DepthFrame = erl::geometry::DepthFrame3Dd;

    struct Options : public erl::common::Yamlable<Options, typename Super::Options> {
        std::string data_dir;

        ERL_REFLECT_SCHEMA(Options, ERL_REFLECT_MEMBER(Options, data_dir));

        bool
        PostDeserialization() override {
            if (!Super::Options::PostDeserialization()) { return false; }
            ERL_ASSERTM(
                !data_dir.empty(),
                "Please provide the Newer College dataset directory via --data_dir");
            return true;
        }
    };

private:
    std::shared_ptr<Options> options = nullptr;
    CowAndLady dataset;
    CowAndLady::Frame frame;
    DepthFrame depth_frame;

public:
    explicit TestOccupancyOctreeBuildWithCowAndLady(const std::shared_ptr<Options> &options_in)
        : Super(options_in), options(options_in), dataset(options->data_dir), depth_frame([] {
              auto depth_frame_setting = std::make_shared<DepthFrame::Setting>();
              depth_frame_setting->camera_intrinsic.image_height = CowAndLady::kImageHeight;
              depth_frame_setting->camera_intrinsic.image_width = CowAndLady::kImageWidth;
              depth_frame_setting->camera_intrinsic.camera_fx = CowAndLady::kCameraFx;
              depth_frame_setting->camera_intrinsic.camera_fy = CowAndLady::kCameraFy;
              depth_frame_setting->camera_intrinsic.camera_cx = CowAndLady::kCameraCx;
              depth_frame_setting->camera_intrinsic.camera_cy = CowAndLady::kCameraCy;
              return depth_frame_setting;
          }()) {

        options->min_range = 0.05f;
        options->max_range = 8.0f;

        if (options->max_wp_idx < 0) {
            options->max_wp_idx = CowAndLady::kEndIdx;
        } else {
            options->max_wp_idx = std::min(options->max_wp_idx, CowAndLady::kEndIdx);
        }
    }

    TestOccupancyOctreeBuildWithCowAndLady(const TestOccupancyOctreeBuildWithCowAndLady &) = delete;
    TestOccupancyOctreeBuildWithCowAndLady &
    operator=(const TestOccupancyOctreeBuildWithCowAndLady &) = delete;
    TestOccupancyOctreeBuildWithCowAndLady(TestOccupancyOctreeBuildWithCowAndLady &&) = delete;
    TestOccupancyOctreeBuildWithCowAndLady &
    operator=(const TestOccupancyOctreeBuildWithCowAndLady &&) = delete;

    ~TestOccupancyOctreeBuildWithCowAndLady() override = default;

    [[nodiscard]] Eigen::Vector3d
    GetMapMin() const override {
        return dataset.GetMapMin();
    }

    [[nodiscard]] Eigen::Vector3d
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

TEST(OccupancyOctree, BuildCowAndLady) {
    GTEST_PREPARE_OUTPUT_DIR();
    using Test = TestOccupancyOctreeBuildWithCowAndLady<float>;
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
