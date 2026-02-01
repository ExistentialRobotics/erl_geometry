#include "test_occupancy_octree_build_impl.hpp"

#include "erl_common/test_helper.hpp"
#include "erl_geometry/newer_college.hpp"

template<typename Dtype>
struct TestOccupancyOctreeBuildWithNewerCollege : TestOccupancyOctreeBuildImpl<Dtype> {
    using Super = TestOccupancyOctreeBuildImpl<Dtype>;

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
    erl::geometry::NewerCollege dataset;
    erl::geometry::NewerCollege::Frame frame;

public:
    explicit TestOccupancyOctreeBuildWithNewerCollege(std::shared_ptr<Options> options_in)
        : Super(options_in), options(options_in), dataset(options->data_dir) {

        // options->min_range = 0.06f;
        // options->max_range = 35.0f;

        if (options->max_wp_idx < 0) {
            options->max_wp_idx = erl::geometry::NewerCollege::kNumFrames;
        } else {
            options->max_wp_idx =
                std::min(options->max_wp_idx, erl::geometry::NewerCollege::kNumFrames);
        }
    }

    TestOccupancyOctreeBuildWithNewerCollege(const TestOccupancyOctreeBuildWithNewerCollege &) =
        delete;
    TestOccupancyOctreeBuildWithNewerCollege &
    operator=(const TestOccupancyOctreeBuildWithNewerCollege &) = delete;
    TestOccupancyOctreeBuildWithNewerCollege(TestOccupancyOctreeBuildWithNewerCollege &&) = delete;
    TestOccupancyOctreeBuildWithNewerCollege &
    operator=(const TestOccupancyOctreeBuildWithNewerCollege &&) = delete;

    ~TestOccupancyOctreeBuildWithNewerCollege() override = default;

    Eigen::Vector3d
    GetMapMin() const override {
        return erl::geometry::NewerCollege::GetMapMin();
    }

    Eigen::Vector3d
    GetMapMax() const override {
        return erl::geometry::NewerCollege::GetMapMax();
    }

    Eigen::Matrix3Xd
    LoadPcdInWorldFrame() override {
        frame = dataset[this->idx];
        return frame.GetPointsInWorldFrame();
    }

    Eigen::Vector3d
    LoadSensorTranslation() override {
        return frame.translation;
    }
};

static int g_argc = 0;
static char **g_argv = nullptr;

TEST(OccupancyOctree, BuildNewerCollege) {
    GTEST_PREPARE_OUTPUT_DIR();
    using Test = TestOccupancyOctreeBuildWithNewerCollege<float>;
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
