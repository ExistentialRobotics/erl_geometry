#include "erl_common/test_helper.hpp"
#include "erl_geometry/city_street_map.hpp"

TEST(CityStreetMap, LoadMap) {
    GTEST_PREPARE_OUTPUT_DIR();

    const cv::Mat map = erl::geometry::CityStreetMap::LoadMap(gtest_src_dir / "Berlin_0_1024.map");
    cv::imshow("city street map", map);
    cv::waitKey(0);
}

TEST(CityStreetMap, LoadScenes) {
    GTEST_PREPARE_OUTPUT_DIR();

    const std::vector scenes =
        erl::geometry::CityStreetMap::LoadScenes(gtest_src_dir / "Berlin_0_1024.map.scen");
    ERL_INFO("{} scenes loaded", scenes.size());
}
