#include "erl_common/test_helper.hpp"
#include "erl_geometry/city_street_map.hpp"

TEST(CityStreetMap, Load) {
    GTEST_PREPARE_OUTPUT_DIR();

    const cv::Mat map = erl::geometry::CityStreetMap::Load(gtest_src_dir / "Berlin_0_1024.map");
    cv::imshow("city street map", map);
    cv::waitKey(0);
}
