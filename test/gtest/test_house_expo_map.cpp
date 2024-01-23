#include "erl_common/test_helper.hpp"
#include "erl_geometry/house_expo_map.hpp"
#include "erl_geometry/earcut.hpp"

TEST(HouseExpoMap, ExtrudeTo3D) {
    using namespace erl::geometry;

    std::filesystem::path path = __FILE__;
    path = path.parent_path() / "house_expo_room_1451.json";
    HouseExpoMap house_expo_map(path.string().c_str());
    auto mesh = house_expo_map.ExtrudeTo3D(3);
}
