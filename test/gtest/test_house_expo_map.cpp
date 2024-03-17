#include <open3d/io/TriangleMeshIO.h>

#include "erl_common/test_helper.hpp"
#include "erl_geometry/house_expo_map.hpp"
#include "erl_geometry/earcut.hpp"

TEST(HouseExpoMap, ExtrudeTo3D) {
    using namespace erl::geometry;

    std::filesystem::path path = __FILE__;
    path = path.parent_path() / "house_expo_room_1451.json";
    HouseExpoMap house_expo_map(path.string().c_str());
    auto mesh = house_expo_map.ExtrudeTo3D(3);
    bool write_ascii = false;
    bool compressed = false;
    bool write_vertex_normals = true;
    bool write_vertex_colors = true;
    bool write_triangle_uvs = true;
    bool print_progress = false;
    open3d::io::WriteTriangleMesh(
        (path.parent_path() / "house_expo_room_1451.ply").string(),
        *mesh,
        write_ascii,
        compressed,
        write_vertex_normals,
        write_vertex_colors,
        write_triangle_uvs,
        print_progress);
}
