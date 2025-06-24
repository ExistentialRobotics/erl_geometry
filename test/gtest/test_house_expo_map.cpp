#include "erl_common/test_helper.hpp"
#include "erl_geometry/earcut.hpp"
#include "erl_geometry/house_expo_map.hpp"

#include <open3d/io/TriangleMeshIO.h>

TEST(HouseExpoMap, ExtrudeTo3D) {
    using namespace erl::geometry;

    std::filesystem::path path = ERL_GEOMETRY_ROOT_DIR;
    path /= "data/house_expo_room_0000.json";
    const HouseExpoMap house_expo_map(path);
    const std::shared_ptr<open3d::geometry::TriangleMesh> mesh = house_expo_map.ExtrudeTo3D(3);
    constexpr bool write_ascii = false;
    constexpr bool compressed = false;
    constexpr bool write_vertex_normals = true;
    constexpr bool write_vertex_colors = true;
    constexpr bool write_triangle_uvs = true;
    constexpr bool print_progress = false;
    open3d::io::WriteTriangleMesh(
        (path.parent_path() / "house_expo_room_0000.ply").string(),
        *mesh,
        write_ascii,
        compressed,
        write_vertex_normals,
        write_vertex_colors,
        write_triangle_uvs,
        print_progress);
}
