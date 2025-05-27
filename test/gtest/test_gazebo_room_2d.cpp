#include "erl_common/test_helper.hpp"
#include "erl_geometry/gazebo_room_2d.hpp"

#include <open3d/io/TriangleMeshIO.h>
#include <open3d/visualization/utility/DrawGeometry.h>

TEST(GazeboRoom2D, ExtrudeTo3D) {
    GTEST_PREPARE_OUTPUT_DIR();
    using namespace erl::geometry;

    // Extrude to 3D with a height of 3.0 and add ceiling
    auto room_mesh = GazeboRoom2D::ExtrudeTo3D(3.0, false);

    // Check if the mesh is not null
    ASSERT_NE(room_mesh, nullptr);

    // Check if the mesh has vertices and triangles
    ASSERT_GT(room_mesh->vertices_.size(), 0);
    ASSERT_GT(room_mesh->triangles_.size(), 0);

    open3d::visualization::DrawGeometries({room_mesh});
    open3d::io::WriteTriangleMesh(
        test_output_dir / "gazebo_room_3d_no_ceiling.ply",
        *room_mesh,
        true);

    room_mesh = GazeboRoom2D::ExtrudeTo3D(3.0, true);
    open3d::visualization::DrawGeometries({room_mesh});
    open3d::io::WriteTriangleMesh(
        test_output_dir / "gazebo_room_3d_with_ceiling.ply",
        *room_mesh,
        true);
}
