#include "erl_common/block_timer.hpp"
#include "erl_common/grid_map_info.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/marching_cubes.hpp"
#include "erl_geometry/mesh_sdf.hpp"
#include "erl_geometry/open3d_helper.hpp"

#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/t/geometry/PointCloud.h>
#include <open3d/visualization/utility/DrawGeometry.h>

#include <bitset>

TEST(MarchingCubes, SingleCube) {
    GTEST_PREPARE_OUTPUT_DIR();
    using MC = erl::geometry::MarchingCubes;

    Eigen::Matrix<double, 3, 8> vertex_coords;
    for (int i = 0; i < 8; ++i) {
        const int *vtx = MC::kCubeVertexCodes[i];
        vertex_coords.col(i) << vtx[0], vtx[1], vtx[2];
    }

    auto box_org = erl::geometry::CreateUnitBoxFrameMesh(0.01);
    box_org->PaintUniformColor({0.0, 1.0, 0.0});

    auto all_cubes = std::make_shared<open3d::geometry::TriangleMesh>();
    auto all_boxes = std::make_shared<open3d::geometry::TriangleMesh>();

    for (int cfg = 1; cfg <= 254; ++cfg) {
        std::bitset<8> flags(cfg);
        Eigen::Vector<double, 8> sdf_values(8);
        sdf_values.setOnes();
        for (int i = 0; i < 8; ++i) {
            if (flags[i]) { sdf_values[i] = -1.0; }
        }
        auto extracted_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        MC::SingleCube(
            vertex_coords,
            sdf_values,
            extracted_mesh->vertices_,
            extracted_mesh->triangles_,
            extracted_mesh->triangle_normals_);
        extracted_mesh->PaintUniformColor({1.0, 0.0, 0.0});

        // 8x8x4, col-major
        int z = cfg / 64;
        int rem = cfg % 64;
        int y = rem / 8;
        int x = rem % 8;
        Eigen::Vector3d origin(
            static_cast<double>(x) * 1.5,
            static_cast<double>(y) * 1.5,
            static_cast<double>(z) * 1.5);
        extracted_mesh->Translate(origin);
        *all_cubes += *extracted_mesh;

        open3d::geometry::TriangleMesh box = *box_org;
        box.Translate(origin);
        *all_boxes += box;
    }

    auto axes = open3d::geometry::TriangleMesh::CreateCoordinateFrame();

    open3d::visualization::DrawGeometries({all_cubes, all_boxes, axes}, "Open3D", 1280, 960);
    open3d::io::WriteTriangleMesh(
        test_output_dir / "single_cube.ply",
        *all_cubes + *all_boxes + *axes,
        true);
}

TEST(MarchingCubes, Sphere) {
    GTEST_PREPARE_OUTPUT_DIR();
    constexpr double radius = 1.0;
    constexpr int n = 71;
    constexpr int resolution = 100;
    auto sphere = open3d::geometry::TriangleMesh::CreateSphere(radius, resolution);
    open3d::io::WriteTriangleMesh(test_output_dir / "sphere.ply", *sphere, true);

    // generate test data
    erl::common::GridMapInfo3Dd grid_map_info(
        Eigen::Vector3i(n, n, n),
        Eigen::Vector3d(-2, -2, -2),
        Eigen::Vector3d(2, 2, 2));
    // axis varying order: x, y, z
    constexpr bool row_major = false;
    Eigen::Matrix3Xd positions = grid_map_info.GenerateMeterCoordinates(row_major);
    Eigen::VectorXd sdf_gt_values = positions.colwise().norm().array() - radius;
    // test
    erl::geometry::MeshSdf sdf(sphere->vertices_, sphere->triangles_, true);
    Eigen::VectorXd sdf_values;
    {
        ERL_BLOCK_TIMER_MSG("compute sdf");
        sdf_values = sdf(positions);
    }
    const double mean_sdf_error = (sdf_values - sdf_gt_values).array().abs().mean();
    std::cout << "Mean SDF error: " << mean_sdf_error << std::endl;

    using MC = erl::geometry::MarchingCubes;
    constexpr bool parallel = true;
    std::vector<std::vector<MC::ValidCube>> cubes_vec;
    {
        ERL_BLOCK_TIMER_MSG("collect valid cubes");
        cubes_vec = MC::CollectValidCubes(grid_map_info.Shape(), sdf_values, row_major, parallel);
    }

    // check by visualizing the valid cubes
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries = {sphere};
    open3d::geometry::TriangleMesh boxes;
    const Eigen::Vector3d &cube_size = grid_map_info.Resolution();
    for (auto &cubes: cubes_vec) {
        for (auto &cube: cubes) {
            Eigen::Vector3d cube_origin = grid_map_info.GridToMeterForPoints(cube.coords);
            auto box =
                open3d::geometry::TriangleMesh::CreateBox(cube_size[0], cube_size[1], cube_size[2]);
            box->Translate(cube_origin);
            box->PaintUniformColor({1.0, 0.0, 0.0});
            geometries.push_back(box);
            boxes += *box;
        }
    }
    // open3d::io::WriteTriangleMesh(test_output_dir / "valid_cubes.ply", boxes, true);
    // open3d::visualization::DrawGeometries(geometries);

    auto extracted_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    {
        ERL_BLOCK_TIMER_MSG("compute extracted mesh");
        MC::ProcessValidCubes(
            cubes_vec,
            grid_map_info.GetMinMeterCoords() /*coords_min*/,
            grid_map_info.Resolution(),
            grid_map_info.Shape(),
            sdf_values,
            row_major,
            extracted_mesh->vertices_,
            extracted_mesh->triangles_,
            extracted_mesh->triangle_normals_,
            parallel);
    }
    extracted_mesh->PaintUniformColor({0.0, 0.0, 1.0});
    // auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    // pcd->points_ = extracted_mesh->vertices_;
    // pcd->PaintUniformColor({0.0, 1.0, 0.0});
    // open3d::visualization::DrawGeometries({pcd});
    // open3d::io::WritePointCloud(test_output_dir / "vertices.ply", *pcd, {true});
    open3d::io::WriteTriangleMesh(test_output_dir / "extracted_mesh.ply", *extracted_mesh, true);
    // open3d::visualization::DrawGeometries({extracted_mesh});

    constexpr std::size_t num_points = 1000000;
    using PointCloud = open3d::t::geometry::PointCloud;
    auto gt_pcd = PointCloud::FromLegacy(*sphere->SamplePointsUniformly(num_points));
    auto extracted_pcd = PointCloud::FromLegacy(*extracted_mesh->SamplePointsUniformly(num_points));
    // open3d::t::geometry::MetricParameters metric_params;  // set fscore_radius
    auto scores_tensor = gt_pcd.ComputeMetrics(
        extracted_pcd,
        {open3d::t::geometry::Metric::ChamferDistance,
         open3d::t::geometry::Metric::HausdorffDistance,
         open3d::t::geometry::Metric::FScore});
    auto scores = scores_tensor.ToFlatVector<float>();
    std::cout << "Chamfer distance: " << scores[0] << '\n'
              << "Hausdorff distance: " << scores[1] << '\n'
              << "F-Score: " << scores[2] << std::endl;
    EXPECT_LE(scores[0], 0.004);
    EXPECT_LE(scores[1], 0.0085);
    EXPECT_GE(scores[2], 99);
}

TEST(MarchingCubes, HouseExpo) {
    std::filesystem::path kProjectDir = ERL_GEOMETRY_ROOT_DIR;
    std::filesystem::path kDataDir = kProjectDir / "data";
    std::filesystem::path kMeshFile = kDataDir / "house_expo_room_0000_water_tight.ply";
    // std::filesystem::path kMeshFile = kDataDir / "house_expo_room_1451_water_tight.ply";
    // std::filesystem::path kMeshFile = kDataDir / "Stage_v3_sc0_staging.ply";
    GTEST_PREPARE_OUTPUT_DIR();
    constexpr int n = 401;
    auto mesh = open3d::io::CreateMeshFromFile(kMeshFile);
    ASSERT_TRUE(mesh != nullptr);
    // for (auto &triangle: mesh->triangles_) { std::swap(triangle[1], triangle[2]); }

    // scale mesh to [-1.2, 1.2]
    // auto aabb = mesh->GetAxisAlignedBoundingBox();
    // mesh->Translate(-aabb.GetCenter(), true);
    // const double scale_org = (aabb.GetExtent() / 2).maxCoeff();
    // mesh->Scale(1.2 / scale_org, Eigen::Vector3d::Zero());
    mesh->ComputeTriangleNormals();
    mesh->ComputeVertexNormals();
    open3d::io::WriteTriangleMesh(test_output_dir / "house_expo.ply", *mesh, true);
    // open3d::visualization::DrawGeometries({mesh}, "House Expo", 640, 480, 50, 50, true);

    // generate test data
    erl::common::GridMapInfo3Dd grid_map_info(
        Eigen::Vector3i(n, n, n),
        mesh->GetMinBound().array() - 0.2,
        mesh->GetMaxBound().array() + 0.2);

    constexpr bool c_stride = false;  // if false, axis varying order: x, y, z. Otherwise, z, y, x
    Eigen::Matrix3Xd positions = grid_map_info.GenerateMeterCoordinates(c_stride);

    // test
    Eigen::VectorXd sdf_gt_values =
        erl::geometry::MeshSdf(mesh->vertices_, mesh->triangles_, true)(positions);

    auto extracted_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    using MC = erl::geometry::MarchingCubes;
    MC::Run(
        grid_map_info.Min(),
        grid_map_info.Resolution(),
        grid_map_info.Shape(),
        sdf_gt_values,
        c_stride,
        extracted_mesh->vertices_,
        extracted_mesh->triangles_,
        extracted_mesh->triangle_normals_,
        true);
    open3d::io::WriteTriangleMesh(test_output_dir / "extracted_mesh.ply", *extracted_mesh, true);
}

TEST(MarchingCubes, FromArray) {
    GTEST_PREPARE_OUTPUT_DIR();
    std::filesystem::path kProjectDir = ERL_GEOMETRY_ROOT_DIR;
    std::filesystem::path kDataDir = kProjectDir / "data";
    std::filesystem::path kArrayFile = kDataDir / "sdf_array.dat";

    auto extracted_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    Eigen::VectorXd sdf_values =
        erl::common::LoadEigenMatrixFromBinaryFile<float>(kArrayFile).cast<double>();
    erl::geometry::MarchingCubes::Run(
        Eigen::Vector3d(-1, -1, -1),
        Eigen::Vector3d(2.0 / 256, 2.0 / 256, 2.0 / 256),
        Eigen::Vector3i(256, 256, 256),
        sdf_values,
        true,
        extracted_mesh->vertices_,
        extracted_mesh->triangles_,
        extracted_mesh->triangle_normals_,
        true);
    open3d::io::WriteTriangleMesh(test_output_dir / "extracted_mesh.ply", *extracted_mesh, true);
}
