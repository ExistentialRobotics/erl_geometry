#include "erl_common/grid_map_info.hpp"
#include "erl_common/plplot_fig.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_geometry/mesh_sdf.hpp"

#include <open3d/geometry/BoundingVolume.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/t/geometry/RaycastingScene.h>

TEST(MeshSdf, Sphere) {
    GTEST_PREPARE_OUTPUT_DIR();
    constexpr double radius = 1.0;
    constexpr int n = 41;
    constexpr int resolution = 100;
    auto mesh = open3d::geometry::TriangleMesh::CreateSphere(radius, resolution);
    // generate test data
    erl::common::GridMapInfo3Dd grid_map_info(
        Eigen::Vector3i(n, n, n),
        Eigen::Vector3d(-2, -2, -2),
        Eigen::Vector3d(2, 2, 2));
    // axis varying order: x, y, z
    Eigen::Matrix3Xd positions = grid_map_info.GenerateMeterCoordinates(false);
    Eigen::VectorXd sdf_gt_values = positions.colwise().norm().array() - radius;
    // test
    erl::geometry::MeshSdf sdf(mesh->vertices_, mesh->triangles_, true);
    Eigen::VectorXd sdf_values = sdf(positions);
    // check
    Eigen::VectorXd abs_err = (sdf_values - sdf_gt_values).cwiseAbs();
    double max_abs_err = abs_err.maxCoeff();
    double mean_abs_err = abs_err.mean();
    // std::cout << "max abs err: " << max_abs_err << std::endl;    // 0.000245438
    // std::cout << "mean abs err: " << mean_abs_err << std::endl;  // 0.00010277
    ASSERT_TRUE(max_abs_err < 2.5e-4);
    ASSERT_TRUE(mean_abs_err < 1.1e-4);

    Eigen::VectorXd sdf_gt_values_xy = sdf_gt_values.segment<n * n>(n * n * 20);
    Eigen::VectorXd sdf_values_xy = sdf_values.segment<n * n>(n * n * 20);
    Eigen::Matrix3Xd positions_xy = positions.block<3, n * n>(0, n * n * 20);

    const double xmin = positions_xy.row(0).minCoeff();
    const double xmax = positions_xy.row(0).maxCoeff();
    const double ymin = positions_xy.row(1).minCoeff();
    const double ymax = positions_xy.row(1).maxCoeff();

    using namespace erl::common;
    PlplotFig fig(640, 640, true);
    PlplotFig::ShadesOpt shades_opt;
    shades_opt.SetColorLevels(sdf_gt_values_xy.data(), n, n, 127)
        .SetXMin(xmin)
        .SetXMax(xmax)
        .SetYMin(ymin)
        .SetYMax(ymax);
    PlplotFig::ColorBarOpt color_bar_opt;
    color_bar_opt.SetLabelOpts({PL_COLORBAR_LABEL_BOTTOM})
        .SetLabelTexts({"SDF"})
        .AddColorMap(0, shades_opt.color_levels, 10);
    fig.Clear()
        .SetMargin(0.15, 0.85, 0.15, 0.85)
        .SetAxisLimits(xmin, xmax, ymin, ymax)
        .SetCurrentColor(PlplotFig::Color0::Black)
        .SetColorMap(1, PlplotFig::ColorMap::Jet)
        .Shades(sdf_gt_values_xy.data(), n, n, true, shades_opt)
        .SetCurrentColor(PlplotFig::Color0::Black)
        .DrawContour(sdf_gt_values_xy.data(), n, n, xmin, xmax, ymin, ymax, true, {0.0})
        .ColorBar(color_bar_opt)
        .DrawAxesBox(PlplotFig::AxisOpt(), PlplotFig::AxisOpt().DrawPerpendicularTickLabels())
        .SetAxisLabelX("x")
        .SetAxisLabelY("y")
        .SetTitle("Ground Truth SDF");
    cv::imshow(test_info_->name() + std::string(": SDF G.T."), fig.ToCvMat());
    cv::imwrite(test_output_dir / "sdf_gt.png", fig.ToCvMat());

    shades_opt.SetColorLevels(sdf_values_xy.data(), n, n, 127);
    color_bar_opt.SetLabelOpts({PL_COLORBAR_LABEL_BOTTOM})
        .SetLabelTexts({"SDF"})
        .AddColorMap(0, shades_opt.color_levels, 10);
    fig.Clear()
        .SetMargin(0.15, 0.85, 0.15, 0.85)
        .SetAxisLimits(xmin, xmax, ymin, ymax)
        .SetCurrentColor(PlplotFig::Color0::Black)
        .SetColorMap(1, PlplotFig::ColorMap::Jet)
        .Shades(sdf_values_xy.data(), n, n, true, shades_opt)
        .SetCurrentColor(PlplotFig::Color0::Black)
        .DrawContour(sdf_values_xy.data(), n, n, xmin, xmax, ymin, ymax, true, {0.0})
        .ColorBar(color_bar_opt)
        .DrawAxesBox(PlplotFig::AxisOpt(), PlplotFig::AxisOpt().DrawPerpendicularTickLabels())
        .SetAxisLabelX("x")
        .SetAxisLabelY("y")
        .SetTitle("MeshSdf Output");
    cv::imshow(test_info_->name() + std::string(": SDF"), fig.ToCvMat());
    cv::imwrite(test_output_dir / "sdf.png", fig.ToCvMat());

    cv::waitKey(100);

    // open3d::visualization::DrawGeometries({mesh});
}

TEST(MeshSdf, Bunny) {
    std::filesystem::path kProjectDir = ERL_GEOMETRY_ROOT_DIR;
    std::filesystem::path kDataDir = kProjectDir / "data";
    std::filesystem::path kMeshFile = kDataDir / "bunny_z_up.ply";
    GTEST_PREPARE_OUTPUT_DIR();
    constexpr int n = 41;
    auto mesh = open3d::io::CreateMeshFromFile(kMeshFile);
    ASSERT_TRUE(mesh != nullptr);
    // scale mesh to [-1.2, 1.2]
    auto aabb = mesh->GetAxisAlignedBoundingBox();
    mesh->Translate(-aabb.GetCenter(), true);
    const double scale_org = (aabb.GetExtent() / 2).maxCoeff();
    mesh->Scale(1.2 / scale_org, Eigen::Vector3d::Zero());
    mesh->ComputeTriangleNormals();
    mesh->ComputeVertexNormals();
    // open3d::io::WriteTriangleMesh(test_output_dir / "bunny.ply", *mesh, true);
    // open3d::visualization::DrawGeometries({mesh});

    // generate test data
    erl::common::GridMapInfo3Dd grid_map_info(
        Eigen::Vector3i(n, n, n),
        Eigen::Vector3d(-2, -2, -2),
        Eigen::Vector3d(2, 2, 2));
    // axis varying order: z, y, x
    Eigen::Matrix3Xd positions = grid_map_info.GenerateMeterCoordinates(true);
    open3d::t::geometry::RaycastingScene scene;
    scene.AddTriangles(open3d::t::geometry::TriangleMesh::FromLegacy(*mesh));
    open3d::core::Tensor positions_tensor({positions.cols(), 3}, open3d::core::Dtype::Float32);
    Eigen::Map<Eigen::Matrix3Xf>(
        static_cast<float *>(positions_tensor.GetDataPtr()),
        3,
        positions.cols()) = Eigen::Matrix3Xf(positions.cast<float>());
    auto sdf_gt_tensor = scene.ComputeSignedDistance(
        positions_tensor,
        static_cast<int>(std::thread::hardware_concurrency()));
    Eigen::VectorXd sdf_gt_values = Eigen::Map<Eigen::VectorXf>(
                                        static_cast<float *>(sdf_gt_tensor.GetDataPtr()),
                                        positions.cols())
                                        .cast<double>();

    // test
    erl::geometry::MeshSdf sdf(mesh->vertices_, mesh->triangles_, true);
    Eigen::VectorXd sdf_values = sdf(positions);

    // check
    Eigen::VectorXd abs_err = (sdf_values - sdf_gt_values).cwiseAbs();
    double max_abs_err = abs_err.maxCoeff();
    double mean_abs_err = abs_err.mean();
    // std::cout << "max abs err: " << max_abs_err << std::endl;    // 0.00151204
    // std::cout << "mean abs err: " << mean_abs_err << std::endl;  // 3.33084e-07
    ASSERT_LE(max_abs_err, 0.0016);
    ASSERT_LE(mean_abs_err, 1.0e-6);

    Eigen::VectorXd sdf_gt_values_yz = sdf_gt_values.segment<n * n>(n * n * 20);  // an x-slice
    Eigen::VectorXd sdf_values_yz = sdf_values.segment<n * n>(n * n * 20);        // an x-slice
    Eigen::Matrix3Xd positions_yz = positions.block<3, n * n>(0, n * n * 20);

    const double ymin = positions_yz.row(1).minCoeff();
    const double ymax = positions_yz.row(1).maxCoeff();
    const double zmin = positions_yz.row(2).minCoeff();
    const double zmax = positions_yz.row(2).maxCoeff();

    using namespace erl::common;
    PlplotFig fig(640, 640, true);
    PlplotFig::ShadesOpt shades_opt;
    shades_opt.SetColorLevels(sdf_gt_values_yz.data(), n, n, 127)
        .SetXMin(ymin)
        .SetXMax(ymax)
        .SetYMin(zmin)
        .SetYMax(zmax);
    //.SetContourColor0(PlplotFig::Color0::White)
    //.SetContourLineWidth(1);
    PlplotFig::ColorBarOpt color_bar_opt;
    color_bar_opt.SetLabelOpts({PL_COLORBAR_LABEL_BOTTOM})
        .SetLabelTexts({"SDF"})
        .AddColorMap(0, shades_opt.color_levels, 10);

    fig.Clear()
        .SetMargin(0.15, 0.85, 0.15, 0.85)
        .SetAxisLimits(ymin, ymax, zmin, zmax)
        .SetCurrentColor(PlplotFig::Color0::Black)
        .SetAreaFillPattern(PlplotFig::AreaFillPattern::Solid)
        .SetColorMap(1, PlplotFig::ColorMap::Jet)
        .Shades(sdf_gt_values_yz.data(), n, n, true, shades_opt)
        .SetCurrentColor(PlplotFig::Color0::Black)
        .DrawContour(sdf_gt_values_yz.data(), n, n, ymin, ymax, zmin, zmax, true, {0.0})
        .ColorBar(color_bar_opt)
        .DrawAxesBox(PlplotFig::AxisOpt(), PlplotFig::AxisOpt().DrawPerpendicularTickLabels())
        .SetAxisLabelX("y")
        .SetAxisLabelY("z")
        .SetTitle("Ground Truth SDF");
    cv::imshow(test_info_->name() + std::string(": SDF G.T."), fig.ToCvMat());
    cv::imwrite(test_output_dir / "sdf_gt.png", fig.ToCvMat());

    shades_opt.SetColorLevels(sdf_values_yz.data(), n, n, 127);
    color_bar_opt.SetLabelOpts({PL_COLORBAR_LABEL_BOTTOM})
        .SetLabelTexts({"SDF"})
        .AddColorMap(0, shades_opt.color_levels, 10);
    fig.Clear()
        .SetMargin(0.15, 0.85, 0.15, 0.85)
        .SetAxisLimits(ymin, ymax, zmin, zmax)
        .SetCurrentColor(PlplotFig::Color0::Black)
        .SetColorMap(1, PlplotFig::ColorMap::Jet)
        .Shades(sdf_values_yz.data(), n, n, true, shades_opt)
        .SetCurrentColor(PlplotFig::Color0::Black)
        .DrawContour(sdf_values_yz.data(), n, n, ymin, ymax, zmin, zmax, true, {0.0})
        .ColorBar(color_bar_opt)
        .DrawAxesBox(PlplotFig::AxisOpt(), PlplotFig::AxisOpt().DrawPerpendicularTickLabels())
        .SetAxisLabelX("y")
        .SetAxisLabelY("z")
        .SetTitle("MeshSdf Output");
    cv::imshow(test_info_->name() + std::string(": SDF X=0"), fig.ToCvMat());
    cv::imwrite(test_output_dir / "sdf.png", fig.ToCvMat());
    cv::waitKey(100);
}

TEST(MeshSdf, HouseExpo) {
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
    Eigen::VectorXd sdf_values =
        erl::geometry::MeshSdf(mesh->vertices_, mesh->triangles_, false, false)(positions);

    // check
    Eigen::VectorXd abs_err = (sdf_values - sdf_gt_values).cwiseAbs();
    double max_abs_err = abs_err.maxCoeff();
    double mean_abs_err = abs_err.mean();
    std::cout << "max abs err: " << max_abs_err << std::endl;
    std::cout << "mean abs err: " << mean_abs_err << std::endl;
    // EXPECT_LE(max_abs_err, 0.0016);
    // EXPECT_LE(mean_abs_err, 1.0e-6);

    Eigen::VectorXd sdf_gt_values_xy = sdf_gt_values.segment<n * n>(n * n * (n / 2));  // a z-slice
    Eigen::VectorXd sdf_values_xy = sdf_values.segment<n * n>(n * n * (n / 2));        // a z-slice
    Eigen::Matrix3Xd positions_xy = positions.block<3, n * n>(0, n * n * (n / 2));

    const double xmin = positions_xy.row(c_stride ? 1 : 0).minCoeff();
    const double xmax = positions_xy.row(c_stride ? 1 : 0).maxCoeff();
    const double ymin = positions_xy.row(c_stride ? 2 : 1).minCoeff();
    const double ymax = positions_xy.row(c_stride ? 2 : 1).maxCoeff();

    using namespace erl::common;
    PlplotFig fig(640, 640, true);
    PlplotFig::ShadesOpt shades_opt;
    shades_opt.SetColorLevels(sdf_gt_values_xy.data(), n, n, 127)
        .SetXMin(xmin)
        .SetXMax(xmax)
        .SetYMin(ymin)
        .SetYMax(ymax);
    PlplotFig::ColorBarOpt color_bar_opt;
    color_bar_opt.SetLabelOpts({PL_COLORBAR_LABEL_BOTTOM})
        .SetLabelTexts({"SDF"})
        .AddColorMap(0, shades_opt.color_levels, 10);

    fig.Clear()
        .SetMargin(0.15, 0.85, 0.15, 0.85)
        .SetAxisLimits(xmin, xmax, ymin, ymax)
        .SetCurrentColor(PlplotFig::Color0::Black)
        .SetAreaFillPattern(PlplotFig::AreaFillPattern::Solid)
        .SetColorMap(1, PlplotFig::ColorMap::Jet)
        .Shades(sdf_gt_values_xy.data(), n, n, true, shades_opt)
        .SetCurrentColor(PlplotFig::Color0::Black)
        .DrawContour(sdf_gt_values_xy.data(), n, n, xmin, xmax, ymin, ymax, true, {0.0})
        .ColorBar(color_bar_opt)
        .DrawAxesBox(PlplotFig::AxisOpt(), PlplotFig::AxisOpt().DrawPerpendicularTickLabels())
        .SetAxisLabelX(c_stride ? "y" : "x")
        .SetAxisLabelY(c_stride ? "z" : "y")
        .SetTitle("Ground Truth SDF");
    cv::imshow(test_info_->name() + std::string(": SDF G.T."), fig.ToCvMat());
    cv::imwrite(test_output_dir / "sdf_gt.png", fig.ToCvMat());

    shades_opt.SetColorLevels(sdf_values_xy.data(), n, n, 127);
    color_bar_opt.SetLabelOpts({PL_COLORBAR_LABEL_BOTTOM})
        .SetLabelTexts({"SDF"})
        .AddColorMap(0, shades_opt.color_levels, 10);
    fig.Clear()
        .SetMargin(0.15, 0.85, 0.15, 0.85)
        .SetAxisLimits(xmin, xmax, ymin, ymax)
        .SetCurrentColor(PlplotFig::Color0::Black)
        .SetColorMap(1, PlplotFig::ColorMap::Jet)
        .Shades(sdf_values_xy.data(), n, n, true, shades_opt)
        .SetCurrentColor(PlplotFig::Color0::Black)
        .DrawContour(sdf_values_xy.data(), n, n, xmin, xmax, ymin, ymax, true, {0.0})
        .ColorBar(color_bar_opt)
        .DrawAxesBox(PlplotFig::AxisOpt(), PlplotFig::AxisOpt().DrawPerpendicularTickLabels())
        .SetAxisLabelX("y")
        .SetAxisLabelY("z")
        .SetTitle("MeshSdf Output");
    cv::imshow(test_info_->name() + std::string(": SDF X=0"), fig.ToCvMat());
    cv::imwrite(test_output_dir / "sdf.png", fig.ToCvMat());
    cv::waitKey(100);
}
