#include "erl_common/pybind11.hpp"
#include "erl_geometry/space_2d.hpp"

void
BindSpace2D(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    auto py_space = py::class_<Space2D, std::shared_ptr<Space2D>>(m, "Space2D");

    py::enum_<Space2D::SignMethod>(py_space, "SignMethod", py::arithmetic(), "Algorithm to determine SDF sign.")
        .value(
            Space2D::GetSignMethodName(Space2D::SignMethod::kPointNormal),
            Space2D::SignMethod::kPointNormal,
            "Use the normal of the nearest vertex to determine the sign.")
        .value(
            Space2D::GetSignMethodName(Space2D::SignMethod::kLineNormal),
            Space2D::SignMethod::kLineNormal,
            "Use the normal of the nearest line segment to determine the sign.")
        .value(
            Space2D::GetSignMethodName(Space2D::SignMethod::kPolygon),
            Space2D::SignMethod::kPolygon,
            "Use the nearest object polygon and the winding number algorithm to determine the sign.")
        .export_values();

    py_space
        .def(
            py::init<const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &, const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &>(),
            py::arg("ordered_object_vertices"),
            py::arg("ordered_object_normals"))
        .def(
            py::init<const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &, const Eigen::Ref<const Eigen::VectorXb> &, double, bool>(),
            py::arg("ordered_object_vertices"),
            py::arg("outside_flags"),
            py::arg("delta") = 0.01,
            py::arg("parallel") = false)
        .def(
            py::init<const Eigen::Ref<const Eigen::MatrixXd> &, const GridMapInfo2Dd &, double, double, bool>(),
            py::arg("map_image"),
            py::arg("grid_map_info"),
            py::arg("free_threshold"),
            py::arg("delta") = 0.01,
            py::arg("parallel") = false)
        .def(py::init<const Space2D &>(), py::arg("space2d"))
        .def_static("get_sign_method_name", &Space2D::GetSignMethodName, py::arg("sign_method"))
        .def_static("get_sign_method_from_name", &Space2D::GetSignMethodFromName, py::arg("sign_method_name"))
        .def_property_readonly("surface", &Space2D::GetSurface)
        .def("generate_map_image", &Space2D::GenerateMapImage, py::arg("grid_map_info"), py::arg("anti_aliased") = false)
        .def(
            "compute_sdf_image",
            &Space2D::ComputeSdfImage,
            py::arg("grid_map_info"),
            py::arg("sign_method") = Space2D::SignMethod::kLineNormal,
            py::arg("use_kdtree") = false,
            py::arg("parallel") = false)
        .def(
            "compute_sdf",
            &Space2D::ComputeSdf,
            py::arg("query_points"),
            py::arg("sign_method") = Space2D::SignMethod::kLineNormal,
            py::arg("use_kd_tree") = false,
            py::arg("parallel") = false)
        .def("compute_sdf_with_kdtree", &Space2D::ComputeSdfWithKdtree, py::arg("q"), py::arg("sign_method"))
        .def("compute_sdf_greedily", &Space2D::ComputeSdfGreedily, py::arg("q"), py::arg("sign_method"))
        .def(
            "compute_ddf",
            [](const Space2D &space, const GridMapInfo2Dd &grid_map_info, const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions, const bool parallel) {
                auto result = space.ComputeDdf(grid_map_info, query_directions, parallel);
                const long n_rows = result.rows();
                const long n_cols = result.cols();
                const long n_dirs = query_directions.cols();
                py::array_t<double> out({n_rows, n_cols, n_dirs});
                for (long i = 0; i < n_rows; i++) {
                    for (long j = 0; j < n_cols; j++) {
                        for (long k = 0; k < n_dirs; k++) { out.mutable_at(i, j, k) = result(i, j)[k]; }
                    }
                }
                return out;
            },
            py::arg("grid_map_info"),
            py::arg("query_directions"),
            py::arg("parallel") = false)
        .def(
            "compute_ddf",
            py::overload_cast<const Eigen::Ref<const Eigen::Matrix2Xd> &, const Eigen::Ref<const Eigen::Matrix2Xd> &, bool>(&Space2D::ComputeDdf, py::const_),
            py::arg("query_points"),
            py::arg("query_directions"),
            py::arg("parallel") = false)
        .def(
            "compute_sddf_v1",
            [](const Space2D &space, const GridMapInfo2Dd &grid_map_info, const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions, const bool parallel) {
                auto result = space.ComputeSddfV1(grid_map_info, query_directions, parallel);
                const long n_rows = result.rows();
                const long n_cols = result.cols();
                const long n_dirs = query_directions.cols();
                py::array_t<double> out({n_rows, n_cols, n_dirs});
                for (long i = 0; i < n_rows; i++) {
                    for (long j = 0; j < n_cols; j++) {
                        for (long k = 0; k < n_dirs; k++) { out.mutable_at(i, j, k) = result(i, j)[k]; }
                    }
                }
                return out;
            },
            py::arg("grid_map_info"),
            py::arg("query_directions"),
            py::arg("parallel") = false)
        .def(
            "compute_sddf_v1",
            py::overload_cast<const Eigen::Ref<const Eigen::Matrix2Xd> &, const Eigen::Ref<const Eigen::Matrix2Xd> &, bool>(
                &Space2D::ComputeSddfV1,
                py::const_),
            py::arg("query_points"),
            py::arg("query_directions"),
            py::arg("parallel") = false)
        .def(
            "compute_sddf_v2",
            [](const Space2D &space,
               const GridMapInfo2Dd &grid_map_info,
               const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions,
               const Space2D::SignMethod sign_method,
               const bool parallel) {
                auto result = space.ComputeSddfV2(grid_map_info, query_directions, sign_method, parallel);
                const long n_rows = result.rows();
                const long n_cols = result.cols();
                const long n_dirs = query_directions.cols();
                py::array_t<double> out({n_rows, n_cols, n_dirs});
                for (long i = 0; i < n_rows; i++) {
                    for (long j = 0; j < n_cols; j++) {
                        for (long k = 0; k < n_dirs; k++) { out.mutable_at(i, j, k) = result(i, j)[k]; }
                    }
                }
                return out;
            },
            py::arg("map_image_info"),
            py::arg("query_directions"),
            py::arg("sign_method") = Space2D::SignMethod::kLineNormal,
            py::arg("parallel") = false)
        .def(
            "compute_sddf_v2",
            py::overload_cast<const Eigen::Ref<const Eigen::Matrix2Xd> &, const Eigen::Ref<const Eigen::Matrix2Xd> &, Space2D::SignMethod, bool>(
                &Space2D::ComputeSddfV2,
                py::const_),
            py::arg("query_points"),
            py::arg("query_directions"),
            py::arg("sign_method") = Space2D::SignMethod::kLineNormal,
            py::arg("parallel") = false);
}
