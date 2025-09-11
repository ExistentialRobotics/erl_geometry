#include "erl_common/pybind11.hpp"
#include "erl_geometry/marching_cubes.hpp"

void
BindMarchingCubes(const py::module &m) {
    using MC = erl::geometry::MarchingCubes;
    py::class_<MC> mc(m, "MarchingCubes");
    py::class_<MC::ValidCube>(mc, "ValidCube")
        .def(py::init<>())
        .def_readwrite("coords", &MC::ValidCube::coords)
        .def_readwrite("cfg_index", &MC::ValidCube::cfg_index)
        .def_readwrite("edges", &MC::ValidCube::edges);
    mc.def_static(
          "get_vertex_offsets",
          []() {
              py::array_t<int> arr({8, 3});
              for (int i = 0; i < 8; ++i) {
                  for (int j = 0; j < 3; ++j) { arr.mutable_at(i, j) = MC::kCubeVertexCodes[i][j]; }
              }
              return arr;
          })
        .def_static(
            "single_cube",
            [](const Eigen::Ref<const Eigen::Matrix<double, 3, 8>> &vertex_coords,
               const Eigen::Ref<const Eigen::Vector<double, 8>> &grid_values,
               const double iso_value) {
                std::vector<Eigen::Vector3d> vertices;
                std::vector<Eigen::Vector3i> triangles;
                std::vector<Eigen::Vector3d> face_normals;
                MC::SingleCube(
                    vertex_coords,
                    grid_values,
                    iso_value,
                    vertices,
                    triangles,
                    face_normals);
                return py::make_tuple(vertices, triangles, face_normals);
            },
            py::arg("vertex_coords"),
            py::arg("grid_values"),
            py::arg("iso_value"))
        .def_static(
            "calculate_cube_cfg_index",
            [](const Eigen::Vector<double, 8> &vertex_values, const double iso_value) {
                return MC::CalculateVertexConfigIndex(vertex_values.data(), iso_value);
            },
            py::arg("vertex_values"),
            py::arg("iso_value"))
        .def_static(
            "collect_valid_cubes",
            [](const Eigen::Ref<const Eigen::Vector3i> &grid_shape,
               const Eigen::Ref<const Eigen::VectorXd> &grid_values,
               const double iso_value,
               const bool row_major,
               const bool parallel) {
                return MC::CollectValidCubes(
                    grid_shape,
                    grid_values,
                    iso_value,
                    row_major,
                    parallel);
            },
            py::arg("grid_shape"),
            py::arg("grid_values"),
            py::arg("iso_value"),
            py::arg("row_major") = true,
            py::arg("parallel") = false)
        .def_static(
            "process_valid_cubes",
            [](const std::vector<std::vector<MC::ValidCube>> &valid_cubes,
               const Eigen::Ref<const Eigen::Vector3d> &coords_min,
               const Eigen::Ref<const Eigen::Vector3d> &grid_res,
               const Eigen::Ref<const Eigen::Vector3i> &grid_shape,
               const Eigen::Ref<const Eigen::VectorXd> &grid_values,
               const double iso_value,
               const bool row_major,
               const bool parallel) {
                std::vector<Eigen::Vector3d> vertices;
                std::vector<Eigen::Vector3i> triangles;
                std::vector<Eigen::Vector3d> face_normals;
                MC::ProcessValidCubes(
                    valid_cubes,
                    coords_min,
                    grid_res,
                    grid_shape,
                    grid_values,
                    iso_value,
                    row_major,
                    parallel,
                    vertices,
                    triangles,
                    face_normals);
                return py::make_tuple(vertices, triangles, face_normals);
            },
            py::arg("valid_cubes"),
            py::arg("coords_min"),
            py::arg("grid_res"),
            py::arg("grid_shape"),
            py::arg("grid_values"),
            py::arg("iso_value"),
            py::arg("row_major") = true,
            py::arg("parallel") = false)
        .def_static(
            "run",
            [](const Eigen::Ref<const Eigen::Vector3d> &coords_min,
               const Eigen::Ref<const Eigen::Vector3d> &grid_res,
               const Eigen::Ref<const Eigen::Vector3i> &grid_shape,
               const Eigen::Ref<const Eigen::VectorXd> &grid_values,
               const float iso_value,
               const bool row_major,
               const bool parallel) {
                std::vector<Eigen::Vector3d> vertices;
                std::vector<Eigen::Vector3i> triangles;
                std::vector<Eigen::Vector3d> face_normals;
                MC::Run(
                    coords_min,
                    grid_res,
                    grid_shape,
                    grid_values,
                    iso_value,
                    row_major,
                    parallel,
                    vertices,
                    triangles,
                    face_normals);
                return py::make_tuple(vertices, triangles, face_normals);
            },
            py::arg("coords_min"),
            py::arg("grid_res"),
            py::arg("grid_shape"),
            py::arg("grid_values"),
            py::arg("iso_value"),
            py::arg("row_major") = true,
            py::arg("parallel") = false);
}
