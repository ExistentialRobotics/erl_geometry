#include "erl_common/pybind11.hpp"
#include "erl_geometry/hidden_point_removal.hpp"

void
BindHiddenPointRemoval(py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    m.def(
         "hidden_point_removal",
         [](const Eigen::Ref<const Eigen::Matrix3Xd> &points,
            const Eigen::Ref<const Eigen::Vector3d> &view_position,
            const double radius,
            const bool fast,
            const bool joggle_inputs,
            const bool return_meshes) {
             std::vector<long> visible_point_indices;
             py::dict result;
             if (return_meshes) {
                 Eigen::Matrix3Xl mesh_triangles;
                 Eigen::Matrix3Xd mesh_vertices;
                 HiddenPointRemoval(points, view_position, radius, mesh_triangles, mesh_vertices, visible_point_indices, fast, joggle_inputs);
                 result["triangles"] = mesh_triangles;
                 result["vertices"] = mesh_vertices;
             } else {
                 HiddenPointRemoval(points, view_position, radius, visible_point_indices, fast, joggle_inputs);
             }
             result["visible_point_indices"] = visible_point_indices;
             return result;
         },
         py::arg("points"),
         py::arg("view_position"),
         py::arg("radius"),
         py::arg("fast") = false,
         py::arg("joggle_inputs") = false,
         py::arg("return_meshes") = false,
         "Remove hidden points from the point cloud.\n\n"
         "Args:\n"
         "    points: A 3xN matrix of points.\n"
         "    view_position: The position of the camera.\n"
         "    radius: The radius of the sphere.\n"
         "    fast: If true, will run QHull with `Q3 Q5 Q8`.\n"
         "    joggle_inputs: If true, will run QHull with `QJ`.\n"
         "Returns:\n"
         "    A dictionary containing the visible point indices and optionally the mesh vertices and triangles.")
        .def(
            "parallel_hidden_point_removal",
            [](const Eigen::Ref<const Eigen::Matrix3Xd> &points,
               const Eigen::Ref<const Eigen::Matrix3Xd> &view_positions,
               const Eigen::Ref<const Eigen::VectorXd> &radii,
               const bool fast,
               const bool joggle_inputs,
               const bool return_meshes) {
                std::vector<std::vector<long>> visible_point_indices;
                py::dict result;
                if (return_meshes) {
                    std::vector<Eigen::Matrix3Xl> mesh_triangles;
                    std::vector<Eigen::Matrix3Xd> mesh_vertices;
                    ParallelHiddenPointRemoval(points, view_positions, radii, mesh_triangles, mesh_vertices, visible_point_indices, fast, joggle_inputs);
                    result["triangles"] = mesh_triangles;
                    result["vertices"] = mesh_vertices;
                } else {
                    ParallelHiddenPointRemoval(points, view_positions, radii, visible_point_indices, fast, joggle_inputs);
                }
                result["visible_point_indices"] = visible_point_indices;
                return result;
            },
            py::arg("points"),
            py::arg("view_positions"),
            py::arg("radii"),
            py::arg("fast") = false,
            py::arg("joggle_inputs") = false,
            py::arg("return_meshes") = false,
            "Remove hidden points from the point cloud w.r.t. multiple camera positions. The function is parallelized.\n\n"
            "Args:\n"
            "    points: A 3xN matrix of points.\n"
            "    view_positions: A 3xM matrix of camera positions.\n"
            "    radii: The radius of the spherical projection.\n"
            "    fast: If true, will run QHull with `Q3 Q5 Q8`.\n"
            "    joggle_inputs: If true, will run QHull with `QJ`.\n"
            "Returns:\n"
            "    A dictionary containing the visible point indices and optionally the mesh vertices and triangles for each camera position.");
}
