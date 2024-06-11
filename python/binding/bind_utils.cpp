#include "erl_common/pybind11.hpp"
#include "erl_common/random.hpp"
#include "erl_geometry/bresenham_2d.hpp"
#include "erl_geometry/marching_square.hpp"
#include "erl_geometry/utils.hpp"
#include "erl_geometry/winding_number.hpp"

void
BindUtils(py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    m.def("manually_set_seed", &ManuallySetSeed, py::arg("seed"))
        .def(
            "bresenham_2d",
            [](const Eigen::Ref<const Eigen::Vector2i> &start,
               const Eigen::Ref<const Eigen::Vector2i> &end,
               const std::optional<std::function<bool(long, long)>> &stop) {
                return stop.has_value() ? Bresenham2D(start, end, stop.value()) : Bresenham2D(start, end);
            },
            py::arg("start"),
            py::arg("end"),
            py::arg("stop") = py::none())
        .def("compute_pixels_of_polygon_contour", &ComputePixelsOfPolygonContour, py::arg("polygon_vertices"))
        .def(
            "marching_square",
            [](const Eigen::Ref<const Eigen::MatrixXd> &img, const double iso_value) {
                Eigen::Matrix2Xd vertices;
                Eigen::Matrix2Xi lines_to_vertices;
                Eigen::Matrix2Xi objects_to_lines;
                MarchingSquare(img, iso_value, vertices, lines_to_vertices, objects_to_lines);

                return py::make_tuple(vertices, lines_to_vertices, objects_to_lines);
            },
            py::arg("img"),
            py::arg("iso_value"))
        .def("winding_number", &WindingNumber, py::arg("p"), py::arg("vertices"))
        .def(
            "compute_nearest_distance_from_point_to_line_segment_2d",
            &ComputeNearestDistanceFromPointToLineSegment2D,
            py::arg("point_x"),
            py::arg("point_y"),
            py::arg("line_segment_x1"),
            py::arg("line_segment_y1"),
            py::arg("line_segment_x2"),
            py::arg("line_segment_y2"))
        .def(
            "compute_intersection_between_ray_and_segment_2d",
            [](const Eigen::Ref<const Eigen::Vector2d> &ray_start_point,
               const Eigen::Ref<const Eigen::Vector2d> &ray_direction,
               const Eigen::Ref<const Eigen::Vector2d> &segment_point1,
               const Eigen::Ref<const Eigen::Vector2d> &segment_point2) {
                double lambda = 0;
                double distance = 0;
                ComputeIntersectionBetweenRayAndSegment2D(ray_start_point, ray_direction, segment_point1, segment_point2, lambda, distance);
                return py::make_tuple(lambda, distance);
            },
            py::arg("ray_start_point"),
            py::arg("ray_direction"),
            py::arg("segment_point1"),
            py::arg("segment_point2"))
        .def(
            "compute_intersection_between_ray_and_aabb_2d",
            [](const Eigen::Ref<const Eigen::Vector2d> &p,
               const Eigen::Ref<const Eigen::Vector2d> &r,
               const Eigen::Ref<const Eigen::Vector2d> &box_min,
               const Eigen::Ref<const Eigen::Vector2d> &box_max) {
                double d1 = 0.0, d2 = 0.0;
                bool intersected = false;
                ComputeIntersectionBetweenRayAndAabb2D(p, r.cwiseInverse(), box_min, box_max, d1, d2, intersected);
                return py::make_tuple(d1, d2, intersected);
            },
            py::arg("ray_start_point"),
            py::arg("ray_direction"),
            py::arg("aabb_min"),
            py::arg("aabb_max"))
        .def(
            "compute_intersection_between_ray_and_aabb_3d",
            [](const Eigen::Ref<const Eigen::Vector3d> &p,
               const Eigen::Ref<const Eigen::Vector3d> &r,
               const Eigen::Ref<const Eigen::Vector3d> &box_min,
               const Eigen::Ref<const Eigen::Vector3d> &box_max) {
                double d1 = 0.0, d2 = 0.0;
                bool intersected = false;
                ComputeIntersectionBetweenRayAndAabb3D(p, r.cwiseInverse(), box_min, box_max, d1, d2, intersected);
                return py::make_tuple(d1, d2, intersected);
            },
            py::arg("ray_start_point"),
            py::arg("ray_direction"),
            py::arg("aabb_min"),
            py::arg("aabb_max"))
        .def("convert_path_2d_to_3d", &ConvertPath2dTo3d, py::arg("path_2d"), py::arg("z"));
}
