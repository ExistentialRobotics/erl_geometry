#include "erl_common/pybind11.hpp"
#include "erl_geometry/intersection.hpp"

void
BindIntersection(py::module &m) {
    using namespace erl::geometry;

    m.def(
         "compute_nearest_distance_from_point_to_line_segment_2d",
         &ComputeNearestDistanceFromPointToLineSegment2D,
         py::arg("point_x"),
         py::arg("point_y"),
         py::arg("line_segment_x1"),
         py::arg("line_segment_y1"),
         py::arg("line_segment_x2"),
         py::arg("line_segment_y2"))
        .def(
            "compute_intersection_between_ray_and_line_2d",
            [](const Eigen::Ref<const Eigen::Vector2d> &ray_start_point,
               const Eigen::Ref<const Eigen::Vector2d> &ray_direction,
               const Eigen::Ref<const Eigen::Vector2d> &segment_point1,
               const Eigen::Ref<const Eigen::Vector2d> &segment_point2) {
                double lambda = 0;
                double distance = 0;
                bool intersected = false;
                ComputeIntersectionBetweenRayAndLine2D(ray_start_point, ray_direction, segment_point1, segment_point2, lambda, distance, intersected);
                return py::make_tuple(lambda, distance, intersected);
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
        .def(
            "compute_intersection_between_line_and_ellipse_2d",
            [](const double x0, const double y0, const double x1, const double y1, const double a, const double b) {
                double lam1 = 0.0, lam2 = 0.0;
                bool intersected = false;
                ComputeIntersectionBetweenLineAndEllipse2D(x0, y0, x1, y1, a, b, lam1, lam2, intersected);
                return py::make_tuple(lam1, lam2, intersected);
            },
            py::arg("x0"),
            py::arg("y0"),
            py::arg("x1"),
            py::arg("y1"),
            py::arg("a"),
            py::arg("b"))
        .def(
            "compute_intersection_between_line_and_ellipsoid_3d",
            [](const double x0,
               const double y0,
               const double z0,
               const double x1,
               const double y1,
               const double z1,
               const double a,
               const double b,
               const double c) {
                double lam1 = 0.0, lam2 = 0.0;
                bool intersected = false;
                ComputeIntersectionBetweenLineAndEllipsoid3D(x0, y0, z0, x1, y1, z1, a, b, c, lam1, lam2, intersected);
                return py::make_tuple(lam1, lam2, intersected);
            },
            py::arg("x0"),
            py::arg("y0"),
            py::arg("z0"),
            py::arg("x1"),
            py::arg("y1"),
            py::arg("z1"),
            py::arg("a"),
            py::arg("b"),
            py::arg("c"));
}
