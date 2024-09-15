#include "erl_common/pybind11.hpp"
#include "erl_geometry/primitives_2d.hpp"

using namespace erl::geometry;

void
BindPrimitives2D(const py::module &m) {
    auto primitive = py::class_<Primitive2D>(m, "Primitive2D");
    py::enum_<Primitive2D::Type>(primitive, "Type")
        .value("kLine2D", Primitive2D::Type::kLine2D)
        .value("kSegment2D", Primitive2D::Type::kSegment2D)
        .value("kRay2D", Primitive2D::Type::kRay2D)
        .value("kAxisAlignedRectangle", Primitive2D::Type::kAxisAlignedRectangle)
        .value("kRectangle", Primitive2D::Type::kRectangle)
        .value("kEllipse", Primitive2D::Type::kEllipse)
        .export_values();
    primitive.def_readwrite("id", &Primitive2D::id)
        .def_property_readonly("type", &Primitive2D::GetType)
        .def("is_inside", &Primitive2D::IsInside, py::arg("point"))
        .def("is_on_boundary", &Primitive2D::IsOnBoundary, py::arg("point"))
        .def("compute_intersections", py::overload_cast<const Line2D &>(&Primitive2D::ComputeIntersections, py::const_), py::arg("line"))
        .def("compute_intersections", py::overload_cast<const Segment2D &>(&Primitive2D::ComputeIntersections, py::const_), py::arg("segment"))
        .def("compute_intersections", py::overload_cast<const Ray2D &>(&Primitive2D::ComputeIntersections, py::const_), py::arg("ray"))
        .def_property_readonly("orientation_angle", &Primitive2D::GetOrientationAngle);

    py::class_<Line2D, Primitive2D>(m, "Line2D")
        .def(py::init<int, Eigen::Vector2d, Eigen::Vector2d>(), py::arg("id"), py::arg("p0"), py::arg("p1"))
        .def_readwrite("p0", &Line2D::p0)
        .def_readwrite("p1", &Line2D::p1);
    py::class_<Segment2D, Line2D>(m, "Segment2D").def(py::init<int, Eigen::Vector2d, Eigen::Vector2d>(), py::arg("id"), py::arg("p0"), py::arg("p1"));
    py::class_<Ray2D, Primitive2D>(m, "Ray2D")
        .def(py::init<int, Eigen::Vector2d, Eigen::Vector2d>(), py::arg("id"), py::arg("origin"), py::arg("direction"))
        .def_readwrite("origin", &Ray2D::origin)
        .def_readwrite("direction", &Ray2D::direction);
    py::class_<AxisAlignedRectangle2D, Primitive2D, Aabb2D>(m, "AxisAlignedRectangle2D")
        .def(py::init<int, const Eigen::Vector2d &, const Eigen::Vector2d &>(), py::arg("id"), py::arg("center"), py::arg("half_sizes"));
    py::class_<Rectangle2D, Primitive2D>(m, "Rectangle2D")
        .def(py::init<int, Eigen::Vector2d, Eigen::Vector2d, double>(), py::arg("id"), py::arg("center"), py::arg("half_sizes"), py::arg("angle"));
    py::class_<Ellipse2D, Primitive2D>(m, "Ellipse2D")
        .def(py::init<int, Eigen::Vector2d, double, double, double>(), py::arg("id"), py::arg("center"), py::arg("a"), py::arg("b"), py::arg("angle"))
        .def_property("center", &Ellipse2D::GetCenter, &Ellipse2D::SetCenter)
        .def_property_readonly("radius", &Ellipse2D::GetRadius)
        .def_property_readonly("rotation_matrix", &Ellipse2D::GetRotationMatrix)
        .def("translate", &Ellipse2D::Translate, py::arg("translation"))
        .def("rotate", &Ellipse2D::Rotate, py::arg("angle"))
        .def_property("orientation_angle", &Ellipse2D::GetOrientationAngle, &Ellipse2D::SetOrientationAngle)
        .def("compute_points_on_boundary", &Ellipse2D::ComputePointsOnBoundary, py::arg("num_points"), py::arg("start_angle"), py::arg("end_angle"));
}
