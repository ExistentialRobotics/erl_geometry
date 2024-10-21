#include "erl_common/pybind11.hpp"
#include "erl_geometry/primitives_3d.hpp"

using namespace erl::geometry;

void
BindPrimitives3D(const py::module &m) {
    auto primitive = py::class_<Primitive3D>(m, "Primitive3D");
    py::enum_<Primitive3D::Type>(primitive, "Type")
        .value("kLine3D", Primitive3D::Type::kLine3D)
        .value("kSegment3D", Primitive3D::Type::kSegment3D)
        .value("kRay3D", Primitive3D::Type::kRay3D)
        .value("kPlane", Primitive3D::Type::kPlane)
        .value("kTriangle", Primitive3D::Type::kTriangle)
        .value("kAxisAlignedBox", Primitive3D::Type::kAxisAlignedBox)
        .value("kBox", Primitive3D::Type::kBox)
        .value("kEllipsoid", Primitive3D::Type::kEllipsoid);
    primitive.def_readwrite("id", &Primitive3D::id)
        .def_property_readonly("type", &Primitive3D::GetType)
        .def("is_inside", &Primitive3D::IsInside, py::arg("point"));

    py::class_<Box, Primitive3D>(m, "Box")
        .def(py::init<int, Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>(), py::arg("id"), py::arg("center"), py::arg("half_sizes"), py::arg("rotation"))
        .def_property("center", &Box::GetCenter, &Box::SetCenter)
        .def_property_readonly("half_sizes", &Box::GetHalfSizes)
        .def_property("rotation_matrix", &Box::GetRotationMatrix, &Box::SetRotationMatrix)
        .def("translate", &Box::Translate, py::arg("translation"))
        .def("rotate", &Box::Rotate, py::arg("rotation"));

    py::class_<Ellipsoid, Primitive3D>(m, "Ellipsoid")
        .def(py::init<int, Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>(), py::arg("id"), py::arg("center"), py::arg("radius"), py::arg("rotation"))
        .def_property("center", &Ellipsoid::GetCenter, &Ellipsoid::SetCenter)
        .def_property_readonly("radii", &Ellipsoid::GetRadii)
        .def_property("rotation_matrix", &Ellipsoid::GetRotationMatrix, &Ellipsoid::SetRotationMatrix)
        .def("translate", &Ellipsoid::Translate, py::arg("translation"))
        .def("rotate", &Ellipsoid::Rotate, py::arg("rotation"));
}
