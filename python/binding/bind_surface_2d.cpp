#include "erl_common/pybind11.hpp"
#include "erl_geometry/surface_2d.hpp"

void
BindSurface2D(const py::module &m) {
    using namespace erl::geometry;

    py::class_<Surface2D, std::shared_ptr<Surface2D>>(m, "Surface2D")
        .def(
            py::init<Eigen::Matrix2Xd, Eigen::Matrix2Xd, Eigen::Matrix2Xi, Eigen::Matrix2Xi, Eigen::VectorXb>(),
            py::arg("vertices"),
            py::arg("normals"),
            py::arg("lines2vertices"),
            py::arg("objects2lines"),
            py::arg("outside_flags"))
        .def(py::init<const Surface2D &>(), py::arg("surface"))
        .def_property_readonly("num_vertices", &Surface2D::GetNumVertices)
        .def_property_readonly("num_lines", &Surface2D::GetNumLines)
        .def_property_readonly("num_objects", &Surface2D::GetNumObjects)
        .def_readwrite("vertices", &Surface2D::vertices)
        .def_readwrite("normals", &Surface2D::normals)
        .def_readwrite("lines_to_vertices", &Surface2D::lines_to_vertices)
        .def_readwrite("objects_to_lines", &Surface2D::objects_to_lines)
        .def_readwrite("vertices_to_objects", &Surface2D::vertices_to_objects)
        .def_readwrite("outside_flags", &Surface2D::outside_flags)
        .def_property_readonly("normals_available", &Surface2D::NormalsAvailable)
        .def_property_readonly("outside_flags_available", &Surface2D::OutsideFlagsAvailable)
        .def(
            "get_object_vertices",
            [](const Surface2D &surface, const int idx_object) {
                return Eigen::Matrix2Xd(surface.GetObjectVertices(idx_object));
            },  // cast Eigen::Ref -> Eigen::Matrix2Xfp -> numpy array
            py::arg("index_object"))
        .def(
            "get_object_normals",
            [](const Surface2D &surface, const int idx_object) {
                return Eigen::Matrix2Xd(surface.GetObjectNormals(idx_object));
            },  // cast Eigen::Ref -> Eigen::Matrix2Xfp -> numpy array
            py::arg("index_object"))
        .def("get_vertex_neighbors", &Surface2D::GetVertexNeighbors, py::arg("index_vertex"));
}
