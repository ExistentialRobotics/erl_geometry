#include "erl_common/pybind11.hpp"
#include "erl_geometry/open3d_helper.hpp"

void
BindOpen3dHelper(py::module &m) {
    m.def(
        "create_ellipsoid_mesh",
        [](const double a, const double b, const double c, const long num_azimuths, const long num_elevations) {
            const auto mesh = erl::geometry::CreateEllipsoidMesh(a, b, c, num_azimuths, num_elevations);
            return py::make_tuple(mesh->vertices_, mesh->triangles_);
        },
        py::arg("a"),
        py::arg("b"),
        py::arg("c"),
        py::arg("num_azimuths") = 360,
        py::arg("num_elevations") = 180);
}
