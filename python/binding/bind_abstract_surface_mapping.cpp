#include "erl_common/pybind11.hpp"
#include "erl_geometry/abstract_surface_mapping_2d.hpp"
#include "erl_geometry/abstract_surface_mapping_3d.hpp"

void
BindAbstractSurfaceMapping(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<AbstractSurfaceMapping2D, std::shared_ptr<AbstractSurfaceMapping2D>>(m, "AbstractSurfaceMapping2D")
        .def_property_readonly("quadtree", &AbstractSurfaceMapping2D::GetQuadtree)
        .def_property_readonly("sensor_noise", &AbstractSurfaceMapping2D::GetSensorNoise)
        .def_property_readonly("cluster_level", &AbstractSurfaceMapping2D::GetClusterLevel)
        .def("update", &AbstractSurfaceMapping2D::Update, py::arg("rotation"), py::arg("translation"), py::arg("ranges"));

    py::class_<AbstractSurfaceMapping3D, std::shared_ptr<AbstractSurfaceMapping3D>>(m, "AbstractSurfaceMapping3D")
        .def_property_readonly("octree", &AbstractSurfaceMapping3D::GetOctree)
        .def_property_readonly("sensor_noise", &AbstractSurfaceMapping3D::GetSensorNoise)
        .def_property_readonly("cluster_level", &AbstractSurfaceMapping3D::GetClusterLevel)
        .def("update", &AbstractSurfaceMapping3D::Update, py::arg("rotation"), py::arg("translation"), py::arg("ranges"));
}
