#include "erl_common/pybind11.hpp"
#include "erl_geometry/abstract_surface_mapping_2d.hpp"
#include "erl_geometry/abstract_surface_mapping_3d.hpp"

void
BindAbstractSurfaceMapping(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<AbstractSurfaceMapping, std::shared_ptr<AbstractSurfaceMapping>> abstract_surface_mapping(m, "AbstractSurfaceMapping");
    py::class_<AbstractSurfaceMapping::Setting, YamlableBase, std::shared_ptr<AbstractSurfaceMapping::Setting>>(abstract_surface_mapping, "Setting")
        .def(py::init<>(&AbstractSurfaceMapping::Setting::Create<AbstractSurfaceMapping::Setting>), py::arg("mapping_type"));
    abstract_surface_mapping.def_static("create_surface_mapping", &AbstractSurfaceMapping::CreateSurfaceMapping<>, py::arg("mapping_type"), py::arg("setting"))
        .def("write", py::overload_cast<const std::string &>(&AbstractSurfaceMapping::Write, py::const_), py::arg("filename"))
        .def("read", py::overload_cast<const std::string &>(&AbstractSurfaceMapping::Read), py::arg("filename"));

    py::class_<AbstractSurfaceMapping2D, AbstractSurfaceMapping, std::shared_ptr<AbstractSurfaceMapping2D>>(m, "AbstractSurfaceMapping2D")
        .def_property_readonly("quadtree", &AbstractSurfaceMapping2D::GetQuadtree)
        .def_property_readonly("sensor_noise", &AbstractSurfaceMapping2D::GetSensorNoise)
        .def_property_readonly("cluster_level", &AbstractSurfaceMapping2D::GetClusterLevel)
        .def("update", &AbstractSurfaceMapping2D::Update, py::arg("rotation"), py::arg("translation"), py::arg("ranges"));

    py::class_<AbstractSurfaceMapping3D, AbstractSurfaceMapping, std::shared_ptr<AbstractSurfaceMapping3D>>(m, "AbstractSurfaceMapping3D")
        .def_property_readonly("octree", &AbstractSurfaceMapping3D::GetOctree)
        .def_property_readonly("sensor_noise", &AbstractSurfaceMapping3D::GetSensorNoise)
        .def_property_readonly("cluster_level", &AbstractSurfaceMapping3D::GetClusterLevel)
        .def("update", &AbstractSurfaceMapping3D::Update, py::arg("rotation"), py::arg("translation"), py::arg("ranges"));
}
