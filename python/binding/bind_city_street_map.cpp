#include "erl_common/pybind11.hpp"
#include "erl_geometry/city_street_map.hpp"


void BindCityStreetMap(const py::module &m) {

    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<CityStreetMap>(m, "CityStreetMap")
        .def_property_readonly_static("kFree", [](py::object) { return CityStreetMap::kFree; })
        .def_property_readonly_static("kObstacle", [](py::object) { return CityStreetMap::kObstacle; })
        .def_property_readonly_static("kPassableDot", [](py::object) { return CityStreetMap::kPassableDot; })
        .def_property_readonly_static("kPassableG", [](py::object) { return CityStreetMap::kPassableG; })
        .def_property_readonly_static("kOutOfBoundAt", [](py::object) { return CityStreetMap::kOutOfBoundAt; })
        .def_property_readonly_static("kOutOfBoundO", [](py::object) { return CityStreetMap::kOutOfBoundO; })
        .def_property_readonly_static("kTree", [](py::object) { return CityStreetMap::kTree; })
        .def_property_readonly_static("kSwamp", [](py::object) { return CityStreetMap::kSwamp; })
        .def_property_readonly_static("kWater", [](py::object) { return CityStreetMap::kWater; })
        .def_static("load", &CityStreetMap::Load, py::arg("filename"));
}
