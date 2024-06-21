#include "erl_common/pybind11.hpp"
#include "erl_geometry/city_street_map.hpp"

void
BindCityStreetMap(const py::module &m) {

    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<CityStreetMap> city_street_map(m, "CityStreetMap");
    city_street_map.def_property_readonly_static("kFree", [](const py::object &) { return CityStreetMap::kFree; })
        .def_property_readonly_static("kObstacle", [](const py::object &) { return CityStreetMap::kObstacle; })
        .def_property_readonly_static("kPassableDot", [](const py::object &) { return CityStreetMap::kPassableDot; })
        .def_property_readonly_static("kPassableG", [](const py::object &) { return CityStreetMap::kPassableG; })
        .def_property_readonly_static("kOutOfBoundAt", [](const py::object &) { return CityStreetMap::kOutOfBoundAt; })
        .def_property_readonly_static("kOutOfBoundO", [](const py::object &) { return CityStreetMap::kOutOfBoundO; })
        .def_property_readonly_static("kTree", [](const py::object &) { return CityStreetMap::kTree; })
        .def_property_readonly_static("kSwamp", [](const py::object &) { return CityStreetMap::kSwamp; })
        .def_property_readonly_static("kWater", [](const py::object &) { return CityStreetMap::kWater; })
        .def_static("load_map", &CityStreetMap::LoadMap, py::arg("filename"))
        .def_static("load_scenes", &CityStreetMap::LoadScenes, py::arg("filename"));
    py::class_<CityStreetMap::Scene>(city_street_map, "Scene")
        .def_readwrite("bucket", &CityStreetMap::Scene::bucket)
        .def_readwrite("map", &CityStreetMap::Scene::map)
        .def_readwrite("map_width", &CityStreetMap::Scene::map_width)
        .def_readwrite("map_height", &CityStreetMap::Scene::map_height)
        .def_readwrite("start_x", &CityStreetMap::Scene::start_x)
        .def_readwrite("start_y", &CityStreetMap::Scene::start_y)
        .def_readwrite("goal_x", &CityStreetMap::Scene::goal_x)
        .def_readwrite("goal_y", &CityStreetMap::Scene::goal_y)
        .def_readwrite("optimal_length", &CityStreetMap::Scene::optimal_length);
}
