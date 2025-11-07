#include "erl_common/pybind11.hpp"
#include "erl_geometry/city_street_map.hpp"

void
BindCityStreetMap(const py::module &m) {

    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<CityStreetMap> city_street_map(m, "CityStreetMap");
    city_street_map.def_readonly_static("kFree", &CityStreetMap::kFree)
        .def_readonly_static("kObstacle", &CityStreetMap::kObstacle)
        .def_readonly_static("kPassableDot", &CityStreetMap::kPassableDot)
        .def_readonly_static("kPassableG", &CityStreetMap::kPassableG)
        .def_readonly_static("kOutOfBoundAt", &CityStreetMap::kOutOfBoundAt)
        .def_readonly_static("kOutOfBoundO", &CityStreetMap::kOutOfBoundO)
        .def_readonly_static("kTree", &CityStreetMap::kTree)
        .def_readonly_static("kSwamp", &CityStreetMap::kSwamp)
        .def_readonly_static("kWater", &CityStreetMap::kWater)
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
