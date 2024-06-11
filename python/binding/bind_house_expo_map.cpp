#include "erl_common/pybind11.hpp"
#include "erl_geometry/house_expo_map.hpp"

#include <open3d/io/TriangleMeshIO.h>

void
BindHouseExpoMap(const py::module &m) {
    using namespace erl::geometry;

    py::class_<HouseExpoMap>(m, "HouseExpoMap")
        .def(py::init<const char *>(), py::arg("file"))
        .def(py::init<const char *, double>(), py::arg("file"), py::arg("wall_thickness"))
        .def_property_readonly("file", &HouseExpoMap::GetFile)
        .def_property_readonly("room_id", &HouseExpoMap::GetRoomId)
        .def_property_readonly("meter_space", &HouseExpoMap::GetMeterSpace)
        .def(
            "to_json",
            [](const HouseExpoMap &map) {
                nlohmann::json json_data;
                HouseExpoMap::ToJson(json_data, map);
                return json_data.dump();
            })
        .def(
            "extrude_to_3d",
            [](const HouseExpoMap &map, const double room_height, const std::string &filename) {
                const std::shared_ptr<open3d::geometry::TriangleMesh> mesh = map.ExtrudeTo3D(room_height);
                open3d::io::WriteTriangleMesh(filename, *mesh, false, false, true, true, true, false);
            },
            py::arg("room_height"),
            py::arg("filename"));
}
