#include "erl_common/pybind11.hpp"
#include "erl_common/random.hpp"
#include "erl_geometry/bresenham_2d.hpp"
#include "erl_geometry/utils.hpp"
#include "erl_geometry/winding_number.hpp"

void
BindUtils(py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    m.def(
         "bresenham_2d",
         [](const Eigen::Ref<const Eigen::Vector2i> &start,
            const Eigen::Ref<const Eigen::Vector2i> &end,
            const std::optional<std::function<bool(long, long)>> &stop) {
             return stop.has_value() ? Bresenham2D(start, end, stop.value())
                                     : Bresenham2D(start, end);
         },
         py::arg("start"),
         py::arg("end"),
         py::arg("stop") = py::none())
        .def(
            "compute_pixels_of_polygon_contour",
            &ComputePixelsOfPolygonContour,
            py::arg("polygon_vertices"))
        .def("winding_number", &WindingNumber<double>, py::arg("p"), py::arg("vertices"))
        .def(
            "convert_path_2d_to_3d_float64",
            &ConvertPath2dTo3d<double>,
            py::arg("path_2d").noconvert(),
            py::arg("z"))
        .def(
            "convert_path_2d_to_3d_float32",
            &ConvertPath2dTo3d<float>,
            py::arg("path_2d").noconvert(),
            py::arg("z"));
}
