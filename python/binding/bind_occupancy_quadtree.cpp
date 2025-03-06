#include "erl_common/grid_map_info.hpp"
#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/pybind11_occupancy_quadtree.hpp"

void
BindOccupancyQuadtree(const py::module& m) {
    using namespace erl::common;
    using namespace erl::geometry;

    auto tree_d = BindOccupancyQuadtree<OccupancyQuadtreeD, OccupancyQuadtreeNode>(m, "OccupancyQuadtreeD");
    tree_d.def(
        py::init<>([](const std::shared_ptr<GridMapInfo2Dd>& map_info, const cv::Mat& image_map, const double occupied_threshold, const int padding) {
            return std::make_shared<OccupancyQuadtreeD>(map_info, image_map, occupied_threshold, padding);
        }),
        py::arg("map_info"),
        py::arg("image_map"),
        py::arg("occupied_threshold"),
        py::arg("padding") = 0);

    auto tree_f = BindOccupancyQuadtree<OccupancyQuadtreeF, OccupancyQuadtreeNode>(m, "OccupancyQuadtreeF");
    tree_f.def(
        py::init<>([](const std::shared_ptr<GridMapInfo2Df>& map_info, const cv::Mat& image_map, const float occupied_threshold, const int padding) {
            return std::make_shared<OccupancyQuadtreeF>(map_info, image_map, occupied_threshold, padding);
        }),
        py::arg("map_info"),
        py::arg("image_map"),
        py::arg("occupied_threshold"),
        py::arg("padding") = 0);
}
