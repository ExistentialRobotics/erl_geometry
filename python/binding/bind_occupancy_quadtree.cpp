#include "erl_common/grid_map_info.hpp"
#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/pybind11_occupancy_quadtree.hpp"

void
BindOccupancyQuadtree(const py::module& m) {
    using namespace erl::common;
    using namespace erl::geometry;
    auto tree = BindOccupancyQuadtree<OccupancyQuadtree, OccupancyQuadtreeNode>(m, "OccupancyQuadtree");
    tree.def(
        py::init<>([](const std::shared_ptr<GridMapInfo2D>& map_info, const cv::Mat& image_map, const double occupied_threshold, const int padding) {
            return std::make_shared<OccupancyQuadtree>(map_info, image_map, occupied_threshold, padding);
        }),
        py::arg("map_info"),
        py::arg("image_map"),
        py::arg("occupied_threshold"),
        py::arg("padding") = 0);
}
