#include "erl_common/pybind11.hpp"
#include "erl_geometry/pybind11_occupancy_octree.hpp"
#include "erl_geometry/surface_mapping_octree.hpp"

void
BindSurfaceMappingOctree(const py::module& m) {
    using namespace erl::geometry;
    BindOccupancyOctree<SurfaceMappingOctree, SurfaceMappingOctreeNode>(m, "SurfaceMappingOctree");
}
