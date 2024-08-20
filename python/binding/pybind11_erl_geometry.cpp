#include "erl_common/pybind11.hpp"

void
BindAabb(const py::module &m);

void
BindAbstractSurfaceMapping(const py::module &m);

void
BindCityStreetMap(const py::module &m);

void
BindNdTreeSetting(const py::module &m);

void
BindOccupancyNdTreeSetting(const py::module &m);

void
BindQuadtreeKey(const py::module &m);

void
BindAbstractQuadtreeNode(const py::module &m);

void
BindAbstractQuadtree(const py::module &m);

void
BindOccupancyQuadtreeNode(const py::module &m);

void
BindOccupancyQuadtreeBaseSetting(const py::module &m);

void
BindAbstractOccupancyQuadtree(const py::module &m);

void
BindOccupancyQuadtree(const py::module &m);

void
BindSurfaceMappingQuadtreeNode(const py::module &m);

void
BindSurfaceMappingQuadtree(const py::module &m);

void
BindOctreeKey(const py::module &m);

void
BindAbstractOctreeNode(const py::module &m);

void
BindAbstractOctree(const py::module &m);

void
BindOccupancyOctreeNode(const py::module &m);

void
BindOccupancyOctreeBaseSetting(const py::module &m);

void
BindAbstractOccupancyOctree(const py::module &m);

void
BindOccupancyOctree(const py::module &m);

void
BindSurfaceMappingOctreeNode(const py::module &m);

void
BindSurfaceMappingOctree(const py::module &m);

void
BindSurface2D(const py::module &m);

void
BindSpace2D(const py::module &m);

void
BindLidar2D(const py::module &m);

void
BindLidarFrame2D(const py::module &m);

void
BindLogOddMap2D(const py::module &m);

void
BindHouseExpoMap(const py::module &m);

void
BindLidar3D(const py::module &m);

void
BindDepthCamera3D(const py::module &m);

void
BindRangeSensorFrame3D(const py::module &m);

void
BindLidarFrame3D(const py::module &m);

void
BindDepthFrame3D(const py::module &m);

void
BindUtils(py::module &m);

void
BindIntersection(py::module &m);

void
BindPrimitives2D(const py::module &m);

PYBIND11_MODULE(PYBIND_MODULE_NAME, m) {
    m.doc() = "Python 3 Interface of erl_geometry";

    BindAabb(m);
    BindAbstractSurfaceMapping(m);
    BindCityStreetMap(m);
    BindNdTreeSetting(m);
    BindOccupancyNdTreeSetting(m);
    BindQuadtreeKey(m);
    BindAbstractQuadtreeNode(m);
    BindAbstractQuadtree(m);
    BindOccupancyQuadtreeNode(m);
    BindOccupancyQuadtreeBaseSetting(m);
    BindAbstractOccupancyQuadtree(m);
    BindOccupancyQuadtree(m);
    BindSurfaceMappingQuadtreeNode(m);
    BindSurfaceMappingQuadtree(m);
    BindOctreeKey(m);
    BindAbstractOctreeNode(m);
    BindAbstractOctree(m);
    BindOccupancyOctreeNode(m);
    BindOccupancyOctreeBaseSetting(m);
    BindAbstractOccupancyOctree(m);
    BindOccupancyOctree(m);
    BindSurfaceMappingOctreeNode(m);
    BindSurfaceMappingOctree(m);
    BindSurface2D(m);
    BindSpace2D(m);
    BindLidar2D(m);
    BindLidarFrame2D(m);
    BindLogOddMap2D(m);
    BindHouseExpoMap(m);
    BindLidar3D(m);
    BindDepthCamera3D(m);
    BindRangeSensorFrame3D(m);
    BindLidarFrame3D(m);
    BindDepthFrame3D(m);
    BindUtils(m);
    BindIntersection(m);
    BindPrimitives2D(m);
}
