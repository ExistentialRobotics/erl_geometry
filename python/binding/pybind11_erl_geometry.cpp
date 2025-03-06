#include "pyobject_occupancy_octree.hpp"
#include "pyobject_occupancy_quadtree.hpp"

#include "erl_common/pybind11.hpp"
#include "erl_geometry/init.hpp"

void
BindAabb(const py::module &m);

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
BindPyObjectOccupancyQuadtreeNode(const py::module &m);

void
BindOccupancyQuadtreeBaseSetting(const py::module &m);

void
BindAbstractOccupancyQuadtree(const py::module &m);

void
BindOccupancyQuadtree(const py::module &m);

void
BindPyObjectOccupancyQuadtree(const py::module &m);

void
BindOctreeKey(const py::module &m);

void
BindAbstractOctreeNode(const py::module &m);

void
BindAbstractOctree(const py::module &m);

void
BindOccupancyOctreeNode(const py::module &m);

void
BindPyObjectOccupancyOctreeNode(const py::module &m);

void
BindOccupancyOctreeBaseSetting(const py::module &m);

void
BindAbstractOccupancyOctree(const py::module &m);

void
BindOccupancyOctree(const py::module &m);

void
BindPyObjectOccupancyOctree(const py::module &m);

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
BindRangeSensor3D(const py::module &m);

void
BindLidar3D(const py::module &m);

void
BindCameraIntrinsic(const py::module &m);

void
BindCameraBase3D(const py::module &m);

void
BindDepthCamera3D(const py::module &m);

void
BindRgbdCamera3D(const py::module &m);

void
BindRangeSensorFrame3D(const py::module &m);

void
BindLidarFrame3D(const py::module &m);

void
BindDepthFrame3D(const py::module &m);

void
BindRgbdFrame3D(const py::module &m);

void
BindTrajectory(const py::module &m);

void
BindUtils(py::module &m);

void
BindIntersection(py::module &m);

void
BindPrimitives2D(const py::module &m);

void
BindPrimitives3D(const py::module &m);

void
BindOpen3dHelper(py::module &m);

void
BindHiddenPointRemoval(py::module &m);

void
PyInit() {

#define REGISTER(x) (void) x::Register<x>()

    using namespace erl::geometry;
    REGISTER(PyObjectOccupancyQuadtreeNode);
    REGISTER(PyObjectOccupancyOctreeNode);
    REGISTER(PyObjectOccupancyQuadtreeD);
    REGISTER(PyObjectOccupancyQuadtreeF);
    REGISTER(PyObjectOccupancyOctreeD);
    REGISTER(PyObjectOccupancyOctreeF);

    Init();
}

PYBIND11_MODULE(PYBIND_MODULE_NAME, m) {
    PyInit();

    m.doc() = "Python 3 Interface of erl_geometry";

    BindAabb(m);

    BindCityStreetMap(m);

    BindNdTreeSetting(m);
    BindOccupancyNdTreeSetting(m);
    BindOccupancyQuadtreeBaseSetting(m);
    BindOccupancyOctreeBaseSetting(m);

    BindQuadtreeKey(m);

    BindAbstractQuadtreeNode(m);
    BindOccupancyQuadtreeNode(m);
    BindPyObjectOccupancyQuadtreeNode(m);

    BindAbstractQuadtree(m);
    BindAbstractOccupancyQuadtree(m);
    BindOccupancyQuadtree(m);
    BindPyObjectOccupancyQuadtree(m);

    BindOctreeKey(m);

    BindAbstractOctreeNode(m);
    BindOccupancyOctreeNode(m);
    BindPyObjectOccupancyOctreeNode(m);

    BindAbstractOctree(m);
    BindAbstractOccupancyOctree(m);
    BindOccupancyOctree(m);
    BindPyObjectOccupancyOctree(m);

    BindSurface2D(m);
    BindSpace2D(m);
    BindLidar2D(m);
    BindLidarFrame2D(m);
    BindLogOddMap2D(m);
    BindHouseExpoMap(m);
    BindRangeSensor3D(m);
    BindLidar3D(m);
    BindCameraIntrinsic(m);
    BindCameraBase3D(m);
    BindDepthCamera3D(m);
    BindRgbdCamera3D(m);
    BindRangeSensorFrame3D(m);
    BindLidarFrame3D(m);
    BindDepthFrame3D(m);
    BindRgbdFrame3D(m);
    BindTrajectory(m);
    BindUtils(m);
    BindIntersection(m);
    BindPrimitives2D(m);
    BindPrimitives3D(m);
    BindOpen3dHelper(m);
    BindHiddenPointRemoval(m);
}
