#include "erl_common/pybind11.hpp"

void
BindAabb(const py::module &m);

void
BindCityStreetMap(const py::module &m);

void
BindNdTreeDeps(const py::module &m);

void
BindOccupancyTrees(const py::module &m);

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
BindRgbdFrame3D(const py::module &m);

void
BindUtils(py::module &m);

PYBIND11_MODULE(PYBIND_MODULE_NAME, m) {
    m.doc() = "Python 3 Interface of erl_geometry";

    BindAabb(m);
    BindCityStreetMap(m);
    BindNdTreeDeps(m);
    BindOccupancyTrees(m);
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
    BindRgbdFrame3D(m);
    BindUtils(m);
}
