#include "erl_common/pybind11.hpp"
#include "erl_geometry/semi_sparse_nd_tree_setting.hpp"

void
BindSemiSparseNdTreeSetting(const py::module& m) {
    using namespace erl::common;
    using namespace erl::geometry;
    py::class_<SemiSparseNdTreeSetting, NdTreeSetting, std::shared_ptr<SemiSparseNdTreeSetting>>(
        m,
        "SemiSparseNdTreeSetting")
        .def(py::init<>())
        .def_readwrite("full_depth", &SemiSparseNdTreeSetting::full_depth)
        .def_readwrite("init_voxel_num", &SemiSparseNdTreeSetting::init_voxel_num);
}
