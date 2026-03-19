#include "erl_common/pybind11.hpp"
#include "erl_geometry/mesh_sdf.hpp"

void
BindMeshSdf(const py::module &m) {
    using Sdf = erl::geometry::MeshSdf;

    py::class_<Sdf>(m, "MeshSdf")
        .def(
            py::init<
                const std::vector<Eigen::Vector3d> &,
                const std::vector<Eigen::Vector3i> &,
                const bool,
                const bool>(),
            py::arg("verts"),
            py::arg("faces"),
            py::arg("use_open3d") = true,
            py::arg("robust") = true)
        .def(
            "__call__",
            py::overload_cast<const Eigen::Matrix3Xd &, bool, std::size_t>(
                &Sdf::operator()<double>,
                py::const_),
            py::arg("points"),
            py::arg("trunc_aabb") = false,
            py::arg("n_threads") = std::thread::hardware_concurrency());
}
