#include "erl_geometry/grid_collision_checker_3d.hpp"
#include "erl_geometry/grid_collision_checker_se2.hpp"
#include "erl_geometry/point_collision_checker.hpp"

static void
BindCollisionCheckers(const py::module &m) {
    py::class_<CollisionCheckerBase>(m, "CollisionCheckerBase").def("is_collided", &CollisionCheckerBase::IsCollided, py::arg("grid_coords"));

    py::class_<PointCollisionChecker2D, CollisionCheckerBase>(m, "PointCollisionChecker2D")
        .def(py::init<std::shared_ptr<GridMap<uint8_t, 2>>>(), py::arg("grid_map"));

    py::class_<PointCollisionChecker3D, CollisionCheckerBase>(m, "PointCollisionChecker3D")
        .def(py::init<std::shared_ptr<GridMap<uint8_t, 3>>>(), py::arg("grid_map"));

    py::class_<GridCollisionCheckerSe2, CollisionCheckerBase>(m, "GridCollisionCheckerSe2")
        .def(
            py::init<std::shared_ptr<GridMap<uint8_t, 2>>, const std::shared_ptr<GridMapInfo3D> &, Eigen::Matrix2Xd>(),
            py::arg("grid_map"),
            py::arg("se2_grid_map_info"),
            py::arg("metric_shape"))
        .def("is_collided", py::overload_cast<const Eigen::Ref<const Eigen::Matrix3d> &>(&GridCollisionCheckerSe2::IsCollided, py::const_), py::arg("pose"));

    py::class_<GridCollisionChecker3D, CollisionCheckerBase>(m, "GridCollisionChecker3D")
        .def(py::init<std::shared_ptr<GridMap<uint8_t, 3>>, Eigen::Matrix3Xd>(), py::arg("grid_map"), py::arg("metric_voxels"))
        .def("is_collided", py::overload_cast<const Eigen::Ref<const Eigen::Matrix4d> &>(&GridCollisionChecker3D::IsCollided, py::const_), py::arg("pose"));
}
