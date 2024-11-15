#include "erl_common/pybind11.hpp"
#include "erl_geometry/range_sensor_frame_3d.hpp"

void
BindRangeSensorFrame3D(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<RangeSensorFrame3D, std::shared_ptr<RangeSensorFrame3D>> range_sensor_frame(m, "RangeSensorFrame3D");
    py::class_<RangeSensorFrame3D::Setting, YamlableBase, std::shared_ptr<RangeSensorFrame3D::Setting>>(range_sensor_frame, "Setting")
        .def_readwrite("row_margin", &RangeSensorFrame3D::Setting::row_margin)
        .def_readwrite("col_margin", &RangeSensorFrame3D::Setting::col_margin)
        .def_readwrite("valid_range_min", &RangeSensorFrame3D::Setting::valid_range_min)
        .def_readwrite("valid_range_max", &RangeSensorFrame3D::Setting::valid_range_max)
        .def_readwrite("discontinuity_factor", &RangeSensorFrame3D::Setting::discontinuity_factor)
        .def_readwrite("rolling_diff_discount", &RangeSensorFrame3D::Setting::rolling_diff_discount)
        .def_readwrite("min_partition_size", &RangeSensorFrame3D::Setting::min_partition_size);
    range_sensor_frame.def_property_readonly("num_rays", &RangeSensorFrame3D::GetNumRays)
        .def_property_readonly("num_hit_rays", &RangeSensorFrame3D::GetNumHitRays)
        .def_property_readonly("rotation_matrix", &RangeSensorFrame3D::GetRotationMatrix)
        .def_property_readonly("translation_vector", &RangeSensorFrame3D::GetTranslationVector)
        .def_property_readonly("pose_matrix", &RangeSensorFrame3D::GetPoseMatrix)
        .def_property_readonly("ranges", [](const RangeSensorFrame3D &self) { return self.GetRanges(); })
        .def_property_readonly(
            "frame_coords",
            [](const RangeSensorFrame3D &self) {
                const Eigen::MatrixX<Eigen::Vector2d> &frame_coords = self.GetFrameCoords();
                py::array_t<double> py_frame_coords({frame_coords.rows(), frame_coords.cols(), 2l});
                for (long i = 0; i < frame_coords.rows(); ++i) {
                    for (long j = 0; j < frame_coords.cols(); ++j) {
                        py_frame_coords.mutable_at(i, j, 0) = frame_coords(i, j)[0];
                        py_frame_coords.mutable_at(i, j, 1) = frame_coords(i, j)[1];
                    }
                }
                return py_frame_coords;
            })
        .def_property_readonly("ranges", &RangeSensorFrame3D::GetRanges)
        .def_property_readonly(
            "ray_directions_in_frame",
            [](const RangeSensorFrame3D &self) -> py::array_t<double> {
                const Eigen::MatrixX<Eigen::Vector3d> &ray_directions_in_frame = self.GetRayDirectionsInFrame();
                return py::cast_to_array(ray_directions_in_frame);
            })
        .def_property_readonly(
            "ray_directions_in_world",
            [](const RangeSensorFrame3D &self) -> py::array_t<double> {
                const Eigen::MatrixX<Eigen::Vector3d> &ray_directions_in_world = self.GetRayDirectionsInWorld();
                return py::cast_to_array(ray_directions_in_world);
            })
        .def_property_readonly(
            "end_points_in_frame",
            [](const RangeSensorFrame3D &self) -> py::array_t<double> {
                const Eigen::MatrixX<Eigen::Vector3d> &end_points_in_frame = self.GetEndPointsInFrame();
                return py::cast_to_array(end_points_in_frame);
            })
        .def_property_readonly(
            "end_points_in_world",
            [](const RangeSensorFrame3D &self) -> py::array_t<double> {
                const Eigen::MatrixX<Eigen::Vector3d> &end_points_in_world = self.GetEndPointsInWorld();
                return py::cast_to_array(end_points_in_world);
            })
        .def_property_readonly("hit_ray_indices", &RangeSensorFrame3D::GetHitRayIndices)
        .def_property_readonly("hit_points_world", &RangeSensorFrame3D::GetHitPointsWorld)
        .def_property_readonly("max_valid_range", [](const RangeSensorFrame3D &self) { return self.GetMaxValidRange(); })
        .def_property_readonly("hit_mask", [](const RangeSensorFrame3D &self) { return self.GetHitMask(); })
        .def_property_readonly("is_valid", [](const RangeSensorFrame3D &self) { return self.IsValid(); })
        .def("point_is_in_frame", &RangeSensorFrame3D::PointIsInFrame, py::arg("xyz_frame"))
        .def("coords_is_in_frame", &RangeSensorFrame3D::CoordsIsInFrame, py::arg("frame_coords"))
        .def("compute_frame_coords", &RangeSensorFrame3D::ComputeFrameCoords, py::arg("xyz_frame"))
        .def("world_to_frame_so3", &RangeSensorFrame3D::WorldToFrameSo3, py::arg("dir_world"))
        .def("frame_to_world_so3", &RangeSensorFrame3D::FrameToWorldSo3, py::arg("dir_frame"))
        .def("world_to_frame_se3", &RangeSensorFrame3D::WorldToFrameSe3, py::arg("xyz_world"))
        .def("frame_to_world_se3", &RangeSensorFrame3D::FrameToWorldSe3, py::arg("xyz_frame"))
        .def(
            "compute_closest_end_point",
            [](RangeSensorFrame3D &self, const Eigen::Ref<const Eigen::Vector3d> &position_world, const bool brute_force) -> py::dict {
                long row_index = -1;
                long col_index = -1;
                double distance = 0.0;
                self.ComputeClosestEndPoint(position_world, row_index, col_index, distance, brute_force);
                py::dict out;
                out["row_index"] = row_index;
                out["col_index"] = col_index;
                out["distance"] = distance;
                return out;
            },
            py::arg("position_world"),
            py::arg("brute_force") = false)
        .def(
            "sample_along_rays",
            [](const RangeSensorFrame3D &self, const long num_samples_per_ray, const double max_in_obstacle_dist, const double sampled_rays_ratio) -> py::dict {
                Eigen::Matrix3Xd positions_world;
                Eigen::Matrix3Xd directions_world;
                Eigen::VectorXd distances;
                self.SampleAlongRays(num_samples_per_ray, max_in_obstacle_dist, sampled_rays_ratio, positions_world, directions_world, distances);
                py::dict out;
                out["positions_world"] = positions_world;
                out["directions_world"] = directions_world;
                out["distances"] = distances;
                return out;
            },
            py::arg("num_samples_per_ray"),
            py::arg("max_in_obstacle_dist"),
            py::arg("sampled_rays_ratio"))
        .def(
            "sample_along_rays",
            [](const RangeSensorFrame3D &self, const double range_step, const double max_in_obstacle_dist, const double sampled_rays_ratio) -> py::dict {
                Eigen::Matrix3Xd positions_world;
                Eigen::Matrix3Xd directions_world;
                Eigen::VectorXd distances;
                self.SampleAlongRays(range_step, max_in_obstacle_dist, sampled_rays_ratio, positions_world, directions_world, distances);
                py::dict out;
                out["positions_world"] = positions_world;
                out["directions_world"] = directions_world;
                out["distances"] = distances;
                return out;
            },
            py::arg("range_step"),
            py::arg("max_in_obstacle_dist"),
            py::arg("sampled_rays_ratio"))
        .def(
            "sample_near_surface",
            [](const RangeSensorFrame3D &self, const long num_samples_per_ray, const double max_offset, const double sampled_rays_ratio) -> py::dict {
                Eigen::Matrix3Xd positions_world;
                Eigen::Matrix3Xd directions_world;
                Eigen::VectorXd distances;
                self.SampleNearSurface(num_samples_per_ray, max_offset, sampled_rays_ratio, positions_world, directions_world, distances);
                py::dict out;
                out["positions_world"] = positions_world;
                out["directions_world"] = directions_world;
                out["distances"] = distances;
                return out;
            },
            py::arg("num_samples_per_ray"),
            py::arg("max_offset"),
            py::arg("sampled_rays_ratio"))
        .def(
            "sample_in_region_hpr",
            [](const RangeSensorFrame3D &self,
               const long num_positions,
               const long num_near_surface_samples_per_ray,
               const long num_along_ray_samples_per_ray,
               const double max_in_obstacle_dist,
               const bool parallel) -> py::dict {
                Eigen::Matrix3Xd positions_world;
                Eigen::Matrix3Xd directions_world;
                Eigen::VectorXd distances;
                self.SampleInRegionHpr(
                    num_positions,
                    num_near_surface_samples_per_ray,
                    num_along_ray_samples_per_ray,
                    max_in_obstacle_dist,
                    positions_world,
                    directions_world,
                    distances,
                    parallel);
                py::dict out;
                out["positions_world"] = positions_world;
                out["directions_world"] = directions_world;
                out["distances"] = distances;
                return out;
            },
            py::arg("num_positions"),
            py::arg("num_near_surface_samples_per_ray"),
            py::arg("num_along_ray_samples_per_ray"),
            py::arg("max_in_obstacle_dist"),
            py::arg("parallel"))
        .def(
            "sample_in_region_vrs",
            [](const RangeSensorFrame3D &self,
               const long num_hit_points,
               const long num_samples_per_azimuth_segment,
               const long num_azimuth_segments,
               const bool parallel) -> py::dict {
                Eigen::Matrix3Xd positions_world;
                Eigen::Matrix3Xd directions_world;
                Eigen::VectorXd distances;
                self.SampleInRegionVrs(
                    num_hit_points,
                    num_samples_per_azimuth_segment,
                    num_azimuth_segments,
                    positions_world,
                    directions_world,
                    distances,
                    parallel);

                py::dict out;
                out["positions_world"] = positions_world;
                out["directions_world"] = directions_world;
                out["distances"] = distances;
                return out;
            },
            py::arg("num_hit_points"),
            py::arg("num_samples_per_azimuth_segment"),
            py::arg("num_azimuth_segments"),
            py::arg("parallel"))
        .def(
            "compute_rays_at",
            [](const RangeSensorFrame3D &self, const Eigen::Ref<const Eigen::Vector3d> &position_world) -> py::dict {
                Eigen::Matrix3Xd directions_world;
                Eigen::VectorXd distances;
                std::vector<long> visible_hit_point_indices;
                self.ComputeRaysAt(position_world, directions_world, distances, visible_hit_point_indices);
                py::dict out;
                out["directions_world"] = directions_world;
                out["distances"] = distances;
                out["visible_hit_point_indices"] = visible_hit_point_indices;
                return out;
            },
            py::arg("position_world"));
}
