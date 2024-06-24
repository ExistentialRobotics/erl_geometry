#include "erl_common/pybind11.hpp"
#include "erl_geometry/lidar_frame_3d.hpp"

void
BindLidarFrame3D(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<LidarFrame3D, std::shared_ptr<LidarFrame3D>> lidar_frame(m, "LidarFrame3D");
    py::class_<LidarFrame3D::Setting, YamlableBase, std::shared_ptr<LidarFrame3D::Setting>>(lidar_frame, "Setting")
        .def(py::init<>())
        .def_readwrite("valid_range_min", &LidarFrame3D::Setting::valid_range_min)
        .def_readwrite("valid_range_max", &LidarFrame3D::Setting::valid_range_max)
        .def_readwrite("valid_azimuth_min", &LidarFrame3D::Setting::valid_azimuth_min)
        .def_readwrite("valid_azimuth_max", &LidarFrame3D::Setting::valid_azimuth_max)
        .def_readwrite("valid_elevation_min", &LidarFrame3D::Setting::valid_elevation_min)
        .def_readwrite("valid_elevation_max", &LidarFrame3D::Setting::valid_elevation_max)
        .def_readwrite("discontinuity_factor", &LidarFrame3D::Setting::discontinuity_factor)
        .def_readwrite("rolling_diff_discount", &LidarFrame3D::Setting::rolling_diff_discount)
        .def_readwrite("min_partition_size", &LidarFrame3D::Setting::min_partition_size);
    lidar_frame.def(py::init<std::shared_ptr<LidarFrame3D::Setting>>(), py::arg("setting").none(false))
        .def("reset", &LidarFrame3D::Reset)
        .def(
            "update",
            &LidarFrame3D::Update,
            py::arg("rotation"),
            py::arg("translation"),
            py::arg("azimuths"),
            py::arg("elevations"),
            py::arg("ranges"),
            py::arg("partition_rays") = false)
        .def_property_readonly("setting", [](const LidarFrame3D &self) { return self.GetSetting<LidarFrame3D::Setting>(); })
        .def_property_readonly("num_rays", &LidarFrame3D::GetNumRays)
        .def_property_readonly("num_hit_rays", &LidarFrame3D::GetNumHitRays)
        .def_property_readonly("rotation_matrix", &LidarFrame3D::GetRotationMatrix)
        .def_property_readonly("translation_vector", &LidarFrame3D::GetTranslationVector)
        .def_property_readonly("pose_matrix", &LidarFrame3D::GetPoseMatrix)
        .def_property_readonly("azimuth_angles_in_frame", &LidarFrame3D::GetAzimuthAnglesInFrame)
        .def_property_readonly("elevation_angles_in_frame", &LidarFrame3D::GetElevationAnglesInFrame)
        .def_property_readonly("ranges", &LidarFrame3D::GetRanges)
        .def_property_readonly(
            "ray_directions_in_frame",
            [](const LidarFrame3D &self) -> py::array_t<double> {
                const long n_azimuths = self.GetAzimuthAnglesInFrame().size();
                const long n_elevations = self.GetElevationAnglesInFrame().size();
                py::array_t<double> out({n_azimuths, n_elevations, 3l});
                const Eigen::MatrixX<Eigen::Vector3d> &get_ray_directions_in_frame = self.GetRayDirectionsInFrame();
                for (long i = 0; i < n_azimuths; ++i) {
                    for (long j = 0; j < n_elevations; ++j) {
                        const auto &dir = get_ray_directions_in_frame(i, j);
                        out.mutable_at(i, j, 0) = dir[0];
                        out.mutable_at(i, j, 1) = dir[1];
                        out.mutable_at(i, j, 2) = dir[2];
                    }
                }
                return out;
            })
        .def_property_readonly(
            "ray_directions_in_world",
            [](const LidarFrame3D &self) -> py::array_t<double> {
                const long n_azimuths = self.GetAzimuthAnglesInFrame().size();
                const long n_elevations = self.GetElevationAnglesInFrame().size();
                py::array_t<double> out({n_azimuths, n_elevations, 3l});
                const Eigen::MatrixX<Eigen::Vector3d> &ray_directions_in_world = self.GetRayDirectionsInWorld();
                for (long i = 0; i < n_azimuths; ++i) {
                    for (long j = 0; j < n_elevations; ++j) {
                        const auto &dir = ray_directions_in_world(i, j);
                        out.mutable_at(i, j, 0) = dir[0];
                        out.mutable_at(i, j, 1) = dir[1];
                        out.mutable_at(i, j, 2) = dir[2];
                    }
                }
                return out;
            })
        .def_property_readonly(
            "end_points_in_frame",
            [](const LidarFrame3D &self) -> py::array_t<double> {
                const long n_azimuths = self.GetAzimuthAnglesInFrame().size();
                const long n_elevations = self.GetElevationAnglesInFrame().size();
                py::array_t<double> out({n_azimuths, n_elevations, 3l});
                const Eigen::MatrixX<Eigen::Vector3d> &end_points_in_frame = self.GetEndPointsInFrame();
                for (long i = 0; i < n_azimuths; ++i) {
                    for (long j = 0; j < n_elevations; ++j) {
                        const auto &point = end_points_in_frame(i, j);
                        out.mutable_at(i, j, 0) = point[0];
                        out.mutable_at(i, j, 1) = point[1];
                        out.mutable_at(i, j, 2) = point[2];
                    }
                }
                return out;
            })
        .def_property_readonly(
            "end_points_in_world",
            [](const LidarFrame3D &self) -> py::array_t<double> {
                const long n_azimuths = self.GetNumAzimuthLines();
                const long n_elevations = self.GetNumElevationLines();
                py::array_t<double> out({n_azimuths, n_elevations, 3l});
                const Eigen::MatrixX<Eigen::Vector3d> &end_points_in_world = self.GetEndPointsInWorld();
                for (long i = 0; i < n_azimuths; ++i) {
                    for (long j = 0; j < n_elevations; ++j) {
                        const auto &point = end_points_in_world(i, j);
                        out.mutable_at(i, j, 0) = point[0];
                        out.mutable_at(i, j, 1) = point[1];
                        out.mutable_at(i, j, 2) = point[2];
                    }
                }
                return out;
            })
        .def_property_readonly("hit_ray_indices", &LidarFrame3D::GetHitRayIndices)
        .def_property_readonly("hit_points_world", &LidarFrame3D::GetHitPointsWorld)
        .def_property_readonly("max_valid_range", &LidarFrame3D::GetMaxValidRange)
        .def_property_readonly("hit_mask", &LidarFrame3D::GetHitMask)
        .def_property_readonly("is_valid", &LidarFrame3D::IsValid)
        .def_property_readonly("is_partitioned", &LidarFrame3D::IsPartitioned)
        .def_property_readonly("partitions", &LidarFrame3D::GetPartitions)
        .def(
            "compute_closest_end_point",
            [](LidarFrame3D &self, const Eigen::Ref<const Eigen::Vector3d> &position_world, const bool brute_force) -> py::dict {
                long azimuth_index = -1;
                long elevation_index = -1;
                double distance = 0.0;
                self.ComputeClosestEndPoint(position_world, azimuth_index, elevation_index, distance, brute_force);
                py::dict out;
                out["azimuth_index"] = azimuth_index;
                out["elevation_index"] = elevation_index;
                out["distance"] = distance;
                return out;
            },
            py::arg("position_world"),
            py::arg("brute_force") = false)
        .def(
            "sample_along_rays",
            [](const LidarFrame3D &self, const long num_samples_per_ray, const double max_in_obstacle_dist, const double sampled_rays_ratio) -> py::dict {
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
            [](const LidarFrame3D &self, const double range_step, const double max_in_obstacle_dist, const double sampled_rays_ratio) -> py::dict {
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
            [](const LidarFrame3D &self, const long num_samples_per_ray, const double max_offset, const double sampled_rays_ratio) -> py::dict {
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
            [](const LidarFrame3D &self,
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
            [](const LidarFrame3D &self,
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
            [](const LidarFrame3D &self, const Eigen::Ref<const Eigen::Vector3d> &position_world) -> py::dict {
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
