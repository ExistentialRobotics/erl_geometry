#include "erl_common/pybind11.hpp"
#include "erl_geometry/lidar_frame_2d.hpp"

void
BindLidarFrame2D(const py::module &m) {
    using namespace erl::common;
    using namespace erl::geometry;

    py::class_<LidarFramePartition2D>(m, "LidarFramePartition2D")
        .def_property_readonly("index_begin", &LidarFramePartition2D::GetIndexBegin)
        .def_property_readonly("index_end", &LidarFramePartition2D::GetIndexEnd)
        .def("angle_in_partition", &LidarFramePartition2D::AngleInPartition, py::arg("angle_world"));

    py::class_<LidarFrame2D, std::shared_ptr<LidarFrame2D>> lidar_frame_2d(m, "LidarFrame2D");

    py::class_<LidarFrame2D::Setting, YamlableBase, std::shared_ptr<LidarFrame2D::Setting>>(lidar_frame_2d, "Setting")
        .def(py::init<>())
        .def_readwrite("valid_range_min", &LidarFrame2D::Setting::valid_range_min)
        .def_readwrite("valid_range_max", &LidarFrame2D::Setting::valid_range_max)
        .def_readwrite("valid_angle_min", &LidarFrame2D::Setting::valid_angle_min)
        .def_readwrite("valid_angle_max", &LidarFrame2D::Setting::valid_angle_max)
        .def_readwrite("discontinuity_factor", &LidarFrame2D::Setting::discontinuity_factor)
        .def_readwrite("rolling_diff_discount", &LidarFrame2D::Setting::rolling_diff_discount)
        .def_readwrite("min_partition_size", &LidarFrame2D::Setting::min_partition_size);

    lidar_frame_2d.def(py::init<std::shared_ptr<LidarFrame2D::Setting>>(), py::arg("setting"))
        .def(
            "update",
            &LidarFrame2D::Update,
            py::arg("rotation"),
            py::arg("translation"),
            py::arg("angles"),
            py::arg("ranges"),
            py::arg("partition_rays") = false)
        .def_property_readonly("setting", &LidarFrame2D::GetSetting)
        .def_property_readonly("num_rays", &LidarFrame2D::GetNumRays)
        .def_property_readonly("num_hit_rays", &LidarFrame2D::GetNumHitRays)
        .def_property_readonly("rotation_matrix", &LidarFrame2D::GetRotationMatrix)
        .def_property_readonly("rotation_angle", &LidarFrame2D::GetRotationAngle)
        .def_property_readonly("translation_vector", &LidarFrame2D::GetTranslationVector)
        .def_property_readonly("pose_matrix", &LidarFrame2D::GetPoseMatrix)
        .def_property_readonly("angles_in_frame", &LidarFrame2D::GetAnglesInFrame)
        .def_property_readonly("angles_in_world", &LidarFrame2D::GetAnglesInWorld)
        .def_property_readonly("ranges", &LidarFrame2D::GetRanges)
        .def_property_readonly("ray_directions_in_frame", &LidarFrame2D::GetRayDirectionsInFrame)
        .def_property_readonly("ray_directions_in_world", &LidarFrame2D::GetRayDirectionsInWorld)
        .def_property_readonly("end_points_in_frame", &LidarFrame2D::GetEndPointsInFrame)
        .def_property_readonly("end_points_in_world", &LidarFrame2D::GetEndPointsInWorld)
        .def_property_readonly("hit_ray_indices", &LidarFrame2D::GetHitRayIndices)
        .def_property_readonly("hit_points_world", &LidarFrame2D::GetHitPointsWorld)
        .def_property_readonly("max_valid_range", &LidarFrame2D::GetMaxValidRange)
        .def_property_readonly("partitions", &LidarFrame2D::GetPartitions)
        .def_property_readonly("is_partitioned", &LidarFrame2D::IsPartitioned)
        .def_property_readonly("is_valid", &LidarFrame2D::IsValid)
        .def(
            "compute_closest_end_point",
            [](LidarFrame2D &self, const Eigen::Ref<const Eigen::Vector2d> &position) {
                long end_point_index = -1;
                double distance = 0.0;
                self.ComputeClosestEndPoint(position, end_point_index, distance);
                py::dict out;
                out["end_point_index"] = end_point_index;
                out["distance"] = distance;
                return out;
            },
            py::arg("position"))
        .def(
            "sample_along_rays",
            [](const LidarFrame2D &self, const long num_samples_per_ray, const double max_in_obstacle_dist, const double sampled_rays_ratio) {
                Eigen::Matrix2Xd positions_world;
                Eigen::Matrix2Xd directions_world;
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
            [](const LidarFrame2D &self, const double range_step, const double max_in_obstacle_dist, const double sampled_rays_ratio) {
                Eigen::Matrix2Xd positions_world;
                Eigen::Matrix2Xd directions_world;
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
            [](const LidarFrame2D &self, const long num_samples_per_ray, const double max_offset, const double sampled_rays_ratio) {
                Eigen::Matrix2Xd positions_world;
                Eigen::Matrix2Xd directions_world;
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
            "sample_in_region",
            [](const LidarFrame2D &self,
               const long num_positions,
               const long num_along_ray_samples_per_ray,
               const long num_near_surface_samples_per_ray,
               const double max_in_obstacle_dist) {
                Eigen::Matrix2Xd positions_world;
                Eigen::Matrix2Xd directions_world;
                Eigen::VectorXd distances;
                self.SampleInRegion(
                    num_positions,
                    num_along_ray_samples_per_ray,
                    num_near_surface_samples_per_ray,
                    max_in_obstacle_dist,
                    positions_world,
                    directions_world,
                    distances);
                py::dict out;
                out["positions_world"] = positions_world;
                out["directions_world"] = directions_world;
                out["distances"] = distances;
                return out;
            },
            py::arg("num_positions"),
            py::arg("num_along_ray_samples_per_ray"),
            py::arg("num_near_surface_samples_per_ray"),
            py::arg("max_in_obstacle_dist"))
        .def(
            "compute_rays_at",
            [](const LidarFrame2D &self, const Eigen::Ref<const Eigen::Vector2d> &position_world) {
                Eigen::Matrix2Xd directions_world;
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
