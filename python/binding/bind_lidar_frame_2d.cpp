#include "erl_common/pybind11.hpp"
#include "erl_geometry/lidar_frame_2d.hpp"

template<typename Dtype>
void
BindLidarFrame2DImpl(const py::module &m, const char *name) {
    using namespace erl::common;
    using namespace erl::geometry;

    using Frame = LidarFrame2D<Dtype>;
    using Partition = typename Frame::Partition;
    using Matrix2X = Eigen::Matrix2X<Dtype>;
    using Vector2 = Eigen::Vector2<Dtype>;
    using VectorX = Eigen::VectorX<Dtype>;

    py::class_<Frame, std::shared_ptr<Frame>> lidar_frame_2d(m, name);

    py::class_<Partition>(lidar_frame_2d, "Partition")
        .def_property_readonly("index_begin", &Partition::GetIndexBegin)
        .def_property_readonly("index_end", &Partition::GetIndexEnd)
        .def("angle_in_partition", &Partition::AngleInPartition, py::arg("angle_world"));

    py::class_<typename Frame::Setting, YamlableBase, std::shared_ptr<typename Frame::Setting>>(
        lidar_frame_2d,
        "Setting")
        .def(py::init<>())
        .def_readwrite("valid_range_min", &Frame::Setting::valid_range_min)
        .def_readwrite("valid_range_max", &Frame::Setting::valid_range_max)
        .def_readwrite("angle_min", &Frame::Setting::angle_min)
        .def_readwrite("angle_max", &Frame::Setting::angle_max)
        .def_readwrite("num_rays", &Frame::Setting::num_rays)
        .def_readwrite("discontinuity_factor", &Frame::Setting::discontinuity_factor)
        .def_readwrite("rolling_diff_discount", &Frame::Setting::rolling_diff_discount)
        .def_readwrite("min_partition_size", &Frame::Setting::min_partition_size);

    lidar_frame_2d.def(py::init<std::shared_ptr<typename Frame::Setting>>(), py::arg("setting"))
        .def("coords_is_in_frame", &Frame::CoordsIsInFrame, py::arg("angle_frame"))
        .def("position_is_in_frame", &Frame::PosIsInFrame, py::arg("xy_frame"))
        .def("dir_world_to_frame", &Frame::DirWorldToFrame, py::arg("dir_world"))
        .def("dir_frame_to_world", &Frame::DirFrameToWorld, py::arg("dir_frame"))
        .def("pos_world_to_frame", &Frame::PosWorldToFrame, py::arg("xy_world"))
        .def("pos_frame_to_world", &Frame::PosFrameToWorld, py::arg("xy_frame"))
        .def(
            "update_ranges",
            &Frame::UpdateRanges,
            py::arg("rotation"),
            py::arg("translation"),
            py::arg("ranges"))
        .def_property_readonly("setting", &Frame::GetSetting)
        .def_property_readonly("num_rays", &Frame::GetNumRays)
        .def_property_readonly("num_hit_rays", &Frame::GetNumHitRays)
        .def_property_readonly("rotation_matrix", &Frame::GetRotationMatrix)
        .def_property_readonly("rotation_angle", &Frame::GetRotationAngle)
        .def_property_readonly("translation_vector", &Frame::GetTranslationVector)
        .def_property_readonly("pose_matrix", &Frame::GetPoseMatrix)
        .def_property_readonly("angles_in_frame", &Frame::GetAnglesInFrame)
        .def_property_readonly("angles_in_world", &Frame::GetAnglesInWorld)
        .def_property_readonly("ranges", &Frame::GetRanges)
        .def_property_readonly("ray_directions_in_frame", &Frame::GetRayDirectionsInFrame)
        .def_property_readonly("ray_directions_in_world", &Frame::GetRayDirectionsInWorld)
        .def_property_readonly("end_points_in_frame", &Frame::GetEndPointsInFrame)
        .def_property_readonly("end_points_in_world", &Frame::GetEndPointsInWorld)
        .def_property_readonly("hit_ray_indices", &Frame::GetHitRayIndices)
        .def_property_readonly("hit_points_world", &Frame::GetHitPointsWorld)
        .def_property_readonly("min_valid_range", &Frame::GetMinValidRange)
        .def_property_readonly("max_valid_range", &Frame::GetMaxValidRange)
        .def_property_readonly("partitions", &Frame::GetPartitions)
        .def_property_readonly("is_partitioned", &Frame::IsPartitioned)
        .def_property_readonly("is_valid", &Frame::IsValid)
        .def(
            "compute_closest_end_point",
            [](Frame &self, const Eigen::Ref<const Vector2> &position) {
                long end_point_index = -1;
                Dtype distance = 0.0;
                self.ComputeClosestEndPoint(position, end_point_index, distance);
                py::dict out;
                out["end_point_index"] = end_point_index;
                out["distance"] = distance;
                return out;
            },
            py::arg("position"))
        .def(
            "sample_along_rays",
            [](const Frame &self,
               const long num_samples_per_ray,
               const Dtype max_in_obstacle_dist,
               const Dtype sampled_rays_ratio) {
                Matrix2X positions_world;
                Matrix2X directions_world;
                VectorX distances;
                self.SampleAlongRays(
                    num_samples_per_ray,
                    max_in_obstacle_dist,
                    sampled_rays_ratio,
                    positions_world,
                    directions_world,
                    distances);
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
            [](const Frame &self,
               const Dtype range_step,
               const Dtype max_in_obstacle_dist,
               const Dtype sampled_rays_ratio) {
                Matrix2X positions_world;
                Matrix2X directions_world;
                VectorX distances;
                self.SampleAlongRays(
                    range_step,
                    max_in_obstacle_dist,
                    sampled_rays_ratio,
                    positions_world,
                    directions_world,
                    distances);
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
            [](const Frame &self,
               const long num_samples_per_ray,
               const Dtype max_offset,
               const Dtype sampled_rays_ratio) {
                Matrix2X positions_world;
                Matrix2X directions_world;
                VectorX distances;
                self.SampleNearSurface(
                    num_samples_per_ray,
                    max_offset,
                    sampled_rays_ratio,
                    positions_world,
                    directions_world,
                    distances);
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
            [](const Frame &self,
               const long num_positions,
               const long num_along_ray_samples_per_ray,
               const long num_near_surface_samples_per_ray,
               const Dtype max_in_obstacle_dist) {
                Matrix2X positions_world;
                Matrix2X directions_world;
                VectorX distances;
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
            [](const Frame &self, const Eigen::Ref<const Vector2> &position_world) {
                Matrix2X directions_world;
                VectorX distances;
                std::vector<long> visible_hit_point_indices;
                self.ComputeRaysAt(
                    position_world,
                    directions_world,
                    distances,
                    visible_hit_point_indices);
                py::dict out;
                out["directions_world"] = directions_world;
                out["distances"] = distances;
                out["visible_hit_point_indices"] = visible_hit_point_indices;
                return out;
            },
            py::arg("position_world"));
}

void
BindLidarFrame2D(const py::module &m) {
    BindLidarFrame2DImpl<double>(m, "LidarFrame2Dd");
    BindLidarFrame2DImpl<float>(m, "LidarFrame2Df");
}
