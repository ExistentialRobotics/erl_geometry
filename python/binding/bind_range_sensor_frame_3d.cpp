#include "erl_common/pybind11.hpp"
#include "erl_geometry/range_sensor_frame_3d.hpp"

template<typename Dtype>
void
BindRangeSensorFrame3DImpl(const py::module &m, const char *name) {
    using namespace erl::common;
    using namespace erl::geometry;
    using T = RangeSensorFrame3D<Dtype>;
    using Vector2 = Eigen::Vector2<Dtype>;
    using Vector3 = Eigen::Vector3<Dtype>;
    using VectorX = Eigen::VectorX<Dtype>;
    using Matrix3X = Eigen::Matrix3X<Dtype>;

    py::class_<T, std::shared_ptr<T>> range_sensor_frame(m, name);
    py::class_<typename T::Setting, YamlableBase, std::shared_ptr<typename T::Setting>>(
        range_sensor_frame,
        "Setting")
        .def_readwrite("row_margin", &T::Setting::row_margin)
        .def_readwrite("col_margin", &T::Setting::col_margin)
        .def_readwrite("valid_range_min", &T::Setting::valid_range_min)
        .def_readwrite("valid_range_max", &T::Setting::valid_range_max);
    range_sensor_frame.def_property_readonly("num_rays", &T::GetNumRays)
        .def_property_readonly("num_hit_rays", &T::GetNumHitRays)
        .def_property_readonly("rotation_matrix", &T::GetRotationMatrix)
        .def_property_readonly("translation_vector", &T::GetTranslationVector)
        .def_property_readonly("pose_matrix", &T::GetPoseMatrix)
        .def_property_readonly("ranges", [](const T &self) { return self.GetRanges(); })
        .def_property_readonly(
            "frame_coords",
            [](const T &self) {
                const Eigen::MatrixX<Vector2> &frame_coords = self.GetFrameCoords();
                return py::cast_to_array(frame_coords);
            })
        .def_property_readonly("ranges", &T::GetRanges)
        .def_property_readonly(
            "ray_directions_in_frame",
            [](const T &self) -> py::array_t<Dtype> {
                const Eigen::MatrixX<Vector3> &ray_directions_in_frame =
                    self.GetRayDirectionsInFrame();
                return py::cast_to_array(ray_directions_in_frame);
            })
        .def_property_readonly(
            "ray_directions_in_world",
            [](const T &self) -> py::array_t<Dtype> {
                const Eigen::MatrixX<Vector3> &ray_directions_in_world =
                    self.GetRayDirectionsInWorld();
                return py::cast_to_array(ray_directions_in_world);
            })
        .def_property_readonly(
            "end_points_in_frame",
            [](const T &self) -> py::array_t<Dtype> {
                const Eigen::MatrixX<Vector3> &end_points_in_frame = self.GetEndPointsInFrame();
                return py::cast_to_array(end_points_in_frame);
            })
        .def_property_readonly(
            "end_points_in_world",
            [](const T &self) -> py::array_t<Dtype> {
                const Eigen::MatrixX<Vector3> &end_points_in_world = self.GetEndPointsInWorld();
                return py::cast_to_array(end_points_in_world);
            })
        .def_property_readonly("hit_ray_indices", &T::GetHitRayIndices)
        .def_property_readonly("hit_points_world", &T::GetHitPointsWorld)
        .def_property_readonly("min_valid_range", &T::GetMinValidRange)
        .def_property_readonly("max_valid_range", &T::GetMaxValidRange)
        .def_property_readonly("hit_mask", [](const T &self) { return self.GetHitMask(); })
        .def_property_readonly("is_valid", [](const T &self) { return self.IsValid(); })
        .def("position_is_in_frame", &T::PosIsInFrame, py::arg("xyz_frame"))
        .def("coords_is_in_frame", &T::CoordsIsInFrame, py::arg("frame_coords"))
        .def(
            "compute_frame_coords",
            [](T &self, const Vector3 &xyz_frame) {
                Dtype dist;
                Vector2 frame_coords;
                self.ComputeFrameCoords(xyz_frame, dist, frame_coords);
                py::dict out;
                out["distance"] = dist;
                out["frame_coords"] = frame_coords;
                return out;
            },
            py::arg("xyz_frame"))
        .def("dir_world_to_frame", &T::DirWorldToFrame, py::arg("dir_world"))
        .def("dir_frame_to_world", &T::DirFrameToWorld, py::arg("dir_frame"))
        .def("pos_world_to_frame", &T::PosWorldToFrame, py::arg("pos_world"))
        .def("pos_frame_to_world", &T::PosFrameToWorld, py::arg("pos_frame"))
        .def(
            "compute_closest_end_point",
            [](T &self,
               const Eigen::Ref<const Vector3> &position_world,
               const bool brute_force) -> py::dict {
                long row_index = -1;
                long col_index = -1;
                Dtype distance = 0.0;
                self.ComputeClosestEndPoint(
                    position_world,
                    row_index,
                    col_index,
                    distance,
                    brute_force);
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
            [](const T &self,
               const long num_samples_per_ray,
               const Dtype max_in_obstacle_dist,
               const Dtype sampled_rays_ratio) -> py::dict {
                Matrix3X positions_world;
                Matrix3X directions_world;
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
            [](const T &self,
               const Dtype range_step,
               const Dtype max_in_obstacle_dist,
               const Dtype sampled_rays_ratio) -> py::dict {
                Matrix3X positions_world;
                Matrix3X directions_world;
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
            [](const T &self,
               const long num_samples_per_ray,
               const Dtype max_offset,
               const Dtype sampled_rays_ratio) -> py::dict {
                Matrix3X positions_world;
                Matrix3X directions_world;
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
            "sample_in_region_hpr",
            [](const T &self,
               const long num_positions,
               const long num_near_surface_samples_per_ray,
               const long num_along_ray_samples_per_ray,
               const Dtype max_in_obstacle_dist,
               const bool parallel) -> py::dict {
                Matrix3X positions_world;
                Matrix3X directions_world;
                VectorX distances;
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
            [](const T &self,
               const long num_hit_points,
               const long num_samples_per_azimuth_segment,
               const long num_azimuth_segments,
               const bool parallel) -> py::dict {
                Matrix3X positions_world;
                Matrix3X directions_world;
                VectorX distances;
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
            [](const T &self, const Eigen::Ref<const Vector3> &position_world) -> py::dict {
                Matrix3X directions_world;
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
BindRangeSensorFrame3D(const py::module &m) {
    BindRangeSensorFrame3DImpl<double>(m, "RangeSensorFrame3Dd");
    BindRangeSensorFrame3DImpl<float>(m, "RangeSensorFrame3Df");
}
