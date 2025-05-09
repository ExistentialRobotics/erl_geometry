#pragma once

#include "kdtree_eigen_adaptor.hpp"

#include "erl_common/factory_pattern.hpp"
#include "erl_common/yaml.hpp"

#include <absl/container/flat_hash_map.h>

namespace erl::geometry {

    template<typename Dtype>
    class RangeSensorFrame3D {
    public:
        struct Setting : public common::Yamlable<Setting> {
            long row_margin = 0;
            long col_margin = 0;
            Dtype valid_range_min = 0.0f;
            Dtype valid_range_max = std::numeric_limits<Dtype>::max();

            struct YamlConvertImpl {
                static YAML::Node
                encode(const Setting &setting);

                static bool
                decode(const YAML::Node &node, Setting &setting);
            };
        };

        using MatrixX = Eigen::MatrixX<Dtype>;
        using Matrix3X = Eigen::Matrix3X<Dtype>;
        using Matrix3 = Eigen::Matrix3<Dtype>;
        using Matrix4 = Eigen::Matrix4<Dtype>;
        using VectorX = Eigen::VectorX<Dtype>;
        using Vector3 = Eigen::Vector3<Dtype>;
        using Vector2 = Eigen::Vector2<Dtype>;
        using KdTree = KdTreeEigenAdaptor<Dtype, 3>;

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;

    protected:
        Matrix3 m_rotation_ = {};
        Vector3 m_translation_ = {};

        // (row_coord, col_coord) in the frame, e.g. (azimuth, elevation) for LiDAR, (v, u) for RGBD
        Eigen::MatrixX<Vector2> m_frame_coords_ = {};
        MatrixX m_ranges_ = {};

        // the memory layout of the following matrices is (azimuth, elevation, 3), and it is okay
        // to access the data using a raw pointer directly because the memory is contiguous.

        Eigen::MatrixX<Vector3> m_dirs_frame_ = {};  // normalized directions in frame, (vx, vy, vz)
        Eigen::MatrixX<Vector3> m_dirs_world_ = {};  // normalized directions in world, (vx, vy, vz)
        Eigen::MatrixX<Vector3> m_end_pts_frame_ = {};  // end points in the frame, (x, y, z)
        Eigen::MatrixX<Vector3> m_end_pts_world_ = {};  // end points in the world, (x, y, z)
        Eigen::MatrixXb m_mask_hit_ = {};  // if i-th element is true, then i-th vertex is a hit
        std::vector<std::pair<long, long>> m_hit_ray_indices_ = {};  // hit ray indices
        std::vector<Vector3> m_hit_points_frame_ = {};               // hit points in the frame
        std::vector<Vector3> m_hit_points_world_ = {};               // hit points in the world
        Dtype m_max_valid_range_ = std::numeric_limits<Dtype>::min();
        std::shared_ptr<KdTree> m_kd_tree_ = std::make_shared<KdTree>();

    public:
        using Factory = common::
            FactoryPattern<RangeSensorFrame3D, false, false, const std::shared_ptr<Setting> &>;

        explicit RangeSensorFrame3D(std::shared_ptr<Setting> setting);

        virtual ~RangeSensorFrame3D() = default;

        // factory method
        static std::shared_ptr<RangeSensorFrame3D>
        Create(const std::string &type, const std::shared_ptr<Setting> &setting);

        template<typename Derived>
        static bool
        Register(std::string frame_type = "");

        [[nodiscard]] long
        GetNumRays() const;

        [[nodiscard]] long
        GetNumHitRays() const;

        [[nodiscard]] const Matrix3 &
        GetRotationMatrix() const;

        [[nodiscard]] const Vector3 &
        GetTranslationVector() const;

        [[nodiscard]] Matrix4
        GetPoseMatrix() const;

        [[nodiscard]] const Eigen::MatrixX<Vector2> &
        GetFrameCoords() const;

        [[nodiscard]] virtual bool
        PointIsInFrame(const Vector3 &xyz_frame) const = 0;

        [[nodiscard]] bool
        CoordsIsInFrame(const Vector2 &frame_coords) const;

        [[nodiscard]] virtual Vector2
        ComputeFrameCoords(const Vector3 &xyz_frame) const = 0;

        [[nodiscard]] virtual Vector3
        DirWorldToFrame(const Vector3 &dir_world) const;

        [[nodiscard]] virtual Vector3
        DirFrameToWorld(const Vector3 &dir_frame) const;

        [[nodiscard]] virtual Vector3
        PosWorldToFrame(const Vector3 &pos_world) const;

        [[nodiscard]] virtual Vector3
        PosFrameToWorld(const Vector3 &pos_frame) const;

        virtual void
        UpdateRanges(
            const Eigen::Ref<const Matrix3> &rotation,
            const Eigen::Ref<const Vector3> &translation,
            MatrixX ranges) = 0;

        [[nodiscard]] const MatrixX &
        GetRanges() const;

        [[nodiscard]] const Eigen::MatrixX<Vector3> &
        GetRayDirectionsInFrame() const;

        [[nodiscard]] const Eigen::MatrixX<Vector3> &
        GetRayDirectionsInWorld() const;

        [[nodiscard]] const Eigen::MatrixX<Vector3> &
        GetEndPointsInFrame() const;

        [[nodiscard]] const Eigen::MatrixX<Vector3> &
        GetEndPointsInWorld() const;

        [[nodiscard]] const std::vector<std::pair<long, long>> &
        GetHitRayIndices() const;

        [[nodiscard]] const std::vector<Vector3> &
        GetHitPointsFrame() const;

        [[nodiscard]] const std::vector<Vector3> &
        GetHitPointsWorld() const;

        [[nodiscard]] Dtype
        GetMaxValidRange() const;

        [[nodiscard]] const Eigen::MatrixXb &
        GetHitMask() const;

        [[nodiscard]] bool
        IsValid() const;

        void
        ComputeClosestEndPoint(
            const Eigen::Ref<const Vector3> &position_world,
            long &end_point_row_index,
            long &end_point_col_index,
            Dtype &distance,
            bool brute_force = false);

        void
        SampleAlongRays(
            long num_samples_per_ray,
            Dtype max_in_obstacle_dist,
            Dtype sampled_rays_ratio,
            Matrix3X &positions_world,
            Matrix3X &directions_world,
            VectorX &distances) const;

        void
        SampleAlongRays(
            Dtype range_step,
            Dtype max_in_obstacle_dist,
            Dtype sampled_rays_ratio,
            Matrix3X &positions_world,
            Matrix3X &directions_world,
            VectorX &distances) const;

        void
        SampleNearSurface(
            long num_samples_per_ray,
            Dtype max_offset,
            Dtype sampled_rays_ratio,
            Matrix3X &positions_world,
            Matrix3X &directions_world,
            VectorX &distances) const;

        void
        SampleInRegionHpr(  // HPR: hidden point removal
            long num_positions,
            long num_along_ray_samples_per_ray,
            long num_near_surface_samples_per_ray,
            Dtype max_in_obstacle_dist,
            Matrix3X &positions_world,
            Matrix3X &directions_world,
            VectorX &distances,
            bool parallel = false) const;

        void
        SampleInRegionVrs(  // VRS: visible ray synthesis
            long num_hit_points,
            long num_samples_per_azimuth_segment,
            long num_azimuth_segments,
            Matrix3X &positions_world,
            Matrix3X &directions_world,
            VectorX &distances,
            bool parallel = false) const;

        void
        ComputeRaysAt(
            const Eigen::Ref<const Vector3> &position_world,
            Matrix3X &directions_world,
            VectorX &distances,
            std::vector<long> &visible_hit_point_indices) const;

        [[nodiscard]] virtual bool
        operator==(const RangeSensorFrame3D &other) const;

        [[nodiscard]] virtual bool
        operator!=(const RangeSensorFrame3D &other) const;

        [[nodiscard]] virtual bool
        Write(std::ostream &s) const;

        [[nodiscard]] virtual bool
        Read(std::istream &s);

    protected:
        void
        SampleInRegionHprThread(
            uint64_t seed,
            long num_positions,
            long num_along_ray_samples_per_ray,
            long num_near_surface_samples_per_ray,
            Dtype max_in_obstacle_dist,
            Matrix3X *positions_world_ptr,
            Matrix3X *directions_world_ptr,
            VectorX *distances_ptr) const;

        void
        SampleInRegionVrsThread(
            uint64_t seed,
            const long *hit_point_index_start,
            const long *hit_point_index_end,
            long num_samples_per_azimuth_segment,
            long num_azimuth_segments,
            Matrix3X *positions_world_ptr,
            Matrix3X *directions_world_ptr,
            VectorX *distances_ptr) const;
    };

    using RangeSensorFrame3Dd = RangeSensorFrame3D<double>;
    using RangeSensorFrame3Df = RangeSensorFrame3D<float>;
}  // namespace erl::geometry

#include "range_sensor_frame_3d.tpp"

template<>
struct YAML::convert<erl::geometry::RangeSensorFrame3D<double>::Setting>
    : erl::geometry::RangeSensorFrame3D<double>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::geometry::RangeSensorFrame3D<float>::Setting>
    : erl::geometry::RangeSensorFrame3D<float>::Setting::YamlConvertImpl {};
