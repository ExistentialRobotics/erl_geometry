#pragma once

#include "kdtree_eigen_adaptor.hpp"

#include "erl_common/eigen.hpp"
#include "erl_common/yaml.hpp"

namespace erl::geometry {

    /**
     * 1. detects discontinuities in a sensor frame.
     * 2. detects out-of-max-range measurements
     */
    template<typename Dtype>
    class LidarFrame2D {
    public:
        using Matrix2 = Eigen::Matrix2<Dtype>;
        using Matrix3 = Eigen::Matrix3<Dtype>;
        using Matrix2X = Eigen::Matrix2X<Dtype>;
        using Vector2 = Eigen::Vector2<Dtype>;
        using VectorX = Eigen::VectorX<Dtype>;
        using KdTree = KdTreeEigenAdaptor<Dtype, 2>;

        struct Setting : public common::Yamlable<Setting> {
            Dtype valid_range_min = 0.0;
            Dtype valid_range_max = std::numeric_limits<Dtype>::infinity();
            Dtype angle_min = -M_PI;
            Dtype angle_max = M_PI;
            long num_rays = 360;
            bool discontinuity_detection = true;
            Dtype discontinuity_factor = 10;
            Dtype rolling_diff_discount = 0.9;
            long min_partition_size = 5;

            struct YamlConvertImpl {
                static YAML::Node
                encode(const Setting &setting);

                static bool
                decode(const YAML::Node &node, Setting &setting);
            };

            long
            Resize(Dtype factor);
        };

        class Partition {
            LidarFrame2D *m_frame_ = nullptr;
            long m_index_begin_ = -1;
            long m_index_end_ = -1;  // inclusive

            friend class LidarFrame2D;

        public:
            Partition(LidarFrame2D *frame, long index_begin, long index_end);

            [[nodiscard]] long
            GetIndexBegin() const;

            [[nodiscard]] long
            GetIndexEnd() const;

            [[nodiscard]] bool
            AngleInPartition(Dtype angle_world) const;
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        Matrix2 m_rotation_ = {};
        Dtype m_rotation_angle_ = 0.0;
        Vector2 m_translation_ = {};

        VectorX m_angles_frame_ = {};
        VectorX m_angles_world_ = {};
        VectorX m_ranges_ = {};

        std::vector<Vector2> m_dirs_frame_ = {};
        std::vector<Vector2> m_dirs_world_ = {};

        std::vector<Vector2> m_end_pts_frame_ = {};
        std::vector<Vector2> m_end_pts_world_ = {};

        // if i-th element is true, then i-th vertex is a hit
        Eigen::VectorXb m_mask_hit_ = {};
        // if i-th element is true, then (i-1, i) and (i, i+1) edges are continuous
        Eigen::VectorXb m_mask_continuous_ = {};
        std::vector<long> m_hit_ray_indices_ = {};      // hit ray indices
        std::vector<Vector2> m_hit_points_frame_ = {};  // hit points in the frame
        std::vector<Vector2> m_hit_points_world_ = {};  // hit points in the world

        Dtype m_max_valid_range_ = 0.0;
        bool m_partitioned_ = false;
        std::vector<Partition> m_partitions_ = {};
        std::shared_ptr<KdTree> m_kd_tree_ = std::make_shared<KdTree>();

    public:
        explicit LidarFrame2D(std::shared_ptr<Setting> setting);

        [[nodiscard]] bool
        AngleIsInFrame(Dtype angle_frame) const;

        [[nodiscard]] bool
        PointIsInFrame(const Vector2 &xy_frame) const;

        [[nodiscard]] Vector2
        DirWorldToFrame(const Vector2 &dir_world) const;

        [[nodiscard]] Vector2
        DirFrameToWorld(const Vector2 &dir_frame) const;

        [[nodiscard]] Vector2
        PosWorldToFrame(const Vector2 &pos_world) const;

        [[nodiscard]] Vector2
        PosFrameToWorld(const Vector2 &pos_local) const;

        void
        UpdateRanges(
            const Eigen::Ref<const Matrix2> &rotation,
            const Eigen::Ref<const Vector2> &translation,
            VectorX ranges);

        /**
         * Convert a point cloud to ranges.
         * @param rotation The rotation of the sensor, which is a 2x2 matrix.
         * @param translation The translation of the sensor, which is a 2x1 vector.
         * @param points The point cloud, which is a 2xN matrix.
         * @param are_local If true, the points are in the local frame.
         * @return The ranges, which is a vector of size N.
         */
        [[nodiscard]] VectorX
        PointCloudToRanges(
            const Matrix2 &rotation,
            const Vector2 &translation,
            const Eigen::Ref<const Matrix2X> &points,
            bool are_local) const;

        [[nodiscard]] const std::shared_ptr<Setting> &
        GetSetting() const;

        [[nodiscard]] long
        GetNumRays() const;

        [[nodiscard]] long
        GetNumHitRays() const;

        [[nodiscard]] const Matrix2 &
        GetRotationMatrix() const;

        [[nodiscard]] Dtype
        GetRotationAngle() const;

        [[nodiscard]] const Vector2 &
        GetTranslationVector() const;

        [[nodiscard]] Matrix3
        GetPoseMatrix() const;

        [[nodiscard]] const VectorX &
        GetAnglesInFrame() const;

        [[nodiscard]] const VectorX &
        GetAnglesInWorld() const;

        [[nodiscard]] const VectorX &
        GetRanges() const;

        [[nodiscard]] const std::vector<Vector2> &
        GetRayDirectionsInFrame() const;

        [[nodiscard]] const std::vector<Vector2> &
        GetRayDirectionsInWorld() const;

        [[nodiscard]] const std::vector<Vector2> &
        GetEndPointsInFrame() const;

        [[nodiscard]] const std::vector<Vector2> &
        GetEndPointsInWorld() const;

        [[nodiscard]] const Eigen::VectorXb &
        GetHitMask() const;

        [[nodiscard]] const Eigen::VectorXb &
        GetContinuityMask() const;

        [[nodiscard]] const std::vector<long> &
        GetHitRayIndices() const;

        [[nodiscard]] const std::vector<Vector2> &
        GetHitPointsFrame() const;

        [[nodiscard]] const std::vector<Vector2> &
        GetHitPointsWorld() const;

        [[nodiscard]] Dtype
        GetMinValidRange() const;

        [[nodiscard]] Dtype
        GetMaxValidRange() const;

        [[nodiscard]] const std::vector<Partition> &
        GetPartitions() const;

        [[nodiscard]] bool
        IsPartitioned() const;

        [[nodiscard]] bool
        IsValid() const;

        void
        ComputeClosestEndPoint(
            const Eigen::Ref<const Vector2> &position_world,
            long &end_point_index,
            Dtype &distance,
            bool brute_force = false);

        void
        SampleAlongRays(
            long n_samples_per_ray,
            Dtype max_in_obstacle_dist,
            Dtype sampled_rays_ratio,
            Matrix2X &positions_world,
            Matrix2X &directions_world,
            VectorX &distances) const;

        void
        SampleAlongRays(
            Dtype range_step,
            Dtype max_in_obstacle_dist,
            Dtype sampled_rays_ratio,
            Matrix2X &positions_world,
            Matrix2X &directions_world,
            VectorX &distances) const;

        void
        SampleNearSurface(
            long num_samples_per_ray,
            Dtype max_offset,
            Dtype sampled_rays_ratio,
            Matrix2X &positions_world,
            Matrix2X &directions_world,
            VectorX &distances) const;

        void
        SampleInRegion(
            long num_positions,
            long num_along_ray_samples_per_ray,
            long num_near_surface_samples_per_ray,
            Dtype max_in_obstacle_dist,
            Matrix2X &positions_world,
            Matrix2X &directions_world,
            VectorX &distances) const;

        void
        ComputeRaysAt(
            const Eigen::Ref<const Vector2> &position_world,
            Matrix2X &directions_world,
            VectorX &distances,
            std::vector<long> &visible_hit_point_indices) const;

        [[nodiscard]] bool
        operator==(const LidarFrame2D &other) const;

        [[nodiscard]] bool
        operator!=(const LidarFrame2D &other) const;

        [[nodiscard]] bool
        Write(std::ostream &s) const;

        [[nodiscard]] bool
        Read(std::istream &s);

        void
        PartitionRays();
    };

    using LidarFrame2Dd = LidarFrame2D<double>;
    using LidarFrame2Df = LidarFrame2D<float>;

    extern template class LidarFrame2D<double>;
    extern template class LidarFrame2D<float>;
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::LidarFrame2D<double>::Setting>
    : erl::geometry::LidarFrame2Dd::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::geometry::LidarFrame2D<float>::Setting>
    : erl::geometry::LidarFrame2Df::Setting::YamlConvertImpl {};
