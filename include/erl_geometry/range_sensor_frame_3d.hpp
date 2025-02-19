#pragma once

#include "kdtree_eigen_adaptor.hpp"

#include "erl_common/factory_pattern.hpp"
#include "erl_common/yaml.hpp"

#include <absl/container/flat_hash_map.h>

namespace erl::geometry {

    template<typename Dtype>
    class RangeSensorFrame3D {
    public:
        struct Setting : common::Yamlable<Setting> {
            long row_margin = 0;
            long col_margin = 0;
            Dtype valid_range_min = 0.0;
            Dtype valid_range_max = std::numeric_limits<Dtype>::max();
            Dtype discontinuity_factor = 10;
            Dtype rolling_diff_discount = 0.9;
            int min_partition_size = 5;

            struct YamlConvertImpl {
                static YAML::Node
                encode(const Setting &setting);

                static bool
                decode(const YAML::Node &node, Setting &setting);
            };
        };

        using Matrix = Eigen::MatrixX<Dtype>;
        using Matrix3X = Eigen::Matrix3X<Dtype>;
        using Matrix3 = Eigen::Matrix3<Dtype>;
        using Vector = Eigen::VectorX<Dtype>;
        using Vector3 = Eigen::Vector3<Dtype>;
        using Vector2 = Eigen::Vector2<Dtype>;
        using KdTree = KdTreeEigenAdaptor<Dtype, 3>;

    private:
        inline static const volatile bool kSettingRegistered = common::YamlableBase::Register<Setting>();
        inline static const std::string kFileHeader = fmt::format("# erl::geometry::RangeSensorFrame3D<{}>", type_name<Dtype>());
        std::shared_ptr<Setting> m_setting_ = nullptr;

    protected:
        Matrix3 m_rotation_ = {};
        Vector3 m_translation_ = {};

        Eigen::MatrixX<Vector2> m_frame_coords_ = {};  // (row_coord, col_coord) in frame, e.g. (azimuth, elevation) for LiDAR, (v, u) for RGBD
        Matrix m_ranges_ = {};

        // the memory layout of the following matrices is (azimuth, elevation, 3), and it is okay to access the data using raw pointer directly
        // because the memory is contiguous.

        Eigen::MatrixX<Vector3> m_dirs_frame_ = {};  // directions in frame, (vx, vy, vz), normalized
        Eigen::MatrixX<Vector3> m_dirs_world_ = {};  // directions in world, (vx, vy, vz), normalized

        Eigen::MatrixX<Vector3> m_end_pts_frame_ = {};  // end points in frame, (x, y, z)
        Eigen::MatrixX<Vector3> m_end_pts_world_ = {};  // end points in world, (x, y, z)

        Eigen::MatrixXb m_mask_hit_ = {};                            // if i-th element is true, then i-th vertex is a hit
        Eigen::MatrixXb m_mask_continuous_ = {};                     // if i-th element is true, then (i-1, i) edge is continuous
        std::vector<std::pair<long, long>> m_hit_ray_indices_ = {};  // hit ray indices
        std::vector<Vector3> m_hit_points_world_ = {};               // hit points in world
        Dtype m_max_valid_range_ = std::numeric_limits<Dtype>::min();
        std::shared_ptr<KdTree> m_kd_tree_ = std::make_shared<KdTree>();

    public:
        using Factory = common::FactoryPattern<RangeSensorFrame3D, false, false, const std::shared_ptr<Setting> &>;

        explicit RangeSensorFrame3D(std::shared_ptr<Setting> setting)
            : m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr when creating RangeSensorFrame3D.");
        }

        virtual ~RangeSensorFrame3D() = default;

        // factory method
        static std::shared_ptr<RangeSensorFrame3D>
        Create(const std::string &type, const std::shared_ptr<Setting> &setting);

        template<typename Derived>
        static bool
        Register(std::string frame_type = "");

        [[nodiscard]] long
        GetNumRays() const {
            return m_ranges_.size();
        }

        [[nodiscard]] long
        GetNumHitRays() const {
            return static_cast<long>(m_hit_ray_indices_.size());
        }

        [[nodiscard]] const Matrix3 &
        GetRotationMatrix() const {
            return m_rotation_;
        }

        [[nodiscard]] const Vector3 &
        GetTranslationVector() const {
            return m_translation_;
        }

        [[nodiscard]] Eigen::Matrix4d
        GetPoseMatrix() const {
            Eigen::Isometry3d pose;
            pose.linear() = m_rotation_;
            pose.translation() = m_translation_;
            return pose.matrix();
        }

        [[nodiscard]] const Eigen::MatrixX<Vector2> &
        GetFrameCoords() const {
            return m_frame_coords_;
        }

        [[nodiscard]] virtual bool
        PointIsInFrame(const Vector3 &xyz_frame) const = 0;

        [[nodiscard]] bool
        CoordsIsInFrame(const Vector2 &frame_coords) const {
            const Vector2 &top_left = m_frame_coords_(m_setting_->row_margin, m_setting_->col_margin);
            const Vector2 &bottom_right = m_frame_coords_(  //
                m_frame_coords_.rows() - m_setting_->row_margin - 1,
                m_frame_coords_.cols() - m_setting_->col_margin - 1);
            return frame_coords[0] >= top_left[0] && frame_coords[0] <= bottom_right[0] &&  //
                   frame_coords[1] >= top_left[1] && frame_coords[1] <= bottom_right[1];
        }

        [[nodiscard]] virtual Vector2
        ComputeFrameCoords(const Vector3 &xyz_frame) const = 0;

        [[nodiscard]] virtual Vector3
        WorldToFrameSo3(const Vector3 &dir_world) const {
            return m_rotation_.transpose() * dir_world;
        }

        [[nodiscard]] virtual Vector3
        FrameToWorldSo3(const Vector3 &dir_frame) const {
            return m_rotation_ * dir_frame;
        }

        [[nodiscard]] virtual Vector3
        WorldToFrameSe3(const Vector3 &xyz_world) const {
            return m_rotation_.transpose() * (xyz_world - m_translation_);
        }

        [[nodiscard]] virtual Vector3
        FrameToWorldSe3(const Vector3 &xyz_frame) const {
            return m_rotation_ * xyz_frame + m_translation_;
        }

        virtual void
        UpdateRanges(const Eigen::Ref<const Matrix3> &rotation, const Eigen::Ref<const Vector3> &translation, Matrix ranges, bool partition_rays) = 0;

        [[nodiscard]] const Matrix &
        GetRanges() const {
            return m_ranges_;
        }

        [[nodiscard]] const Eigen::MatrixX<Vector3> &
        GetRayDirectionsInFrame() const {
            return m_dirs_frame_;
        }

        [[nodiscard]] const Eigen::MatrixX<Vector3> &
        GetRayDirectionsInWorld() const {
            return m_dirs_world_;
        }

        [[nodiscard]] const Eigen::MatrixX<Vector3> &
        GetEndPointsInFrame() const {
            return m_end_pts_frame_;
        }

        [[nodiscard]] const Eigen::MatrixX<Vector3> &
        GetEndPointsInWorld() const {
            return m_end_pts_world_;
        }

        [[nodiscard]] const std::vector<std::pair<long, long>> &
        GetHitRayIndices() const {
            return m_hit_ray_indices_;
        }

        [[nodiscard]] const std::vector<Vector3> &
        GetHitPointsWorld() const {
            return m_hit_points_world_;
        }

        [[nodiscard]] Dtype
        GetMaxValidRange() const {
            return m_max_valid_range_;
        }

        [[nodiscard]] const Eigen::MatrixXb &
        GetHitMask() const {
            return m_mask_hit_;
        }

        [[nodiscard]] bool
        IsValid() const {
            return m_max_valid_range_ > 0.0;
        }

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
            Vector &distances) const;

        void
        SampleAlongRays(
            Dtype range_step,
            Dtype max_in_obstacle_dist,
            Dtype sampled_rays_ratio,
            Matrix3X &positions_world,
            Matrix3X &directions_world,
            Vector &distances) const;

        void
        SampleNearSurface(
            long num_samples_per_ray,
            Dtype max_offset,
            Dtype sampled_rays_ratio,
            Matrix3X &positions_world,
            Matrix3X &directions_world,
            Vector &distances) const;

        void
        SampleInRegionHpr(  // HPR: hidden point removal
            long num_positions,
            long num_along_ray_samples_per_ray,
            long num_near_surface_samples_per_ray,
            Dtype max_in_obstacle_dist,
            Matrix3X &positions_world,
            Matrix3X &directions_world,
            Vector &distances,
            bool parallel = false) const;

        void
        SampleInRegionVrs(  // VRS: visible ray synthesis
            long num_hit_points,
            long num_samples_per_azimuth_segment,
            long num_azimuth_segments,
            Matrix3X &positions_world,
            Matrix3X &directions_world,
            Vector &distances,
            bool parallel = false) const;

        void
        ComputeRaysAt(
            const Eigen::Ref<const Vector3> &position_world,
            Matrix3X &directions_world,
            Vector &distances,
            std::vector<long> &visible_hit_point_indices) const;

        [[nodiscard]] virtual bool
        operator==(const RangeSensorFrame3D &other) const;

        [[nodiscard]] virtual bool
        operator!=(const RangeSensorFrame3D &other) const {
            return !(*this == other);
        }

        [[nodiscard]] virtual bool
        Write(const std::string &filename) const;

        [[nodiscard]] virtual bool
        Write(std::ostream &s) const;

        [[nodiscard]] virtual bool
        Read(const std::string &filename);

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
            Vector *distances_ptr) const;

        void
        SampleInRegionVrsThread(
            uint64_t seed,
            const long *hit_point_index_start,
            const long *hit_point_index_end,
            long num_samples_per_azimuth_segment,
            long num_azimuth_segments,
            Matrix3X *positions_world_ptr,
            Matrix3X *directions_world_ptr,
            Vector *distances_ptr) const;
    };

#define ERL_REGISTER_RANGE_SENSOR_FRAME_3D(Derived) inline const volatile bool kRegistered##Derived = Derived::Register<Derived>()
}  // namespace erl::geometry

#include "range_sensor_frame_3d.tpp"

template<>
struct YAML::convert<erl::geometry::RangeSensorFrame3D<double>::Setting> : erl::geometry::RangeSensorFrame3D<double>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::geometry::RangeSensorFrame3D<float>::Setting> : erl::geometry::RangeSensorFrame3D<float>::Setting::YamlConvertImpl {};
