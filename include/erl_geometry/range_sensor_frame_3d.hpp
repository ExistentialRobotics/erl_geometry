#pragma once

#include "kdtree_eigen_adaptor.hpp"

#include "erl_common/yaml.hpp"

namespace erl::geometry {
    class RangeSensorFrame3D {
    public:
        struct Setting : common::Yamlable<Setting> {
            long row_margin = 0;
            long col_margin = 0;
            double valid_range_min = 0.0;
            double valid_range_max = std::numeric_limits<double>::max();
            double discontinuity_factor = 10;
            double rolling_diff_discount = 0.9;
            int min_partition_size = 5;
        };

    protected:
        inline static std::map<std::string, std::function<std::shared_ptr<RangeSensorFrame3D>(const std::shared_ptr<Setting> &)>> s_class_id_mapping_ = {};

        std::shared_ptr<Setting> m_setting_ = nullptr;

        Eigen::Matrix3d m_rotation_ = {};
        Eigen::Vector3d m_translation_ = {};

        Eigen::MatrixX<Eigen::Vector2d> m_frame_coords_ = {};  // (row_coord, col_coord) in frame, e.g. (azimuth, elevation) for LiDAR, (v, u) for RGBD
        Eigen::MatrixXd m_ranges_ = {};

        // the memory layout of the following matrices is (azimuth, elevation, 3), and it is okay to access the data using raw pointer directly
        // because the memory is contiguous.

        Eigen::MatrixX<Eigen::Vector3d> m_dirs_frame_ = {};  // directions in frame, (vx, vy, vz), normalized
        Eigen::MatrixX<Eigen::Vector3d> m_dirs_world_ = {};  // directions in world, (vx, vy, vz), normalized

        Eigen::MatrixX<Eigen::Vector3d> m_end_pts_frame_ = {};  // end points in frame, (x, y, z)
        Eigen::MatrixX<Eigen::Vector3d> m_end_pts_world_ = {};  // end points in world, (x, y, z)

        Eigen::MatrixXb m_mask_hit_ = {};                            // if i-th element is true, then i-th vertex is a hit
        Eigen::MatrixXb m_mask_continuous_ = {};                     // if i-th element is true, then (i-1, i) edge is continuous
        std::vector<std::pair<long, long>> m_hit_ray_indices_ = {};  // hit ray indices
        std::vector<Eigen::Vector3d> m_hit_points_world_ = {};       // hit points in world
        double m_max_valid_range_ = std::numeric_limits<double>::min();
        std::shared_ptr<KdTree3d> m_kd_tree_ = std::make_shared<KdTree3d>();

    public:
        explicit RangeSensorFrame3D(std::shared_ptr<Setting> setting)
            : m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr when creating RangeSensorFrame3D.");
        }

        virtual ~RangeSensorFrame3D() = default;

        // factory method
        static std::shared_ptr<RangeSensorFrame3D>
        Create(const std::string &type, const std::shared_ptr<Setting> &setting);

        template<typename Derived>
        static std::enable_if_t<std::is_base_of_v<RangeSensorFrame3D, Derived>, bool>
        RegisterFrameType() {
            if (const std::string &frame_type = Derived::GetFrameType(); s_class_id_mapping_.find(frame_type) != s_class_id_mapping_.end()) {
                ERL_WARN("Derived RangeSensorFrame3D of type {} is already registered.", frame_type);
                return false;
            }
            s_class_id_mapping_[Derived::GetFrameType()] = [](const std::shared_ptr<Setting> &setting) -> std::shared_ptr<RangeSensorFrame3D> {
                if (setting == nullptr) {
                    ERL_WARN("setting is nullptr before creating a derived RangeSensorFrame3D of type {}.", Derived::GetFrameType());
                    return nullptr;
                }
                auto frame_setting = std::dynamic_pointer_cast<typename Derived::Setting>(setting);
                if (frame_setting == nullptr) {
                    ERL_WARN("Failed to cast setting for derived RangeSensorFrame3D of type {}.", Derived::GetFrameType());
                    return nullptr;
                }
                return std::make_shared<Derived>(frame_setting);
            };
            ERL_DEBUG("Registered RangeSensorFrame3D of type {}.", Derived::GetFrameType());
            return true;
        }

        [[nodiscard]] long
        GetNumRays() const {
            return m_ranges_.size();
        }

        [[nodiscard]] long
        GetNumHitRays() const {
            return static_cast<long>(m_hit_ray_indices_.size());
        }

        [[nodiscard]] const Eigen::Matrix3d &
        GetRotationMatrix() const {
            return m_rotation_;
        }

        [[nodiscard]] const Eigen::Vector3d &
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

        [[nodiscard]] const Eigen::MatrixX<Eigen::Vector2d> &
        GetFrameCoords() const {
            return m_frame_coords_;
        }

        [[nodiscard]] virtual bool
        PointIsInFrame(const Eigen::Vector3d &xyz_frame) const = 0;

        [[nodiscard]] bool
        CoordsIsInFrame(const Eigen::Vector2d &frame_coords) const {
            const Eigen::Vector2d &top_left = m_frame_coords_(m_setting_->row_margin, m_setting_->col_margin);
            const Eigen::Vector2d &bottom_right = m_frame_coords_(  //
                m_frame_coords_.rows() - m_setting_->row_margin - 1,
                m_frame_coords_.cols() - m_setting_->col_margin - 1);
            return frame_coords[0] >= top_left[0] && frame_coords[0] <= bottom_right[0] &&  //
                   frame_coords[1] >= top_left[1] && frame_coords[1] <= bottom_right[1];
        }

        [[nodiscard]] virtual Eigen::Vector2d
        ComputeFrameCoords(const Eigen::Vector3d &xyz_frame) const = 0;

        [[nodiscard]] virtual Eigen::Vector3d
        WorldToFrameSo3(const Eigen::Vector3d &dir_world) const = 0;

        [[nodiscard]] virtual Eigen::Vector3d
        FrameToWorldSo3(const Eigen::Vector3d &dir_frame) const = 0;

        [[nodiscard]] virtual Eigen::Vector3d
        WorldToFrameSe3(const Eigen::Vector3d &xyz_world) const = 0;

        [[nodiscard]] virtual Eigen::Vector3d
        FrameToWorldSe3(const Eigen::Vector3d &xyz_frame) const = 0;

        virtual void
        UpdateRanges(
            const Eigen::Ref<const Eigen::Matrix3d> &rotation,
            const Eigen::Ref<const Eigen::Vector3d> &translation,
            Eigen::MatrixXd ranges,
            bool partition_rays) = 0;

        [[nodiscard]] const Eigen::MatrixXd &
        GetRanges() const {
            return m_ranges_;
        }

        [[nodiscard]] const Eigen::MatrixX<Eigen::Vector3d> &
        GetRayDirectionsInFrame() const {
            return m_dirs_frame_;
        }

        [[nodiscard]] const Eigen::MatrixX<Eigen::Vector3d> &
        GetRayDirectionsInWorld() const {
            return m_dirs_world_;
        }

        [[nodiscard]] const Eigen::MatrixX<Eigen::Vector3d> &
        GetEndPointsInFrame() const {
            return m_end_pts_frame_;
        }

        [[nodiscard]] const Eigen::MatrixX<Eigen::Vector3d> &
        GetEndPointsInWorld() const {
            return m_end_pts_world_;
        }

        [[nodiscard]] const std::vector<std::pair<long, long>> &
        GetHitRayIndices() const {
            return m_hit_ray_indices_;
        }

        [[nodiscard]] const std::vector<Eigen::Vector3d> &
        GetHitPointsWorld() const {
            return m_hit_points_world_;
        }

        [[nodiscard]] double
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
            const Eigen::Ref<const Eigen::Vector3d> &position_world,
            long &end_point_row_index,
            long &end_point_col_index,
            double &distance,
            bool brute_force = false);

        void
        SampleAlongRays(
            long num_samples_per_ray,
            double max_in_obstacle_dist,
            double sampled_rays_ratio,
            Eigen::Matrix3Xd &positions_world,
            Eigen::Matrix3Xd &directions_world,
            Eigen::VectorXd &distances) const;

        void
        SampleAlongRays(
            double range_step,
            double max_in_obstacle_dist,
            double sampled_rays_ratio,
            Eigen::Matrix3Xd &positions_world,
            Eigen::Matrix3Xd &directions_world,
            Eigen::VectorXd &distances) const;

        void
        SampleNearSurface(
            long num_samples_per_ray,
            double max_offset,
            double sampled_rays_ratio,
            Eigen::Matrix3Xd &positions_world,
            Eigen::Matrix3Xd &directions_world,
            Eigen::VectorXd &distances) const;

        void
        SampleInRegionHpr(  // HPR: hidden point removal
            long num_positions,
            long num_along_ray_samples_per_ray,
            long num_near_surface_samples_per_ray,
            double max_in_obstacle_dist,
            Eigen::Matrix3Xd &positions_world,
            Eigen::Matrix3Xd &directions_world,
            Eigen::VectorXd &distances,
            bool parallel = false) const;

        void
        SampleInRegionVrs(  // VRS: visible ray synthesis
            long num_hit_points,
            long num_samples_per_azimuth_segment,
            long num_azimuth_segments,
            Eigen::Matrix3Xd &positions_world,
            Eigen::Matrix3Xd &directions_world,
            Eigen::VectorXd &distances,
            bool parallel = false) const;

        void
        ComputeRaysAt(
            const Eigen::Ref<const Eigen::Vector3d> &position_world,
            Eigen::Matrix3Xd &directions_world,
            Eigen::VectorXd &distances,
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
            double max_in_obstacle_dist,
            Eigen::Matrix3Xd *positions_world_ptr,
            Eigen::Matrix3Xd *directions_world_ptr,
            Eigen::VectorXd *distances_ptr) const;

        void
        SampleInRegionVrsThread(
            uint64_t seed,
            const long *hit_point_index_start,
            const long *hit_point_index_end,
            long num_samples_per_azimuth_segment,
            long num_azimuth_segments,
            Eigen::Matrix3Xd *positions_world_ptr,
            Eigen::Matrix3Xd *directions_world_ptr,
            Eigen::VectorXd *distances_ptr) const;
    };

#define ERL_REGISTER_RANGE_SENSOR_FRAME_3D(Derived) inline const volatile bool kRegistered##Derived = RangeSensorFrame3D::RegisterFrameType<Derived>()
}  // namespace erl::geometry

// ReSharper disable CppInconsistentNaming
template<>
struct YAML::convert<erl::geometry::RangeSensorFrame3D::Setting> {
    static Node
    encode(const erl::geometry::RangeSensorFrame3D::Setting &rhs) {
        Node node;
        node["row_margin"] = rhs.row_margin;
        node["col_margin"] = rhs.col_margin;
        node["valid_range_min"] = rhs.valid_range_min;
        node["valid_range_max"] = rhs.valid_range_max;
        node["discontinuity_factor"] = rhs.discontinuity_factor;
        node["rolling_diff_discount"] = rhs.rolling_diff_discount;
        node["min_partition_size"] = rhs.min_partition_size;
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::RangeSensorFrame3D::Setting &rhs) {
        if (!node.IsMap()) { return false; }
        rhs.row_margin = node["row_margin"].as<long>();
        rhs.col_margin = node["col_margin"].as<long>();
        rhs.valid_range_min = node["valid_range_min"].as<double>();
        rhs.valid_range_max = node["valid_range_max"].as<double>();
        rhs.discontinuity_factor = node["discontinuity_factor"].as<double>();
        rhs.rolling_diff_discount = node["rolling_diff_discount"].as<double>();
        rhs.min_partition_size = node["min_partition_size"].as<int>();
        return true;
    }
};

// ReSharper restore CppInconsistentNaming
