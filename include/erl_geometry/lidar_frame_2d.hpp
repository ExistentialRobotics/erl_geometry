#pragma once

#include "kdtree_eigen_adaptor.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/eigen.hpp"
#include "erl_common/random.hpp"
#include "erl_common/yaml.hpp"

#include <utility>

namespace erl::geometry {
    class LidarFramePartition2D;

    /**
     * 1. detects discontinuities in a sensor frame.
     * 2. detects out-of-max-range measurements
     */
    class LidarFrame2D {
        friend class LidarFramePartition2D;

    public:
        struct Setting : public common::Yamlable<Setting> {
            double valid_range_min = 0.0;
            double valid_range_max = std::numeric_limits<double>::infinity();
            double valid_angle_min = -M_PI;
            double valid_angle_max = M_PI;
            double discontinuity_factor = 10;
            double rolling_diff_discount = 0.9;
            int min_partition_size = 5;
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        Eigen::Matrix2d m_rotation_ = {};
        double m_rotation_angle_ = 0.0;
        Eigen::Vector2d m_translation_ = {};

        Eigen::VectorXd m_angles_frame_ = {};
        Eigen::VectorXd m_angles_world_ = {};
        Eigen::VectorXd m_ranges_ = {};

        Eigen::Matrix2Xd m_dirs_frame_ = {};
        Eigen::Matrix2Xd m_dirs_world_ = {};

        Eigen::Matrix2Xd m_end_pts_frame_ = {};
        Eigen::Matrix2Xd m_end_pts_world_ = {};

        Eigen::VectorXb m_mask_hit_ = {};           // if i-th element is true, then i-th vertex is a hit
        Eigen::VectorXb m_mask_continuous_ = {};    // if i-th element is true, then (i-1, i) edge is continuous
        Eigen::VectorXl m_hit_ray_indices_ = {};    // hit ray indices
        Eigen::Matrix2Xd m_hit_points_world_ = {};  // hit points in world

        double m_max_valid_range_ = 0.0;
        std::vector<LidarFramePartition2D> m_partitions_ = {};
        bool m_partitioned_ = false;
        std::shared_ptr<KdTree2d> m_kd_tree_ = std::make_shared<KdTree2d>();

    public:
        explicit LidarFrame2D(std::shared_ptr<Setting> setting)
            : m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        void
        Update(
            const Eigen::Ref<const Eigen::Matrix2d> &rotation,
            const Eigen::Ref<const Eigen::Vector2d> &translation,
            Eigen::VectorXd angles,
            Eigen::VectorXd ranges,
            bool partition_rays = false);

        [[nodiscard]] const std::shared_ptr<Setting> &
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] long
        GetNumRays() const {
            return m_angles_frame_.size();
        }

        [[nodiscard]] long
        GetNumHitRays() const {
            return m_hit_ray_indices_.size();
        }

        [[nodiscard]] const Eigen::Matrix2d &
        GetRotationMatrix() const {
            return m_rotation_;
        }

        [[nodiscard]] double
        GetRotationAngle() const {
            return m_rotation_angle_;
        }

        [[nodiscard]] const Eigen::Vector2d &
        GetTranslationVector() const {
            return m_translation_;
        }

        [[nodiscard]] Eigen::Matrix3d
        GetPoseMatrix() const {
            Eigen::Isometry2d pose;
            pose.linear() = m_rotation_;
            pose.translation() = m_translation_;
            return pose.matrix();
        }

        [[nodiscard]] const Eigen::VectorXd &
        GetAnglesInFrame() const {
            return m_angles_frame_;
        }

        [[nodiscard]] const Eigen::VectorXd &
        GetAnglesInWorld() const {
            return m_angles_world_;
        }

        [[nodiscard]] const Eigen::VectorXd &
        GetRanges() const {
            return m_ranges_;
        }

        [[nodiscard]] const Eigen::Matrix2Xd &
        GetRayDirectionsInFrame() const {
            return m_dirs_frame_;
        }

        [[nodiscard]] const Eigen::Matrix2Xd &
        GetRayDirectionsInWorld() const {
            return m_dirs_world_;
        }

        [[nodiscard]] const Eigen::Matrix2Xd &
        GetEndPointsInFrame() const {
            return m_end_pts_frame_;
        }

        [[nodiscard]] const Eigen::Matrix2Xd &
        GetEndPointsInWorld() const {
            return m_end_pts_world_;
        }

        [[nodiscard]] const Eigen::VectorXl &
        GetHitRayIndices() const {
            return m_hit_ray_indices_;
        }

        [[nodiscard]] const Eigen::Matrix2Xd &
        GetHitPointsWorld() const {
            return m_hit_points_world_;
        }

        [[nodiscard]] double
        GetMaxValidRange() const {
            return m_max_valid_range_;
        }

        [[nodiscard]] const std::vector<LidarFramePartition2D> &
        GetPartitions() const {
            ERL_ASSERTM(m_partitioned_, "LidarFrame2D::GetPartitions() is called before partitioning.");
            return m_partitions_;
        }

        [[nodiscard]] bool
        IsPartitioned() const {
            return m_partitioned_;
        }

        [[nodiscard]] bool
        IsValid() const {
            return m_max_valid_range_ > 0;
        }

        void
        ComputeClosestEndPoint(const Eigen::Ref<const Eigen::Vector2d> &position_world, long &end_point_index, double &distance, bool brute_force = false);

        void
        SampleAlongRays(
            long n_samples_per_ray,
            double max_in_obstacle_dist,
            double sampled_rays_ratio,
            Eigen::Matrix2Xd &positions_world,
            Eigen::Matrix2Xd &directions_world,
            Eigen::VectorXd &distances) const;

        void
        SampleAlongRays(
            double range_step,
            double max_in_obstacle_dist,
            double sampled_rays_ratio,
            Eigen::Matrix2Xd &positions_world,
            Eigen::Matrix2Xd &directions_world,
            Eigen::VectorXd &distances) const;

        void
        SampleNearSurface(
            long num_samples_per_ray,
            double max_offset,
            double sampled_rays_ratio,
            Eigen::Matrix2Xd &positions_world,
            Eigen::Matrix2Xd &directions_world,
            Eigen::VectorXd &distances) const;

        void
        SampleInRegion(
            long num_positions,
            long num_along_ray_samples_per_ray,
            long num_near_surface_samples_per_ray,
            double max_in_obstacle_dist,
            Eigen::Matrix2Xd &positions_world,
            Eigen::Matrix2Xd &directions_world,
            Eigen::VectorXd &distances) const;

        void
        ComputeRaysAt(
            const Eigen::Ref<const Eigen::Vector2d> &position_world,
            Eigen::Matrix2Xd &directions_world,
            Eigen::VectorXd &distances,
            std::vector<long> &visible_hit_point_indices) const;

    private:
        void
        PartitionRays();
    };

    class LidarFramePartition2D {
        LidarFrame2D *m_frame_ = nullptr;
        long m_index_begin_ = -1;
        long m_index_end_ = -1;  // inclusive

        friend class LidarFrame2D;

    public:
        LidarFramePartition2D(LidarFrame2D *frame, const long index_begin, const long index_end)
            : m_frame_(frame),
              m_index_begin_(index_begin),
              m_index_end_(index_end) {
            ERL_DEBUG_ASSERT(m_frame_ != nullptr, "frame is nullptr.");
            ERL_DEBUG_ASSERT(m_index_begin_ >= 0, "index_begin is negative.");
            ERL_DEBUG_ASSERT(m_index_end_ >= 0, "index_end is negative.");
        }

        [[nodiscard]] long
        GetIndexBegin() const {
            return m_index_begin_;
        }

        [[nodiscard]] long
        GetIndexEnd() const {
            return m_index_end_ + 1;
        }

        [[nodiscard]] bool
        AngleInPartition(const double angle_world) const {
            const double angle_frame = common::WrapAnglePi(angle_world - m_frame_->m_rotation_angle_);
            return (angle_frame >= m_frame_->m_angles_frame_[m_index_begin_]) && (angle_frame <= m_frame_->m_angles_frame_[m_index_end_]);
        }
    };
}  // namespace erl::geometry

// ReSharper disable CppInconsistentNaming
template<>
struct YAML::convert<erl::geometry::LidarFrame2D::Setting> {
    static Node
    encode(const erl::geometry::LidarFrame2D::Setting &rhs) {
        Node node;
        node["valid_range_min"] = rhs.valid_range_min;
        node["valid_range_max"] = rhs.valid_range_max;
        node["valid_angle_min"] = rhs.valid_angle_min;
        node["valid_angle_max"] = rhs.valid_angle_max;
        node["discontinuity_factor"] = rhs.discontinuity_factor;
        node["rolling_diff_discount"] = rhs.rolling_diff_discount;
        node["min_partition_size"] = rhs.min_partition_size;
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::LidarFrame2D::Setting &rhs) {
        if (!node.IsMap()) { return false; }
        rhs.valid_range_min = node["valid_range_min"].as<double>();
        rhs.valid_range_max = node["valid_range_max"].as<double>();
        rhs.valid_angle_min = node["valid_angle_min"].as<double>();
        rhs.valid_angle_max = node["valid_angle_max"].as<double>();
        rhs.discontinuity_factor = node["discontinuity_factor"].as<double>();
        rhs.rolling_diff_discount = node["rolling_diff_discount"].as<double>();
        rhs.min_partition_size = node["min_partition_size"].as<int>();
        return true;
    }
};  // namespace YAML

// ReSharper restore CppInconsistentNaming
