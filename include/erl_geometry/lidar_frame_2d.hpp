#pragma once

#include <utility>

#include "erl_common/eigen.hpp"
#include "erl_common/yaml.hpp"
#include "erl_common/random.hpp"
#include "erl_common/angle_utils.hpp"

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
            double valid_range_min = 0.01;
            double valid_range_max = 30;
            double valid_angle_min = -135. / 180. * M_PI;
            double valid_angle_max = 135. / 180. * M_PI;
            double discontinuity_factor = 10;
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

        Eigen::VectorXb m_mask_hit_ = {};         // if i-th element is true, then i-th vertex is a hit
        Eigen::VectorXb m_mask_continuous_ = {};  // if i-th element is true, then (i-1, i) edge is continuous

        double m_max_valid_range_ = 0.0;
        std::vector<LidarFramePartition2D> m_partitions_ = {};

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
            Eigen::VectorXd ranges);

        [[nodiscard]] inline const std::shared_ptr<Setting> &
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] inline long
        GetNumRays() const {
            return m_angles_frame_.size();
        }

        [[nodiscard]] inline const Eigen::Matrix2d &
        GetRotationMatrix() const {
            return m_rotation_;
        }

        [[nodiscard]] inline double
        GetRotationAngle() const {
            return m_rotation_angle_;
        }

        [[nodiscard]] inline const Eigen::Vector2d &
        GetTranslationVector() const {
            return m_translation_;
        }

        [[nodiscard]] inline Eigen::Matrix3d
        GetPoseMatrix() const {
            Eigen::Isometry2d pose;
            pose.linear() = m_rotation_;
            pose.translation() = m_translation_;
            return pose.matrix();
        }

        [[nodiscard]] inline const Eigen::VectorXd &
        GetAnglesInFrame() const {
            return m_angles_frame_;
        }

        [[nodiscard]] inline const Eigen::VectorXd &
        GetAnglesInWorld() const {
            return m_angles_world_;
        }

        [[nodiscard]] inline const Eigen::VectorXd &
        GetRanges() const {
            return m_ranges_;
        }

        [[nodiscard]] inline const Eigen::Matrix2Xd &
        GetRayDirectionsInFrame() const {
            return m_dirs_frame_;
        }

        [[nodiscard]] inline const Eigen::Matrix2Xd &
        GetRayDirectionsInWorld() const {
            return m_dirs_world_;
        }

        [[nodiscard]] inline const Eigen::Matrix2Xd &
        GetEndPointsInFrame() const {
            return m_end_pts_frame_;
        }

        [[nodiscard]] inline const Eigen::Matrix2Xd &
        GetEndPointsInWorld() const {
            return m_end_pts_world_;
        }

        [[nodiscard]] inline double
        GetMaxValidRange() const {
            return m_max_valid_range_;
        }

        [[nodiscard]] inline const std::vector<LidarFramePartition2D> &
        GetPartitions() const {
            return m_partitions_;
        }

        [[nodiscard]] inline bool
        IsValid() const {
            return !m_partitions_.empty();
        }

        inline void
        ComputeClosestEndPoint(const Eigen::Ref<const Eigen::Vector2d> &position, long &end_point_index, double &distance) const {
            end_point_index = -1;
            distance = std::numeric_limits<double>::infinity();
            long n_vertices = m_end_pts_world_.cols();
            for (long i = 0; i < n_vertices; ++i) {
                double d = (m_end_pts_world_.col(i) - position).squaredNorm();
                if (d < distance) {
                    end_point_index = i;
                    distance = d;
                }
            }
            distance = std::sqrt(distance);
        }

        void
        SampleAlongRays(
            int num_samples_per_ray,
            double max_in_obstacle_dist,
            Eigen::Matrix2Xd &positions,
            Eigen::Matrix2Xd &directions,
            Eigen::VectorXd &distances) const;

        void
        SampleAlongRays(double range_step, double max_in_obstacle_dist, Eigen::Matrix2Xd &positions, Eigen::Matrix2Xd &directions, Eigen::VectorXd &distances)
            const;

        void
        SampleNearSurface(int num_samples_per_ray, double max_offset, Eigen::Matrix2Xd &positions, Eigen::Matrix2Xd &directions, Eigen::VectorXd &distances)
            const;

        void
        SampleInRegion(int num_samples, Eigen::Matrix2Xd &positions, Eigen::Matrix2Xd &directions, Eigen::VectorXd &distances) const;

        void
        ComputeRaysAt(const Eigen::Ref<const Eigen::Vector2d> &position, Eigen::Matrix2Xd &directions, Eigen::VectorXd &distances) const;
    };

    class LidarFramePartition2D {
        LidarFrame2D *m_frame_ = nullptr;
        long m_index_begin_ = -1;
        long m_index_end_ = -1;  // inclusive

        friend class LidarFrame2D;

    public:
        LidarFramePartition2D(LidarFrame2D *frame, long index_begin, long index_end)
            : m_frame_(frame),
              m_index_begin_(index_begin),
              m_index_end_(index_end) {
            ERL_DEBUG_ASSERT(m_frame_ != nullptr, "frame is nullptr.");
            ERL_DEBUG_ASSERT(m_index_begin_ >= 0, "index_begin is negative.");
            ERL_DEBUG_ASSERT(m_index_end_ >= 0, "index_end is negative.");
        }

        [[nodiscard]] inline long
        GetIndexBegin() const {
            return m_index_begin_;
        }

        [[nodiscard]] inline long
        GetIndexEnd() const {
            return m_index_end_ + 1;
        }

        [[nodiscard]] inline bool
        AngleInPartition(double angle_world) const {
            double angle_frame = common::ClipAngle(angle_world - m_frame_->m_rotation_angle_);
            return (angle_frame >= m_frame_->m_angles_frame_[m_index_begin_]) && (angle_frame <= m_frame_->m_angles_frame_[m_index_end_]);
        }
    };
}  // namespace erl::geometry

namespace YAML {
    template<>
    struct convert<erl::geometry::LidarFrame2D::Setting> {
        static Node
        encode(const erl::geometry::LidarFrame2D::Setting &rhs) {
            Node node;
            node["valid_range_min"] = rhs.valid_range_min;
            node["valid_range_max"] = rhs.valid_range_max;
            node["valid_angle_min"] = rhs.valid_angle_min;
            node["valid_angle_max"] = rhs.valid_angle_max;
            node["discontinuity_factor"] = rhs.discontinuity_factor;
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
            rhs.min_partition_size = node["min_partition_size"].as<int>();
            return true;
        }
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::LidarFrame2D::Setting &rhs) {
        out << BeginMap;
        out << Key << "valid_range_min" << Value << rhs.valid_range_min;
        out << Key << "valid_range_max" << Value << rhs.valid_range_max;
        out << Key << "valid_angle_min" << Value << rhs.valid_angle_min;
        out << Key << "valid_angle_max" << Value << rhs.valid_angle_max;
        out << Key << "discontinuity_factor" << Value << rhs.discontinuity_factor;
        out << Key << "min_partition_size" << Value << rhs.min_partition_size;
        out << EndMap;
        return out;
    }
}  // namespace YAML
