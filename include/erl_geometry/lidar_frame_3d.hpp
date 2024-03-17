#pragma once

#include "erl_common/yaml.hpp"
#include "erl_common/assert.hpp"
#include "kdtree_eigen_adaptor.hpp"

namespace erl::geometry {

    class LidarFramePartition3D;

    class LidarFrame3D {
        friend class LidarFramePartition3D;

    public:
        struct Setting : public common::Yamlable<Setting> {
            double valid_range_min = 0.0;
            double valid_range_max = std::numeric_limits<double>::max();
            double valid_azimuth_min = -M_PI;
            double valid_azimuth_max = M_PI;
            double valid_elevation_min = -M_PI / 2;
            double valid_elevation_max = M_PI / 2;
            double discontinuity_factor = 10;
            double rolling_diff_discount = 0.9;
            int min_partition_size = 5;
        };

    protected:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        Eigen::Matrix3d m_rotation_ = {};
        Eigen::Vector3d m_translation_ = {};

        Eigen::MatrixXd m_azimuth_frame_ = {};
        Eigen::MatrixXd m_elevation_frame_ = {};
        Eigen::MatrixXd m_ranges_ = {};

        // the memory layout of the following matrices is (azimuth, elevation, 3), and it is okay to access the data using raw pointer directly
        // because the memory is contiguous.

        Eigen::MatrixX<Eigen::Vector3d> m_dirs_frame_ = {};  // directions in frame
        Eigen::MatrixX<Eigen::Vector3d> m_dirs_world_ = {};  // directions in world

        Eigen::MatrixX<Eigen::Vector3d> m_end_pts_frame_ = {};  // end points in frame
        Eigen::MatrixX<Eigen::Vector3d> m_end_pts_world_ = {};  // end points in world

        Eigen::MatrixXb m_mask_hit_ = {};           // if i-th element is true, then i-th vertex is a hit
        Eigen::MatrixXb m_mask_continuous_ = {};    // if i-th element is true, then (i-1, i) edge is continuous
        Eigen::Matrix2Xl m_hit_ray_indices_ = {};   // hit ray indices
        Eigen::Matrix3Xd m_hit_points_world_ = {};  // hit points in world

        double m_max_valid_range_ = std::numeric_limits<double>::infinity();
        std::vector<LidarFramePartition3D> m_partitions_ = {};
        bool m_partitioned_ = false;

        std::shared_ptr<KdTree3d> m_kd_tree_ = std::make_shared<KdTree3d>();

    public:
        explicit LidarFrame3D(std::shared_ptr<Setting> setting)
            : m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        void
        Reset() {
            m_max_valid_range_ = 0.0;
            m_partitioned_ = false;
        }

        void
        Update(
            const Eigen::Ref<const Eigen::Matrix3d> &rotation,
            const Eigen::Ref<const Eigen::Vector3d> &translation,
            const Eigen::Ref<Eigen::VectorXd> &azimuths,
            const Eigen::Ref<Eigen::VectorXd> &elevations,
            Eigen::MatrixXd ranges,
            bool partition_rays = false);

        [[nodiscard]] inline std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] inline long
        GetNumRays() const {
            return m_ranges_.size();
        }

        [[nodiscard]] inline long
        GetNumAzimuthLines() const {
            return m_ranges_.rows();
        }

        [[nodiscard]] inline long
        GetNumElevationLines() const {
            return m_ranges_.cols();
        }

        [[nodiscard]] inline long
        GetNumHitRays() const {
            return m_hit_ray_indices_.cols();
        }

        [[nodiscard]] inline const Eigen::Matrix3d &
        GetRotationMatrix() const {
            return m_rotation_;
        }

        [[nodiscard]] inline const Eigen::Vector3d &
        GetTranslationVector() const {
            return m_translation_;
        }

        [[nodiscard]] inline Eigen::Matrix4d
        GetPoseMatrix() const {
            Eigen::Isometry3d pose;
            pose.linear() = m_rotation_;
            pose.translation() = m_translation_;
            return pose.matrix();
        }

        [[nodiscard]] inline const Eigen::MatrixXd &
        GetAzimuthAnglesInFrame() const {
            return m_azimuth_frame_;
        }

        [[nodiscard]] inline const Eigen::MatrixXd &
        GetElevationAnglesInFrame() const {
            return m_elevation_frame_;
        }

        [[nodiscard]] inline const Eigen::MatrixXd &
        GetRanges() const {
            return m_ranges_;
        }

        [[nodiscard]] inline const Eigen::MatrixX<Eigen::Vector3d> &
        GetRayDirectionsInFrame() const {
            return m_dirs_frame_;
        }

        [[nodiscard]] inline const Eigen::MatrixX<Eigen::Vector3d> &
        GetRayDirectionsInWorld() const {
            return m_dirs_world_;
        }

        [[nodiscard]] inline const Eigen::MatrixX<Eigen::Vector3d> &
        GetEndPointsInFrame() const {
            return m_end_pts_frame_;
        }

        [[nodiscard]] inline const Eigen::MatrixX<Eigen::Vector3d> &
        GetEndPointsInWorld() const {
            return m_end_pts_world_;
        }

        [[nodiscard]] inline const Eigen::Matrix2Xl &
        GetHitRayIndices() const {
            return m_hit_ray_indices_;
        }

        [[nodiscard]] inline const Eigen::Matrix3Xd &
        GetHitPointsWorld() const {
            return m_hit_points_world_;
        }

        [[nodiscard]] inline double
        GetMaxValidRange() const {
            return m_max_valid_range_;
        }

        [[nodiscard]] inline const Eigen::MatrixXb &
        GetHitMask() const {
            return m_mask_hit_;
        }

        [[nodiscard]] inline bool
        IsValid() const {
            return m_max_valid_range_ > 0.0;
        }

        [[nodiscard]] inline bool
        IsPartitioned() const {
            return m_partitioned_;
        }

        [[nodiscard]] inline const std::vector<LidarFramePartition3D> &
        GetPartitions() const {
            ERL_ASSERTM(m_partitioned_, "LidarFrame3D::GetPartitions() is called before partitioning.");
            return m_partitions_;
        }

        void
        ComputeClosestEndPoint(
            const Eigen::Ref<const Eigen::Vector3d> &position_world,
            long &end_point_azimuth_index,
            long &end_point_elevation_index,
            double &distance,
            bool brute_force = false) const;

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

    protected:
        void
        PartitionRays();

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

    class LidarFramePartition3D {
        LidarFrame3D *m_lidar_frame_ = nullptr;
        Eigen::Matrix2Xd m_ray_indices_ = {};
    };

}  // namespace erl::geometry

namespace YAML {
    template<>
    struct convert<erl::geometry::LidarFrame3D::Setting> {
        static Node
        encode(const erl::geometry::LidarFrame3D::Setting &rhs) {
            Node node;
            node["valid_range_min"] = rhs.valid_range_min;
            node["valid_range_max"] = rhs.valid_range_max;
            node["valid_azimuth_min"] = rhs.valid_azimuth_min;
            node["valid_azimuth_max"] = rhs.valid_azimuth_max;
            node["valid_elevation_min"] = rhs.valid_elevation_min;
            node["valid_elevation_max"] = rhs.valid_elevation_max;
            node["discontinuity_factor"] = rhs.discontinuity_factor;
            node["rolling_diff_discount"] = rhs.rolling_diff_discount;
            node["min_partition_size"] = rhs.min_partition_size;
            return node;
        }

        static bool
        decode(const Node &node, erl::geometry::LidarFrame3D::Setting &rhs) {
            if (!node.IsMap()) { return false; }
            rhs.valid_range_min = node["valid_range_min"].as<double>();
            rhs.valid_range_max = node["valid_range_max"].as<double>();
            rhs.valid_azimuth_min = node["valid_azimuth_min"].as<double>();
            rhs.valid_azimuth_max = node["valid_azimuth_max"].as<double>();
            rhs.valid_elevation_min = node["valid_elevation_min"].as<double>();
            rhs.valid_elevation_max = node["valid_elevation_max"].as<double>();
            rhs.discontinuity_factor = node["discontinuity_factor"].as<double>();
            rhs.rolling_diff_discount = node["rolling_diff_discount"].as<double>();
            rhs.min_partition_size = node["min_partition_size"].as<int>();
            return true;
        }
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::LidarFrame3D::Setting &rhs) {
        out << BeginMap;
        out << Key << "valid_range_min" << Value << rhs.valid_range_min;
        out << Key << "valid_range_max" << Value << rhs.valid_range_max;
        out << Key << "valid_azimuth_min" << Value << rhs.valid_azimuth_min;
        out << Key << "valid_azimuth_max" << Value << rhs.valid_azimuth_max;
        out << Key << "valid_elevation_min" << Value << rhs.valid_elevation_min;
        out << Key << "valid_elevation_max" << Value << rhs.valid_elevation_max;
        out << Key << "discontinuity_factor" << Value << rhs.discontinuity_factor;
        out << Key << "rolling_diff_discount" << Value << rhs.rolling_diff_discount;
        out << Key << "min_partition_size" << Value << rhs.min_partition_size;
        out << EndMap;
        return out;
    }
}  // namespace YAML
