#pragma once

#include "range_sensor_frame_3d.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/logging.hpp"
#include "erl_common/yaml.hpp"

namespace erl::geometry {

    class LidarFramePartition3D;

    class LidarFrame3D : public RangeSensorFrame3D {
        friend class LidarFramePartition3D;

    public:
        struct Setting : public common::Yamlable<Setting, RangeSensorFrame3D::Setting> {
            double azimuth_min = -M_PI;
            double azimuth_max = M_PI;
            double elevation_min = -M_PI / 2;
            double elevation_max = M_PI / 2;
            long num_azimuth_lines = 360;
            long num_elevation_lines = 180;
        };

    protected:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::vector<LidarFramePartition3D> m_partitions_ = {};
        bool m_partitioned_ = false;

    public:
        explicit LidarFrame3D(std::shared_ptr<Setting> setting);

        static const std::string &
        GetFrameType() {
            static const std::string kFrameType = "lidar";
            return kFrameType;
        }

        void
        Reset() {
            m_max_valid_range_ = std::numeric_limits<double>::min();
            m_partitioned_ = false;
        }

        [[nodiscard]] bool
        PointIsInFrame(const Eigen::Vector3d &xyz_frame) const override {
            if (const double range = xyz_frame.norm(); range < m_setting_->valid_range_min || range > m_setting_->valid_range_max) { return false; }
            return CoordsIsInFrame(ComputeFrameCoords(xyz_frame.normalized()));
        }

        [[nodiscard]] Eigen::Vector2d
        ComputeFrameCoords(const Eigen::Vector3d &dir_frame) const override {
            Eigen::Vector2d frame_coords;
            common::DirectionToAzimuthElevation(dir_frame, frame_coords[0], frame_coords[1]);
            return frame_coords;
        }

        [[nodiscard]] Eigen::Vector3d
        WorldToFrameSo3(const Eigen::Vector3d &dir_world) const override {
            return m_rotation_.transpose() * dir_world;
        }

        [[nodiscard]] Eigen::Vector3d
        FrameToWorldSo3(const Eigen::Vector3d &dir_frame) const override {
            return m_rotation_ * dir_frame;
        }

        [[nodiscard]] Eigen::Vector3d
        WorldToFrameSe3(const Eigen::Vector3d &xyz_world) const override {
            return m_rotation_.transpose() * (xyz_world - m_translation_);
        }

        [[nodiscard]] Eigen::Vector3d
        FrameToWorldSe3(const Eigen::Vector3d &xyz_frame) const override {
            return m_rotation_ * xyz_frame + m_translation_;
        }

        void
        UpdateRanges(
            const Eigen::Ref<const Eigen::Matrix3d> &rotation,
            const Eigen::Ref<const Eigen::Vector3d> &translation,
            Eigen::MatrixXd ranges,
            bool partition_rays) override;

        [[nodiscard]] std::shared_ptr<const Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] long
        GetNumAzimuthLines() const {
            return m_frame_coords_.rows();
        }

        [[nodiscard]] long
        GetNumElevationLines() const {
            return m_frame_coords_.cols();
        }

        [[nodiscard]] bool
        IsPartitioned() const {
            return m_partitioned_;
        }

        [[nodiscard]] const std::vector<LidarFramePartition3D> &
        GetPartitions() const {
            ERL_ASSERTM(m_partitioned_, "LidarFrame3D::GetPartitions() is called before partitioning.");
            return m_partitions_;
        }

        [[nodiscard]] bool
        operator==(const RangeSensorFrame3D &other) const override;

        [[nodiscard]] bool
        Write(const std::string &filename) const override;

        [[nodiscard]] bool
        Write(std::ostream &s) const override;

        [[nodiscard]] bool
        Read(const std::string &filename) override;

        [[nodiscard]] bool
        Read(std::istream &s) override;

    protected:
        void
        PartitionRays();
    };

    class LidarFramePartition3D {
        LidarFrame3D *m_lidar_frame_ = nullptr;
        Eigen::Matrix2Xd m_ray_indices_ = {};
    };

    ERL_REGISTER_RANGE_SENSOR_FRAME_3D(LidarFrame3D);
}  // namespace erl::geometry

// ReSharper disable CppInconsistentNaming
template<>
struct YAML::convert<erl::geometry::LidarFrame3D::Setting> {
    static Node
    encode(const erl::geometry::LidarFrame3D::Setting &rhs) {
        Node node = convert<erl::geometry::RangeSensorFrame3D::Setting>::encode(rhs);
        node["azimuth_min"] = rhs.azimuth_min;
        node["azimuth_max"] = rhs.azimuth_max;
        node["elevation_min"] = rhs.elevation_min;
        node["elevation_max"] = rhs.elevation_max;
        node["num_azimuth_lines"] = rhs.num_azimuth_lines;
        node["num_elevation_lines"] = rhs.num_elevation_lines;
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::LidarFrame3D::Setting &rhs) {
        if (!convert<erl::geometry::RangeSensorFrame3D::Setting>::decode(node, rhs)) { return false; }
        rhs.azimuth_min = node["azimuth_min"].as<double>();
        rhs.azimuth_max = node["azimuth_max"].as<double>();
        rhs.elevation_min = node["elevation_min"].as<double>();
        rhs.elevation_max = node["elevation_max"].as<double>();
        rhs.num_azimuth_lines = node["num_azimuth_lines"].as<long>();
        rhs.num_elevation_lines = node["num_elevation_lines"].as<long>();
        return true;
    }
};  // namespace YAML

// ReSharper restore CppInconsistentNaming
