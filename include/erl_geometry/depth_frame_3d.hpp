#pragma once

#include "camera_intrinsic.hpp"
#include "range_sensor_frame_3d.hpp"

#include "erl_common/yaml.hpp"

namespace erl::geometry {

    class DepthFramePartition3D;

    class DepthFrame3D : public RangeSensorFrame3D {
    public:
        struct Setting : public common::Yamlable<Setting, RangeSensorFrame3D::Setting> {
            std::shared_ptr<CameraIntrinsic> camera_intrinsic = std::make_shared<CameraIntrinsic>();
        };

        inline static const volatile bool kSettingRegistered = common::YamlableBase::Register<Setting>();

    protected:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::vector<DepthFramePartition3D> m_partitions_ = {};
        bool m_partitioned_ = false;

    public:
        explicit DepthFrame3D(std::shared_ptr<Setting> setting);

        [[nodiscard]] std::shared_ptr<const Setting>
        GetSetting() const {
            return m_setting_;
        }

        void
        Reset() {
            m_max_valid_range_ = std::numeric_limits<double>::min();
            m_partitioned_ = false;
        }

        [[nodiscard]] long
        GetImageHeight() const {
            return m_frame_coords_.rows();
        }

        [[nodiscard]] long
        GetImageWidth() const {
            return m_frame_coords_.cols();
        }

        [[nodiscard]] bool
        IsPartitioned() const {
            return m_partitioned_;
        }

        [[nodiscard]] bool
        PointIsInFrame(const Eigen::Vector3d &xyz_frame) const override {
            if (xyz_frame[2] < 0) { return false; }  // behind the camera
            return CoordsIsInFrame(ComputeFrameCoords(xyz_frame));
        }

        [[nodiscard]] Eigen::Vector2d
        ComputeFrameCoords(const Eigen::Vector3d &dir_frame) const override {
            return {dir_frame[1] / dir_frame[2], dir_frame[0] / dir_frame[2]};
        }

        [[nodiscard]] static Eigen::MatrixXd
        DepthImageToDepth(const Eigen::MatrixXd &depth_img, const double depth_scale) {
            return depth_img / depth_scale;
        }

        [[nodiscard]] static Eigen::MatrixXd
        DepthToDepthImage(const Eigen::MatrixXd &depth, const double depth_scale) {
            return depth * depth_scale;
        }

        /**
         * @brief Update the frame with new depth measurements.
         * @param rotation orientation of the optical frame in the world frame.
         * @param translation translation of the optical frame in the world frame.
         * @param depth depth measurements (not depth image) in the camera frame.
         * @param partition_rays whether to partition the rays.
         */
        void
        UpdateRanges(
            const Eigen::Ref<const Eigen::Matrix3d> &rotation,
            const Eigen::Ref<const Eigen::Vector3d> &translation,
            Eigen::MatrixXd depth,
            bool partition_rays) override;

        void
        UpdateRanges(
            const Eigen::Ref<const Eigen::Matrix3d> &rotation,
            const Eigen::Ref<const Eigen::Vector3d> &translation,
            const std::string &depth_file,
            const double depth_scale,
            const bool partition_rays = false) {
            cv::Mat depth_img = cv::imread(depth_file, cv::IMREAD_UNCHANGED);
            depth_img.convertTo(depth_img, CV_64FC1);  // convert to double
            Eigen::MatrixXd depth;
            cv::cv2eigen(depth_img, depth);
            UpdateRanges(rotation, translation, DepthImageToDepth(depth, depth_scale), partition_rays);
        }

        [[nodiscard]] const std::vector<DepthFramePartition3D> &
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

        void
        UpdateFrameCoords() {
            m_setting_->camera_intrinsic->ComputeFrameDirections(m_frame_coords_, m_dirs_frame_);
        }
    };

    class DepthFramePartition3D {};

    ERL_REGISTER_RANGE_SENSOR_FRAME_3D(DepthFrame3D);
}  // namespace erl::geometry

// ReSharper disable CppInconsistentNaming
template<>
struct YAML::convert<erl::geometry::DepthFrame3D::Setting> {
    static Node
    encode(const erl::geometry::DepthFrame3D::Setting &rhs) {
        Node node = convert<erl::geometry::RangeSensorFrame3D::Setting>::encode(rhs);
        node["camera_intrinsic"] = rhs.camera_intrinsic;
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::DepthFrame3D::Setting &rhs) {
        if (!convert<erl::geometry::RangeSensorFrame3D::Setting>::decode(node, rhs)) { return false; }
        rhs.camera_intrinsic = node["camera_intrinsic"].as<std::shared_ptr<erl::geometry::CameraIntrinsic>>();
        return true;
    }
};  // namespace YAML

// ReSharper restore CppInconsistentNaming
