#pragma once

#include "range_sensor_frame_3d.hpp"

#include "erl_common/yaml.hpp"

namespace erl::geometry {

    class RgbdFramePartition3D;

    class RgbdFrame3D : public RangeSensorFrame3D {
    public:
        struct Setting : public common::OverrideYamlable<RangeSensorFrame3D::Setting, Setting> {
            Eigen::Matrix4d camera_to_optical = Eigen::Matrix4d::Identity();  // used when camera frame is not aligned with optical frame
            // defaults are from https://github.com/cvg/nice-slam/blob/master/configs/Replica/replica.yaml
            long image_height = 680;
            long image_width = 1200;
            double camera_fx = 600.0;
            double camera_fy = 600.0;
            double camera_cx = 599.5;
            double camera_cy = 339.5;
            double depth_scale = 6553.5;

            /**
             * @brief Resize the frame by a factor. Need to call Update() after this.
             * @param factor the factor to resize the frame. (0, 1) to shrink, (1, +inf) to enlarge.
             * @return the new image size (height, width).
             */
            std::pair<long, long>
            Resize(double factor);
        };

    protected:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        Eigen::Matrix4d m_camera_extrinsic_ = Eigen::Matrix4d::Identity();
        std::vector<RgbdFramePartition3D> m_partitions_ = {};
        bool m_partitioned_ = false;

    public:
        explicit RgbdFrame3D(std::shared_ptr<Setting> setting);

        static const std::string &
        GetFrameType() {
            static const std::string frame_type = "rgbd";
            return frame_type;
        }

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
            return m_ranges_.rows();
        }

        [[nodiscard]] long
        GetImageWidth() const {
            return m_ranges_.cols();
        }

        [[nodiscard]] bool
        IsPartitioned() const {
            return m_partitioned_;
        }

        [[nodiscard]] Eigen::Vector2d
        ComputeFrameCoords(const Eigen::Vector3d &dir_frame) const override {
            return {dir_frame[1] / dir_frame[2], dir_frame[0] / dir_frame[2]};
        }

        [[nodiscard]] Eigen::Vector3d
        WorldToFrameSo3(const Eigen::Vector3d &dir_world) const override {
            return m_camera_extrinsic_.topLeftCorner<3, 3>().transpose() * dir_world;
        }

        [[nodiscard]] Eigen::Vector3d
        FrameToWorldSo3(const Eigen::Vector3d &dir_frame) const override {
            return m_camera_extrinsic_.topLeftCorner<3, 3>() * dir_frame;
        }

        [[nodiscard]] Eigen::Vector3d
        WorldToFrameSe3(const Eigen::Vector3d &xyz_world) const override {
            return m_camera_extrinsic_.topLeftCorner<3, 3>().transpose() * (xyz_world - m_camera_extrinsic_.topRightCorner<3, 1>());
        }

        [[nodiscard]] Eigen::Vector3d
        FrameToWorldSe3(const Eigen::Vector3d &xyz_frame) const override {
            return m_camera_extrinsic_.topLeftCorner<3, 3>() * xyz_frame + m_camera_extrinsic_.topRightCorner<3, 1>();
        }

        Eigen::MatrixXd
        DepthImageToDepth(const Eigen::MatrixXd &depth_img) {
            return depth_img / m_setting_->depth_scale;
        }

        Eigen::MatrixXd
        DepthToDepthImage(const Eigen::MatrixXd &depth) {
            return depth * m_setting_->depth_scale;
        }

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
            const bool partition_rays = false) {
            cv::Mat depth_img = cv::imread(depth_file, cv::IMREAD_UNCHANGED);
            depth_img.convertTo(depth_img, CV_64FC1);  // convert to double
            Eigen::MatrixXd depth;
            cv::cv2eigen(depth_img, depth);
            UpdateRanges(rotation, translation, DepthImageToDepth(depth), partition_rays);
        }

        [[nodiscard]] const Eigen::Matrix4d &
        GetCameraExtrinsicMatrix() const {
            return m_camera_extrinsic_;
        }

        [[nodiscard]] Eigen::Matrix3d
        GetCameraIntrinsicMatrix() const {
            Eigen::Matrix3d intrinsic;
            // clang-format off
            intrinsic << m_setting_->camera_fx, 0, m_setting_->camera_cx,
                         0, m_setting_->camera_fy, m_setting_->camera_cy,
                         0, 0, 1;
            // clang-format on
            return intrinsic;
        }

        [[nodiscard]] const std::vector<RgbdFramePartition3D> &
        GetPartitions() const {
            ERL_ASSERTM(m_partitioned_, "LidarFrame3D::GetPartitions() is called before partitioning.");
            return m_partitions_;
        }

    protected:
        void
        PartitionRays();

        void
        UpdateFrameCoords();
    };

    class RgbdFramePartition3D {};

    ERL_REGISTER_RANGE_SENSOR_FRAME_3D(RgbdFrame3D);
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::RgbdFrame3D::Setting> {
    static Node
    encode(const erl::geometry::RgbdFrame3D::Setting &rhs) {
        Node node = convert<erl::geometry::RangeSensorFrame3D::Setting>::encode(rhs);
        node["camera_to_optical"] = rhs.camera_to_optical;
        node["image_height"] = rhs.image_height;
        node["image_width"] = rhs.image_width;
        node["camera_fx"] = rhs.camera_fx;
        node["camera_fy"] = rhs.camera_fy;
        node["camera_cx"] = rhs.camera_cx;
        node["camera_cy"] = rhs.camera_cy;
        node["depth_scale"] = rhs.depth_scale;
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::RgbdFrame3D::Setting &rhs) {
        if (!convert<erl::geometry::RangeSensorFrame3D::Setting>::decode(node, rhs)) { return false; }
        rhs.camera_to_optical = node["camera_to_optical"].as<Eigen::Matrix4d>();
        rhs.image_height = node["image_height"].as<int>();
        rhs.image_width = node["image_width"].as<int>();
        rhs.camera_fx = node["camera_fx"].as<double>();
        rhs.camera_fy = node["camera_fy"].as<double>();
        rhs.camera_cx = node["camera_cx"].as<double>();
        rhs.camera_cy = node["camera_cy"].as<double>();
        rhs.depth_scale = node["depth_scale"].as<double>();
        return true;
    }
};  // namespace YAML
