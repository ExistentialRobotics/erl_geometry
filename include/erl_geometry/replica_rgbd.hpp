#pragma once

#include "erl_common/opencv.hpp"
#include "erl_common/yaml.hpp"

#include <filesystem>

namespace erl::geometry {

    class ReplicaRgbd {
        std::filesystem::path m_directory_;
        std::string m_scene_name_;
        bool m_load_as_bgr_ = false;
        std::filesystem::path m_mesh_path_;
        std::filesystem::path m_traj_path_;
        std::filesystem::path m_rgbd_dir_;
        Eigen::Vector3d m_map_min_{};
        Eigen::Vector3d m_map_max_{};
        Eigen::Matrix<double, 16, Eigen::Dynamic> m_pose_data_;

    public:
        static constexpr long kImageWidth = 1200;
        static constexpr long kImageHeight = 680;
        static constexpr double kCameraFx = 600.0;
        static constexpr double kCameraFy = 600.0;
        static constexpr double kCameraCx = 599.5;
        static constexpr double kCameraCy = 339.5;
        static constexpr double kDepthScale = 6553.5;

        struct Frame {
            long sequence_number = -1;
            Eigen::Matrix3d rotation;     // rotation matrix
            Eigen::Vector3d translation;  // translation vector
            Eigen::MatrixXd depth;        // depth image
            cv::Mat color{};              // camera image
            cv::Mat depth_jet{};          // depth image in jet color for visualization
        };

        /**
         *
         * @param directory directory containing the Replica dataset.
         * @param scene_name name of the scene, e.g., "hotel0".
         * @param load_as_bgr if true, load the color images in BGR format; otherwise in RGB format.
         * @param generate_rgbd if true, generate RGBD images from the mesh and trajectory when.
         * necessary.
         */
        explicit ReplicaRgbd(
            std::filesystem::path directory,
            std::string scene_name,
            bool load_as_bgr = false,
            bool generate_rgbd = false);

        [[nodiscard]] std::filesystem::path
        GetMeshPath() const {
            return m_mesh_path_;
        }

        [[nodiscard]] Eigen::Vector3d
        GetMapMin() const {
            return m_map_min_;
        }

        [[nodiscard]] Eigen::Vector3d
        GetMapMax() const {
            return m_map_max_;
        }

        [[nodiscard]] long
        Size() const {
            return m_pose_data_.cols();
        }

        [[nodiscard]] Frame
        operator[](long index) const;
    };

}  // namespace erl::geometry
