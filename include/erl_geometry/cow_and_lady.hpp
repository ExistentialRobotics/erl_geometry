#pragma once

#include "depth_frame_3d.hpp"

#include "erl_common/opencv.hpp"

#include <open3d/geometry/Geometry3D.h>
#include <open3d/geometry/PointCloud.h>

#include <filesystem>
#include <memory>

namespace erl::geometry {

    class CowAndLady {

        std::filesystem::path m_directory_;
        std::shared_ptr<open3d::geometry::PointCloud> m_pcd_gt_;
        Eigen::MatrixXd m_pose_data_;
        bool m_use_icp_poses_ = false;
        long m_start_idx = 0;
        long m_end_idx = 0;

    public:
        static constexpr long kImageWidth = 640;
        static constexpr long kImageHeight = 480;
        static constexpr double kCameraFx = 525.0;
        static constexpr double kCameraFy = 525.0;
        static constexpr double kCameraCx = 319.5;
        static constexpr double kCameraCy = 239.5;
        static constexpr long kStartIdx = 90;
        static constexpr long kEndIdx = 2684;  // 2829 - 145

        inline static const Eigen::Matrix4d sk_Transform_ = []() -> Eigen::Matrix4d {
            Eigen::Matrix4d transform;
            // clang-format off
            transform << 0.971048,  -0.120915,  0.206023, 0.00114049,
                          0.15701,   0.973037, -0.168959,  0.0450936,
                         -0.180038,  0.196415,   0.96385,  0.0430765,
                                 0,         0,         0,          1;
            // clang-format on
            return transform;
        }();

        struct Frame {
            bool valid = true;
            long sequence_number = -1;
            long time_stamp = 0;
            long header_time_stamp = 0;
            Eigen::Matrix3d rotation;     // rotation matrix
            Eigen::Vector3d translation;  // translation vector
            Eigen::MatrixXd depth;        // depth image
            cv::Mat color{};              // camera image
            cv::Mat depth_jet{};          // depth image in jet color for visualization
        };

        /**
         *
         * @param directory Directory containing the Cow and Lady dataset.
         * @param use_icp_poses If true, use ICP to refine the registration results. Note that this
         * may not improve the results.
         * @note To generate the two folders, pcd and color, run the script:
         * erl_geometry/scripts/rosbag_extract_cow_and_lady.py
         */
        explicit CowAndLady(std::filesystem::path directory, bool use_icp_poses = false);

        [[nodiscard]] std::shared_ptr<open3d::geometry::PointCloud>
        GetGroundTruthPointCloud() const {
            return m_pcd_gt_;
        }

        [[nodiscard]] long
        Size() const {
            return kEndIdx - kStartIdx;
        }

        [[nodiscard]] long
        GetStartIndex() const {
            return m_start_idx;
        }

        [[nodiscard]] long
        GetEndIndex() const {
            return m_end_idx;
        }

        [[nodiscard]] bool
        SetStartIndex(long start_index) {
            if (start_index < 0 || start_index >= kEndIdx) {
                ERL_WARN("Invalid start index: {}", start_index);
                return false;
            }
            m_start_idx = start_index;
            return true;
        }

        [[nodiscard]] bool
        SetEndIndex(long end_index) {
            if (end_index <= m_start_idx || end_index > kEndIdx) {
                ERL_WARN("Invalid end index: {}", end_index);
                return false;
            }
            m_end_idx = end_index;
            return true;
        }

        [[nodiscard]] Eigen::Vector3d
        GetMapMin() const {
            return m_pcd_gt_->GetMinBound();
        }

        [[nodiscard]] Eigen::Vector3d
        GetMapMax() const {
            return m_pcd_gt_->GetMaxBound();
        }

        [[nodiscard]] Frame
        operator[](long index) const;

        /**
         * Use ICP to refine the registration results. Results are saved in the same directory as
         * the input data as "poses_icp.dat".
         */
        void
        GenerateIcpResults() const;

    private:
        [[nodiscard]] bool
        LoadData(
            long index,
            long &sequence_number,
            long &time_stamp,
            long &header_time_stamp,
            Eigen::Matrix3d &rotation,
            Eigen::Vector3d &translation,
            Eigen::MatrixX<Eigen::Vector3f> &points,
            cv::Mat &color) const;
    };
}  // namespace erl::geometry
