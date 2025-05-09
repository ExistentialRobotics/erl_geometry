#include "erl_geometry/cow_and_lady.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/opencv.hpp"
#include "erl_common/progress_bar.hpp"
#include "erl_common/random.hpp"

#include <open3d/core/EigenConverter.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/pipelines/registration/ColoredICP.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/visualization/utility/DrawGeometry.h>

#include <utility>

namespace erl::geometry {
    CowAndLady::CowAndLady(std::filesystem::path directory, const bool use_icp_poses)
        : m_directory_(std::move(directory)),
          m_pcd_gt_(open3d::io::CreatePointCloudFromFile(m_directory_ / "cow_and_lady_gt.ply")),
          m_pose_data_(common::LoadEigenMatrixFromBinaryFile<double>(m_directory_ / "poses.dat")),
          m_use_icp_poses_(use_icp_poses) {
        if (m_use_icp_poses_) {
            const std::string pose_icp_filename = m_directory_ / "poses_icp.dat";
            if (!std::filesystem::exists(pose_icp_filename)) {
                ERL_WARN("ICP poses not found, generating ICP results...");
                if (m_pcd_gt_->points_.empty()) {
                    ERL_WARN(
                        "Failed to load G.T. point cloud from {}, use_icp_poses will be set to "
                        "false",
                        m_directory_ / "cow_and_lady_gt.ply");
                    m_use_icp_poses_ = false;
                }
                GenerateIcpResults();
            }
            m_pose_data_ = common::LoadEigenMatrixFromBinaryFile<double>(pose_icp_filename);
        }
    }

    CowAndLady::Frame
    CowAndLady::operator[](const long index) const {
        Frame frame;
        Eigen::MatrixX<Eigen::Vector3f> points;

        if (frame.valid = LoadData(
                index,
                frame.sequence_number,
                frame.time_stamp,
                frame.header_time_stamp,
                frame.rotation,
                frame.translation,
                points,
                frame.color);
            !frame.valid) {
            return frame;
        }

        frame.depth.resize(kImageHeight, kImageWidth);
        double max_depth = 0;
        for (long j = 0; j < kImageWidth; ++j) {
            double *depth_ptr = frame.depth.col(j).data();
            const Eigen::Vector3f *point_ptr = points.col(j).data();
            for (long i = 0; i < kImageHeight; ++i) {
                if (const Eigen::Vector3f &point = point_ptr[i]; point.hasNaN()) {
                    depth_ptr[i] = -1;
                } else {
                    depth_ptr[i] = point[2];  // z is depth
                    max_depth = std::max(max_depth, depth_ptr[i]);
                }
            }
        }

        const Eigen::MatrixX8U depth_img =
            ((frame.depth.array() + 1) / (max_depth + 1) * 255.0).cast<uint8_t>();
        cv::eigen2cv(depth_img, frame.depth_jet);
        cv::cvtColor(frame.depth_jet, frame.depth_jet, cv::COLOR_GRAY2BGR);
        cv::applyColorMap(frame.depth_jet, frame.depth_jet, cv::COLORMAP_JET);
        return frame;
    }

    void
    CowAndLady::GenerateIcpResults() const {
        ERL_WARN(
            "Cow and lady dataset does not remove camera distortion. ICP does not work well "
            "sometimes.");

        m_pcd_gt_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.05, 30), false);
        m_pcd_gt_->NormalizeNormals();
        Eigen::MatrixXd icp_pose_data = m_pose_data_;
        Eigen::Matrix2Xd error_data(2, icp_pose_data.cols());

        const auto progress_bar_setting = std::make_shared<common::ProgressBar::Setting>();
        progress_bar_setting->description = "CowAndLady::GenerateIcpResults";
        progress_bar_setting->total = icp_pose_data.cols();
        progress_bar_setting->line_width = 160;
        const auto progress_bar = common::ProgressBar::Open(progress_bar_setting);

#pragma omp parallel for default(none) shared(icp_pose_data, error_data, progress_bar)
        for (long i = 0; i < icp_pose_data.cols(); ++i) {
            Eigen::Matrix3d rotation;
            Eigen::Vector3d translation;
            Eigen::MatrixX<Eigen::Vector3f> points;
            cv::Mat color;

            if (long sequence_number, time_stamp, header_time_stamp; !LoadData(
                    i,
                    sequence_number,
                    time_stamp,
                    header_time_stamp,
                    rotation,
                    translation,
                    points,
                    color)) {
                continue;
            }

            const auto pcd = std::make_shared<open3d::geometry::PointCloud>();
            pcd->points_.reserve(kImageHeight * kImageWidth);
            pcd->colors_.reserve(kImageHeight * kImageWidth);
            for (long u = 0; u < kImageWidth; ++u) {
                const Eigen::Vector3f *point_ptr = points.col(u).data();
                for (long v = 0; v < kImageHeight; ++v) {
                    const Eigen::Vector3f &point = point_ptr[v];
                    if (point.hasNaN()) { continue; }
                    pcd->points_.emplace_back(point.cast<double>());
                    const cv::Vec3b &pixel =
                        color.at<cv::Vec3b>(static_cast<int>(v), static_cast<int>(u));
                    pcd->colors_.emplace_back(pixel[2] / 255.0, pixel[1] / 255.0, pixel[0] / 255.0);
                }
            }

            // ICP
            // auto result = open3d::pipelines::registration::RegistrationColoredICP(
            //     *pcd,
            //     *m_pcd_gt_,
            //     0.05,
            //     pose,
            //     open3d::pipelines::registration::TransformationEstimationForColoredICP(),
            //     open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 40));
            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            pose.topLeftCorner<3, 3>() = rotation;
            pose.topRightCorner<3, 1>() = translation;
            auto result = open3d::pipelines::registration::RegistrationICP(
                *pcd,
                *m_pcd_gt_,
                0.01,
                pose,
                open3d::pipelines::registration::TransformationEstimationPointToPlane(),
                open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 1000));

            Eigen::Matrix4d pose_icp = result.transformation_;

            // compute error
            const Eigen::Matrix4d pose_diff = pose_icp * pose.inverse();
            error_data(0, i) = common::RadianToDegree(
                std::acos((pose_diff(0, 0) + pose_diff(1, 1) + pose_diff(2, 2) - 1) / 2));
            error_data(1, i) = pose_diff.topRightCorner<3, 1>().norm();

            // save results
            pose_icp = pose_icp * sk_Transform_.inverse();  // remove the vicon_T_camera transform
            double *icp_pose_ptr = icp_pose_data.col(i).segment<7>(3).data();
            Eigen::Map<Eigen::Vector3d>(icp_pose_ptr, 3, 1) = pose_icp.topRightCorner<3, 1>();
            Eigen::Map<Eigen::Vector4d>(icp_pose_ptr + 3, 4, 1) =
                Eigen::Quaterniond(pose_icp.topLeftCorner<3, 3>()).coeffs();

#pragma omp critical
            { progress_bar->Update(); }
        }

        // save results
        std::string pose_icp_filename = m_directory_ / "poses_icp.dat";
        if (!common::SaveEigenMatrixToBinaryFile(pose_icp_filename, icp_pose_data)) {
            ERL_WARN("Failed to save ICP results to {}", pose_icp_filename);
        }
        pose_icp_filename = m_directory_ / "poses_icp.csv";
        common::SaveEigenMatrixToTextFile<double>(
            pose_icp_filename,
            icp_pose_data.transpose(),
            common::EigenTextFormat::kCsvFmt);
        const std::string error_filename = m_directory_ / "icp_errors.csv";
        common::SaveEigenMatrixToTextFile<double>(
            error_filename,
            error_data.transpose(),
            common::EigenTextFormat::kCsvFmt);
    }

    bool
    CowAndLady::LoadData(
        const long index,
        long &sequence_number,
        long &time_stamp,
        long &header_time_stamp,
        Eigen::Matrix3d &rotation,
        Eigen::Vector3d &translation,
        Eigen::MatrixX<Eigen::Vector3f> &points,
        cv::Mat &color) const {

        const double *data = m_pose_data_.col(index).data();
        sequence_number = static_cast<long>(data[0]);
        time_stamp = static_cast<long>(data[1]);
        header_time_stamp = static_cast<long>(data[2]);

        translation = Eigen::Map<const Eigen::Vector3d>(data + 3, 3, 1);
        const Eigen::Quaterniond quaternion(Eigen::Map<const Eigen::Vector4d>(data + 6, 4, 1));
        rotation = quaternion.toRotationMatrix();

        translation = rotation * sk_Transform_.topRightCorner<3, 1>() + translation;
        rotation = rotation * sk_Transform_.topLeftCorner<3, 3>();

        const std::string pcd_filename =
            m_directory_ / "pcd" / fmt::format("{}.pcd", sequence_number);
        if (!std::filesystem::exists(pcd_filename)) {
            ERL_WARN("{} does not exist", pcd_filename);
            return false;
        }
        std::ifstream ifs(pcd_filename, std::ios::binary);
        if (!common::LoadEigenMatrixOfEigenMatricesFromBinaryStream(ifs, points)) {
            ERL_WARN("Failed to load point cloud data from {}", pcd_filename);
            return false;
        }
        ifs.close();

        const std::string color_filename =
            m_directory_ / "color" / fmt::format("{}.png", sequence_number);
        if (!std::filesystem::exists(color_filename)) {
            ERL_WARN("{} does not exist", color_filename);
            return false;
        }
        color = cv::imread(color_filename);
        return true;
    }

}  // namespace erl::geometry
