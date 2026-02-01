#include "erl_geometry/cow_and_lady.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/opencv.hpp"
#include "erl_common/progress_bar.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/occupancy_octree_drawer.hpp"

#include <open3d/core/EigenConverter.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/pipelines/registration/ColoredICP.h>
#include <open3d/pipelines/registration/Registration.h>

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
        (void) LoadData(index, frame);
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
            Frame frame;

            if (!LoadData(i, frame)) { continue; }

            const auto pcd = std::make_shared<open3d::geometry::PointCloud>();
            pcd->points_.reserve(kImageHeight * kImageWidth);
            pcd->colors_.reserve(kImageHeight * kImageWidth);
            for (long u = 0; u < kImageWidth; ++u) {
                const double *depth_ptr = frame.depth.col(u).data();
                for (long v = 0; v < kImageHeight; ++v) {
                    if (depth_ptr[v] <= 0 || !std::isfinite(depth_ptr[v])) { continue; }

                    pcd->points_.emplace_back(
                        (static_cast<double>(u) - kCameraCx) * depth_ptr[v] / kCameraFx,
                        (static_cast<double>(v) - kCameraCy) * depth_ptr[v] / kCameraFy,
                        depth_ptr[v]);

                    const cv::Vec3b &pixel =
                        frame.color.at<cv::Vec3b>(static_cast<int>(v), static_cast<int>(u));
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
            pose.topLeftCorner<3, 3>() = frame.rotation;
            pose.topRightCorner<3, 1>() = frame.translation;
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

    void
    CowAndLady::ComputePcdPointNormals(const std::string &pcd_path) const {
        const auto frame_setting = std::make_shared<DepthFrame3Dd::Setting>();
        frame_setting->camera_intrinsic.image_height = kImageHeight;
        frame_setting->camera_intrinsic.image_width = kImageWidth;
        frame_setting->camera_intrinsic.camera_fx = kCameraFx;
        frame_setting->camera_intrinsic.camera_fy = kCameraFy;
        frame_setting->camera_intrinsic.camera_cx = kCameraCx;
        frame_setting->camera_intrinsic.camera_cy = kCameraCy;
        frame_setting->valid_range_min = 0.05;
        frame_setting->valid_range_max = 4.0;
        DepthFrame3Dd depth_frame(frame_setting);

        std::cout << "Computing point normals..." << std::endl;
        m_pcd_gt_->EstimateNormals();

        std::cout << "Checking point normals consistency with depth frames..." << std::endl;
        std::vector<long> observation_cnt(m_pcd_gt_->points_.size(), 0);
        std::vector<long> correct_cnt(observation_cnt.size(), 0);
        const auto pbar_setting = std::make_shared<common::ProgressBar::Setting>();
        pbar_setting->description = "Check point normals";
        pbar_setting->total = Size();
        pbar_setting->line_width = 160;
        const auto pbar = common::ProgressBar::Open(pbar_setting);
        KdTree3d kdtree(
            m_pcd_gt_->points_.data()->data(),
            static_cast<long>(m_pcd_gt_->points_.size()));

        for (long i = 0; i < Size(); ++i) {
            const Frame frame = (*this)[i];
            depth_frame.UpdateRanges(frame.rotation, frame.translation, frame.depth);

            // find nearest neighbors for each hit point
            std::vector<long> indices(depth_frame.GetNumHitRays(), -1);
#pragma omp parallel for default(none) shared(depth_frame, kdtree, indices)
            for (long j = 0; j < depth_frame.GetNumHitRays(); ++j) {
                const Eigen::Vector3d &point = depth_frame.GetHitPointsWorld()[j];
                std::vector<long> knn_indices;
                std::vector<double> knn_dists;
                const long k = kdtree.Knn(1, point, knn_indices, knn_dists);
                if (k > 0 && std::sqrt(knn_dists[0]) < 0.01) { indices[j] = knn_indices[0]; }
            }

            // remove duplicate indices
            std::unordered_set<long> unique_indices;
            unique_indices.reserve(indices.size());
            unique_indices.insert(indices.begin(), indices.end());
            indices.clear();
            indices.insert(indices.end(), unique_indices.begin(), unique_indices.end());

            // count observations and correct normals
#pragma omp parallel for default(none) shared(indices, frame, kdtree, observation_cnt, correct_cnt)
            for (const long &j: indices) {
                if (j < 0) { continue; }
                ++observation_cnt[j];
                const Eigen::Vector3d v = m_pcd_gt_->points_[j] - frame.translation;
                if (v.dot(m_pcd_gt_->normals_[j]) < 0) { ++correct_cnt[j]; }
            }
            pbar->Update();
        }
        pbar->Close();

        open3d::geometry::PointCloud pcd;
        pcd.points_.reserve(m_pcd_gt_->points_.size());
        pcd.normals_.reserve(m_pcd_gt_->points_.size());
        pcd.colors_.reserve(m_pcd_gt_->points_.size());
        for (std::size_t i = 0; i < m_pcd_gt_->points_.size(); ++i) {
            if (observation_cnt[i] == 0) { continue; }  // unobserved point
            const double correct_ratio =
                static_cast<double>(correct_cnt[i]) / static_cast<double>(observation_cnt[i]);
            pcd.points_.push_back(m_pcd_gt_->points_[i]);
            pcd.colors_.push_back(m_pcd_gt_->colors_[i]);
            if (correct_ratio < 0.5) {
                // flip normal
                pcd.normals_.emplace_back(-m_pcd_gt_->normals_[i]);
            } else {
                pcd.normals_.push_back(m_pcd_gt_->normals_[i]);
            }
        }

        ERL_INFO(
            "Removed {} points with invalid normals",
            m_pcd_gt_->points_.size() - pcd.points_.size());

        kdtree.SetDataMatrix(pcd.points_.data()->data(), static_cast<long>(pcd.points_.size()));

        std::cout << "Checking point normals consistency with neighbors..." << std::endl;

        constexpr int max_iters = 100;
        int iter = 0;
        while (iter < max_iters) {
            std::cout << "Iteration " << iter << "/" << max_iters << std::endl;
            ++iter;
            volatile bool any_flip = false;  // racing condition is acceptable here
#pragma omp parallel for default(none) shared(pcd, kdtree, any_flip)
            for (std::size_t i = 0; i < pcd.points_.size(); ++i) {
                std::vector<long> indices;
                std::vector<double> dists;
                const long k = kdtree.Knn(15, pcd.points_[i], indices, dists);
                std::size_t votes = 0;
                for (long j = 1; j < k; ++j) {
                    const Eigen::Vector3d &neighbor_normal = pcd.normals_[indices[j]];
                    if (pcd.normals_[i].dot(neighbor_normal) > 0) { votes++; }
                }
                if (votes < static_cast<std::size_t>(k / 2)) {
                    pcd.normals_[i] = -pcd.normals_[i];
                    any_flip = true;
                }
            }
            if (!any_flip) { break; }  // converged
        }

        std::cout << "Saving point cloud with normals to " << pcd_path << std::endl;
        const std::filesystem::path path = pcd_path;
        std::filesystem::create_directories(path.parent_path());
        if (!open3d::io::WritePointCloud(pcd_path, pcd)) {
            ERL_WARN("Failed to save point cloud with normals to {}", pcd_path);
        }
    }

    bool
    CowAndLady::LoadData(long index, Frame &frame) const {

        index = m_start_idx_ + index;

        auto
            &[valid,
              sequence_number,
              time_stamp,
              header_time_stamp,
              rotation,
              translation,
              depth,
              color,
              depth_jet] = frame;

        valid = false;

        const double *data = m_pose_data_.col(index).data();
        sequence_number = static_cast<long>(data[0]);
        time_stamp = static_cast<long>(data[1]);
        header_time_stamp = static_cast<long>(data[2]);

        translation = Eigen::Map<const Eigen::Vector3d>(data + 3, 3, 1);
        const Eigen::Quaterniond quaternion(Eigen::Map<const Eigen::Vector4d>(data + 6, 4, 1));
        rotation = quaternion.toRotationMatrix();

        translation = rotation * sk_Transform_.topRightCorner<3, 1>() + translation;
        rotation = rotation * sk_Transform_.topLeftCorner<3, 3>();

        const std::string depth_filename =
            m_directory_ / "depth" / fmt::format("{}.png", sequence_number);
        if (!std::filesystem::exists(depth_filename)) {
            ERL_WARN("{} does not exist", depth_filename);
            return false;
        }
        cv::Mat depth_img = cv::imread(depth_filename, cv::IMREAD_UNCHANGED);
        depth_img.convertTo(depth_img, CV_64FC1, 0.001);  // mm to m
        cv::cv2eigen(depth_img, depth);

        const std::string color_filename =
            m_directory_ / "color" / fmt::format("{}.png", sequence_number);
        if (!std::filesystem::exists(color_filename)) {
            ERL_WARN("{} does not exist", color_filename);
            return false;
        }
        color = cv::imread(color_filename);

        cv::normalize(depth_img, depth_img, 0, 255, cv::NORM_MINMAX);
        depth_img.convertTo(depth_img, CV_8UC1);
        cv::cvtColor(depth_img, depth_jet, cv::COLOR_GRAY2BGR);
        cv::applyColorMap(depth_jet, depth_jet, cv::COLORMAP_JET);

        valid = true;

        return true;
    }

}  // namespace erl::geometry
