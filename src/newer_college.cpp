#include "erl_geometry/newer_college.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/logging.hpp"

#include <open3d/io/PointCloudIO.h>
#include <open3d/io/TriangleMeshIO.h>

namespace erl::geometry {

    Eigen::MatrixXd
    NewerCollege::Frame::GetRangeMatrix() const {
        Eigen::MatrixXd range_matrix = Eigen::MatrixXd::Zero(kNumAzimuthLines, kNumElevationLines);
        constexpr double a_res = 2.0 * M_PI / static_cast<double>(kNumAzimuthLines);
        constexpr double e_res = kVerticalFov / static_cast<double>(kNumElevationLines);
        constexpr double e_min = -kVerticalFov / 2.0;

        (void) points;
#pragma omp parallel for default(none) schedule(static) shared(range_matrix, a_res, e_res)
        for (long i = 0; i < points.cols(); ++i) {
            const Eigen::Vector3d p = points.col(i);
            const double r = p.norm();
            double azimuth = 0.0;
            double elevation = 0.0;
            common::DirectionToAzimuthElevation<double>(p / r, azimuth, elevation);
            const long a_idx = static_cast<long>(std::floor((azimuth + M_PI) / a_res));
            const long e_idx = static_cast<long>(std::floor((elevation - e_min) / e_res));
            if (a_idx < 0 || a_idx >= kNumAzimuthLines || e_idx < 0 ||
                e_idx >= kNumElevationLines) {
                continue;
            }
            range_matrix(a_idx, e_idx) = r;
        }
        return range_matrix;
    }

    Eigen::Matrix3Xd
    NewerCollege::Frame::GetPointsInWorldFrame() const {
        Eigen::Matrix3Xd points_in_world_frame = rotation * points;
        points_in_world_frame.colwise() += translation;
        return points_in_world_frame;
    }

    NewerCollege::NewerCollege(std::filesystem::path directory)
        : m_directory_(std::move(directory)) {
        const std::filesystem::path csv_path = m_directory_ / "poses.csv";
        m_poses_ = common::LoadEigenMatrixFromTextFile<double, 7>(
            csv_path,
            common::EigenTextFormat::kCsvFmt,
            true);
        m_num_frames_ = m_poses_.cols();
        ERL_ASSERTM(m_num_frames_ > 0, "No frames found in {}", csv_path);
    }

    std::shared_ptr<open3d::geometry::PointCloud>
    NewerCollege::GetGroundTruthPointCloud() const {
        auto pcd_file = m_directory_ / "gt-pointcloud.ply";
        std::shared_ptr<open3d::geometry::PointCloud> pcd =
            open3d::io::CreatePointCloudFromFile(pcd_file);
        ERL_ASSERTM(
            pcd != nullptr && !pcd->points_.empty(),
            "Failed to read ground truth point cloud from {}",
            pcd_file);
        return pcd;
    }

    std::shared_ptr<open3d::geometry::TriangleMesh>
    NewerCollege::GetGroundTruthMesh() const {
        auto mesh_file = m_directory_ / "gt-mesh.ply";
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh =
            open3d::io::CreateMeshFromFile(mesh_file);
        ERL_ASSERTM(
            mesh != nullptr && !mesh->vertices_.empty(),
            "Failed to read ground truth mesh from {}",
            mesh_file);
        return mesh;
    }

    NewerCollege::Frame
    NewerCollege::operator[](long index) const {
        ERL_ASSERTM(
            index >= 0 && index < m_num_frames_,
            "Index out of bounds: {}. Valid range is [0, {})",
            index,
            m_num_frames_);
        Frame frame;
        const double *pose = m_poses_.col(index).data();
        frame.rotation = Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]).toRotationMatrix();
        frame.translation[0] = pose[0];
        frame.translation[1] = pose[1];
        frame.translation[2] = pose[2];

        auto pcd = open3d::io::CreatePointCloudFromFile(
            m_directory_ / "ply" / fmt::format("{:04d}.ply", index));
        ERL_ASSERTM(
            pcd != nullptr && !pcd->points_.empty(),
            "Failed to read point cloud for frame {} from {}",
            index,
            m_directory_ / "ply" / fmt::format("{:04d}.ply", index));
        frame.points = Eigen::Map<const Eigen::Matrix3Xd>(
            pcd->points_.data()->data(),
            3,
            static_cast<Eigen::Index>(pcd->points_.size()));
        return frame;
    }

}  // namespace erl::geometry
