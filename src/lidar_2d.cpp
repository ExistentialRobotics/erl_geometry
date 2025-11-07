#include "erl_geometry/lidar_2d.hpp"

#include <utility>

namespace erl::geometry {
    Lidar2D::Lidar2D(std::shared_ptr<Setting> setting, std::shared_ptr<Space2D> space)
        : m_setting_(std::move(setting)), m_space_(std::move(space)) {
        ERL_ASSERTM(m_space_ != nullptr, "space cannot be nullptr!");
    }

    Eigen::VectorXd
    Lidar2D::GetAngles() const {
        if (m_setting_->max_angle - m_setting_->min_angle == 2.0 * M_PI) {
            const double d = 2.0 * M_PI / m_setting_->num_lines;
            return Eigen::VectorXd::LinSpaced(
                m_setting_->num_lines,
                m_setting_->min_angle,
                m_setting_->max_angle - d);
        }
        return Eigen::VectorXd::LinSpaced(
            m_setting_->num_lines,
            m_setting_->min_angle,
            m_setting_->max_angle);
    }

    Eigen::Matrix2Xd
    Lidar2D::GetRayDirectionsInFrame() const {
        Eigen::Matrix2Xd directions(2, m_setting_->num_lines);
        Eigen::VectorXd angles = GetAngles();
        directions << angles.array().cos().transpose(), angles.array().sin().transpose();
        return directions;
    }

    Eigen::VectorXd
    Lidar2D::Scan(
        const double rotation_angle,
        const Eigen::Ref<const Eigen::Vector2d> &translation,
        const bool parallel) const {
        const Eigen::Matrix2d rotation = Eigen::Rotation2Dd(rotation_angle).toRotationMatrix();
        return Scan(rotation, translation, parallel);
    }

    Eigen::VectorXd
    Lidar2D::Scan(
        const Eigen::Ref<const Eigen::Matrix2d> &rotation,
        const Eigen::Ref<const Eigen::Vector2d> &translation,
        const bool parallel) const {
        const Eigen::Matrix2Xd directions = rotation * GetRayDirectionsInFrame();
        const Eigen::Matrix2Xd positions = translation.replicate(1, m_setting_->num_lines);
        switch (m_setting_->mode) {
            case Mode::kDdf:
                return m_space_->ComputeDdf(positions, directions, parallel);
            case Mode::kSddfV1:
                return m_space_->ComputeSddfV1(positions, directions, parallel);
            case Mode::kSddfV2:
                return m_space_
                    ->ComputeSddfV2(positions, directions, m_setting_->sign_method, parallel);
        }

        throw std::runtime_error(
            "Unknown mode: " + std::to_string(static_cast<int>(m_setting_->mode)));
    }

    std::vector<Eigen::VectorXd>
    Lidar2D::ScanMultiPoses(const std::vector<Eigen::Matrix3d> &poses, const bool parallel) const {
        (void) parallel;
        const auto n = static_cast<int>(poses.size());
        std::vector<Eigen::VectorXd> out(n);

#pragma omp parallel for if (parallel) default(none) shared(poses, n, out, Eigen::Dynamic)
        for (int i = 0; i < n; ++i) {
            Eigen::Matrix2d rotation = poses[i].block<2, 2>(0, 0);
            Eigen::Vector2d translation = poses[i].block<2, 1>(0, 2);
            out[i] = Scan(rotation, translation, false);
        }

        return out;
    }
}  // namespace erl::geometry
