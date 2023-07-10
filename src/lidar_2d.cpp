#include "erl_geometry/lidar_2d.hpp"
#include <memory>

namespace erl::geometry {


    Lidar2D::Lidar2D(std::shared_ptr<Space2D> space)
        : m_space_(std::move(space)) {
        ERL_ASSERTM(m_space_ != nullptr, "space cannot be nullptr!");
        m_pose_.setIdentity();
        UpdateDirections();
    }

    void
    Lidar2D::SetMinAngle(double angle) {
        m_min_angle_ = angle;
        ERL_ASSERTM(m_min_angle_ < m_max_angle_, "minimum angle should be less than maximum angle!");
        ERL_ASSERTM(m_min_angle_ >= -M_PI && m_min_angle_ <= M_PI, "angle should be within [-pi, pi].");
        UpdateDirections();
    }

    void
    Lidar2D::SetMaxAngle(double angle) {
        m_max_angle_ = angle;
        ERL_ASSERTM(m_min_angle_ < m_max_angle_, "minimum angle should be less than maximum angle!");
        ERL_ASSERTM(m_max_angle_ >= -M_PI && m_max_angle_ <= M_PI, "angle should be within [-pi, pi].");
        UpdateDirections();
    }

    void
    Lidar2D::SetNumLines(int num_lines) {
        ERL_ASSERTM(num_lines > 0, "num_lines must be positive integer!");
        m_num_lines_ = num_lines;
        UpdateDirections();
    }

    Eigen::VectorXd
    Lidar2D::Scan(bool parallel) const {
        auto directions = GetOrientedRayDirections();
        switch (m_mode_) {
            case Mode::kDdf:
                return m_space_->ComputeDdf(GetTranslation().replicate(1, m_num_lines_), directions, parallel);
            case Mode::kSddfV1:
                return m_space_->ComputeSddfV1(GetTranslation().replicate(1, m_num_lines_), directions, parallel);
            case Mode::kSddfV2:
                return m_space_->ComputeSddfV2(GetTranslation().replicate(1, m_num_lines_), directions, m_sign_method_, parallel);
        }

        throw std::runtime_error("Unknown mode: " + std::to_string(int(m_mode_)));
    }

    std::vector<Eigen::VectorXd>
    Lidar2D::ScanMultiPoses(const std::vector<Eigen::Matrix3d> &poses, bool parallel) const {
        auto n = int(poses.size());
        std::vector<Eigen::VectorXd> out(n);

#pragma omp parallel for if (parallel) default(none) shared(poses, n, out) schedule(static)
        for (int i = 0; i < n; ++i) {
            auto lidar = *this;
            lidar.SetPose(poses[i]);
            out[i] = lidar.Scan(false);
        }

        return out;
    }

    [[nodiscard]] std::vector<Eigen::VectorXd>
    Lidar2D::ScanMultiPoses(
        const Eigen::Ref<const Eigen::VectorXd> &xs,
        const Eigen::Ref<const Eigen::VectorXd> &ys,
        const Eigen::Ref<const Eigen::VectorXd> &thetas,
        bool parallel) const {
        auto n = xs.size();
        std::vector<Eigen::VectorXd> out(n);

#pragma omp parallel for if (parallel) default(none) shared(xs, ys, thetas, n, out) schedule(static)
        for (int i = 0; i < n; ++i) {
            auto lidar = *this;
            lidar.SetTranslation(Eigen::Vector2d{xs[i], ys[i]});
            lidar.SetRotation(thetas[i]);
            out[i] = lidar.Scan(false);
        }

        return out;
    }

    [[nodiscard]] std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>
    Lidar2D::GetRays(bool parallel) const {
        auto directions = GetOrientedRayDirections();
        Eigen::VectorXd ranges;
        switch (m_mode_) {
            case Mode::kDdf:
                ranges = m_space_->ComputeDdf(GetTranslation().replicate(1, m_num_lines_), directions, parallel);
                break;
            case Mode::kSddfV1:
                ranges = m_space_->ComputeSddfV1(GetTranslation().replicate(1, m_num_lines_), directions, parallel);
                break;
            case Mode::kSddfV2:
                ranges = m_space_->ComputeSddfV2(GetTranslation().replicate(1, m_num_lines_), directions, m_sign_method_, parallel);
                break;
        }

        auto start = GetTranslation();
        auto n_rays = ranges.size();
        std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> rays(n_rays);
        for (int i = 0; i < directions.cols(); ++i) {
            rays[i].first = start;
            rays[i].second = start + ranges[i] * directions.col(i);
        }
        return rays;
    }

    [[nodiscard]] std::vector<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>>
    Lidar2D::GetRaysOfMultiPoses(const std::vector<Eigen::Matrix3d> &poses, bool parallel) const {

        auto n = int(poses.size());
        std::vector<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>> rays(n);

#pragma omp parallel for if (parallel) default(none) shared(poses, n, rays) schedule(static)
        for (int i = 0; i < n; ++i) {
            auto lidar = *this;  // copy this
            lidar.SetPose(poses[i]);
            rays[i] = lidar.GetRays(false);
        }

        return rays;
    }

    [[nodiscard]] std::vector<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>>
    Lidar2D::GetRaysOfMultiPoses(
        const Eigen::Ref<const Eigen::VectorXd> &xs,
        const Eigen::Ref<const Eigen::VectorXd> &ys,
        const Eigen::Ref<const Eigen::VectorXd> &thetas,
        bool parallel) const {

        auto n = xs.size();
        std::vector<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>> rays(n);

#pragma omp parallel for if (parallel) default(none) shared(xs, ys, thetas, n, rays) schedule(static)
        for (int i = 0; i < n; ++i) {
            auto lidar = *this;
            lidar.SetTranslation(Eigen::Vector2d{xs[i], ys[i]});
            lidar.SetRotation(thetas[i]);
            rays[i] = lidar.GetRays(false);
        }

        return rays;
    }

    void
    Lidar2D::UpdateDirections() {
        auto d = (m_max_angle_ - m_min_angle_) / m_num_lines_;
        m_angles_ = Eigen::VectorXd::LinSpaced(m_num_lines_, m_min_angle_, m_max_angle_ - d);
        m_directions_.resize(2, m_num_lines_);
        m_directions_ << m_angles_.array().cos().transpose(), m_angles_.array().sin().transpose();
    }
}  // namespace gpis
