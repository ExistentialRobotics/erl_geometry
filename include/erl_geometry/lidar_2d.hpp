#pragma once

#include <utility>

#include "erl_common/eigen.hpp"
#include "space_2d.hpp"

namespace erl::geometry {

    class Lidar2D {
    public:
        enum class Mode { kDdf = 0, kSddfV1 = 1, kSddfV2 = 2 };

        static inline const char *
        GetModeName(Mode mode) {
            const char *mode_names[3] = {"kDdf", "kSddfV1", "kSddfV2"};
            return mode_names[int(mode)];
        }

        static inline Mode
        GetModeFromName(const std::string &mode_name) {
            if (mode_name == "kDdf") { return Mode::kDdf; }
            if (mode_name == "kSddfV1") { return Mode::kSddfV1; }
            if (mode_name == "kSddfV2") { return Mode::kSddfV2; }

            throw std::runtime_error("Unknown mode: " + mode_name);
        }

    private:
        std::shared_ptr<Space2D> m_space_;
        Eigen::Isometry2d m_pose_;
        double m_min_angle_ = -M_PI;
        double m_max_angle_ = M_PI;
        int m_num_lines_ = 360;
        Eigen::VectorXd m_angles_;
        Eigen::Matrix2Xd m_directions_;
        Mode m_mode_ = Mode::kDdf;
        Space2D::SignMethod m_sign_method_ = Space2D::SignMethod::kLineNormal;

    public:
        Lidar2D() = delete;

        explicit Lidar2D(std::shared_ptr<Space2D> space);

        [[nodiscard]] inline Eigen::Matrix3d
        GetPose() const {
            return m_pose_.matrix();
        }

        inline void
        SetPose(const Eigen::Matrix3d &pose) {
            m_pose_ = pose;
        }

        [[nodiscard]] inline Eigen::Vector2d
        GetTranslation() const {
            return m_pose_.translation();
        }

        inline void
        SetTranslation(const Eigen::Ref<const Eigen::Vector2d> &translation) {
            m_pose_.translation() = translation;
        }

        [[nodiscard]] inline Eigen::Matrix2d
        GetRotation() const {
            return m_pose_.linear();  // faster than .rotation
        }

        inline void
        SetRotation(const Eigen::Ref<const Eigen::Matrix2d> &rotation) {
            m_pose_.matrix().block<2, 2>(0, 0) = rotation;
        }

        inline void
        SetRotation(double angle) {
            m_pose_.matrix().block<2, 2>(0, 0) = Eigen::Rotation2Dd(angle).matrix();
        }

        [[nodiscard]] inline double
        GetMinAngle() const {
            return m_min_angle_;
        }

        void
        SetMinAngle(double angle);

        [[nodiscard]] inline double
        GetMaxAngle() const {
            return m_max_angle_;
        }

        void
        SetMaxAngle(double angle);

        [[nodiscard]] inline int
        GetNumLines() const {
            return m_num_lines_;
        }

        void
        SetNumLines(int num_lines);

        [[nodiscard]] inline Eigen::VectorXd
        GetAngles() const {
            return m_angles_;
        }

        [[nodiscard]] inline Eigen::Matrix2Xd
        GetRayDirections() const {
            return m_directions_;
        }

        [[nodiscard]] inline Eigen::Matrix2Xd
        GetOrientedRayDirections() const {
            auto rotation = GetRotation();
            return rotation * m_directions_;
        }

        [[nodiscard]] inline Mode
        GetMode() const {
            return m_mode_;
        }

        inline void
        SetMode(Mode mode) {
            m_mode_ = mode;
        }

        [[nodiscard]] inline Space2D::SignMethod
        GetSignMethod() const {
            return m_sign_method_;
        }

        inline void
        SetSignMethod(Space2D::SignMethod sign_method) {
            m_sign_method_ = sign_method;
        }

        [[nodiscard]] Eigen::VectorXd
        Scan(bool parallel = false) const;

        [[nodiscard]] std::vector<Eigen::VectorXd>
        ScanMultiPoses(const std::vector<Eigen::Matrix3d> &poses, bool parallel = false) const;

        [[nodiscard]] std::vector<Eigen::VectorXd>
        ScanMultiPoses(
            const Eigen::Ref<const Eigen::VectorXd> &xs,
            const Eigen::Ref<const Eigen::VectorXd> &ys,
            const Eigen::Ref<const Eigen::VectorXd> &thetas,
            bool parallel = false) const;

        [[nodiscard]] std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>
        GetRays(bool parallel = false) const;

        [[nodiscard]] std::vector<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>>
        GetRaysOfMultiPoses(const std::vector<Eigen::Matrix3d> &poses, bool parallel = false) const;

        [[nodiscard]] std::vector<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>>
        GetRaysOfMultiPoses(
            const Eigen::Ref<const Eigen::VectorXd> &xs,
            const Eigen::Ref<const Eigen::VectorXd> &ys,
            const Eigen::Ref<const Eigen::VectorXd> &thetas,
            bool parallel = false) const;

    private:
        void
        UpdateDirections();
    };
}  // namespace erl::geometry
