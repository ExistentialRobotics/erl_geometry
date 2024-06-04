#pragma once

#include "space_2d.hpp"

#include "erl_common/eigen.hpp"

#include <utility>

namespace erl::geometry {

    class Lidar2D {
    public:
        enum class Mode { kDdf = 0, kSddfV1 = 1, kSddfV2 = 2 };

        static const char *
        GetModeName(Mode mode) {
            const char *mode_names[3] = {"kDdf", "kSddfV1", "kSddfV2"};
            return mode_names[static_cast<int>(mode)];
        }

        static Mode
        GetModeFromName(const std::string &mode_name) {
            if (mode_name == "kDdf") { return Mode::kDdf; }
            if (mode_name == "kSddfV1") { return Mode::kSddfV1; }
            if (mode_name == "kSddfV2") { return Mode::kSddfV2; }

            throw std::runtime_error("Unknown mode: " + mode_name);
        }

        struct Setting : common::Yamlable<Setting> {
            double min_angle = -M_PI;
            double max_angle = M_PI;  // [min_angle, max_angle) by default
            int num_lines = 360;
            Mode mode = Mode::kDdf;
            Space2D::SignMethod sign_method = Space2D::SignMethod::kLineNormal;
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<Space2D> m_space_ = nullptr;

    public:
        Lidar2D() = delete;

        Lidar2D(std::shared_ptr<Setting> setting, std::shared_ptr<Space2D> space)
            : m_setting_(std::move(setting)),
              m_space_(std::move(space)) {
            ERL_ASSERTM(m_space_ != nullptr, "space cannot be nullptr!");
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] Eigen::VectorXd
        GetAngles() const {
            if (m_setting_->max_angle - m_setting_->min_angle == 2.0 * M_PI) {
                const double d = 2.0 * M_PI / m_setting_->num_lines;
                return Eigen::VectorXd::LinSpaced(m_setting_->num_lines, m_setting_->min_angle, m_setting_->max_angle - d);
            }
            return Eigen::VectorXd::LinSpaced(m_setting_->num_lines, m_setting_->min_angle, m_setting_->max_angle);
        }

        [[nodiscard]] Eigen::Matrix2Xd
        GetRayDirectionsInFrame() const {
            Eigen::Matrix2Xd directions(2, m_setting_->num_lines);
            Eigen::VectorXd angles = GetAngles();
            directions << angles.array().cos().transpose(), angles.array().sin().transpose();
            return directions;
        }

        [[nodiscard]] Eigen::VectorXd
        Scan(const double rotation_angle, const Eigen::Ref<const Eigen::Vector2d> &translation, const bool parallel) const {
            const Eigen::Matrix2d rotation = Eigen::Rotation2Dd(rotation_angle).toRotationMatrix();
            return Scan(rotation, translation, parallel);
        }

        [[nodiscard]] Eigen::VectorXd
        Scan(const Eigen::Ref<const Eigen::Matrix2d> &rotation, const Eigen::Ref<const Eigen::Vector2d> &translation, const bool parallel) const {
            const Eigen::Matrix2Xd directions = rotation * GetRayDirectionsInFrame();
            const Eigen::Matrix2Xd positions = translation.replicate(1, m_setting_->num_lines);
            switch (m_setting_->mode) {
                case Mode::kDdf:
                    return m_space_->ComputeDdf(positions, directions, parallel);
                case Mode::kSddfV1:
                    return m_space_->ComputeSddfV1(positions, directions, parallel);
                case Mode::kSddfV2:
                    return m_space_->ComputeSddfV2(positions, directions, m_setting_->sign_method, parallel);
            }

            throw std::runtime_error("Unknown mode: " + std::to_string(static_cast<int>(m_setting_->mode)));
        }

        [[nodiscard]] std::vector<Eigen::VectorXd>
        ScanMultiPoses(const std::vector<Eigen::Matrix3d> &poses, const bool parallel = false) const {
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
    };
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::Lidar2D::Mode> {
    static Node
    encode(const erl::geometry::Lidar2D::Mode &rhs) {
        Node node;
        node = erl::geometry::Lidar2D::GetModeName(rhs);
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::Lidar2D::Mode &rhs) {
        if (!node.IsScalar()) { return false; }
        rhs = erl::geometry::Lidar2D::GetModeFromName(node.as<std::string>());
        return true;
    }
};

template<>
struct YAML::convert<erl::geometry::Lidar2D::Setting> {
    static Node
    encode(const erl::geometry::Lidar2D::Setting &rhs) {
        Node node;
        node["min_angle"] = rhs.min_angle;
        node["max_angle"] = rhs.max_angle;
        node["num_lines"] = rhs.num_lines;
        node["mode"] = rhs.mode;
        node["sign_method"] = rhs.sign_method;
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::Lidar2D::Setting &rhs) {
        if (!node.IsMap()) { return false; }
        rhs.min_angle = node["min_angle"].as<double>();
        rhs.max_angle = node["max_angle"].as<double>();
        rhs.num_lines = node["num_lines"].as<int>();
        rhs.mode = node["mode"].as<erl::geometry::Lidar2D::Mode>();
        rhs.sign_method = node["sign_method"].as<erl::geometry::Space2D::SignMethod>();
        return true;
    }
};
