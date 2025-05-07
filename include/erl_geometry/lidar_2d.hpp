#pragma once

#include "space_2d.hpp"

#include <fmt/format.h>

namespace erl::geometry {

    class Lidar2D {
    public:
        enum class Mode { kDdf = 0, kSddfV1 = 1, kSddfV2 = 2 };

        struct Setting : public common::Yamlable<Setting> {
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

        Lidar2D(std::shared_ptr<Setting> setting, std::shared_ptr<Space2D> space);

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] Eigen::VectorXd
        GetAngles() const;

        [[nodiscard]] Eigen::Matrix2Xd
        GetRayDirectionsInFrame() const;

        [[nodiscard]] Eigen::VectorXd
        Scan(
            double rotation_angle,
            const Eigen::Ref<const Eigen::Vector2d> &translation,
            bool parallel) const;

        [[nodiscard]] Eigen::VectorXd
        Scan(
            const Eigen::Ref<const Eigen::Matrix2d> &rotation,
            const Eigen::Ref<const Eigen::Vector2d> &translation,
            bool parallel) const;

        [[nodiscard]] std::vector<Eigen::VectorXd>
        ScanMultiPoses(const std::vector<Eigen::Matrix3d> &poses, bool parallel = false) const;
    };
}  // namespace erl::geometry

template<>
struct fmt::formatter<erl::geometry::Lidar2D::Mode> {
    template<typename ParseContext>
    constexpr auto
    parse(ParseContext &ctx) {
        return ctx.begin();
    }

    template<typename FormatContext>
    auto
    format(erl::geometry::Lidar2D::Mode mode, FormatContext &ctx) const {
        static const char *mode_names[3] = {"kDdf", "kSddfV1", "kSddfV2"};
        return format_to(ctx.out(), mode_names[static_cast<int>(mode)]);
    }
};

template<>
struct YAML::convert<erl::geometry::Lidar2D::Mode> {
    static Node
    encode(const erl::geometry::Lidar2D::Mode &mode);

    static bool
    decode(const Node &node, erl::geometry::Lidar2D::Mode &mode);
};

template<>
struct YAML::convert<erl::geometry::Lidar2D::Setting> {
    static Node
    encode(const erl::geometry::Lidar2D::Setting &setting);

    static bool
    decode(const Node &node, erl::geometry::Lidar2D::Setting &setting);
};

// ReSharper restore CppInconsistentNaming
