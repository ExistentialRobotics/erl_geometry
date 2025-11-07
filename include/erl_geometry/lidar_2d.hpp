#pragma once

#include "space_2d.hpp"

#include "erl_common/enum_parse.hpp"
#include "erl_common/fmt.hpp"
#include "erl_common/yaml.hpp"

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

            ERL_REFLECT_SCHEMA(
                Setting,
                ERL_REFLECT_MEMBER(Setting, min_angle),
                ERL_REFLECT_MEMBER(Setting, max_angle),
                ERL_REFLECT_MEMBER(Setting, num_lines),
                ERL_REFLECT_MEMBER(Setting, mode),
                ERL_REFLECT_MEMBER(Setting, sign_method));
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

ERL_REFLECT_ENUM_SCHEMA(
    erl::geometry::Lidar2D::Mode,
    3,
    ERL_REFLECT_ENUM_MEMBER("ddf", erl::geometry::Lidar2D::Mode::kDdf),
    ERL_REFLECT_ENUM_MEMBER("sddf_v1", erl::geometry::Lidar2D::Mode::kSddfV1),
    ERL_REFLECT_ENUM_MEMBER("sddf_v2", erl::geometry::Lidar2D::Mode::kSddfV2));
ERL_PARSE_ENUM(erl::geometry::Lidar2D::Mode, 3);
