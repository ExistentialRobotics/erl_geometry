#pragma once

#include "range_sensor_3d.hpp"

#include "erl_common/yaml.hpp"

#include <open3d/t/geometry/RaycastingScene.h>

namespace erl::geometry {

    class DepthCamera3D : public RangeSensor3D {

    public:
        struct Setting : common::Yamlable<Setting> {
            int image_height = 680;
            int image_width = 1200;
            double camera_fx = 600.0;
            double camera_fy = 600.0;
            double camera_cx = 599.5;
            double camera_cy = 339.5;
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;

    public:
        DepthCamera3D() = delete;

        DepthCamera3D(std::shared_ptr<Setting> setting, const Eigen::Ref<const Eigen::Matrix3Xd> &vertices, const Eigen::Ref<const Eigen::Matrix3Xi> &triangles)
            : RangeSensor3D(vertices, triangles),
              m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        DepthCamera3D(std::shared_ptr<Setting> setting, const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3i> &triangles)
            : RangeSensor3D(vertices, triangles),
              m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        DepthCamera3D(std::shared_ptr<Setting> setting, const std::shared_ptr<open3d::t::geometry::RaycastingScene> &o3d_scene)
            : RangeSensor3D(o3d_scene),
              m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        [[nodiscard]] std::shared_ptr<const Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] Eigen::MatrixX<Eigen::Vector3d>
        GetRayDirectionsInFrame() const override;

        [[nodiscard]] static Eigen::Matrix4d
        CameraToOptical() {
            Eigen::Matrix4d camera_to_optical;
            // clang-format off
            camera_to_optical << 0,  0, 1, 0,
                                -1,  0, 0, 0,
                                 0, -1, 0, 0,
                                 0,  0, 0, 1;
            // clang-format on
            return camera_to_optical;
        }
    };

}  // namespace erl::geometry

// ReSharper disable CppInconsistentNaming
template<>
struct YAML::convert<erl::geometry::DepthCamera3D::Setting> {
    static Node
    encode(const erl::geometry::DepthCamera3D::Setting &rhs) {
        Node node;
        node["image_height"] = rhs.image_height;
        node["image_width"] = rhs.image_width;
        node["camera_fx"] = rhs.camera_fx;
        node["camera_fy"] = rhs.camera_fy;
        node["camera_cx"] = rhs.camera_cx;
        node["camera_cy"] = rhs.camera_cy;
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::DepthCamera3D::Setting &rhs) {
        if (!node.IsMap()) { return false; }
        rhs.image_height = node["image_height"].as<int>();
        rhs.image_width = node["image_width"].as<int>();
        rhs.camera_fx = node["camera_fx"].as<double>();
        rhs.camera_fy = node["camera_fy"].as<double>();
        rhs.camera_cx = node["camera_cx"].as<double>();
        rhs.camera_cy = node["camera_cy"].as<double>();
        return true;
    }
};  // namespace YAML

// ReSharper disable CppInconsistentNaming
