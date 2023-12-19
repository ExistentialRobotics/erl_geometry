#pragma once

#include "erl_common/yaml.hpp"
#include "range_sensor_3d.hpp"
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

        [[nodiscard]] inline std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] Eigen::MatrixX<Eigen::Vector3d>
        GetRayDirectionsInFrame() const override ;
    };

}  // namespace erl::geometry

namespace YAML {
    template<>
    struct convert<erl::geometry::DepthCamera3D::Setting> {
        inline static Node
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

        inline static bool
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
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::DepthCamera3D::Setting &rhs) {
        out << BeginMap;
        out << Key << "image_height" << Value << rhs.image_height;
        out << Key << "image_width" << Value << rhs.image_width;
        out << Key << "camera_fx" << Value << rhs.camera_fx;
        out << Key << "camera_fy" << Value << rhs.camera_fy;
        out << Key << "camera_cx" << Value << rhs.camera_cx;
        out << Key << "camera_cy" << Value << rhs.camera_cy;
        out << EndMap;
        return out;
    }
}  // namespace YAML
