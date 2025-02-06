#pragma once

#include "open3d_helper.hpp"

#include "erl_common/yaml.hpp"

#include <open3d/geometry/Geometry.h>
#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/visualization/visualizer/Visualizer.h>
#include <open3d/visualization/visualizer/VisualizerWithKeyCallback.h>

#include <functional>

namespace erl::geometry {
    class Open3dVisualizerWrapper {

    public:
        struct Setting : public common::Yamlable<Setting> {
            std::string window_name = "Open3D Visualizer";
            int window_width = 1920;
            int window_height = 1080;
            int window_left = 50;
            int window_top = 50;
            std::string screenshot_filename = "screenshot.png";
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            double translate_step = 0.1;
            double angle_step = 0.1;
            bool mesh_show_back_face = false;
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<open3d::visualization::VisualizerWithKeyCallback> m_visualizer_ = nullptr;
        std::shared_ptr<open3d::geometry::TriangleMesh> m_axis_mesh_ = nullptr;
        std::function<bool(Open3dVisualizerWrapper *, open3d::visualization::Visualizer *)> m_keyboard_callback_ = nullptr;
        std::function<bool(Open3dVisualizerWrapper *, open3d::visualization::Visualizer *)> m_animation_callback_ = nullptr;

    public:
        explicit Open3dVisualizerWrapper(std::shared_ptr<Setting> setting = nullptr)
            : m_setting_(std::move(setting)) {
            if (!m_setting_) { m_setting_ = std::make_shared<Setting>(); }
            m_axis_mesh_ = CreateAxisMesh(Eigen::Matrix4d::Identity(), 0.1);
            Init();
        }

        void
        Reset() {
            m_visualizer_->DestroyVisualizerWindow();
            m_visualizer_ = nullptr;
            m_keyboard_callback_ = nullptr;
            m_animation_callback_ = nullptr;
            ClearGeometries();
            Init();
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[maybe_unused]] [[nodiscard]] std::shared_ptr<open3d::visualization::VisualizerWithKeyCallback>
        GetVisualizer() const {
            return m_visualizer_;
        }

        void
        SetKeyboardCallback(std::function<bool(Open3dVisualizerWrapper *, open3d::visualization::Visualizer *)> callback) {
            m_keyboard_callback_ = std::move(callback);
        }

        void
        SetAnimationCallback(std::function<bool(Open3dVisualizerWrapper *, open3d::visualization::Visualizer *)> callback) {
            m_animation_callback_ = std::move(callback);
            if (m_animation_callback_) {
                m_visualizer_->RegisterAnimationCallback([this](open3d::visualization::Visualizer *) -> bool {
                    if (m_animation_callback_) { return m_animation_callback_(this, m_visualizer_.get()); }
                    return true;
                });
            } else {
                m_visualizer_->RegisterAnimationCallback(nullptr);
            }
        }

        void
        AddGeometries(const std::vector<std::shared_ptr<open3d::geometry::Geometry>> &geometries) const {
            for (const auto &geometry: geometries) { m_visualizer_->AddGeometry(geometry); }
        }

        void
        ClearGeometries() const {
            if (m_visualizer_ == nullptr) { return; }
            m_visualizer_->ClearGeometries();
            m_visualizer_->AddGeometry(m_axis_mesh_);
        }

        void
        Show(const int wait_time_seconds = -1) {

            if (wait_time_seconds > 0) {
                m_visualizer_->BuildUtilities();
                m_visualizer_->UpdateWindowTitle();
                const auto start_time = std::chrono::system_clock::now();
                if (m_keyboard_callback_) { m_keyboard_callback_(this, m_visualizer_.get()); }
                while (true) {
                    if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start_time).count() >= wait_time_seconds) { break; }
                    m_visualizer_->PollEvents();
                    m_visualizer_->UpdateRender();
                }
            } else {
                if (m_keyboard_callback_) { m_keyboard_callback_(this, m_visualizer_.get()); }
                m_visualizer_->Run();  // blocking
            }
            m_visualizer_->DestroyVisualizerWindow();
        }

    private:
        void
        Init();

        void
        AddKeyboardCallbacks();
    };
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::Open3dVisualizerWrapper::Setting> {
    static Node
    encode(const erl::geometry::Open3dVisualizerWrapper::Setting &rhs) {
        Node node;
        node["window_name"] = rhs.window_name;
        node["window_width"] = rhs.window_width;
        node["window_height"] = rhs.window_height;
        node["window_left"] = rhs.window_left;
        node["window_top"] = rhs.window_top;
        node["x"] = rhs.x;
        node["y"] = rhs.y;
        node["z"] = rhs.z;
        node["roll"] = rhs.roll;
        node["pitch"] = rhs.pitch;
        node["yaw"] = rhs.yaw;
        node["translate_step"] = rhs.translate_step;
        node["angle_step"] = rhs.angle_step;
        node["mesh_show_back_face"] = rhs.mesh_show_back_face;
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::Open3dVisualizerWrapper::Setting &rhs) {
        if (!node.IsMap()) { return false; }
        rhs.window_name = node["window_name"].as<std::string>();
        rhs.window_width = node["window_width"].as<int>();
        rhs.window_height = node["window_height"].as<int>();
        rhs.window_left = node["window_left"].as<int>();
        rhs.window_top = node["window_top"].as<int>();
        rhs.x = node["x"].as<double>();
        rhs.y = node["y"].as<double>();
        rhs.z = node["z"].as<double>();
        rhs.roll = node["roll"].as<double>();
        rhs.pitch = node["pitch"].as<double>();
        rhs.yaw = node["yaw"].as<double>();
        rhs.translate_step = node["translate_step"].as<double>();
        rhs.angle_step = node["angle_step"].as<double>();
        rhs.mesh_show_back_face = node["mesh_show_back_face"].as<bool>();
        return true;
    }
};
