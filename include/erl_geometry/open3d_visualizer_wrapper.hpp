#pragma once

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
        std::function<bool(Open3dVisualizerWrapper *, open3d::visualization::Visualizer *)>
            m_keyboard_callback_ = nullptr;
        std::function<bool(Open3dVisualizerWrapper *, open3d::visualization::Visualizer *)>
            m_animation_callback_ = nullptr;

    public:
        explicit Open3dVisualizerWrapper(std::shared_ptr<Setting> setting = nullptr);

        void
        Reset();

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const;

        [[nodiscard]] std::shared_ptr<open3d::visualization::VisualizerWithKeyCallback>
        GetVisualizer() const;

        void
        SetKeyboardCallback(
            std::function<bool(Open3dVisualizerWrapper *, open3d::visualization::Visualizer *)>
                callback);

        void
        SetAnimationCallback(
            std::function<bool(Open3dVisualizerWrapper *, open3d::visualization::Visualizer *)>
                callback);

        void
        AddGeometries(
            const std::vector<std::shared_ptr<open3d::geometry::Geometry>> &geometries) const;

        void
        ClearGeometries() const;

        void
        Show(int wait_time_seconds = -1);

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
    encode(const erl::geometry::Open3dVisualizerWrapper::Setting &setting);

    static bool
    decode(const Node &node, erl::geometry::Open3dVisualizerWrapper::Setting &setting);
};
