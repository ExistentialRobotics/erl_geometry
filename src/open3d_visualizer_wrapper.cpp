#include "erl_geometry/open3d_visualizer_wrapper.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/logging.hpp"
#include "erl_geometry/open3d_helper.hpp"

namespace erl::geometry {

    Open3dVisualizerWrapper::Open3dVisualizerWrapper(std::shared_ptr<Setting> setting)
        : m_setting_(std::move(setting)) {
        if (!m_setting_) { m_setting_ = std::make_shared<Setting>(); }
        m_axis_mesh_ = CreateAxisMesh(Eigen::Matrix4d::Identity(), 0.1);
        Init();
    }

    void
    Open3dVisualizerWrapper::Reset() {
        m_visualizer_->DestroyVisualizerWindow();
        m_visualizer_ = nullptr;
        m_keyboard_callback_ = nullptr;
        m_animation_callback_ = nullptr;
        ClearGeometries();
        Init();
    }

    std::shared_ptr<Open3dVisualizerWrapper::Setting>
    Open3dVisualizerWrapper::GetSetting() const {
        return m_setting_;
    }

    std::shared_ptr<open3d::visualization::VisualizerWithKeyCallback>
    Open3dVisualizerWrapper::GetVisualizer() const {
        return m_visualizer_;
    }

    void
    Open3dVisualizerWrapper::SetKeyboardCallback(
        std::function<bool(Open3dVisualizerWrapper *, open3d::visualization::Visualizer *)>
            callback) {
        m_keyboard_callback_ = std::move(callback);
    }

    void
    Open3dVisualizerWrapper::SetAnimationCallback(
        std::function<bool(Open3dVisualizerWrapper *, open3d::visualization::Visualizer *)>
            callback) {
        m_animation_callback_ = std::move(callback);
        if (m_animation_callback_) {
            m_visualizer_->RegisterAnimationCallback(
                [this](open3d::visualization::Visualizer *) -> bool {
                    if (m_animation_callback_) {
                        return m_animation_callback_(this, m_visualizer_.get());
                    }
                    return true;
                });
        } else {
            m_visualizer_->RegisterAnimationCallback(nullptr);
        }
    }

    void
    Open3dVisualizerWrapper::AddGeometries(
        const std::vector<std::shared_ptr<open3d::geometry::Geometry>> &geometries) const {
        for (const auto &geometry: geometries) { m_visualizer_->AddGeometry(geometry); }
    }

    void
    Open3dVisualizerWrapper::ClearGeometries() const {
        if (m_visualizer_ == nullptr) { return; }
        m_visualizer_->ClearGeometries();
        m_visualizer_->AddGeometry(m_axis_mesh_);
    }

    void
    Open3dVisualizerWrapper::Show(const int wait_time_seconds) {

        if (wait_time_seconds > 0) {
            m_visualizer_->BuildUtilities();
            m_visualizer_->UpdateWindowTitle();
            const auto start_time = std::chrono::system_clock::now();
            if (m_keyboard_callback_) { m_keyboard_callback_(this, m_visualizer_.get()); }
            while (true) {
                if (std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::system_clock::now() - start_time)
                        .count() >= wait_time_seconds) {
                    break;
                }
                m_visualizer_->PollEvents();
                m_visualizer_->UpdateRender();
            }
        } else {
            if (m_keyboard_callback_) { m_keyboard_callback_(this, m_visualizer_.get()); }
            m_visualizer_->Run();  // blocking
        }
        m_visualizer_->DestroyVisualizerWindow();
    }

    void
    Open3dVisualizerWrapper::Init() {
        m_visualizer_ = std::make_shared<open3d::visualization::VisualizerWithKeyCallback>();
        ERL_ASSERTM(
            m_visualizer_->CreateVisualizerWindow(
                m_setting_->window_name,
                m_setting_->window_width,
                m_setting_->window_height,
                m_setting_->window_left,
                m_setting_->window_top),
            "Failed creating OpenGL window.");
        m_visualizer_->GetRenderOption().mesh_show_back_face_ = m_setting_->mesh_show_back_face;

        AddKeyboardCallbacks();
        m_visualizer_->AddGeometry(m_axis_mesh_);
    }

    void
    Open3dVisualizerWrapper::AddKeyboardCallbacks() {
        m_visualizer_->RegisterKeyCallback(
            GLFW_KEY_J,
            [this](open3d::visualization::Visualizer *vis) -> bool {  // roll-
                m_setting_->roll = common::WrapAnglePi(m_setting_->roll - m_setting_->angle_step);
                ERL_INFO(
                    "xyz: ({:.3f}, {:.3f}, {:.3f}), rpy: ({:.3f}, {:.3f}, {:.3f})",
                    m_setting_->x,
                    m_setting_->y,
                    m_setting_->z,
                    m_setting_->roll,
                    m_setting_->pitch,
                    m_setting_->yaw);
                if (m_keyboard_callback_) { m_keyboard_callback_(this, vis); }
                return true;
            });
        m_visualizer_->RegisterKeyCallback(
            GLFW_KEY_L,
            [this](open3d::visualization::Visualizer *vis) -> bool {  // roll+
                m_setting_->roll = common::WrapAnglePi(m_setting_->roll + m_setting_->angle_step);
                ERL_INFO(
                    "xyz: ({:.3f}, {:.3f}, {:.3f}), rpy: ({:.3f}, {:.3f}, {:.3f})",
                    m_setting_->x,
                    m_setting_->y,
                    m_setting_->z,
                    m_setting_->roll,
                    m_setting_->pitch,
                    m_setting_->yaw);
                if (m_keyboard_callback_) { m_keyboard_callback_(this, vis); }
                return true;
            });
        m_visualizer_->RegisterKeyCallback(
            GLFW_KEY_K,
            [this](open3d::visualization::Visualizer *vis) -> bool {  // pitch-
                m_setting_->pitch = common::WrapAnglePi(m_setting_->pitch - m_setting_->angle_step);
                ERL_INFO(
                    "xyz: ({:.3f}, {:.3f}, {:.3f}), rpy: ({:.3f}, {:.3f}, {:.3f})",
                    m_setting_->x,
                    m_setting_->y,
                    m_setting_->z,
                    m_setting_->roll,
                    m_setting_->pitch,
                    m_setting_->yaw);
                if (m_keyboard_callback_) { m_keyboard_callback_(this, vis); }
                return true;
            });
        m_visualizer_->RegisterKeyCallback(
            GLFW_KEY_I,
            [this](open3d::visualization::Visualizer *vis) -> bool {  // pitch+
                m_setting_->pitch = common::WrapAnglePi(m_setting_->pitch + m_setting_->angle_step);
                ERL_INFO(
                    "xyz: ({:.3f}, {:.3f}, {:.3f}), rpy: ({:.3f}, {:.3f}, {:.3f})",
                    m_setting_->x,
                    m_setting_->y,
                    m_setting_->z,
                    m_setting_->roll,
                    m_setting_->pitch,
                    m_setting_->yaw);
                if (m_keyboard_callback_) { m_keyboard_callback_(this, vis); }
                return true;
            });
        m_visualizer_->RegisterKeyCallback(
            GLFW_KEY_U,
            [this](open3d::visualization::Visualizer *vis) -> bool {  // yaw-
                m_setting_->yaw = common::WrapAnglePi(m_setting_->yaw - m_setting_->angle_step);
                ERL_INFO(
                    "xyz: ({:.3f}, {:.3f}, {:.3f}), rpy: ({:.3f}, {:.3f}, {:.3f})",
                    m_setting_->x,
                    m_setting_->y,
                    m_setting_->z,
                    m_setting_->roll,
                    m_setting_->pitch,
                    m_setting_->yaw);
                if (m_keyboard_callback_) { m_keyboard_callback_(this, vis); }
                return true;
            });
        m_visualizer_->RegisterKeyCallback(
            GLFW_KEY_O,
            [this](open3d::visualization::Visualizer *vis) -> bool {  // yaw+
                m_setting_->yaw = common::WrapAnglePi(m_setting_->yaw + m_setting_->angle_step);
                ERL_INFO(
                    "xyz: ({:.3f}, {:.3f}, {:.3f}), rpy: ({:.3f}, {:.3f}, {:.3f})",
                    m_setting_->x,
                    m_setting_->y,
                    m_setting_->z,
                    m_setting_->roll,
                    m_setting_->pitch,
                    m_setting_->yaw);
                if (m_keyboard_callback_) { m_keyboard_callback_(this, vis); }
                return true;
            });
        m_visualizer_->RegisterKeyCallback(
            GLFW_KEY_LEFT,
            [this](open3d::visualization::Visualizer *vis) -> bool {  // x-
                m_setting_->x -= m_setting_->translate_step;
                ERL_INFO(
                    "xyz: ({:.3f}, {:.3f}, {:.3f}), rpy: ({:.3f}, {:.3f}, {:.3f})",
                    m_setting_->x,
                    m_setting_->y,
                    m_setting_->z,
                    m_setting_->roll,
                    m_setting_->pitch,
                    m_setting_->yaw);
                if (m_keyboard_callback_) { m_keyboard_callback_(this, vis); }
                return true;
            });
        m_visualizer_->RegisterKeyCallback(
            GLFW_KEY_RIGHT,
            [this](open3d::visualization::Visualizer *vis) -> bool {  // x+
                m_setting_->x += m_setting_->translate_step;
                ERL_INFO(
                    "xyz: ({:.3f}, {:.3f}, {:.3f}), rpy: ({:.3f}, {:.3f}, {:.3f})",
                    m_setting_->x,
                    m_setting_->y,
                    m_setting_->z,
                    m_setting_->roll,
                    m_setting_->pitch,
                    m_setting_->yaw);
                if (m_keyboard_callback_) { m_keyboard_callback_(this, vis); }
                return true;
            });
        m_visualizer_->RegisterKeyCallback(
            GLFW_KEY_DOWN,
            [this](open3d::visualization::Visualizer *vis) -> bool {  // y-
                m_setting_->y -= m_setting_->translate_step;
                ERL_INFO(
                    "xyz: ({:.3f}, {:.3f}, {:.3f}), rpy: ({:.3f}, {:.3f}, {:.3f})",
                    m_setting_->x,
                    m_setting_->y,
                    m_setting_->z,
                    m_setting_->roll,
                    m_setting_->pitch,
                    m_setting_->yaw);
                if (m_keyboard_callback_) { m_keyboard_callback_(this, vis); }
                return true;
            });
        m_visualizer_->RegisterKeyCallback(
            GLFW_KEY_UP,
            [this](open3d::visualization::Visualizer *vis) -> bool {  // y+
                m_setting_->y += m_setting_->translate_step;
                ERL_INFO(
                    "xyz: ({:.3f}, {:.3f}, {:.3f}), rpy: ({:.3f}, {:.3f}, {:.3f})",
                    m_setting_->x,
                    m_setting_->y,
                    m_setting_->z,
                    m_setting_->roll,
                    m_setting_->pitch,
                    m_setting_->yaw);
                if (m_keyboard_callback_) { m_keyboard_callback_(this, vis); }
                return true;
            });
        m_visualizer_->RegisterKeyCallback(
            GLFW_KEY_PAGE_DOWN,
            [this](open3d::visualization::Visualizer *vis) -> bool {  // z-
                m_setting_->z -= m_setting_->translate_step;
                ERL_INFO(
                    "xyz: ({:.3f}, {:.3f}, {:.3f}), rpy: ({:.3f}, {:.3f}, {:.3f})",
                    m_setting_->x,
                    m_setting_->y,
                    m_setting_->z,
                    m_setting_->roll,
                    m_setting_->pitch,
                    m_setting_->yaw);
                if (m_keyboard_callback_) { m_keyboard_callback_(this, vis); }
                return true;
            });
        m_visualizer_->RegisterKeyCallback(
            GLFW_KEY_PAGE_UP,
            [this](open3d::visualization::Visualizer *vis) -> bool {  // z+
                m_setting_->z += m_setting_->translate_step;
                ERL_INFO(
                    "xyz: ({:.3f}, {:.3f}, {:.3f}), rpy: ({:.3f}, {:.3f}, {:.3f})",
                    m_setting_->x,
                    m_setting_->y,
                    m_setting_->z,
                    m_setting_->roll,
                    m_setting_->pitch,
                    m_setting_->yaw);
                if (m_keyboard_callback_) { m_keyboard_callback_(this, vis); }
                return true;
            });
        m_visualizer_->RegisterKeyActionCallback(
            GLFW_KEY_S,
            [this](open3d::visualization::Visualizer *vis, const int action, const int mod)
                -> bool {
                if (action != GLFW_PRESS) { return false; }
                if (mod != GLFW_MOD_CONTROL) { return false; }
                vis->CaptureScreenImage(m_setting_->screenshot_filename);
                return true;
            });
        m_visualizer_->RegisterKeyCallback(
            GLFW_KEY_H,
            [this](open3d::visualization::Visualizer *vis) -> bool {
                vis->PrintVisualizerHelp();
                std::cout << "Extra help messages:" << std::endl
                          << "[H]: print this help message." << std::endl
                          << "[Ctrl+S]: save the view as an image to " +
                                 m_setting_->screenshot_filename
                          << "." << std::endl
                          << "[I]: increase pitch." << std::endl
                          << "[K]: decrease pitch." << std::endl
                          << "[J]: decrease roll." << std::endl
                          << "[L]: increase roll." << std::endl
                          << "[U]: decrease yaw." << std::endl
                          << "[O]: increase yaw." << std::endl
                          << "[Right arrow]: increase x." << std::endl
                          << "[Left arrow]: decrease x." << std::endl
                          << "[Up arrow]: increase y." << std::endl
                          << "[Down arrow]: decrease y." << std::endl
                          << "[Page up]: increase z." << std::endl
                          << "[Page down]: decrease z." << std::endl;
                return true;
            });
    }
}  // namespace erl::geometry

YAML::Node
YAML::convert<erl::geometry::Open3dVisualizerWrapper::Setting>::encode(
    const erl::geometry::Open3dVisualizerWrapper::Setting &setting) {
    Node node;
    ERL_YAML_SAVE_ATTR(node, setting, window_name);
    ERL_YAML_SAVE_ATTR(node, setting, window_width);
    ERL_YAML_SAVE_ATTR(node, setting, window_height);
    ERL_YAML_SAVE_ATTR(node, setting, window_left);
    ERL_YAML_SAVE_ATTR(node, setting, window_top);
    ERL_YAML_SAVE_ATTR(node, setting, x);
    ERL_YAML_SAVE_ATTR(node, setting, y);
    ERL_YAML_SAVE_ATTR(node, setting, z);
    ERL_YAML_SAVE_ATTR(node, setting, roll);
    ERL_YAML_SAVE_ATTR(node, setting, pitch);
    ERL_YAML_SAVE_ATTR(node, setting, yaw);
    ERL_YAML_SAVE_ATTR(node, setting, translate_step);
    ERL_YAML_SAVE_ATTR(node, setting, angle_step);
    ERL_YAML_SAVE_ATTR(node, setting, mesh_show_back_face);
    return node;
}

bool
YAML::convert<erl::geometry::Open3dVisualizerWrapper::Setting>::decode(
    const Node &node,
    erl::geometry::Open3dVisualizerWrapper::Setting &setting) {
    if (!node.IsMap()) { return false; }
    ERL_YAML_LOAD_ATTR(node, setting, window_name);
    ERL_YAML_LOAD_ATTR(node, setting, window_width);
    ERL_YAML_LOAD_ATTR(node, setting, window_height);
    ERL_YAML_LOAD_ATTR(node, setting, window_left);
    ERL_YAML_LOAD_ATTR(node, setting, window_top);
    ERL_YAML_LOAD_ATTR(node, setting, x);
    ERL_YAML_LOAD_ATTR(node, setting, y);
    ERL_YAML_LOAD_ATTR(node, setting, z);
    ERL_YAML_LOAD_ATTR(node, setting, roll);
    ERL_YAML_LOAD_ATTR(node, setting, pitch);
    ERL_YAML_LOAD_ATTR(node, setting, yaw);
    ERL_YAML_LOAD_ATTR(node, setting, translate_step);
    ERL_YAML_LOAD_ATTR(node, setting, angle_step);
    ERL_YAML_LOAD_ATTR(node, setting, mesh_show_back_face);
    return true;
}
