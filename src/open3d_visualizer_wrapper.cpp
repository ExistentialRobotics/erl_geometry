#include "erl_geometry/open3d_visualizer_wrapper.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/logging.hpp"

namespace erl::geometry {

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
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_J, [this](open3d::visualization::Visualizer *vis) -> bool {  // roll-
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
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_L, [this](open3d::visualization::Visualizer *vis) -> bool {  // roll+
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
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_K, [this](open3d::visualization::Visualizer *vis) -> bool {  // pitch-
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
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_I, [this](open3d::visualization::Visualizer *vis) -> bool {  // pitch+
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
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_U, [this](open3d::visualization::Visualizer *vis) -> bool {  // yaw-
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
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_O, [this](open3d::visualization::Visualizer *vis) -> bool {  // yaw+
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
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_LEFT, [this](open3d::visualization::Visualizer *vis) -> bool {  // x-
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
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_RIGHT, [this](open3d::visualization::Visualizer *vis) -> bool {  // x+
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
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_DOWN, [this](open3d::visualization::Visualizer *vis) -> bool {  // y-
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
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_UP, [this](open3d::visualization::Visualizer *vis) -> bool {  // y+
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
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_PAGE_DOWN, [this](open3d::visualization::Visualizer *vis) -> bool {  // z-
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
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_PAGE_UP, [this](open3d::visualization::Visualizer *vis) -> bool {  // z+
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
        m_visualizer_->RegisterKeyActionCallback(GLFW_KEY_S, [this](open3d::visualization::Visualizer *vis, const int action, const int mod) -> bool {
            if (action != GLFW_PRESS) { return false; }
            if (mod != GLFW_MOD_CONTROL) { return false; }
            vis->CaptureScreenImage(m_setting_->screenshot_filename);
            return true;
        });
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_H, [this](open3d::visualization::Visualizer *vis) -> bool {
            vis->PrintVisualizerHelp();
            std::cout << "Extra help messages:" << std::endl
                      << "[H]: print this help message." << std::endl
                      << "[Ctrl+S]: save the view as an image to " + m_setting_->screenshot_filename << "." << std::endl
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
