#include "erl_geometry/open3d_visualizer_wrapper.hpp"

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

        AddKeyboardCallbacks();
        m_visualizer_->AddGeometry(m_axis_mesh_);
    }

    void
    Open3dVisualizerWrapper::AddKeyboardCallbacks() {
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_J, [this](open3d::visualization::Visualizer *vis) -> bool {  // roll-
            m_setting_->roll = common::WrapAnglePi(m_setting_->roll - m_setting_->angle_step);
            ERL_INFO(
                "xyz: (%.3f, %.3f, %.3f), rpy: (%.3f, %.3f, %.3f)",
                m_setting_->x,
                m_setting_->y,
                m_setting_->z,
                m_setting_->roll,
                m_setting_->pitch,
                m_setting_->yaw);
            if (m_update_callback_) { m_update_callback_(this, vis); }
            return true;
        });
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_L, [this](open3d::visualization::Visualizer *vis) -> bool {  // roll+
            m_setting_->roll = common::WrapAnglePi(m_setting_->roll + m_setting_->angle_step);
            ERL_INFO(
                "xyz: (%.3f, %.3f, %.3f), rpy: (%.3f, %.3f, %.3f)",
                m_setting_->x,
                m_setting_->y,
                m_setting_->z,
                m_setting_->roll,
                m_setting_->pitch,
                m_setting_->yaw);
            if (m_update_callback_) { m_update_callback_(this, vis); }
            return true;
        });
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_K, [this](open3d::visualization::Visualizer *vis) -> bool {  // pitch-
            m_setting_->pitch = common::WrapAnglePi(m_setting_->pitch - m_setting_->angle_step);
            ERL_INFO(
                "xyz: (%.3f, %.3f, %.3f), rpy: (%.3f, %.3f, %.3f)",
                m_setting_->x,
                m_setting_->y,
                m_setting_->z,
                m_setting_->roll,
                m_setting_->pitch,
                m_setting_->yaw);
            if (m_update_callback_) { m_update_callback_(this, vis); }
            return true;
        });
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_I, [this](open3d::visualization::Visualizer *vis) -> bool {  // pitch+
            m_setting_->pitch = common::WrapAnglePi(m_setting_->pitch + m_setting_->angle_step);
            ERL_INFO(
                "xyz: (%.3f, %.3f, %.3f), rpy: (%.3f, %.3f, %.3f)",
                m_setting_->x,
                m_setting_->y,
                m_setting_->z,
                m_setting_->roll,
                m_setting_->pitch,
                m_setting_->yaw);
            if (m_update_callback_) { m_update_callback_(this, vis); }
            return true;
        });
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_U, [this](open3d::visualization::Visualizer *vis) -> bool {  // yaw-
            m_setting_->yaw = common::WrapAnglePi(m_setting_->yaw - m_setting_->angle_step);
            ERL_INFO(
                "xyz: (%.3f, %.3f, %.3f), rpy: (%.3f, %.3f, %.3f)",
                m_setting_->x,
                m_setting_->y,
                m_setting_->z,
                m_setting_->roll,
                m_setting_->pitch,
                m_setting_->yaw);
            if (m_update_callback_) { m_update_callback_(this, vis); }
            return true;
        });
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_O, [this](open3d::visualization::Visualizer *vis) -> bool {  // yaw+
            m_setting_->yaw = common::WrapAnglePi(m_setting_->yaw + m_setting_->angle_step);
            ERL_INFO(
                "xyz: (%.3f, %.3f, %.3f), rpy: (%.3f, %.3f, %.3f)",
                m_setting_->x,
                m_setting_->y,
                m_setting_->z,
                m_setting_->roll,
                m_setting_->pitch,
                m_setting_->yaw);
            if (m_update_callback_) { m_update_callback_(this, vis); }
            return true;
        });
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_LEFT, [this](open3d::visualization::Visualizer *vis) -> bool {  // x-
            m_setting_->x -= m_setting_->translate_step;
            ERL_INFO(
                "xyz: (%.3f, %.3f, %.3f), rpy: (%.3f, %.3f, %.3f)",
                m_setting_->x,
                m_setting_->y,
                m_setting_->z,
                m_setting_->roll,
                m_setting_->pitch,
                m_setting_->yaw);
            if (m_update_callback_) { m_update_callback_(this, vis); }
            return true;
        });
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_RIGHT, [this](open3d::visualization::Visualizer *vis) -> bool {  // x+
            m_setting_->x += m_setting_->translate_step;
            ERL_INFO(
                "xyz: (%.3f, %.3f, %.3f), rpy: (%.3f, %.3f, %.3f)",
                m_setting_->x,
                m_setting_->y,
                m_setting_->z,
                m_setting_->roll,
                m_setting_->pitch,
                m_setting_->yaw);
            if (m_update_callback_) { m_update_callback_(this, vis); }
            return true;
        });
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_DOWN, [this](open3d::visualization::Visualizer *vis) -> bool {  // y-
            m_setting_->y -= m_setting_->translate_step;
            ERL_INFO(
                "xyz: (%.3f, %.3f, %.3f), rpy: (%.3f, %.3f, %.3f)",
                m_setting_->x,
                m_setting_->y,
                m_setting_->z,
                m_setting_->roll,
                m_setting_->pitch,
                m_setting_->yaw);
            if (m_update_callback_) { m_update_callback_(this, vis); }
            return true;
        });
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_UP, [this](open3d::visualization::Visualizer *vis) -> bool {  // y+
            m_setting_->y += m_setting_->translate_step;
            ERL_INFO(
                "xyz: (%.3f, %.3f, %.3f), rpy: (%.3f, %.3f, %.3f)",
                m_setting_->x,
                m_setting_->y,
                m_setting_->z,
                m_setting_->roll,
                m_setting_->pitch,
                m_setting_->yaw);
            if (m_update_callback_) { m_update_callback_(this, vis); }
            return true;
        });
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_PAGE_DOWN, [this](open3d::visualization::Visualizer *vis) -> bool {  // z-
            m_setting_->z -= m_setting_->translate_step;
            ERL_INFO(
                "xyz: (%.3f, %.3f, %.3f), rpy: (%.3f, %.3f, %.3f)",
                m_setting_->x,
                m_setting_->y,
                m_setting_->z,
                m_setting_->roll,
                m_setting_->pitch,
                m_setting_->yaw);
            if (m_update_callback_) { m_update_callback_(this, vis); }
            return true;
        });
        m_visualizer_->RegisterKeyCallback(GLFW_KEY_PAGE_UP, [this](open3d::visualization::Visualizer *vis) -> bool {  // z+
            m_setting_->z += m_setting_->translate_step;
            ERL_INFO(
                "xyz: (%.3f, %.3f, %.3f), rpy: (%.3f, %.3f, %.3f)",
                m_setting_->x,
                m_setting_->y,
                m_setting_->z,
                m_setting_->roll,
                m_setting_->pitch,
                m_setting_->yaw);
            if (m_update_callback_) { m_update_callback_(this, vis); }
            return true;
        });
        m_visualizer_->RegisterKeyActionCallback(GLFW_KEY_S, [this](open3d::visualization::Visualizer *vis, int action, int mod) -> bool {
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

    std::shared_ptr<open3d::geometry::TriangleMesh>
    Open3dVisualizerWrapper::CreateAxisMesh(const Eigen::Ref<Eigen::Matrix4d> &pose, double axis_length) {
        auto axis_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        double cylinder_radius = 0.0025;
        double cone_radius = 0.0075;
        double cone_height = 0.04;
        auto axis_x = open3d::geometry::TriangleMesh::CreateArrow(cylinder_radius, cone_radius, axis_length, cone_height);
        axis_x->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));  // red
        axis_x->Rotate(open3d::geometry::TriangleMesh::GetRotationMatrixFromXYZ(Eigen::Vector3d(0, M_PI_2, 0)), Eigen::Vector3d(0, 0, 0));
        auto axis_y = open3d::geometry::TriangleMesh::CreateArrow(cylinder_radius, cone_radius, axis_length, cone_height);
        axis_y->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));  // green
        axis_y->Rotate(open3d::geometry::TriangleMesh::GetRotationMatrixFromXYZ(Eigen::Vector3d(-M_PI_2, 0, 0)), Eigen::Vector3d(0, 0, 0));
        auto axis_z = open3d::geometry::TriangleMesh::CreateArrow(cylinder_radius, cone_radius, axis_length, cone_height);
        axis_z->PaintUniformColor(Eigen::Vector3d(0.0, 0.0, 1.0));  // blue
        *axis_mesh += *axis_x;
        *axis_mesh += *axis_y;
        *axis_mesh += *axis_z;
        axis_mesh->Transform(pose);
        return axis_mesh;
    }
}  // namespace erl::geometry
