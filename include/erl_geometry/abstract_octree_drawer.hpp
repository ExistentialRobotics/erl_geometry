#pragma once

#include <utility>
#include "erl_common/grid_map_info.hpp"
#include "erl_common/yaml.hpp"
#include "abstract_octree.hpp"
#include "open3d_visualizer_wrapper.hpp"

namespace erl::geometry {

    class AbstractOctreeDrawer {
    public:
        struct Setting : public common::Yamlable<Setting> {
            Eigen::Vector3d area_min = {-1.0, -1.0, -1.0};
            Eigen::Vector3d area_max = {1.0, 1.0, 1.0};
            Eigen::Vector3d bg_color = {1.0, 1.0, 1.0};      // white
            Eigen::Vector3d border_color = {0.0, 0.0, 0.0};  // black
        };

    private:
        std::shared_ptr<Setting> m_setting_ = {};

    protected:
        std::shared_ptr<const AbstractOctree> m_octree_ = {};

    public:
        explicit AbstractOctreeDrawer(std::shared_ptr<Setting> setting, std::shared_ptr<const AbstractOctree> octree = nullptr)
            : m_setting_(std::move(setting)),
              m_octree_(std::move(octree)) {
            ERL_ASSERTM(m_setting_, "setting is nullptr.");
        }

        virtual ~AbstractOctreeDrawer() = default;

        void
        SetOctree(std::shared_ptr<const AbstractOctree> octree) {
            m_octree_ = std::move(octree);
        }

        void
        DrawTree(const std::string &filename) const {
            auto visualizer_setting = std::make_shared<Open3dVisualizerWrapper::Setting>();
            visualizer_setting->window_name = "Press Ctrl+S to save the view as an image";
            visualizer_setting->screenshot_filename = filename;
            Open3dVisualizerWrapper visualizer(visualizer_setting);
            DrawTree(visualizer.GetVisualizer().get());
            visualizer.Show();
        }

        virtual void
        DrawTree(open3d::visualization::Visualizer *visualizer) const = 0;

        void
        DrawLeaves(const std::string &filename) const {
            auto visualizer_setting = std::make_shared<Open3dVisualizerWrapper::Setting>();
            visualizer_setting->window_name = "Press Ctrl+S to save the view as an image";
            visualizer_setting->screenshot_filename = filename;
            Open3dVisualizerWrapper visualizer(visualizer_setting);
            DrawLeaves(visualizer.GetVisualizer().get());
            visualizer.Show();
        }

        virtual void
        DrawLeaves(open3d::visualization::Visualizer *visualizer) const = 0;
    };
}  // namespace erl::geometry

namespace YAML {

    template<>
    struct convert<erl::geometry::AbstractOctreeDrawer::Setting> {
        inline static Node
        encode(const erl::geometry::AbstractOctreeDrawer::Setting &rhs) {
            Node node;
            node["area_min"] = rhs.area_min;
            node["area_max"] = rhs.area_max;
            node["bg_color"] = rhs.bg_color;
            node["border_color"] = rhs.border_color;
            return node;
        }

        inline static bool
        decode(const Node &node, erl::geometry::AbstractOctreeDrawer::Setting &rhs) {
            if (!node.IsMap()) { return false; }
            rhs.area_min = node["area_min"].as<Eigen::Vector3d>();
            rhs.area_max = node["area_max"].as<Eigen::Vector3d>();
            rhs.bg_color = node["bg_color"].as<Eigen::Vector3d>();
            rhs.border_color = node["border_color"].as<Eigen::Vector3d>();
            return true;
        }
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::AbstractOctreeDrawer::Setting &rhs) {
        out << BeginMap;
        out << Key << "area_min" << Value << rhs.area_min;
        out << Key << "area_max" << Value << rhs.area_max;
        out << Key << "bg_color" << Value << rhs.bg_color;
        out << Key << "border_color" << Value << rhs.border_color;
        out << EndMap;
        return out;
    }
}  // namespace YAML
