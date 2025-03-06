#pragma once

#include "erl_common/grid_map_info.hpp"
#include "erl_common/yaml.hpp"

namespace erl::geometry {

    class AbstractQuadtreeDrawer {
    public:
        struct Setting : common::Yamlable<Setting> {

            int padding = 1;
            cv::Scalar bg_color = {128, 128, 128, 255};  // gray
            cv::Scalar fg_color = {255, 255, 255, 255};  // white
            cv::Scalar border_color = {0, 0, 0, 255};    // black
            int border_thickness = 1;
        };

        inline static const volatile bool kSettingRegistered = common::YamlableBase::Register<Setting>();

    private:
        std::shared_ptr<Setting> m_setting_ = {};

    public:
        explicit AbstractQuadtreeDrawer(std::shared_ptr<Setting> setting)
            : m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_, "setting is nullptr.");
        }

        virtual ~AbstractQuadtreeDrawer() = default;

        void
        DrawTree(const std::string &filename) const {
            cv::Mat img;
            DrawTree(img);
            cv::imwrite(filename, img);
        }

        virtual void
        DrawTree(cv::Mat &mat) const = 0;

        void
        DrawLeaves(const std::string &filename) const {
            cv::Mat img;
            DrawLeaves(img);
            cv::imwrite(filename, img);
        }

        virtual void
        DrawLeaves(cv::Mat &mat) const = 0;
    };
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::AbstractQuadtreeDrawer::Setting> {
    static Node
    encode(const erl::geometry::AbstractQuadtreeDrawer::Setting &setting) {
        Node node;
        node["padding"] = setting.padding;
        node["bg_color"] = setting.bg_color;
        node["fg_color"] = setting.fg_color;
        node["border_color"] = setting.border_color;
        node["border_thickness"] = setting.border_thickness;
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::AbstractQuadtreeDrawer::Setting &setting) {
        if (!node.IsMap()) { return false; }
        setting.padding = node["padding"].as<int>();
        setting.bg_color = node["bg_color"].as<cv::Scalar>();
        setting.fg_color = node["fg_color"].as<cv::Scalar>();
        setting.border_color = node["border_color"].as<cv::Scalar>();
        setting.border_thickness = node["border_thickness"].as<int>();
        return true;
    }
};  // namespace YAML
