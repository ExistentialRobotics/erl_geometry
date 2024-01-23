#pragma once

#include "erl_common/grid_map_info.hpp"
#include "erl_common/yaml.hpp"
#include "abstract_quadtree.hpp"

namespace erl::geometry {

    class AbstractQuadtreeDrawer {
    public:
        struct Setting : public common::Yamlable<Setting> {
            Eigen::Vector2d area_min = {0.0, 0.0};
            Eigen::Vector2d area_max = {1.0, 1.0};
            double resolution = 0.1;
            int padding = 1;
            cv::Scalar bg_color = {128, 128, 128, 255};  // gray
            cv::Scalar fg_color = {255, 255, 255, 255};  // white
            cv::Scalar border_color = {0, 0, 0, 255};    // black
            int border_thickness = 1;
        };

    private:
        std::shared_ptr<Setting> m_setting_ = {};

    protected:
        std::shared_ptr<const AbstractQuadtree> m_quadtree_ = {};

    public:
        explicit AbstractQuadtreeDrawer(std::shared_ptr<Setting> setting, std::shared_ptr<const AbstractQuadtree> quadtree = nullptr)
            : m_setting_(std::move(setting)),
              m_quadtree_(std::move(quadtree)) {
            ERL_ASSERTM(m_setting_, "setting is nullptr.");
        }

        virtual ~AbstractQuadtreeDrawer() = default;

        [[nodiscard]] inline std::shared_ptr<common::GridMapInfo2D>
        GetGridMapInfo() const {
            return std::make_shared<common::GridMapInfo2D>(
                m_setting_->area_min,
                m_setting_->area_max,
                Eigen::Vector2d(m_setting_->resolution, m_setting_->resolution),
                Eigen::Vector2i(m_setting_->padding, m_setting_->padding));
        }

        void
        SetQuadtree(std::shared_ptr<const AbstractQuadtree> quadtree) {
            m_quadtree_ = quadtree;
        }

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

namespace YAML {

    template<>
    struct convert<erl::geometry::AbstractQuadtreeDrawer::Setting> {
        inline static Node
        encode(const erl::geometry::AbstractQuadtreeDrawer::Setting &rhs) {
            Node node;
            node["area_min"] = rhs.area_min;
            node["area_max"] = rhs.area_max;
            node["resolution"] = rhs.resolution;
            node["padding"] = rhs.padding;
            node["bg_color"] = rhs.bg_color;
            node["fg_color"] = rhs.fg_color;
            node["border_color"] = rhs.border_color;
            node["border_thickness"] = rhs.border_thickness;
            return node;
        }

        inline static bool
        decode(const Node &node, erl::geometry::AbstractQuadtreeDrawer::Setting &rhs) {
            if (!node.IsMap()) { return false; }
            rhs.area_min = node["area_min"].as<Eigen::Vector2d>();
            rhs.area_max = node["area_max"].as<Eigen::Vector2d>();
            rhs.resolution = node["resolution"].as<double>();
            rhs.padding = node["padding"].as<int>();
            rhs.bg_color = node["bg_color"].as<cv::Scalar>();
            rhs.fg_color = node["fg_color"].as<cv::Scalar>();
            rhs.border_color = node["border_color"].as<cv::Scalar>();
            rhs.border_thickness = node["border_thickness"].as<int>();
            return true;
        }
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::AbstractQuadtreeDrawer::Setting &rhs) {
        out << BeginMap;
        out << Key << "area_min" << Value << rhs.area_min;
        out << Key << "area_max" << Value << rhs.area_max;
        out << Key << "resolution" << Value << rhs.resolution;
        out << Key << "padding" << Value << rhs.padding;
        out << Key << "bg_color" << Value << rhs.bg_color;
        out << Key << "fg_color" << Value << rhs.fg_color;
        out << Key << "border_color" << Value << rhs.border_color;
        out << Key << "border_thickness" << Value << rhs.border_thickness;
        out << EndMap;
        return out;
    }
}  // namespace YAML
