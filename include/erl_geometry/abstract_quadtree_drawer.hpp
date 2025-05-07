#pragma once

#include "erl_common/grid_map_info.hpp"
#include "erl_common/yaml.hpp"

namespace erl::geometry {

    class AbstractQuadtreeDrawer {
    public:
        struct Setting : public common::Yamlable<Setting> {
            int padding = 1;
            cv::Scalar bg_color = {128, 128, 128, 255};  // gray
            cv::Scalar fg_color = {255, 255, 255, 255};  // white
            cv::Scalar border_color = {0, 0, 0, 255};    // black
            int border_thickness = 1;
        };

    private:
        std::shared_ptr<Setting> m_setting_ = {};

    public:
        explicit AbstractQuadtreeDrawer(std::shared_ptr<Setting> setting);

        virtual ~AbstractQuadtreeDrawer() = default;

        void
        DrawTree(const std::string &filename) const;

        virtual void
        DrawTree(cv::Mat &mat) const = 0;

        void
        DrawLeaves(const std::string &filename) const;

        virtual void
        DrawLeaves(cv::Mat &mat) const = 0;
    };
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::AbstractQuadtreeDrawer::Setting> {
    static Node
    encode(const erl::geometry::AbstractQuadtreeDrawer::Setting &setting);

    static bool
    decode(const Node &node, erl::geometry::AbstractQuadtreeDrawer::Setting &setting);
};  // namespace YAML
