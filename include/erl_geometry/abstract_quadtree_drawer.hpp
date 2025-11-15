#pragma once

#include "erl_common/grid_map_info.hpp"
#include "erl_common/yaml.hpp"

namespace erl::geometry {

    class AbstractQuadtreeDrawer {
    public:
        struct Setting : public common::Yamlable<Setting> {
            Eigen::Vector2f area_min = {0.0f, 0.0f};
            Eigen::Vector2f area_max = {1.0f, 1.0f};
            float resolution = 0.1f;
            float scaling = 1.0f;
            int padding = 1;
            cv::Scalar bg_color = {128, 128, 128, 255};  // gray
            cv::Scalar fg_color = {255, 255, 255, 255};  // white
            cv::Scalar border_color = {0, 0, 0, 255};    // black
            int border_thickness = 1;

            ERL_REFLECT_SCHEMA(
                Setting,
                ERL_REFLECT_MEMBER(Setting, area_min),
                ERL_REFLECT_MEMBER(Setting, area_max),
                ERL_REFLECT_MEMBER(Setting, resolution),
                ERL_REFLECT_MEMBER(Setting, scaling),
                ERL_REFLECT_MEMBER(Setting, padding),
                ERL_REFLECT_MEMBER(Setting, bg_color),
                ERL_REFLECT_MEMBER(Setting, fg_color),
                ERL_REFLECT_MEMBER(Setting, border_color),
                ERL_REFLECT_MEMBER(Setting, border_thickness));
        };

    private:
        std::shared_ptr<Setting> m_setting_ = {};

    protected:
        std::shared_ptr<common::GridMapInfo2Df> m_grid_map_info_ = nullptr;

    public:
        explicit AbstractQuadtreeDrawer(std::shared_ptr<Setting> setting);

        virtual ~AbstractQuadtreeDrawer() = default;

        [[nodiscard]] std::shared_ptr<const common::GridMapInfo2Df>
        GetGridMapInfo() const;

        /**
         * Compute the pixel coordinates for the given positions.
         * @param positions matrix of positions (2 x N)
         * @param scaled_position if true, the positions are already scaled by the scaling factor.
         * e.g., positions = positions_org * scaling.
         * @return matrix of pixel coordinates (2 x N)
         */
        template<typename Dtype>
        [[nodiscard]] Eigen::Matrix2Xi
        GetPixelCoordsForPositions(
            const Eigen::Ref<const Eigen::Matrix2X<Dtype>> &positions,
            const bool scaled_position) const {
            if (scaled_position) {
                return m_grid_map_info_->MeterToPixelForPoints(
                    positions.template cast<float>().array() / m_setting_->scaling);
            }
            return m_grid_map_info_->MeterToPixelForPoints(positions.template cast<float>());
        }

        template<typename Dtype>
        [[nodiscard]] Eigen::Matrix2Xi
        GetPixelCoordsForVectors(const Eigen::Ref<const Eigen::Matrix2X<Dtype>> &vectors) const {
            return m_grid_map_info_->MeterToPixelForVectors(vectors.template cast<float>());
        }

        /**
         * Compute the meter coordinates for the given pixel coordinates.
         * @param pixel_coords matrix of pixel coordinates (2 x N).
         * @param scaled_position if true, scale the returned meter coordinates with the scaling
         * factor. i.e., meter_coords = meter_coords_org * scaling.
         * @return matrix of meter coordinates (2 x N)
         */
        template<typename Dtype>
        [[nodiscard]] Eigen::Matrix2X<Dtype>
        GetMeterCoordsForPositions(const Eigen::Matrix2Xi &pixel_coords, const bool scaled_position)
            const {
            if (scaled_position) {
                return (m_grid_map_info_->PixelToMeterForPoints(pixel_coords).array() *
                        m_setting_->scaling)
                    .cast<Dtype>();
            }
            return m_grid_map_info_->PixelToMeterForPoints(pixel_coords).cast<Dtype>();
        }

        [[nodiscard]] Eigen::Matrix2Xf
        GetMeterCoordsForVectors(const Eigen::Matrix2Xi &pixel_coords) const {
            return m_grid_map_info_->PixelToMeterForVectors(pixel_coords);
        }

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
