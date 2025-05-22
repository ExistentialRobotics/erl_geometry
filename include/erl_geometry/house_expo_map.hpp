#pragma once

#include "space_2d.hpp"

#include "erl_common/json.hpp"

#include <open3d/geometry/TriangleMesh.h>

#include <string>

namespace erl::geometry {

    class HouseExpoMap {
        std::string m_file_;
        std::string m_room_id_;
        Eigen::Matrix2d m_bbox_;
        std::shared_ptr<Space2D> m_meter_space_;

    public:
        explicit HouseExpoMap(std::string file);

        HouseExpoMap(const std::string &file, double wall_thickness);

        [[nodiscard]] std::string
        GetFile() const {
            return m_file_;
        }

        [[nodiscard]] std::string
        GetRoomId() const {
            return m_room_id_;
        }

        [[nodiscard]] std::shared_ptr<Space2D>
        GetMeterSpace() {
            return m_meter_space_;
        }

        [[nodiscard]] std::shared_ptr<const Space2D>
        GetMeterSpace() const {
            return m_meter_space_;
        }

        [[nodiscard]] Eigen::Vector2d
        GetMapMin() const {
            return m_meter_space_->GetSurface()->vertices.rowwise().minCoeff();
        }

        [[nodiscard]] Eigen::Vector2d
        GetMapMax() const {
            return m_meter_space_->GetSurface()->vertices.rowwise().maxCoeff();
        }

        void
        Translate(const Eigen::Vector2d &translation) {
            m_meter_space_->Translate(translation);
            m_bbox_.colwise() += translation;
        }

        [[nodiscard]] std::shared_ptr<open3d::geometry::TriangleMesh>
        ExtrudeTo3D(double room_height) const;

        static void
        ToJson(nlohmann::json &json_data, const HouseExpoMap &map);

        static void
        FromJson(const nlohmann::json &json_data, HouseExpoMap &map);
    };
}  // namespace erl::geometry
