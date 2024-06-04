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
        explicit HouseExpoMap(const char *file);

        HouseExpoMap(const char *file, double wall_thickness);

        [[nodiscard]] std::string
        GetFile() const {
            return m_file_;
        }

        [[nodiscard]] std::string
        GetRoomId() const {
            return m_room_id_;
        }

        [[nodiscard]] const std::shared_ptr<Space2D> &
        GetMeterSpace() {
            return m_meter_space_;
        }

        [[nodiscard]] std::shared_ptr<open3d::geometry::TriangleMesh>
        ExtrudeTo3D(double room_height) const;

        static void
        ToJson(nlohmann::json &json_data, const HouseExpoMap &map);

        static void
        FromJson(const nlohmann::json &json_data, HouseExpoMap &map);
    };
}  // namespace erl::geometry
