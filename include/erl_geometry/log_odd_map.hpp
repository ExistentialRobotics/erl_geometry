#pragma once

#include "erl_common/enum_parse.hpp"
#include "erl_common/reflection.hpp"

#include <stdexcept>
#include <string>

namespace erl::geometry {

    struct LogOddMap {
        // occupancy grid cell types.
        // the number is consistent with nav_msgs/OccupancyGrid.
        // https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html
        enum class CellType { kOccupied = 100, kFree = 0, kUnexplored = 255 };
    };
}  // namespace erl::geometry

ERL_REFLECT_ENUM_SCHEMA(
    erl::geometry::LogOddMap::CellType,
    3,
    ERL_REFLECT_ENUM_MEMBER("occupied", erl::geometry::LogOddMap::CellType::kOccupied),
    ERL_REFLECT_ENUM_MEMBER("free", erl::geometry::LogOddMap::CellType::kFree),
    ERL_REFLECT_ENUM_MEMBER("unexplored", erl::geometry::LogOddMap::CellType::kUnexplored));
ERL_PARSE_ENUM(erl::geometry::LogOddMap::CellType, 3);
