#pragma once

#include <stdexcept>
#include <string>

namespace erl::geometry {

    struct LogOddMap {
        // occupancy grid cell types.
        // the number is consistent with nav_msgs/OccupancyGrid.
        // https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html
        enum CellType { kOccupied = 100, kFree = 0, kUnexplored = 255 };

        static const char *
        GetCellTypeName(const CellType type) {
            static const char *names[] = {"kOccupied", "kUnexplored", "kFree"};

            const int i = (static_cast<int>(type) + 1) / 128;
            return names[i];
        }

        static CellType
        GetCellTypeFromName(const std::string &name) {
            if (name == "kOccupied") { return kOccupied; }
            if (name == "kFree") { return kFree; }
            if (name == "kUnexplored") { return kUnexplored; }
            throw std::runtime_error("Unknown cell type: " + name);
        }
    };
}  // namespace erl::geometry
