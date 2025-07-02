#pragma once

#include <stdexcept>
#include <string>

namespace erl::geometry {

    struct LogOddMap {
        enum CellType { kOccupied = 0, kFree = 255, kUnexplored = 128 };

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
