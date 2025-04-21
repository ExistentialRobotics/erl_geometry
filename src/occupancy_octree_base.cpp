#include "erl_geometry/occupancy_octree_base.hpp"

namespace erl::geometry {
    bool
    OccupancyOctreeBaseSetting::operator==(const NdTreeSetting &rhs) const {
        if (OccupancyNdTreeSetting::operator==(rhs)) {
            const auto that = reinterpret_cast<const OccupancyOctreeBaseSetting &>(rhs);
            return use_change_detection == that.use_change_detection &&  //
                   use_aabb_limit == that.use_aabb_limit &&              //
                   aabb == that.aabb;
        }
        return false;
    }
}  // namespace erl::geometry
