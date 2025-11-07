#include "erl_geometry/occupancy_nd_tree_setting.hpp"

bool
erl::geometry::OccupancyNdTreeSetting::operator==(const NdTreeSetting &other) const {
    if (NdTreeSetting::operator==(other)) {
        const auto that = reinterpret_cast<const OccupancyNdTreeSetting &>(other);
        return log_odd_min == that.log_odd_min && log_odd_max == that.log_odd_max &&    //
               log_odd_hit == that.log_odd_hit && log_odd_miss == that.log_odd_miss &&  //
               log_odd_occ_threshold == that.log_odd_occ_threshold;
    }
    return false;
}
