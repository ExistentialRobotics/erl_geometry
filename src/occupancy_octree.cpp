#include "erl_geometry/occupancy_octree.hpp"

namespace erl::geometry {
    OccupancyOctree::OccupancyOctree(double resolution)
        : OccupancyOctreeBase<OccupancyOctreeNode>(resolution) {
        s_init_.EnsureLinking();
    }

    OccupancyOctree::OccupancyOctree(const std::shared_ptr<Setting> &setting)
        : OccupancyOctreeBase<OccupancyOctreeNode>(setting) {}

    OccupancyOctree::OccupancyOctree(const std::string &filename)
        : OccupancyOctreeBase<OccupancyOctreeNode>(0.1) {  // resolution will be set by readBinary
        ERL_ASSERTM(this->ReadBinary(filename), "Failed to read %s from file: %s", GetTreeType().c_str(), filename.c_str());
    }
}  // namespace erl::geometry
