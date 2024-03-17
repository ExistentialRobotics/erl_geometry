#include "erl_geometry/occupancy_quadtree.hpp"

namespace erl::geometry {
    OccupancyQuadtree::OccupancyQuadtree(double resolution)
        : OccupancyQuadtreeBase<OccupancyQuadtreeNode>(resolution) {
        s_init_.EnsureLinking();
    }

    OccupancyQuadtree::OccupancyQuadtree(const std::shared_ptr<Setting> &setting)
        : OccupancyQuadtreeBase<OccupancyQuadtreeNode>(setting) {}

    OccupancyQuadtree::OccupancyQuadtree(const std::string &filename)
        : OccupancyQuadtreeBase<OccupancyQuadtreeNode>(0.1) {  // resolution will be set by readBinary
        ERL_ASSERTM(this->ReadBinary(filename), "Failed to read %s from file: %s", GetTreeType().c_str(), filename.c_str());
    }
}  // namespace erl::geometry
