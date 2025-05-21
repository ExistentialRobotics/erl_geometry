#pragma once

#include "erl_geometry/abstract_occupancy_octree.hpp"
#include "erl_geometry/abstract_occupancy_quadtree.hpp"
#include "erl_geometry/OccupancyTreeMsg.h"

namespace erl::geometry {
    template<typename Dtype>
    bool
    LoadFromOccupancyTreeMsg(
        const erl_geometry::OccupancyTreeMsg& msg,
        std::shared_ptr<AbstractOccupancyOctree<Dtype>>& tree) {
        std::istringstream s;  // create a istream to wrap msg->data
        s.str(std::string(msg.data.begin(), msg.data.end()));
        if (msg.binary) { return tree->ReadBinary(s); }
        return tree->Read(s);
    }

    template<typename Dtype>
    bool
    LoadFromOccupancyTreeMsg(
        const erl_geometry::OccupancyTreeMsg& msg,
        std::shared_ptr<AbstractOccupancyQuadtree<Dtype>>& tree) {
        std::istringstream s;  // create a istream to wrap msg->data
        s.str(std::string(msg.data.begin(), msg.data.end()));
        if (msg.binary) { return tree->ReadBinary(s); }
        return tree->Read(s);
    }

    template<typename Dtype>
    bool
    SaveToOccupancyTreeMsg(
        const std::shared_ptr<AbstractOccupancyOctree<Dtype>>& tree,
        erl_geometry::OccupancyTreeMsg& msg) {
        std::ostringstream s;  // create a ostream to wrap msg->data
        if (msg.binary) {
            if (tree->WriteBinary(s)) {
                msg.data = std::vector<int8_t>(s.str().begin(), s.str().end());
                return true;
            }
        } else {
            if (tree->Write(s)) {
                msg.data = std::vector<int8_t>(s.str().begin(), s.str().end());
                return true;
            }
        }
        return false;
    }

    template<typename Dtype>
    bool
    SaveToOccupancyTreeMsg(
        const std::shared_ptr<AbstractOccupancyQuadtree<Dtype>>& tree,
        erl_geometry::OccupancyTreeMsg& msg) {
        std::ostringstream s;  // create a ostream to wrap msg->data
        if (msg.binary) {
            if (tree->WriteBinary(s)) {
                msg.data = std::vector<int8_t>(s.str().begin(), s.str().end());
                return true;
            }
        } else {
            if (tree->Write(s)) {
                msg.data = std::vector<int8_t>(s.str().begin(), s.str().end());
                return true;
            }
        }
        return false;
    }
}  // namespace erl::geometry
