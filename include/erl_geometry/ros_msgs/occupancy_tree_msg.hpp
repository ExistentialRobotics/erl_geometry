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
        const std::shared_ptr<const AbstractOccupancyOctree<Dtype>>& tree,
        bool binary,
        erl_geometry::OccupancyTreeMsg& msg) {
        std::ostringstream s;  // create a ostream to wrap msg->data
        msg.dim = 3;
        msg.tree_type = tree->GetTreeType();
        msg.is_double = std::is_same_v<Dtype, double>;
        msg.binary = binary;
        if (binary) {
            if (tree->WriteBinary(s)) {
                std::string str = s.str();
                msg.data.clear();
                msg.data.insert(msg.data.end(), str.begin(), str.end());
                return true;
            }
        } else {
            if (tree->Write(s)) {
                std::string str = s.str();
                msg.data.clear();
                msg.data.insert(msg.data.end(), str.begin(), str.end());
                return true;
            }
        }
        return false;
    }

    template<typename Dtype>
    bool
    SaveToOccupancyTreeMsg(
        const std::shared_ptr<const AbstractOccupancyQuadtree<Dtype>>& tree,
        bool binary,
        erl_geometry::OccupancyTreeMsg& msg) {
        std::ostringstream s;  // create a ostream to wrap msg->data
        msg.dim = 2;
        msg.tree_type = tree->GetTreeType();
        msg.is_double = std::is_same_v<Dtype, double>;
        msg.binary = binary;
        if (binary) {
            if (tree->WriteBinary(s)) {
                std::string str = s.str();
                msg.data.clear();
                msg.data.insert(msg.data.end(), str.begin(), str.end());
                return true;
            }
        } else {
            if (tree->Write(s)) {
                std::string str = s.str();
                msg.data.clear();
                msg.data.insert(msg.data.end(), str.begin(), str.end());
                return true;
            }
        }
        return false;
    }
}  // namespace erl::geometry
