#pragma once

#include "abstract_octree.hpp"
#include "occupancy_nd_tree_setting.hpp"
#include "occupancy_octree_node.hpp"

namespace erl::geometry {

    /**
     * AbstractOccupancyOctree is a base class that implements generic occupancy octree
     * functionality.
     */
    template<typename Dtype>
    class AbstractOccupancyOctree : public AbstractOctree<Dtype> {
        std::shared_ptr<OccupancyNdTreeSetting> m_setting_ = nullptr;

    public:
        using DataType = Dtype;
        using Super = AbstractOctree<Dtype>;

        AbstractOccupancyOctree() = delete;  // no default constructor

        explicit AbstractOccupancyOctree(std::shared_ptr<OccupancyNdTreeSetting> setting);

        AbstractOccupancyOctree(const AbstractOccupancyOctree&) = default;
        AbstractOccupancyOctree&
        operator=(const AbstractOccupancyOctree&) = default;
        AbstractOccupancyOctree(AbstractOccupancyOctree&&) = default;
        AbstractOccupancyOctree&
        operator=(AbstractOccupancyOctree&&) = default;

        //--IO
        /**
         * Write the tree as a binary sequence to stream.
         * @param s
         * @param prune If true, the tree is pruned before writing.
         * @return
         */
        bool
        WriteBinary(std::ostream& s, bool prune);

        /**
         * Write the tree to a binary stream. The tree is not pruned before writing.
         * @param s
         * @return
         */
        [[nodiscard]] bool
        WriteBinary(std::ostream& s) const;

        /**
         * Write the actual tree data to a binary stream.
         * @param s
         * @return
         */
        virtual bool
        WriteBinaryData(std::ostream& s) const = 0;

        /**
         * Read the tree from a binary stream.
         * @param s
         * @return
         */
        bool
        ReadBinary(std::istream& s);

        virtual bool
        ReadBinaryData(std::istream& s) = 0;

        //-- occupancy queries
        [[nodiscard]] bool
        IsNodeOccupied(const OccupancyOctreeNode* node) const;

        [[nodiscard]] bool
        IsNodeAtThreshold(const OccupancyOctreeNode* node) const;

        //-- search
        const OccupancyOctreeNode*
        GetHitOccupiedNode(
            const Eigen::Ref<typename Super::Vector3>& p,
            const Eigen::Ref<typename Super::Vector3>& v,
            bool ignore_unknown,
            Dtype max_range,
            typename Super::Vector3& hit_position);

        [[nodiscard]] virtual const OccupancyOctreeNode*
        GetHitOccupiedNode(
            Dtype px,
            Dtype py,
            Dtype pz,
            Dtype vx,
            Dtype vy,
            Dtype vz,
            bool ignore_unknown,
            Dtype max_range,
            Dtype& ex,
            Dtype& ey,
            Dtype& ez) const = 0;

        //-- update functions
        virtual void
        ToMaxLikelihood() = 0;
    };
}  // namespace erl::geometry

#include "abstract_occupancy_octree.tpp"
