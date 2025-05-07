#pragma once

#include "abstract_quadtree.hpp"
#include "occupancy_nd_tree_setting.hpp"
#include "occupancy_quadtree_node.hpp"
#include "quadtree_key.hpp"

namespace erl::geometry {

    /**
     * AbstractOccupancyQuadtree is a base class that implements generic occupancy quadtree
     * functionality.
     */
    template<typename Dtype>
    class AbstractOccupancyQuadtree : public AbstractQuadtree<Dtype> {
        std::shared_ptr<OccupancyNdTreeSetting> m_setting_ = nullptr;

    public:
        using DataType = Dtype;
        using Super = AbstractQuadtree<Dtype>;

        AbstractOccupancyQuadtree() = delete;  // no default constructor

        explicit AbstractOccupancyQuadtree(std::shared_ptr<OccupancyNdTreeSetting> setting);

        AbstractOccupancyQuadtree(const AbstractOccupancyQuadtree& other) = default;
        AbstractOccupancyQuadtree&
        operator=(const AbstractOccupancyQuadtree& other) = default;
        AbstractOccupancyQuadtree(AbstractOccupancyQuadtree&& other) = default;
        AbstractOccupancyQuadtree&
        operator=(AbstractOccupancyQuadtree&& other) = default;

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
        IsNodeOccupied(const OccupancyQuadtreeNode* node) const;

        [[nodiscard]] bool
        IsNodeAtThreshold(const OccupancyQuadtreeNode* node) const;

        //-- search
        const OccupancyQuadtreeNode*
        GetHitOccupiedNode(
            const Eigen::Ref<typename Super::Vector2>& p,
            const Eigen::Ref<typename Super::Vector2>& v,
            bool ignore_unknown,
            Dtype max_range,
            typename Super::Vector2& hit_position);

        [[nodiscard]] virtual const OccupancyQuadtreeNode*
        GetHitOccupiedNode(
            Dtype px,
            Dtype py,
            Dtype vx,
            Dtype vy,
            bool ignore_unknown,
            Dtype max_range,
            Dtype& ex,
            Dtype& ey) const = 0;

        //-- update functions
        virtual void
        ToMaxLikelihood() = 0;
    };
}  // namespace erl::geometry

#include "abstract_occupancy_quadtree.tpp"
