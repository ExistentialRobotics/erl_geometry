#pragma once

#include "erl_common/assert.hpp"
#include "abstract_octree.hpp"
#include "octree_key.hpp"
#include "occupancy_octree_node.hpp"
#include "occupancy_nd_tree_setting.hpp"

namespace erl::geometry {

    /**
     * AbstractOccupancyOctree is a base class that implements generic occupancy quadtree functionality.
     */
    class AbstractOccupancyOctree : public AbstractOctree {
    public:
        using Setting = OccupancyNdTreeSetting;

    protected:
        std::shared_ptr<Setting> m_setting_ = std::make_shared<Setting>();
        inline static const std::string sk_BinaryFileHeader_ = "# OccupancyOctree binary file";  // binary file header identifier

    public:
        AbstractOccupancyOctree() = delete;  // no default constructor

        explicit AbstractOccupancyOctree(const std::shared_ptr<Setting>& setting)
            : AbstractOctree(setting),
              m_setting_(setting) {}

        AbstractOccupancyOctree(const AbstractOccupancyOctree&) = delete;  // no copy constructor

        //--IO
        /**
         * Write the tree as a binary sequence to file. Before writing, the tree is pruned.
         * @param filename
         * @return
         */
        bool
        WriteBinary(const std::string& filename);
        /**
         * Write the tree as a binary sequence to stream. Before writing, the tree is pruned.
         * @param s
         * @return
         */
        std::ostream&
        WriteBinary(std::ostream& s);
        /**
         * Write the tree to a binary file. The tree is not pruned before writing.
         * @param filename
         * @return
         */
        [[nodiscard]] bool
        WriteBinary(const std::string& filename) const;
        /**
         * Write the tree to a binary stream. The tree is not pruned before writing.
         * @param s
         * @return
         */
        std::ostream&
        WriteBinary(std::ostream& s) const;
        /**
         * Write the actual tree data to a binary stream.
         * @param s
         * @return
         */
        virtual std::ostream&
        WriteBinaryData(std::ostream& s) const = 0;
        /**
         * Read the tree from a binary file.
         * @param filename
         * @return
         */
        bool
        ReadBinary(const std::string& filename);
        /**
         * Read the tree from a binary stream.
         * @param s
         * @return
         */
        bool
        ReadBinary(std::istream& s);

    private:
        virtual std::istream&
        ReadBinaryData(std::istream& s) = 0;

    public:
        //-- occupancy queries
        [[nodiscard]] inline bool
        IsNodeOccupied(const OccupancyOctreeNode* node) const {
            return node->GetLogOdds() > m_setting_->log_odd_occ_threshold;
        }

        [[nodiscard]] inline bool
        IsNodeAtThreshold(const OccupancyOctreeNode* node) const {
            float log_odds = node->GetLogOdds();
            return log_odds >= m_setting_->log_odd_max || log_odds <= m_setting_->log_odd_min;
        }

        //-- update functions
        virtual void
        ToMaxLikelihood() = 0;
    };
}  // namespace erl::geometry
