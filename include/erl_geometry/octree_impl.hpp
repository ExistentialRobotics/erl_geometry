#pragma once

#include "abstract_octree.hpp"
#include "abstract_octree_node.hpp"
#include "octree_key.hpp"
#include "utils.hpp"
#include <omp.h>
#include <Eigen/Dense>
#include <stack>
#include <tuple>
#include <bitset>

namespace erl::geometry {

    /**
     * OctreeImpl is a template class that implements generic quadtree functionality.
     * @tparam Node
     * @tparam Interface
     */
    template<class Node, class Interface>
    class OctreeImpl : public Interface {
        static_assert(std::is_base_of<AbstractOctreeNode, Node>::value, "Node must be derived from AbstractOctreeNode");
        static_assert(std::is_base_of<AbstractOctree, Interface>::value, "Interface must be derived from AbstractOctree");

    protected:
        std::shared_ptr<Node> m_root_ = nullptr;  // root node of the quadtree, nullptr if the quadtree is empty
        // constants of the tree
        const unsigned int mk_TreeDepth_;      // depth of the tree
        const unsigned int mk_TreeKeyOffset_;  // offset of the tree key
        // parameters of the tree
        double m_resolution_ = 0.0;                // resolution of the tree
        double m_resolution_inv_ = 0.0;            // inverse resolution of the tree
        std::size_t m_tree_size_ = 0;              // number of nodes in the tree
        bool m_size_changed_ = false;              // flag indicating if the metric size of the tree has changed
        double m_tree_center_[3] = {0, 0, 0};      // metric coordinate of the center
        double m_metric_max_[3] = {};              // max metric coordinate of x, y and z
        double m_metric_min_[3] = {};              // min metric coordinate of x, y and z
        std::vector<double> m_size_lookup_table_;  // the size of a quadrant at depth i (0: root node, mk_TreeDepth_: smallest leaf node)
        std::vector<OctreeKeyRay> m_key_rays_;     // data structure for parallel ray casting

    public:
        using ImplType = OctreeImpl<Node, Interface>;

        //-- constructors
        explicit OctreeImpl(double resolution)
            : mk_TreeDepth_(16),
              mk_TreeKeyOffset_(1 << (mk_TreeDepth_ - 1)),
              m_resolution_(resolution) {
            Init();
        }

        OctreeImpl(const ImplType &other)
            : Interface(),
              mk_TreeDepth_(other.mk_TreeDepth_),
              mk_TreeKeyOffset_(other.mk_TreeKeyOffset_),
              m_resolution_(other.m_resolution_),
              m_tree_size_(other.m_tree_size_) {
            Init();
            if (other.m_root_ == nullptr) { return; }
            m_root_ = std::make_shared<Node>(*other.m_root_);  // the copy constructor of Node will deep-copy the whole tree
        }

        //-- comparison operators
        [[nodiscard]] bool
        operator==(const ImplType &other) const {
            if (mk_TreeDepth_ != other.mk_TreeDepth_) { return false; }
            if (mk_TreeKeyOffset_ != other.mk_TreeKeyOffset_) { return false; }
            if (m_resolution_ != other.m_resolution_) { return false; }
            if (m_tree_size_ != other.m_tree_size_) { return false; }

            // traverse all nodes, check if structure is the same
            TreeIterator it = BeginTree();
            TreeIterator end = EndTree();
            TreeIterator other_it = other.BeginTree();
            TreeIterator other_end = other.EndTree();

            for (; it != end; ++it, ++other_it) {
                if (other_it == other_end) { return false; }
                if (it.GetDepth() != other_it.GetDepth()) { return false; }
                if (it.GetKey() != other_it.GetKey()) { return false; }
                if (**it != **other_it) { return false; }
            }

            if (other_it != other_end) { return false; }

            return true;
        }

        //-- get tree info
        [[nodiscard]] std::string
        GetTreeType() const override {
            return "OctreeImpl";
        }

        [[nodiscard]] std::size_t
        GetSize() const override {
            return m_tree_size_;
        }

        inline void
        SetResolution(double resolution) override {
            m_resolution_ = resolution;
            m_resolution_inv_ = 1. / resolution;
            m_tree_center_[0] = m_tree_center_[1] = m_tree_center_[2] = double(mk_TreeKeyOffset_) * m_resolution_;
            // init node size lookup table
            m_size_lookup_table_.resize(mk_TreeDepth_ + 1);
            for (unsigned int i = 0; i <= mk_TreeDepth_; ++i) { m_size_lookup_table_[i] = m_resolution_ * double(1 << (mk_TreeDepth_ - i)); }
            m_size_changed_ = true;
        }

        [[nodiscard]] inline double
        GetResolution() const override {
            return m_resolution_;
        }

        [[nodiscard]] inline Eigen::Vector3d
        GetTreeCenter() const {
            return {m_tree_center_[0], m_tree_center_[1], m_tree_center_[2]};
        }

        [[nodiscard]] inline unsigned int
        GetTreeDepth() const {
            return mk_TreeDepth_;
        }

        [[nodiscard]] inline OctreeKey::KeyType
        GetTreeKeyOffset() const {
            return mk_TreeKeyOffset_;
        }

        inline void
        GetMetricMin(double &min_x, double &min_y, double &min_z) override {
            ComputeMinMax();
            min_x = m_metric_min_[0];
            min_y = m_metric_min_[1];
            min_z = m_metric_min_[2];
        }

        void
        GetMetricMin(double &min_x, double &min_y, double &min_z) const override {
            if (!m_size_changed_) {
                min_x = m_metric_min_[0];
                min_y = m_metric_min_[1];
                min_z = m_metric_min_[2];
                return;
            }

            if (m_root_ == nullptr) {
                min_x = min_y = min_z = 0;
                return;
            }

            min_x = min_y = min_z = std::numeric_limits<double>::infinity();
            for (auto it = this->begin(), end = this->end(); it != end; ++it) {
                double half_size = it.GetNodeSize() / 2.;
                double node_min_x = it.GetX() - half_size;
                double node_min_y = it.GetY() - half_size;
                double node_min_z = it.GetZ() - half_size;
                if (node_min_x < min_x) { min_x = node_min_x; }
                if (node_min_y < min_y) { min_y = node_min_y; }
                if (node_min_z < min_z) { min_z = node_min_z; }
            }
        }

        inline void
        GetMetricMax(double &max_x, double &max_y, double &max_z) override {
            ComputeMinMax();
            max_x = m_metric_max_[0];
            max_y = m_metric_max_[1];
            max_z = m_metric_max_[2];
        }

        void
        GetMetricMax(double &max_x, double &max_y, double &max_z) const override {
            if (!m_size_changed_) {
                max_x = m_metric_max_[0];
                max_y = m_metric_max_[1];
                max_z = m_metric_max_[2];
                return;
            }

            if (m_root_ == nullptr) {
                max_x = max_y = max_z = 0;
                return;
            }

            max_x = max_y = max_z = -std::numeric_limits<double>::infinity();
            for (auto it = this->begin(), end = this->end(); it != end; ++it) {
                double half_size = it.GetNodeSize() / 2.;
                double node_max_x = it.GetX() + half_size;
                double node_max_y = it.GetY() + half_size;
                double node_max_z = it.GetZ() + half_size;
                if (node_max_x > max_x) { max_x = node_max_x; }
                if (node_max_y > max_y) { max_y = node_max_y; }
                if (node_max_z > max_z) { max_z = node_max_z; }
            }
        }

        inline void
        GetMetricMinMax(double &min_x, double &min_y, double &min_z, double &max_x, double &max_y, double &max_z) override {
            ComputeMinMax();
            min_x = m_metric_min_[0];
            min_y = m_metric_min_[1];
            min_z = m_metric_min_[2];
            max_x = m_metric_max_[0];
            max_y = m_metric_max_[1];
            max_z = m_metric_max_[2];
        }

        void
        GetMetricMinMax(double &min_x, double &min_y, double &min_z, double &max_x, double &max_y, double &max_z) const override {
            if (!m_size_changed_) {
                min_x = m_metric_min_[0];
                min_y = m_metric_min_[1];
                min_z = m_metric_min_[2];
                max_x = m_metric_max_[0];
                max_y = m_metric_max_[1];
                max_z = m_metric_max_[2];
                return;
            }

            if (m_root_ == nullptr) {
                min_x = min_y = min_z = max_x = max_y = max_z = 0;
                return;
            }

            min_x = min_y = min_z = std::numeric_limits<double>::infinity();
            max_x = max_y = max_z = -std::numeric_limits<double>::infinity();
            for (auto it = this->begin(), end = this->end(); it != end; ++it) {
                double size = it.GetNodeSize();
                double half_size = size / 2.;
                double node_max_x = it.GetX() + half_size;
                double node_max_y = it.GetY() + half_size;
                double node_max_z = it.GetZ() + half_size;
                double node_min_x = node_max_x - size;
                double node_min_y = node_max_y - size;
                double node_min_z = node_max_z - size;
                if (node_max_x > max_x) { max_x = node_max_x; }
                if (node_max_y > max_y) { max_y = node_max_y; }
                if (node_max_z > max_z) { max_z = node_max_z; }
                if (node_min_x < min_x) { min_x = node_min_x; }
                if (node_min_y < min_y) { min_y = node_min_y; }
                if (node_min_z < min_z) { min_z = node_min_z; }
            }
        }

        inline void
        GetMetricSize(double &x, double &y, double &z) override {
            double min_x, min_y, min_z, max_x, max_y, max_z;
            GetMetricMinMax(min_x, min_y, min_z, max_x, max_y, max_z);
            x = max_x - min_x;
            y = max_y - min_y;
            z = max_z - min_z;
        }

        void
        GetMetricSize(double &x, double &y, double &z) const override {
            if (!m_size_changed_) {
                x = m_metric_max_[0] - m_metric_min_[0];
                y = m_metric_max_[1] - m_metric_min_[1];
                z = m_metric_max_[2] - m_metric_min_[2];
                return;
            }

            if (m_root_ == nullptr) {
                x = y = z = 0;
                return;
            }

            double min_x, min_y, min_z, max_x, max_y, max_z;
            min_x = min_y = min_z = std::numeric_limits<double>::infinity();
            max_x = max_y = max_z = -std::numeric_limits<double>::infinity();
            for (auto it = this->begin(), end = this->end(); it != end; ++it) {
                double size = it.GetNodeSize();
                double half_size = size / 2.;
                double node_max_x = it.GetX() + half_size;
                double node_max_y = it.GetY() + half_size;
                double node_max_z = it.GetZ() + half_size;
                double node_min_x = node_max_x - size;
                double node_min_y = node_max_y - size;
                double node_min_z = node_max_z - size;
                if (node_max_x > max_x) { max_x = node_max_x; }
                if (node_max_y > max_y) { max_y = node_max_y; }
                if (node_max_z > max_z) { max_z = node_max_z; }
                if (node_min_x < min_x) { min_x = node_min_x; }
                if (node_min_y < min_y) { min_y = node_min_y; }
                if (node_min_z < min_z) { min_z = node_min_z; }
            }
            x = max_x - min_x;
            y = max_y - min_y;
            z = max_z - min_z;
        }

        [[nodiscard]] inline double
        GetNodeSize(unsigned int depth) const {
            ERL_ASSERTM(depth <= mk_TreeDepth_, "Depth must be in [0, %u], but got %u.\n", mk_TreeDepth_, depth);
            return m_size_lookup_table_[depth];
        }

        [[nodiscard]] inline std::size_t
        ComputeNumberOfLeafNodes() const {
            if (m_root_ == nullptr) { return 0; }

            std::vector<std::shared_ptr<const Node>> nodes_stack;
            nodes_stack.emplace_back(m_root_);
            unsigned int num_leaf_nodes = 0;
            while (!nodes_stack.empty()) {
                auto node = nodes_stack.back();
                nodes_stack.pop_back();

                if (node->HasAnyChild()) {
                    for (unsigned int i = 0; i < 8; ++i) {
                        auto child = GetNodeChild(node, i);
                        if (child != nullptr) { nodes_stack.emplace_back(child); }
                    }
                } else {
                    num_leaf_nodes++;
                }
            }
            return num_leaf_nodes;
        }

        [[nodiscard]] inline std::size_t
        GetMemoryUsage() const override {
            std::size_t number_of_leaf_nodes = ComputeNumberOfLeafNodes();
            std::size_t number_of_inner_nodes = m_tree_size_ - number_of_leaf_nodes;
            return sizeof(ImplType) + GetMemoryUsagePerNode() * m_tree_size_ + number_of_inner_nodes * sizeof(std::shared_ptr<Node>) * 4;
        }

        [[nodiscard]] inline std::size_t
        GetMemoryUsagePerNode() const override {
            return sizeof(Node);
        }

        [[nodiscard]] inline unsigned int
        ComputeNumberOfNodes() const {
            if (m_root_ == nullptr) { return 0; }

            unsigned int num_nodes = 0;
            std::vector<std::shared_ptr<const Node>> nodes_stack;
            nodes_stack.emplace_back(m_root_);
            while (!nodes_stack.empty()) {
                auto node = nodes_stack.back();
                nodes_stack.pop_back();
                num_nodes++;

                if (node->HasAnyChild()) {  // if the node has any child, push them into the stack
                    for (unsigned int i = 0; i < 8; ++i) {
                        auto child = GetNodeChild(node, i);
                        if (child != nullptr) { nodes_stack.emplace_back(child); }
                    }
                }
            }
            return num_nodes;
        }

    public:
        //-- key / coordinate operations
        // ref: https://www.tandfonline.com/doi/abs/10.1080/10867651.2002.10487560?journalCode=ujgt19
        /**
         * Convert 1-dim coordinate to key at depth N.
         * @param coordinate
         * @return
         */
        [[nodiscard]] inline OctreeKey::KeyType
        CoordToKey(double coordinate) const {
            return std::floor(coordinate * m_resolution_inv_) + mk_TreeKeyOffset_;
        }

        /**
         * Convert 1-dim coordinate to key at a given depth.
         * @param coordinate
         * @param depth
         * @return
         */
        [[nodiscard]] inline OctreeKey::KeyType
        CoordToKey(double coordinate, unsigned int depth) const {
            ERL_ASSERTM(depth <= mk_TreeDepth_, "Depth must be in [0, %u], but got %u.\n", mk_TreeDepth_, depth);
            long keyval = std::floor(coordinate * m_resolution_inv_);
            unsigned int diff = mk_TreeDepth_ - depth;
            if (!diff) { return keyval + mk_TreeKeyOffset_; }

            return ((keyval >> diff) << diff) + (1 << (diff - 1)) + mk_TreeKeyOffset_;
        }

        /**
         * Convert 3-dim coordinate to key at depth N.
         * @param x
         * @param y
         * @param z
         * @return
         */
        [[nodiscard]] inline OctreeKey
        CoordToKey(double x, double y, double z) const {
            return {CoordToKey(x), CoordToKey(y), CoordToKey(z)};
        }

        /**
         * Convert 3-dim coordinate to key at a given depth.
         * @param x
         * @param y
         * @param z
         * @param depth
         * @return
         */
        [[nodiscard]] inline OctreeKey
        CoordToKey(double x, double y, double z, unsigned int depth) const {
            if (depth == mk_TreeDepth_) { return CoordToKey(x, y, z); }
            return {CoordToKey(x, depth), CoordToKey(y, depth), CoordToKey(z, depth)};
        }

        /**
         * Convert 1-dim coordinate to key at depth N with boundary check.
         * @param coordinate
         * @param key
         * @return
         */
        [[nodiscard]] inline bool
        CoordToKeyChecked(double coordinate, OctreeKey::KeyType &key) const {
            int scaled_coord = std::floor(coordinate * m_resolution_inv_) + mk_TreeKeyOffset_;
            if ((scaled_coord >= 0) && ((unsigned int) scaled_coord) < (mk_TreeKeyOffset_ << 1)) {
                key = scaled_coord;
                return true;
            }
            return false;
        }

        /**
         * Convert 1-dim coordinate to key at a given depth with boundary check.
         * @param coordinate
         * @param depth
         * @param key
         * @return
         */
        [[nodiscard]] inline bool
        CoordToKeyChecked(double coordinate, unsigned int depth, OctreeKey::KeyType &key) const {
            int scaled_coord = std::floor(coordinate * m_resolution_inv_) + mk_TreeKeyOffset_;
            if ((scaled_coord >= 0) && ((unsigned int) scaled_coord) < (mk_TreeKeyOffset_ << 1)) {
                key = AdjustKeyToDepth((OctreeKey::KeyType) scaled_coord, depth);
                return true;
            }
            return false;
        }

        /**
         * Convert 3-dim coordinate to key at depth N with boundary check.
         * @param x
         * @param y
         * @param z
         * @param key
         * @return
         */
        [[nodiscard]] inline bool
        CoordToKeyChecked(double x, double y, double z, OctreeKey &key) const {
            if (!CoordToKeyChecked(x, key[0])) { return false; }
            if (!CoordToKeyChecked(y, key[1])) { return false; }
            if (!CoordToKeyChecked(z, key[2])) { return false; }
            return true;
        }

        /**
         * Convert 3-dim coordinate to key at a given depth with boundary check.
         * @param x
         * @param y
         * @param z
         * @param depth
         * @param key
         * @return
         */
        [[nodiscard]] inline bool
        CoordToKeyChecked(double x, double y, double z, unsigned int depth, OctreeKey &key) const {
            ERL_DEBUG_ASSERT(depth != 0, "When depth = 0, key is 0x0, which is useless!");
            if (depth == mk_TreeDepth_) { return CoordToKeyChecked(x, y, z, key); }
            if (!CoordToKeyChecked(x, depth, key[0])) { return false; }
            if (!CoordToKeyChecked(y, depth, key[1])) { return false; }
            if (!CoordToKeyChecked(z, depth, key[2])) { return false; }
            return true;
        }

        /**
         * Adjust 1-dim key from the lowest level (max depth) to a given depth.
         * @param key the key at the lowest level
         * @param depth the target depth
         * @return
         */
        [[nodiscard]] inline OctreeKey::KeyType
        AdjustKeyToDepth(OctreeKey::KeyType key, unsigned int depth) const {
            ERL_ASSERTM(depth <= mk_TreeDepth_, "Depth must be in [0, %u], but got %u.\n", mk_TreeDepth_, depth);
            unsigned int diff = mk_TreeDepth_ - depth;
            if (!diff) { return key; }
            return (((key - mk_TreeKeyOffset_) >> diff) << diff) + (1 << (diff - 1)) + mk_TreeKeyOffset_;
        }

        /**
         * Adjust 3-dim key from the lowest level (max depth) to a given depth.
         * @param key the key at the lowest level
         * @param depth the target depth
         * @return
         */
        [[nodiscard]] inline OctreeKey
        AdjustKeyToDepth(const OctreeKey &key, unsigned int depth) const {
            if (depth == mk_TreeDepth_) { return key; }
            return {AdjustKeyToDepth(key[0], depth), AdjustKeyToDepth(key[1], depth), AdjustKeyToDepth(key[2], depth)};
        }

        inline void
        ComputeCommonAncestorKey(const OctreeKey &key1, const OctreeKey &key2, OctreeKey &ancestor_key, unsigned int &ancestor_depth) const {
            OctreeKey::KeyType mask = (key1[0] ^ key2[0]) | (key1[1] ^ key2[1]) | (key1[2] ^ key2[2]);  // 0: same bit, 1: different bit
            if (!mask) {                                                                                // keys are identical
                ancestor_key = key1;
                ancestor_depth = mk_TreeDepth_;
                return;
            }
            // from bit-max_depth to bit-0, find first 1
            unsigned int level = mk_TreeDepth_;
            while (level > 0 && !(mask & (1 << (level - 1)))) { --level; }
            ancestor_depth = mk_TreeDepth_ - level;  // bit[level] = 0, bit[level-1] = 1
            OctreeKey::KeyType ancestor_mask = ((1 << mk_TreeDepth_) - 1) << level;
            ancestor_key[0] = key1[0] & ancestor_mask;
            ancestor_key[1] = key1[1] & ancestor_mask;
            ancestor_key[2] = key1[2] & ancestor_mask;
        }

        /**
         * Compute the key of the west(left) neighbor of a node.
         * @param key
         * @param depth 0 means root
         * @param neighbor_key
         */
        inline bool
        ComputeWestNeighborKey(const OctreeKey &key, unsigned int depth, OctreeKey &neighbor_key) const {
            OctreeKey::KeyType offset = 1 << (mk_TreeDepth_ - depth);
            if (key[0] < offset) { return false; }  // no west neighbor
            neighbor_key[0] = key[0] - offset;
            neighbor_key[1] = key[1];
            neighbor_key[2] = key[2];
            return true;
        }

        inline bool
        ComputeEastNeighborKey(const OctreeKey &key, unsigned int depth, OctreeKey &neighbor_key) const {
            OctreeKey::KeyType offset = 1 << (mk_TreeDepth_ - depth);
            if ((1 << mk_TreeDepth_) - key[0] <= offset) { return false; }  // no east neighbor (key[0] + offset >= 2^max_depth)
            neighbor_key[0] = key[0] + offset;
            neighbor_key[1] = key[1];
            neighbor_key[2] = key[2];
            return true;
        }

        inline bool
        ComputeNorthNeighborKey(const OctreeKey &key, unsigned int depth, OctreeKey &neighbor_key) const {
            OctreeKey::KeyType offset = 1 << (mk_TreeDepth_ - depth);
            if ((1 << mk_TreeDepth_) - key[1] <= offset) { return false; }  // no north neighbor (key[1] + offset >= 2^max_depth)
            neighbor_key[0] = key[0];
            neighbor_key[1] = key[1] + offset;
            neighbor_key[2] = key[2];
            return true;
        }

        inline bool
        ComputeSouthNeighborKey(const OctreeKey &key, unsigned int depth, OctreeKey &neighbor_key) const {
            OctreeKey::KeyType offset = 1 << (mk_TreeDepth_ - depth);
            if (key[1] < offset) { return false; }  // no south neighbor
            neighbor_key[0] = key[0];
            neighbor_key[1] = key[1] - offset;
            neighbor_key[2] = key[2];
            return true;
        }

        inline bool
        ComputeBottomNeighborKey(const OctreeKey &key, unsigned int depth, OctreeKey &neighbor_key) const {
            OctreeKey::KeyType offset = 1 << (mk_TreeDepth_ - depth);
            if (key[2] < offset) { return false; }  // no bottom neighbor
            neighbor_key[0] = key[0];
            neighbor_key[1] = key[1];
            neighbor_key[2] = key[2] - offset;
            return true;
        }

        inline bool
        ComputeTopNeighborKey(const OctreeKey &key, unsigned int depth, OctreeKey &neighbor_key) const {
            OctreeKey::KeyType offset = 1 << (mk_TreeDepth_ - depth);
            if ((1 << mk_TreeDepth_) - key[2] <= offset) { return false; }  // no top neighbor (key[2] + offset >= 2^max_depth)
            neighbor_key[0] = key[0];
            neighbor_key[1] = key[1];
            neighbor_key[2] = key[2] + offset;
            return true;
        }

        /**
         * Convert 1-dim key to coordinate.
         * @param key
         * @return
         */
        [[nodiscard]] inline double
        KeyToCoord(OctreeKey::KeyType key) const {
            return (double(int(key) - int(mk_TreeKeyOffset_)) + 0.5) * m_resolution_;
        }

        /**
         * Convert 1-dim key to coordinate at a given depth.
         * @param key
         * @param depth
         * @return
         */
        [[nodiscard]] inline double
        KeyToCoord(OctreeKey::KeyType key, unsigned int depth) const {
            if (depth == 0) { return 0.0; }
            if (depth == mk_TreeDepth_) { return KeyToCoord(key); }
            return (std::floor((double(key) - double(mk_TreeKeyOffset_)) / double(1 << (mk_TreeDepth_ - depth))) + 0.5) * GetNodeSize(depth);
        }

        /**
         * Convert 3-dim key to coordinate.
         * @param key
         * @param x
         * @param y
         * @param z
         */
        inline void
        KeyToCoord(const OctreeKey &key, double &x, double &y, double &z) const {
            x = KeyToCoord(key[0]);
            y = KeyToCoord(key[1]);
            z = KeyToCoord(key[2]);
        }

        /**
         * Convert 3-dim key to coordinate at a given depth.
         * @param key
         * @param depth
         * @param x
         * @param y
         * @param z
         */
        inline void
        KeyToCoord(const OctreeKey &key, unsigned int depth, double &x, double &y, double &z) const {
            if (depth == 0) {
                x = y = z = 0.0;
                return;
            }
            if (depth == mk_TreeDepth_) {
                KeyToCoord(key, x, y, z);
                return;
            }
            x = KeyToCoord(key[0], depth);
            y = KeyToCoord(key[1], depth);
            z = KeyToCoord(key[2], depth);
        }

        //-- iterator implementation
        class IteratorBase {
        public:
            struct StackElement {
                std::shared_ptr<Node> node = nullptr;
                OctreeKey key = {};
                unsigned int depth = 0;
                std::shared_ptr<void> data = nullptr;  // data pointer for derived classes

                StackElement() = default;

                StackElement(std::shared_ptr<Node> node, OctreeKey key, unsigned int depth)
                    : node(node),
                      key(key),
                      depth(depth) {}
            };

        protected:
            const ImplType *m_tree_;            // the tree this iterator is working on
            unsigned int m_max_depth_;          // the maximum depth to query
            std::deque<StackElement> m_stack_;  // stack for depth first traversal

        public:
            /**
             * Default constructor, only used for the end-iterator.
             */
            IteratorBase()
                : m_tree_(nullptr),
                  m_max_depth_(0) {}

            explicit IteratorBase(const ImplType *tree, unsigned int depth = 0)
                : m_tree_(tree && tree->GetRoot() ? tree : nullptr),
                  m_max_depth_(depth) {
                if (m_tree_ && m_max_depth_ == 0) { m_max_depth_ = m_tree_->GetTreeDepth(); }
                if (m_tree_ && m_tree_->GetRoot()) {  // tree is not empty
                    m_stack_.emplace_back(m_tree_->GetRoot(), m_tree_->CoordToKey(0.0, 0.0, 0.0), 0);
                } else {
                    m_tree_ = nullptr;
                    m_max_depth_ = 0;
                }
            }

            virtual ~IteratorBase() = default;

            [[nodiscard]] bool
            operator==(const IteratorBase &other) const {
                // we do not need to compare m_max_depth_ here, since it is always the same for the same tree
                if (m_tree_ != other.m_tree_) { return false; }
                if (m_stack_.size() != other.m_stack_.size()) { return false; }
                if (m_stack_.empty()) { return true; }

                const StackElement &kTop = m_stack_.back();
                auto &other_top = other.m_stack_.back();
                if (kTop.node != other_top.node) { return false; }
                if (kTop.depth != other_top.depth) { return false; }
                if (kTop.key != other_top.key) { return false; }
                return true;
            }

            [[nodiscard]] inline bool
            operator!=(const IteratorBase &other) const {
                return !operator==(other);
            }

            inline std::shared_ptr<Node> &
            operator->() {
                return m_stack_.back().node;
            }

            [[nodiscard]] inline const std::shared_ptr<const Node> &
            operator->() const {
                return m_stack_.back().node;
            }

            inline std::shared_ptr<Node> &
            operator*() {
                return m_stack_.back().node;
            }

            [[nodiscard]] inline const std::shared_ptr<Node> &
            operator*() const {
                return m_stack_.back().node;
            }

            [[nodiscard]] inline double
            GetX() const {
                const StackElement &kTop = m_stack_.back();
                return m_tree_->KeyToCoord(kTop.key[0], kTop.depth);
            }

            [[nodiscard]] inline double
            GetY() const {
                const StackElement &kTop = m_stack_.back();
                return m_tree_->KeyToCoord(kTop.key[1], kTop.depth);
            }

            [[nodiscard]] inline double
            GetZ() const {
                const StackElement &kTop = m_stack_.back();
                return m_tree_->KeyToCoord(kTop.key[2], kTop.depth);
            }

            [[nodiscard]] inline double
            GetNodeSize() const {
                return m_tree_->GetNodeSize(m_stack_.back().depth);
            }

            [[nodiscard]] inline unsigned int
            GetDepth() const {
                return m_stack_.back().depth;
            }

            [[nodiscard]] inline const OctreeKey &
            GetKey() const {
                return m_stack_.back().key;
            }

            [[nodiscard]] inline OctreeKey
            GetIndexKey() const {
                const StackElement &kTop = m_stack_.back();
                return m_tree_->AdjustKeyToDepth(kTop.key, kTop.depth);
            }

        protected:
            /**
             * One step of depth-first tree traversal.
             */
            virtual void
            SingleIncrement() {
                StackElement top = m_stack_.back();
                m_stack_.pop_back();
                if (top.depth == m_max_depth_) { return; }

                unsigned int next_depth = top.depth + 1;
                ERL_DEBUG_ASSERT(next_depth <= m_max_depth_, "Wrong depth: %u (max: %u).\n", next_depth, m_max_depth_);
                OctreeKey next_key;
                OctreeKey::KeyType center_offset_key = m_tree_->GetTreeKeyOffset() >> next_depth;
                // push on stack in reverse order
                for (int i = 7; i >= 0; --i) {
                    if (top.node->HasChild(i)) {
                        OctreeKey::ComputeChildKey(i, center_offset_key, top.key, next_key);
                        m_stack_.emplace_back(std::const_pointer_cast<Node>(m_tree_->GetNodeChild(top.node, i)), next_key, next_depth);
                    }
                }
            }

            [[nodiscard]] inline bool
            IsLeaf() const {
                const typename IteratorBase::StackElement &kTop = m_stack_.back();
                if (kTop.depth == m_tree_->GetTreeDepth()) { return true; }
                if (!kTop.node->HasAnyChild()) { return true; }
                return false;
            }

            inline void
            Terminate() {
                this->m_stack_.clear();
                this->m_tree_ = nullptr;
                this->m_max_depth_ = 0;
            }
        };

        class TreeIterator : public IteratorBase {
        public:
            TreeIterator() = default;

            explicit TreeIterator(const ImplType *tree, unsigned int depth = 0)
                : IteratorBase(tree, depth) {}

            // post-increment
            inline auto
            operator++(int) {
                const TreeIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            inline TreeIterator &
            operator++() {
                if (!this->m_stack_.empty()) { this->SingleIncrement(); }
                if (this->m_stack_.empty()) { this->Terminate(); }
                return *this;
            }
        };

        class TreeInAabbIterator : public IteratorBase {
        private:
            OctreeKey m_aabb_min_key_;
            OctreeKey m_aabb_max_key_;

        public:
            TreeInAabbIterator() = default;

            TreeInAabbIterator(
                double aabb_mix_x,
                double aabb_min_y,
                double aabb_min_z,
                double aabb_max_x,
                double aabb_max_y,
                double aabb_max_z,
                const ImplType *tree,
                unsigned int depth = 0
            )
                : IteratorBase(tree, depth) {
                if (this->m_stack_.empty()) { return; }
                ERL_ASSERTM(tree != nullptr, "Tree is null.");

                if (this->m_tree_->CoordToKeyChecked(aabb_mix_x, aabb_min_y, aabb_min_z, m_aabb_min_key_) &&
                    this->m_tree_->CoordToKeyChecked(aabb_max_x, aabb_max_y, aabb_max_z, m_aabb_max_key_)) {
                    // check if the root node is in the AABB
                    typename IteratorBase::StackElement top = this->m_stack_.back();
                    OctreeKey::KeyType center_offset_key = this->m_tree_->GetTreeKeyOffset() >> top.depth;
                    if (!OctreeKey::KeyInAabb(top.key, center_offset_key, m_aabb_min_key_, m_aabb_max_key_)) {
                        this->Terminate();
                        return;
                    }
                } else {
                    this->Terminate();
                }
            }

            TreeInAabbIterator(const OctreeKey &aabb_min_key, const OctreeKey &aabb_max_key, const ImplType *tree, unsigned int depth = 0)
                : IteratorBase(tree, depth),
                  m_aabb_min_key_(aabb_min_key),
                  m_aabb_max_key_(aabb_max_key) {
                if (this->m_stack_.empty()) { return; }
                ERL_ASSERTM(tree != nullptr, "Tree is null.");

                // check if the root node is in the AABB
                typename IteratorBase::StackElement top = this->m_stack_.back();
                OctreeKey::KeyType center_offset_key = this->m_tree_->GetTreeKeyOffset() >> top.depth;
                if (!OctreeKey::KeyInAabb(top.key, center_offset_key, m_aabb_min_key_, m_aabb_max_key_)) {
                    this->Terminate();
                    return;
                }
            }

            // post-increment
            inline auto
            operator++(int) {
                const TreeInAabbIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            inline TreeInAabbIterator &
            operator++() {
                if (!this->m_stack_.empty()) { this->SingleIncrement(); }
                if (this->m_stack_.empty()) { this->Terminate(); }
                return *this;
            }

        private:
            void
            SingleIncrement() override {
                typename IteratorBase::StackElement top = this->m_stack_.back();
                this->m_stack_.pop_back();
                if (top.depth == this->m_max_depth_) { return; }

                unsigned int next_depth = top.depth + 1;
                ERL_DEBUG_ASSERT(next_depth <= this->m_max_depth_, "Wrong depth: %u (max: %u).\n", next_depth, this->m_max_depth_);
                OctreeKey next_key;
                OctreeKey::KeyType center_offset_key = this->m_tree_->GetTreeKeyOffset() >> next_depth;
                // push on stack in reverse order
                for (int i = 7; i >= 0; --i) {
                    if (!top.node->HasChild(i)) { continue; }
                    OctreeKey::ComputeChildKey(i, center_offset_key, top.key, next_key);
                    // check if the child node overlaps with the AABB
                    if (OctreeKey::KeyInAabb(next_key, center_offset_key, m_aabb_min_key_, m_aabb_max_key_)) {
                        this->m_stack_.emplace_back(std::const_pointer_cast<Node>(this->m_tree_->GetNodeChild(top.node, i)), next_key, next_depth);
                    }
                }
            }
        };

        class LeafIterator : public IteratorBase {
        public:
            LeafIterator() = default;

            explicit LeafIterator(const ImplType *tree, unsigned int depth = 0)
                : IteratorBase(tree, depth) {
                if (this->m_stack_.empty()) { return; }
                // skip forward to next valid leaf node
                while (!this->IsLeaf()) { this->SingleIncrement(); }
            }

            // post-increment
            auto
            operator++(int) {
                const LeafIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            LeafIterator &
            operator++() {
                if (this->m_stack_.empty()) {
                    this->Terminate();
                    return *this;
                }

                do { this->SingleIncrement(); } while (!this->m_stack_.empty() && !this->IsLeaf());
                if (this->m_stack_.empty()) { this->Terminate(); }
                return *this;
            }
        };

        class LeafOfNodeIterator : public IteratorBase {
        public:
            LeafOfNodeIterator() = default;

            LeafOfNodeIterator(OctreeKey key, unsigned int cluster_depth, const ImplType *tree, unsigned int depth = 0)
                : IteratorBase(tree, depth) {
                ERL_ASSERTM(cluster_depth <= this->m_max_depth_, "Cluster depth %u is greater than max depth %u.\n", cluster_depth, this->m_max_depth_);

                // modify stack top
                auto &s = this->m_stack_.back();
                unsigned int d = cluster_depth;
                s.node = std::const_pointer_cast<Node>(this->m_tree_->Search(key, d));
                if (s.node == nullptr) {
                    this->m_stack_.clear();
                    this->m_tree_ = nullptr;
                    this->m_max_depth_ = 0;
                    return;
                }
                s.depth = cluster_depth;
                s.key = key;

                // skip forward to next valid leaf node
                while (!this->IsLeaf()) { this->SingleIncrement(); }
            }

            // post-increment
            auto
            operator++(int) {
                const LeafOfNodeIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            LeafOfNodeIterator &
            operator++() {
                if (this->m_stack_.empty()) {
                    this->Terminate();
                    return *this;
                }

                do { this->SingleIncrement(); } while (!this->m_stack_.empty() && !this->IsLeaf());
                if (this->m_stack_.empty()) { this->Terminate(); }
                return *this;
            }
        };

        /**
         * Iterate over all leaf nodes that intersect with the given axis-aligned bounding box.
         */
        class LeafInAabbIterator : public IteratorBase {
        private:
            OctreeKey m_aabb_min_key_;
            OctreeKey m_aabb_max_key_;

        public:
            LeafInAabbIterator() = default;

            LeafInAabbIterator(
                double aabb_mix_x,
                double aabb_min_y,
                double aabb_min_z,
                double aabb_max_x,
                double aabb_max_y,
                double aabb_max_z,
                const ImplType *tree,
                unsigned int depth = 0
            )
                : IteratorBase(tree, depth) {
                if (this->m_stack_.empty()) { return; }
                ERL_ASSERTM(tree != nullptr, "Tree is null.");

                if (this->m_tree_->CoordToKeyChecked(aabb_mix_x, aabb_min_y, aabb_min_z, m_aabb_min_key_) &&
                    this->m_tree_->CoordToKeyChecked(aabb_max_x, aabb_max_y, aabb_max_z, m_aabb_max_key_)) {
                    // skip forward to next valid leaf node
                    while (!this->m_stack_.empty() && !this->IsLeaf()) { SingleIncrement(); }
                    if (this->m_stack_.empty()) { this->Terminate(); }
                } else {
                    this->Terminate();
                }
            }

            LeafInAabbIterator(const OctreeKey &aabb_min_key, const OctreeKey &aabb_max_key, const ImplType *tree, unsigned int depth = 0)
                : IteratorBase(tree, depth),
                  m_aabb_min_key_(aabb_min_key),
                  m_aabb_max_key_(aabb_max_key) {
                if (this->m_stack_.empty()) { return; }
                ERL_ASSERTM(tree != nullptr, "Tree is null.");

                // skip forward to next valid leaf node
                while (!this->m_stack_.empty() && !this->IsLeaf()) { SingleIncrement(); }
                if (this->m_stack_.empty()) { this->Terminate(); }
            }

            // post-increment
            auto
            operator++(int) {
                const LeafInAabbIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            LeafInAabbIterator &
            operator++() {
                if (this->m_stack_.empty()) {
                    this->Terminate();
                    return *this;
                }

                do { SingleIncrement(); } while (!this->m_stack_.empty() && !this->IsLeaf());
                if (this->m_stack_.empty()) { this->Terminate(); }
                return *this;
            }

        protected:
            void
            SingleIncrement() override {
                typename IteratorBase::StackElement top = this->m_stack_.back();
                this->m_stack_.pop_back();
                if (top.depth == this->m_max_depth_) { return; }

                unsigned int next_depth = top.depth + 1;
                ERL_DEBUG_ASSERT(next_depth <= this->m_max_depth_, "Wrong depth: %u (max: %u).\n", next_depth, this->m_max_depth_);
                OctreeKey next_key;
                OctreeKey::KeyType center_offset_key = this->m_tree_->GetTreeKeyOffset() >> next_depth;
                // push on stack in reverse order
                for (int i = 7; i >= 0; --i) {
                    if (!top.node->HasChild(i)) { continue; }
                    OctreeKey::ComputeChildKey(i, center_offset_key, top.key, next_key);
                    // check if the child node overlaps with the AABB
                    if (OctreeKey::KeyInAabb(next_key, center_offset_key, m_aabb_min_key_, m_aabb_max_key_)) {
                        this->m_stack_.emplace_back(std::const_pointer_cast<Node>(this->m_tree_->GetNodeChild(top.node, i)), next_key, next_depth);
                    }
                }
            }
        };

        class LeafNeighborIteratorBase : public IteratorBase {
        public:
            LeafNeighborIteratorBase() = default;

            LeafNeighborIteratorBase(const ImplType *tree, unsigned int max_leaf_depth)
                : IteratorBase(tree, max_leaf_depth) {}

        protected:
            OctreeKey m_neighbor_key_ = {};
            OctreeKey::KeyType m_max_key_changing_dim1_ = 0;
            OctreeKey::KeyType m_min_key_changing_dim2_ = 0;
            OctreeKey::KeyType m_max_key_changing_dim2_ = 0;

            /**
             *
             * @param key
             * @param key_depth
             * @param changing_dim1 the dimension that is changing during the search
             * @param changing_dim2 the dimension that is changing during the search
             * @param unchanged_dim the dimension that is unchanged during the search
             * @param increase
             */
            inline void
            Init(OctreeKey key, unsigned int key_depth, int changing_dim1, int changing_dim2, int unchanged_dim, bool increase) {
                if (this->m_tree_ == nullptr) { return; }
                unsigned int max_depth = this->m_tree_->GetTreeDepth();
                unsigned int level = max_depth - key_depth;
                key = this->m_tree_->AdjustKeyToDepth(key, key_depth);
                OctreeKey::KeyType half_offset = (level == 0 ? 0 : 1 << (level - 1));
                int key_unchanged;
                if (increase) {
                    key_unchanged = key[unchanged_dim] + std::max(1, int(half_offset));
                    if (key_unchanged >= (1 << max_depth)) {  // no neighbor on east/north/top
                        this->Terminate();
                        return;
                    }
                } else {
                    key_unchanged = key[unchanged_dim] - half_offset - 1;
                    if (key_unchanged < 0) {  // no neighbor on west/south/bottom
                        this->Terminate();
                        return;
                    }
                }

                // start searching from the smallest neighbor at bottom-left corner
                this->m_neighbor_key_[unchanged_dim] = key_unchanged;
                this->m_neighbor_key_[changing_dim1] = key[changing_dim1] - half_offset;
                this->m_neighbor_key_[changing_dim2] = key[changing_dim2] - half_offset;
                this->m_max_key_changing_dim1_ = std::min(key[changing_dim1] + (level == 0 ? 1 : half_offset), 1 << max_depth);
                this->m_min_key_changing_dim2_ = this->m_neighbor_key_[changing_dim2];
                this->m_max_key_changing_dim2_ = std::min(key[changing_dim2] + (level == 0 ? 1 : half_offset), 1 << max_depth);
                this->m_stack_.resize(1);
                SingleIncrementOf(changing_dim1, changing_dim2);
            }

            inline void
            SingleIncrementOf(int changing_dim1, int changing_dim2) {
                auto &s = this->m_stack_.back();

                OctreeKey::KeyType &key_changing_dim1 = this->m_neighbor_key_[changing_dim1];
                OctreeKey::KeyType &key_changing_dim2 = this->m_neighbor_key_[changing_dim2];
                s.node = nullptr;
                while (s.node == nullptr && key_changing_dim1 < m_max_key_changing_dim1_ && key_changing_dim2 < m_max_key_changing_dim2_) {
                    s.depth = 0;
                    s.node = std::const_pointer_cast<Node>(this->m_tree_->Search(m_neighbor_key_, s.depth));
                    if (s.node == nullptr || s.depth > this->m_max_depth_) {  // not found
                        s.node = nullptr;
                        ++key_changing_dim2;
                        if (key_changing_dim2 >= m_max_key_changing_dim2_) {  // go to next row
                            key_changing_dim2 = m_min_key_changing_dim2_;
                            ++key_changing_dim1;
                        }
                        continue;
                    }

                    // found a neighbor
                    s.key = this->m_tree_->AdjustKeyToDepth(m_neighbor_key_, s.depth);
                    // avoid duplicate
                    if (s.key[changing_dim1] != key_changing_dim1) {  // node's center is not on the current row
                        s.node = nullptr;
                        ++key_changing_dim2;
                        if (key_changing_dim2 >= m_max_key_changing_dim2_) {  // go to next row
                            key_changing_dim2 = m_min_key_changing_dim2_;
                            ++key_changing_dim1;
                        }
                        continue;
                    }
                    // while loop ends here, update key_changing_dim2 to the next value
                    unsigned int max_depth = this->m_tree_->GetTreeDepth();
                    key_changing_dim2 = s.key[changing_dim2] + (s.depth == max_depth ? 1 : (1 << (max_depth - s.depth - 1)));
                }
                // check if we have reached the end
                if (s.node == nullptr && key_changing_dim1 >= m_max_key_changing_dim1_ && key_changing_dim2 >= m_max_key_changing_dim2_) { this->Terminate(); }
            }
        };

        class WestLeafNeighborIterator : public LeafNeighborIteratorBase {
            inline static const int sk_ChangingDim1_ = 1;  // changing the y-dim key during the search
            inline static const int sk_ChangingDim2_ = 2;  // changing the z-dim key during the search
            inline static const int sk_UnchangedDim_ = 0;
            inline static const bool sk_Increase_ = false;  // decrease the x-dim key

        public:
            WestLeafNeighborIterator() = default;

            WestLeafNeighborIterator(double x, double y, double z, const ImplType *tree, unsigned int max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {

                OctreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, z, key)) {
                    this->Terminate();
                    return;
                }

                unsigned int key_depth = 0;
                if (this->m_tree_->Search(key, key_depth) == nullptr) {
                    this->Terminate();
                    return;
                }

                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            WestLeafNeighborIterator(const OctreeKey &key, unsigned int key_depth, const ImplType *tree, unsigned int max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            // post-increment
            inline auto
            operator++(int) {
                const WestLeafNeighborIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            inline WestLeafNeighborIterator &
            operator++() {
                if (this->m_stack_.empty()) { return *this; }
                this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_);
                return *this;
            }
        };

        class EastLeafNeighborIterator : public LeafNeighborIteratorBase {
            inline static const int sk_ChangingDim1_ = 1;  // changing the y-dim key during the search
            inline static const int sk_ChangingDim2_ = 2;  // changing the z-dim key during the search
            inline static const int sk_UnchangedDim_ = 0;
            inline static const bool sk_Increase_ = true;  // increase the x-dim key

        public:
            EastLeafNeighborIterator() = default;

            EastLeafNeighborIterator(double x, double y, double z, const ImplType *tree, unsigned int max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {

                OctreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, z, key)) {
                    this->Terminate();
                    return;
                }

                unsigned int key_depth = 0;
                if (this->m_tree_->Search(key, key_depth) == nullptr) {
                    this->Terminate();
                    return;
                }
                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            EastLeafNeighborIterator(const OctreeKey &key, unsigned int key_depth, const ImplType *tree, unsigned int max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            // post-increment
            inline auto
            operator++(int) {
                const EastLeafNeighborIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            inline EastLeafNeighborIterator &
            operator++() {
                if (this->m_stack_.empty()) { return *this; }
                this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_);
                return *this;
            }
        };

        class NorthLeafNeighborIterator : public LeafNeighborIteratorBase {
            inline static const int sk_ChangingDim1_ = 0;  // changing the x-dim key during the search
            inline static const int sk_ChangingDim2_ = 2;  // changing the z-dim key during the search
            inline static const int sk_UnchangedDim_ = 1;
            inline static const bool sk_Increase_ = true;  // increase the y-dim key

        public:
            NorthLeafNeighborIterator() = default;

            NorthLeafNeighborIterator(double x, double y, double z, const ImplType *tree, unsigned int max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {

                OctreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, z, key)) {
                    this->Terminate();
                    return;
                }

                unsigned int key_depth = 0;
                if (this->m_tree_->Search(key, key_depth) == nullptr) {
                    this->Terminate();
                    return;
                }

                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            NorthLeafNeighborIterator(const OctreeKey &key, unsigned int key_depth, const ImplType *tree, unsigned int max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            // post-increment
            inline auto
            operator++(int) {
                const NorthLeafNeighborIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            inline NorthLeafNeighborIterator &
            operator++() {
                if (this->m_stack_.empty()) { return *this; }
                this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_);
                return *this;
            }
        };

        class SouthLeafNeighborIterator : public LeafNeighborIteratorBase {
            inline static const int sk_ChangingDim1_ = 0;  // changing the x-dim key during the search
            inline static const int sk_ChangingDim2_ = 2;  // changing the z-dim key during the search
            inline static const int sk_UnchangedDim_ = 1;
            inline static const bool sk_Increase_ = false;  // decrease the y-dim key

        public:
            SouthLeafNeighborIterator() = default;

            SouthLeafNeighborIterator(double x, double y, double z, const ImplType *tree, unsigned int max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                OctreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, z, key)) {
                    this->Terminate();
                    return;
                }

                unsigned int key_depth = 0;
                if (this->m_tree_->Search(key, key_depth) == nullptr) {
                    this->Terminate();
                    return;
                }

                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            SouthLeafNeighborIterator(const OctreeKey &key, unsigned int key_depth, const ImplType *tree, unsigned int max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            // post-increment
            inline auto
            operator++(int) {
                const SouthLeafNeighborIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            inline SouthLeafNeighborIterator &
            operator++() {
                if (this->m_stack_.empty()) { return *this; }
                this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_);
                return *this;
            }
        };

        class TopLeafNeighborIterator : public LeafNeighborIteratorBase {
            inline static const int sk_ChangingDim1_ = 0;  // changing the x-dim key during the search
            inline static const int sk_ChangingDim2_ = 1;  // changing the y-dim key during the search
            inline static const int sk_UnchangedDim_ = 2;
            inline static const bool sk_Increase_ = true;  // increase the z-dim key

        public:
            TopLeafNeighborIterator() = default;

            TopLeafNeighborIterator(double x, double y, double z, const ImplType *tree, unsigned int max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {

                OctreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, z, key)) {
                    this->Terminate();
                    return;
                }

                unsigned int key_depth = 0;
                if (this->m_tree_->Search(key, key_depth) == nullptr) {
                    this->Terminate();
                    return;
                }

                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            TopLeafNeighborIterator(const OctreeKey &key, unsigned int key_depth, const ImplType *tree, unsigned int max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            // post-increment
            inline auto
            operator++(int) {
                const TopLeafNeighborIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            inline TopLeafNeighborIterator &
            operator++() {
                if (this->m_stack_.empty()) { return *this; }
                this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_);
                return *this;
            }
        };

        class BottomLeafNeighborIterator : public LeafNeighborIteratorBase {
            inline static const int sk_ChangingDim1_ = 0;  // changing the x-dim key during the search
            inline static const int sk_ChangingDim2_ = 1;  // changing the y-dim key during the search
            inline static const int sk_UnchangedDim_ = 2;
            inline static const bool sk_Increase_ = false;  // decrease the z-dim key

        public:
            BottomLeafNeighborIterator() = default;

            BottomLeafNeighborIterator(double x, double y, double z, const ImplType *tree, unsigned int max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {

                OctreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, z, key)) {
                    this->Terminate();
                    return;
                }

                unsigned int key_depth = 0;
                if (this->m_tree_->Search(key, key_depth) == nullptr) {
                    this->Terminate();
                    return;
                }

                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            BottomLeafNeighborIterator(const OctreeKey &key, unsigned int key_depth, const ImplType *tree, unsigned int max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            // post-increment
            inline auto
            operator++(int) {
                const BottomLeafNeighborIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            inline BottomLeafNeighborIterator &
            operator++() {
                if (this->m_stack_.empty()) { return *this; }
                this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_);
                return *this;
            }
        };

        class LeafOnRayIterator : public IteratorBase {
            Eigen::Vector3d m_origin_ = {};
            Eigen::Vector3d m_dir_ = {};
            Eigen::Vector3d m_dir_inv_ = {};
            double m_max_range_ = 0.;
            bool m_bidirectional_ = false;

        public:
            LeafOnRayIterator() = default;

            LeafOnRayIterator(
                double px,  //
                double py,
                double pz,
                double vx,
                double vy,
                double vz,
                double max_range,
                bool bidirectional,
                const ImplType *tree,
                unsigned int max_leaf_depth
            )
                : IteratorBase(tree, max_leaf_depth),
                  m_origin_(px, py, pz),
                  m_dir_(vx, vy, vz),
                  m_max_range_(max_range),
                  m_bidirectional_(bidirectional) {

                m_dir_inv_ = m_dir_.cwiseInverse();
                this->m_stack_.back().data = std::make_shared<double>(0.);
                this->SingleIncrement();
            }

            [[nodiscard]] inline double
            GetDistance() const {
                if (this->m_stack_.empty()) { return 0.; }
                return *std::reinterpret_pointer_cast<double>(this->m_stack_.back().data);
            }

            // post-increment
            LeafOnRayIterator
            operator++(int) {
                const LeafOnRayIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            LeafOnRayIterator &
            operator++() {
                if (this->m_stack_.empty()) { return *this; }
                this->SingleIncrement();
                return *this;
            }

        protected:
            struct StackElementCompare {  // for min-heap

                inline bool
                operator()(const typename IteratorBase::StackElement &lhs, const typename IteratorBase::StackElement &rhs) const {
                    double lhs_dist = std::abs(*std::reinterpret_pointer_cast<double>(lhs.data));
                    double rhs_dist = std::abs(*std::reinterpret_pointer_cast<double>(rhs.data));
                    return lhs_dist < rhs_dist;
                }
            };

            void
            SingleIncrement() {
                StackElementCompare cmp;
                auto s = this->m_stack_.back();  // s.node is leaf node unless it is root node
                this->m_stack_.pop_back();
                std::pop_heap(this->m_stack_.begin(), this->m_stack_.end(), cmp);

                if (!s.node->HasAnyChild()) {      // leaf node
                    if (this->m_stack_.empty()) {  // end of iteration
                        this->Terminate();
                        return;
                    } else {
                        if (this->IsLeaf()) {
                            return;  // current stack top is leaf node
                        } else {     // current stack top is internal node
                            s = this->m_stack_.back();
                            this->m_stack_.pop_back();
                            std::pop_heap(this->m_stack_.begin(), this->m_stack_.end(), cmp);
                        }
                    }
                }

                // now s.node is internal node
                while (true) {
                    typename IteratorBase::StackElement s_child;
                    s_child.depth = s.depth + 1;
                    OctreeKey::KeyType center_offset_key = this->m_tree_->GetTreeKeyOffset() >> s_child.depth;
                    for (int i = 7; i >= 0; --i) {
                        if (s_child.depth > this->m_max_depth_) { continue; }
                        if (!s.node->HasChild(i)) { continue; }
                        double dist = 0.;
                        bool intersected = false;
                        auto child = this->m_tree_->GetNodeChild(s.node, i);
                        OctreeKey::ComputeChildKey(i, center_offset_key, s.key, s_child.key);

                        double center_x = this->m_tree_->KeyToCoord(s_child.key[0], s_child.depth);
                        double center_y = this->m_tree_->KeyToCoord(s_child.key[1], s_child.depth);
                        double center_z = this->m_tree_->KeyToCoord(s_child.key[2], s_child.depth);
                        double half_size = this->m_tree_->GetNodeSize(s_child.depth) / 2.0;
                        Eigen::Vector3d box_min(center_x - half_size, center_y - half_size, center_z - half_size);
                        Eigen::Vector3d box_max(center_x + half_size, center_y + half_size, center_z + half_size);
                        ComputeIntersectionBetweenRayAndAabb3D(m_origin_, m_dir_inv_, box_min, box_max, dist, intersected);
                        if (!intersected) { continue; }
                        if (!child->HasAnyChild()) {
                            if (!m_bidirectional_ && dist < 0.) { continue; }
                            if (m_max_range_ > 0. && std::abs(dist) > m_max_range_) { continue; }
                        }
                        s_child.node = std::const_pointer_cast<Node>(child);
                        s_child.data = std::make_shared<double>(dist);
                        this->m_stack_.push_back(s_child);
                        std::push_heap(this->m_stack_.begin(), this->m_stack_.end(), cmp);
                    }
                    if (this->m_stack_.empty()) {  // end of iteration
                        this->Terminate();
                        return;
                    }
                    if (this->IsLeaf()) {
                        return;  // current stack top is leaf node
                    } else {     // current stack top is internal node
                        s = this->m_stack_.back();
                        this->m_stack_.pop_back();
                        std::pop_heap(this->m_stack_.begin(), this->m_stack_.end(), cmp);
                    }
                }
            }
        };

        /**
         * Iterator that iterates over all leaf nodes.
         * @param max_depth
         * @return
         */
        [[nodiscard]] inline LeafIterator
        begin(unsigned int max_depth = 0) const {
            return LeafIterator(this, max_depth);
        }

        /**
         * End iterator of LeafIterator.
         */
        [[nodiscard]] inline LeafIterator
        end() const {
            return LeafIterator();
        }

        /**
         * Iterator that iterates over all leaf nodes.
         * @param max_depth
         * @return
         */
        [[nodiscard]] inline LeafIterator
        BeginLeaf(unsigned int max_depth = 0) const {
            return LeafIterator(this, max_depth);
        }

        /**
         * End iterator of LeafIterator.
         */
        [[nodiscard]] inline LeafIterator
        EndLeaf() const {
            return LeafIterator();
        }

        [[nodiscard]] inline LeafOfNodeIterator
        BeginLeafOfNode(OctreeKey key, unsigned int node_depth, unsigned int max_depth = 0) const {
            return LeafOfNodeIterator(key, node_depth, this, max_depth);
        }

        [[nodiscard]] inline LeafOfNodeIterator
        EndLeafOfNode() const {
            return LeafOfNodeIterator();
        }

        [[nodiscard]] inline LeafInAabbIterator
        BeginLeafInAabb(
            double aabb_mix_x,  //
            double aabb_min_y,
            double aabb_min_z,
            double aabb_max_x,
            double aabb_max_y,
            double aabb_max_z,
            unsigned int max_depth = 0
        ) const {
            return LeafInAabbIterator(aabb_mix_x, aabb_min_y, aabb_min_z, aabb_max_x, aabb_max_y, aabb_max_z, this, max_depth);
        }

        [[nodiscard]] inline LeafInAabbIterator
        BeginLeafInAabb(const OctreeKey &aabb_min_key, const OctreeKey &aabb_max_key, unsigned int max_depth = 0) const {
            return LeafInAabbIterator(aabb_min_key, aabb_max_key, this, max_depth);
        }

        [[nodiscard]] inline LeafInAabbIterator
        EndLeafInAabb() const {
            return LeafInAabbIterator();
        }

        [[nodiscard]] inline TreeIterator
        BeginTree(unsigned int max_depth = 0) const {
            return TreeIterator(this, max_depth);
        }

        [[nodiscard]] inline TreeIterator
        EndTree() const {
            return TreeIterator();
        }

        [[nodiscard]] inline TreeInAabbIterator
        BeginTreeInAabb(
            double aabb_mix_x,  //
            double aabb_min_y,
            double aabb_min_z,
            double aabb_max_x,
            double aabb_max_y,
            double aabb_max_z,
            unsigned int max_depth = 0
        ) const {
            return TreeInAabbIterator(aabb_mix_x, aabb_min_y, aabb_min_z, aabb_max_x, aabb_max_y, aabb_max_z, this, max_depth);
        }

        [[nodiscard]] inline TreeInAabbIterator
        BeginTreeInAabb(const OctreeKey &aabb_min_key, const OctreeKey &aabb_max_key, unsigned int max_depth = 0) const {
            return TreeInAabbIterator(aabb_min_key, aabb_max_key, this, max_depth);
        }

        [[nodiscard]] inline TreeInAabbIterator
        EndTreeInAabb() const {
            return TreeInAabbIterator();
        }

        [[nodiscard]] inline WestLeafNeighborIterator
        BeginWestLeafNeighbor(double x, double y, double z, unsigned int max_leaf_depth = 0) const {
            return WestLeafNeighborIterator(x, y, z, this, max_leaf_depth);
        }

        [[nodiscard]] inline WestLeafNeighborIterator
        BeginWestLeafNeighbor(const OctreeKey &key, unsigned int key_depth, unsigned int max_leaf_depth = 0) const {
            return WestLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[nodiscard]] inline WestLeafNeighborIterator
        EndWestLeafNeighbor() const {
            return WestLeafNeighborIterator();
        }

        [[nodiscard]] inline EastLeafNeighborIterator
        BeginEastLeafNeighbor(double x, double y, double z, unsigned int max_leaf_depth = 0) const {
            return EastLeafNeighborIterator(x, y, z, this, max_leaf_depth);
        }

        [[nodiscard]] inline EastLeafNeighborIterator
        BeginEastLeafNeighbor(const OctreeKey &key, unsigned int key_depth, unsigned int max_leaf_depth = 0) const {
            return EastLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[nodiscard]] inline EastLeafNeighborIterator
        EndEastLeafNeighbor() const {
            return EastLeafNeighborIterator();
        }

        [[nodiscard]] inline NorthLeafNeighborIterator
        BeginNorthLeafNeighbor(double x, double y, double z, unsigned int max_leaf_depth = 0) const {
            return NorthLeafNeighborIterator(x, y, z, this, max_leaf_depth);
        }

        [[nodiscard]] inline NorthLeafNeighborIterator
        BeginNorthLeafNeighbor(const OctreeKey &key, unsigned int key_depth, unsigned int max_leaf_depth = 0) const {
            return NorthLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[nodiscard]] inline NorthLeafNeighborIterator
        EndNorthLeafNeighbor() const {
            return NorthLeafNeighborIterator();
        }

        [[nodiscard]] inline SouthLeafNeighborIterator
        BeginSouthLeafNeighbor(double x, double y, double z, unsigned int max_leaf_depth = 0) const {
            return SouthLeafNeighborIterator(x, y, z, this, max_leaf_depth);
        }

        [[nodiscard]] inline SouthLeafNeighborIterator
        BeginSouthLeafNeighbor(const OctreeKey &key, unsigned int key_depth, unsigned int max_leaf_depth = 0) const {
            return SouthLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[nodiscard]] inline SouthLeafNeighborIterator
        EndSouthLeafNeighbor() const {
            return SouthLeafNeighborIterator();
        }

        [[nodiscard]] inline TopLeafNeighborIterator
        BeginTopLeafNeighbor(double x, double y, double z, unsigned int max_leaf_depth = 0) const {
            return TopLeafNeighborIterator(x, y, z, this, max_leaf_depth);
        }

        [[nodiscard]] inline TopLeafNeighborIterator
        BeginTopLeafNeighbor(const OctreeKey &key, unsigned int key_depth, unsigned int max_leaf_depth = 0) const {
            return TopLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[nodiscard]] inline TopLeafNeighborIterator
        EndTopLeafNeighbor() const {
            return TopLeafNeighborIterator();
        }

        [[nodiscard]] inline BottomLeafNeighborIterator
        BeginBottomLeafNeighbor(double x, double y, double z, unsigned int max_leaf_depth = 0) const {
            return BottomLeafNeighborIterator(x, y, z, this, max_leaf_depth);
        }

        [[nodiscard]] inline BottomLeafNeighborIterator
        BeginBottomLeafNeighbor(const OctreeKey &key, unsigned int key_depth, unsigned int max_leaf_depth = 0) const {
            return BottomLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[nodiscard]] inline BottomLeafNeighborIterator
        EndBottomLeafNeighbor() const {
            return BottomLeafNeighborIterator();
        }

        [[nodiscard]] inline LeafOnRayIterator
        BeginLeafOnRay(
            double px,  //
            double py,
            double pz,
            double vx,
            double vy,
            double vz,
            double max_range = -1,
            bool bidirectional = false,
            unsigned int max_leaf_depth = 0
        ) const {
            return LeafOnRayIterator(px, py, pz, vx, vy, vz, max_range, bidirectional, this, max_leaf_depth);
        }

        [[nodiscard]] inline LeafOnRayIterator
        EndLeafOnRay() const {
            return LeafOnRayIterator();
        }

        //-- ray tracing
        /**
         * Trace a ray from origin to end (excluded), returning a OctreeKeyRay of all nodes' OctreeKey traversed by the ray. For each key, the
         * corresponding node may not exist. You can use Search() to check if the node exists.
         * @param sx x coordinate of the origin
         * @param sy y coordinate of the origin
         * @param sz z coordinate of the origin
         * @param ex x coordinate of the end (excluded)
         * @param ey y coordinate of the end (excluded)
         * @param ez z coordinate of the end (excluded)
         * @param ray
         * @return
         */
        [[nodiscard]] bool
        ComputeRayKeys(double sx, double sy, double sz, double ex, double ey, double ez, OctreeKeyRay &ray) const {
            // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo Digital Difference Analyzer (DDA) algorithm
            // Note that we cannot use Bresenham's line algorithm because it may miss some voxels when the ray is not axis-aligned.
            // For example, if the ray is from (0, 0) to (1, 1), Bresenham's algorithm will miss (1, 0) and (0, 1) but the ray should traverse them.
            // Also look at https://en.wikipedia.org/wiki/File:Bresenham.svg for another example of Bresenham's algorithm.

            ray.Reset();
            OctreeKey key_start, key_end;
            if (!CoordToKeyChecked(sx, sy, sz, key_start) || !CoordToKeyChecked(ex, ey, ez, key_end)) {
                ERL_WARN("Ray (%f, %f, %f) -> (%f, %f, %f) is out of range.\n", sx, sy, sz, ex, ey, ez);
                return false;
            }
            if (key_start == key_end) { return true; }

            // initialization phase
            double direction[3];
            double &dx = direction[0];
            double &dy = direction[1];
            double &dz = direction[2];
            dx = ex - sx;
            dy = ey - sy;
            dz = ez - sz;
            double length = std::sqrt(dx * dx + dy * dy + dz * dz);
            dx /= length;
            dy /= length;
            dz /= length;

            // compute step direction
            int step[3];
            if (dx > 0) {
                step[0] = 1;
            } else if (dx < 0) {
                step[0] = -1;
            } else {
                step[0] = 0;
            }
            if (dy > 0) {
                step[1] = 1;
            } else if (dy < 0) {
                step[1] = -1;
            } else {
                step[1] = 0;
            }
            if (dz > 0) {
                step[2] = 1;
            } else if (dz < 0) {
                step[2] = -1;
            } else {
                step[2] = 0;
            }
            if (step[0] == 0 && step[1] == 0 && step[2] == 0) {
                ERL_WARN("Ray casting in direction (0, 0, 0) is impossible!");
                return false;
            }

            // compute t_max and t_delta
            double t_max[3];
            double t_delta[3];
            if (step[0] == 0) {
                t_max[0] = std::numeric_limits<double>::infinity();
                t_delta[0] = std::numeric_limits<double>::infinity();
            } else {
                double voxel_border = KeyToCoord(key_start[0]) + double(step[0]) * 0.5 * m_resolution_;
                t_max[0] = (voxel_border - sx) / dx;
                t_delta[0] = m_resolution_ / std::abs(dx);
            }
            if (step[1] == 0) {
                t_max[1] = std::numeric_limits<double>::infinity();
                t_delta[1] = std::numeric_limits<double>::infinity();
            } else {
                double voxel_border = KeyToCoord(key_start[1]) + double(step[1]) * 0.5 * m_resolution_;
                t_max[1] = (voxel_border - sy) / dy;
                t_delta[1] = m_resolution_ / std::abs(dy);
            }
            if (step[2] == 0) {
                t_max[2] = std::numeric_limits<double>::infinity();
                t_delta[2] = std::numeric_limits<double>::infinity();
            } else {
                double voxel_border = KeyToCoord(key_start[2]) + double(step[2]) * 0.5 * m_resolution_;
                t_max[2] = (voxel_border - sz) / dz;
                t_delta[2] = m_resolution_ / std::abs(dz);
            }

            // incremental phase
            ray.AddKey(key_start);
            OctreeKey current_key = key_start;
            while (true) {
                int idx = 0;
                if (t_max[1] < t_max[0]) { idx = 1; }
                if (t_max[2] < t_max[idx]) { idx = 2; }

                t_max[idx] += t_delta[idx];
                current_key[idx] += step[idx];
                ERL_DEBUG_ASSERT(
                    current_key[idx] < (mk_TreeKeyOffset_ << 1), "current_key[%d] = %d exceeds limit %d.\n", idx, current_key[idx], (mk_TreeKeyOffset_ << 1)
                );

                if (current_key == key_end) { break; }

                // this seems to be unlikely to happen
                double dist_from_origin = std::min(t_max[0], std::min(t_max[1], t_max[2]));
                if (dist_from_origin > length) { break; }  // this happens due to numerical error

                ray.AddKey(current_key);
                ERL_ASSERTM(ray.size() < ray.capacity() - 1, "Ray capacity is not enough.");
            }

            return true;
        }

        /**
         * Trace a ray from origin to end (excluded), returning a list of all nodes' coordinates traversed by the ray. For each coordinate, the corresponding
         * node may not exist. You can use Search() to check if the node exists.
         * @param sx x coordinate of the origin
         * @param sy y coordinate of the origin
         * @param sz z coordinate of the origin
         * @param ex x coordinate of the end (excluded)
         * @param ey y coordinate of the end (excluded)
         * @param ez z coordinate of the end (excluded)
         * @param ray
         * @return
         */
        [[nodiscard]] bool
        ComputeRayCoords(double sx, double sy, double sz, double ex, double ey, double ez, std::vector<std::array<double, 3>> &ray) const {
            ray.clear();
            OctreeKeyRay key_ray;
            if (!ComputeRayKeys(sx, sy, sz, ex, ey, ez, key_ray)) { return false; }
            ray.reserve(key_ray.size());
            for (auto &key: key_ray) { ray.emplace_back(std::array<double, 3>{KeyToCoord(key[0]), KeyToCoord(key[1]), KeyToCoord(key[2])}); }
            return true;
        }

        /**
         * Clear KeyRay vector to minimize unneeded memory. This is only useful for the StaticMemberInitializer classes, don't call it for a quadtree that is
         * actually used.
         */
        inline void
        ClearKeyRays() {
            m_key_rays_.clear();
        }

        //-- tree structure operations
        /**
         * Create a new child node for the given node.
         * @param node
         * @param child_idx
         * @return
         */
        inline std::shared_ptr<Node>
        CreateNodeChild(const std::shared_ptr<Node> &node, unsigned int child_idx) {
            node->AllocateChildrenPtr();                                            // allocate children if necessary
            ERL_DEBUG_ASSERT(!node->HasChild(child_idx), "Child already exists.");  // child must not exist
            auto new_child = std::make_shared<Node>();                              // create new child
            node->SetChild(new_child, child_idx);                                   // set child
            m_tree_size_++;                                                         // increase tree size
            m_size_changed_ = true;                                                 // size of the tree has changed
            return new_child;
        }

        /**
         * Delete a child node of the given node.
         * @param node
         * @param child_idx
         * @return
         */
        inline void
        DeleteNodeChild(std::shared_ptr<Node> &node, unsigned int child_idx) {
            ERL_DEBUG_ASSERT(node->HasChild(child_idx), "Child does not exist.");  // child must exist
            node->SetChild(nullptr, child_idx);                                    // set child to nullptr
            m_tree_size_--;                                                        // decrease tree size
            m_size_changed_ = true;                                                // size of the tree has changed
        }

        /**
         * Get a child node of the given node. Before calling this function, make sure node->HasChildrenPtr() or node->HasAnyChild() returns true.
         * @param node
         * @param child_idx
         * @return
         */
        inline std::shared_ptr<Node>
        GetNodeChild(std::shared_ptr<Node> &node, unsigned int child_idx) {
            return node->template GetChild<Node>(child_idx);
        }

        /**
         * Get a child node of the given node. Before calling this function, make sure node->HasChildrenPtr() or node->HasAnyChild() returns true.
         * @param node
         * @param child_idx
         * @return
         */
        inline std::shared_ptr<const Node>
        GetNodeChild(const std::shared_ptr<const Node> &node, unsigned int child_idx) const {
            return node->template GetChild<Node>(child_idx);
        }

        /**
         * Check if a node is collapsible. For example, for occupancy quadtree, a node is collapsible if all its children exist, none of them have its own
         * children, and they all have the same occupancy value.
         * @param node
         * @return
         */
        virtual bool
        IsNodeCollapsible(const std::shared_ptr<Node> &node) const = 0;

        /**
         * Expand a node: all children are created and their data is copied from the parent.
         * @param node
         * @return
         */
        virtual void
        ExpandNode(std::shared_ptr<Node> &node) {
            ERL_DEBUG_ASSERT(!node->HasAnyChild(), "Node already has children.");
            for (unsigned int i = 0; i < 8; ++i) {
                auto child = CreateNodeChild(node, i);
                OnExpandNode(node, child);
            }
        }

        virtual void
        OnExpandNode(std::shared_ptr<Node> &node, std::shared_ptr<Node> &child) = 0;

        /**
         * Prune a node: delete all children if the node is collapsible.
         * @param node
         * @return
         */
        virtual bool
        PruneNode(std::shared_ptr<Node> &node) {
            if (!IsNodeCollapsible(node)) { return false; }

            OnPruneNode(node);          // call prune node callback
            node->DeleteChildrenPtr();  // delete children
            m_tree_size_ -= 8;
            m_size_changed_ = true;

            return true;
        }

        virtual void
        OnPruneNode(std::shared_ptr<Node> &node) = 0;

        /**
         * Delete a node at the given depth given a point if it exists. Will always delete at the lowest level unless depth != 0, and expand pruned inner nodes
         * as needed. Pruned nodes at depth "depth" will directly be deleted as a whole.
         * @param x
         * @param y
         * @param z
         * @param depth
         * @return
         */
        inline bool
        DeleteNode(double x, double y, double z, unsigned int depth = 0) {
            OctreeKey key;
            if (!CoordToKeyChecked(x, y, z, key)) {
                ERL_WARN("Point (%f, %f, %f) is out of range.", x, y, z);
                return false;
            } else {
                return DeleteNode(key, depth);
            }
        }

        /**
         * Delete a node at the given depth given a key if it exists. Will always delete at the lowest level unless depth != 0, and expand pruned inner nodes
         * as needed. Pruned nodes at depth "depth" will directly be deleted as a whole.
         * @param key
         * @param depth
         * @return
         */
        inline bool
        DeleteNode(const OctreeKey &key, unsigned int depth = 0) {
            if (m_root_ == nullptr) { return true; }
            if (depth == 0) { depth = mk_TreeDepth_; }
            return DeleteNode(m_root_, key, 0, depth);
        }

        virtual void
        OnDeleteNodeChild(std::shared_ptr<Node> &node, unsigned int child_idx) = 0;

    protected:
        /**
         * Delete child nodes down to max_depth matching the given key of the given node that is at the given depth.
         * @param node node at depth, must not be nullptr and it will not be deleted
         * @param key
         * @param depth
         * @param max_depth
         * @return
         */
        bool
        DeleteNode(std::shared_ptr<Node> node, const OctreeKey &key, unsigned int depth, unsigned int max_depth) {
            if (depth >= max_depth) { return true; }
            ERL_DEBUG_ASSERT(node != nullptr, "node should not be nullptr.\n");

            using Element = std::pair<std::shared_ptr<Node>, unsigned int>;  // node, pos
            std::vector<Element> child_nodes_to_delete;
            child_nodes_to_delete.reserve(max_depth - depth);

            // find nodes to delete up to max_depth
            while (depth < max_depth) {
                unsigned int pos = OctreeKey::ComputeChildIndex(key, mk_TreeDepth_ - depth - 1);
                if (!node->HasChild(pos)) {                         // child does not exist, but maybe node is pruned
                    if (!node->HasAnyChild() && node != m_root_) {  // this node is pruned
                        ExpandNode(node);                           // expand it, tree size is updated in ExpandNode
                    } else {                                        // node is not pruned, we are done
                        return false;                               // nothing to delete
                    }
                }

                child_nodes_to_delete.emplace_back(node, pos);  // delete child node at pos if it becomes leaf node later
                node = GetNodeChild(node, pos);                 // go to child node
                ++depth;                                        // go to next depth
            }

            // delete nodes
            ERL_DEBUG_ASSERT(!child_nodes_to_delete.empty(), "child_nodes_to_delete should not be empty.\n");
            for (auto it = child_nodes_to_delete.rbegin(); it != child_nodes_to_delete.rend(); ++it) {
                node = it->first;
                unsigned int pos = it->second;
                DeleteNodeChild(node, pos);
                if (node->HasAnyChild()) {  // stop if node is not pruned, i.e. still has children
                    OnDeleteNodeChild(node, pos);
                    break;
                }
            }

            return true;
        }

        /**
         * Delete all children of a node.
         * @param node
         * @return
         */
        inline void
        DeleteNode(std::shared_ptr<Node> node) {
            ERL_ASSERTM(node != nullptr, "node should not be nullptr.\n");
            if (!node->HasAnyChild()) { return; }

            using Element = std::pair<std::shared_ptr<Node>, bool>;  // node, expanded
            std::vector<Element> node_stack;
            node_stack.emplace_back(node, false);
            while (!node_stack.empty()) {
                auto &pair = node_stack.back();
                node = pair.first;
                bool &expanded = pair.second;

                if (expanded) {  // node's descendants have been all visited
                    node_stack.pop_back();
                    m_tree_size_ -= node->GetNumChildren();
                    m_size_changed_ = true;
                    node->DeleteChildrenPtr();
                } else if (node->HasAnyChild()) {  // node has children, expand it
                    for (unsigned int i = 0; i < 8; ++i) {
                        auto child = GetNodeChild(node, i);
                        if (child != nullptr && child->HasAnyChild()) { node_stack.emplace_back(child, false); }
                    }
                    expanded = true;
                } else {
                    ERL_ERROR("node should have children.\n");
                }
            }
        }

    public:
        /**
         * Delete the whole tree.
         * @return
         */
        void
        Clear() override {
            if (m_root_ == nullptr) { return; }

            DeleteNode(m_root_);
            m_root_ = nullptr;
            m_tree_size_ = 0;
            m_size_changed_ = true;
        }

        /**
         * Lossless compression of the tree: a node will replace all of its children if the node is collapsible.
         */
        void
        Prune() override {
            if (m_root_ == nullptr) { return; }

            // depth first
            struct StackElement {
                std::shared_ptr<Node> node = nullptr;
                unsigned int depth = 0;
                bool expanded = false;

                StackElement() = default;

                StackElement(std::shared_ptr<Node> node, unsigned int depth, bool expanded)
                    : node(std::move(node)),
                      depth(depth),
                      expanded(expanded) {}
            };

            std::list<StackElement> stack;
            stack.emplace_back(m_root_, 0, false);
            while (!stack.empty()) {
                auto &s = stack.back();

                if (s.expanded) {  // node's descendants have been all visited
                    PruneNode(s.node);
                    stack.pop_back();
                    continue;
                }

                if (s.depth < mk_TreeDepth_ - 1) {  // unexpanded inner node has four children, expand it
                    // s changed when emplace_back, so we modify it at first
                    s.expanded = true;
                    unsigned int child_depth = s.depth + 1;
                    bool expanded = (child_depth == mk_TreeDepth_ - 1);
                    auto node = s.node;
                    for (unsigned int i = 0; i < 8; ++i) {
                        auto child = GetNodeChild(node, i);
                        if (child != nullptr && child->HasAnyChild()) { stack.emplace_back(child, child_depth, expanded); }
                    }
                    continue;
                }

                stack.pop_back();
            }
        }

        /**
         * Expand all pruned nodes (reverse operation of Prune).
         * @attention This is an expensive operation, especially when the tree is nearly empty!
         */
        virtual void
        Expand() {
            if (m_root_ == nullptr) { return; }

            using Element = std::pair<std::shared_ptr<Node>, unsigned int>;  // node, depth
            std::vector<Element> nodes_stack;
            nodes_stack.emplace_back(m_root_, 0);

            while (!nodes_stack.empty()) {
                auto &pair = nodes_stack.back();
                nodes_stack.pop_back();
                std::shared_ptr<Node> node = pair.first;
                unsigned int node_depth = pair.second;

                if (!node->HasAnyChild()) { ExpandNode(node); }  // node has no children, expand it
                unsigned int next_depth = node_depth + 1;
                if (next_depth == mk_TreeDepth_) { continue; }  // will reach the max depth, skip it
                for (unsigned int i = 0; i < 8; ++i) {
                    auto child = GetNodeChild(node, i);
                    if (child != nullptr) { nodes_stack.emplace_back(child, next_depth); }
                }
            }
        }

    public:
        inline const std::shared_ptr<Node> &
        GetRoot() const {
            return m_root_;
        }

        //-- Search functions

        inline std::shared_ptr<Node>
        Search(double x, double y, double z) {
            unsigned int depth = 0;
            return Search(x, y, z, depth);
        }

        [[nodiscard]] inline std::shared_ptr<const Node>
        Search(double x, double y, double z) const {
            unsigned int depth = 0;
            return Search(x, y, z, depth);
        }

        /**
         * Search node at specified depth given a point.
         * @param x
         * @param y
         * @param z
         * @param depth specially, depth=0 means searching from root. The output value indicates the depth of the returned node. If -1, search is failed.
         * @return
         */
        [[nodiscard]] inline std::shared_ptr<Node>
        Search(double x, double y, double z, unsigned int &depth) {
            OctreeKey key;
            if (!CoordToKeyChecked(x, y, z, key)) {
                ERL_WARN("Point (%f, %f, %f) is out of range.\n", x, y, z);
                return nullptr;
            }

            return Search(key, depth);
        }

        [[nodiscard]] inline std::shared_ptr<const Node>
        Search(double x, double y, double z, unsigned int &depth) const {
            OctreeKey key;
            if (!CoordToKeyChecked(x, y, z, key)) {
                ERL_WARN("Point (%f, %f, %f) is out of range.\n", x, y, z);
                return nullptr;
            }

            return Search(key, depth);
        }

        inline std::shared_ptr<Node>
        Search(const OctreeKey &key) {
            unsigned int depth = 0;
            return Search(key, depth);
        }

        [[nodiscard]] inline std::shared_ptr<const Node>
        Search(const OctreeKey &key) const {
            unsigned int depth = 0;
            return Search(key, depth);
        }

        /**
         * Search node at specified depth given a key.
         * @param key
         * @param depth specially, depth=0 means searching from root. The output value indicates the depth of the returned node. If -1, search is failed.
         * @return
         */
        inline std::shared_ptr<Node>
        Search(const OctreeKey &key, unsigned int &depth) {
            ERL_ASSERTM(depth <= mk_TreeDepth_, "Depth must be in [0, %u], but got %u.\n", mk_TreeDepth_, depth);

            if (m_root_ == nullptr) { return nullptr; }
            if (depth == 0) { depth = mk_TreeDepth_; }

            // generate appropriate key for the given depth
            OctreeKey key_at_depth = key;
            if (depth != mk_TreeDepth_) { key_at_depth = AdjustKeyToDepth(key_at_depth, depth); }

            // search
            std::shared_ptr<Node> node = m_root_;
            int diff = mk_TreeDepth_ - depth;
            // follow nodes down to the requested level (for level = 0, it is the leaf level)
            for (int level = mk_TreeDepth_ - 1; level >= diff; --level) {
                unsigned int child_index = OctreeKey::ComputeChildIndex(key_at_depth, level);
                if (node->HasChild(child_index)) {
                    node = GetNodeChild(node, child_index);
                    depth = mk_TreeDepth_ - level;
                } else {
                    // we expect a child but did not get it, is the current node a leaf?
                    if (!node->HasAnyChild()) {
                        // current node is a leaf, so we cannot go deeper
                        depth = mk_TreeDepth_ - level - 1;
                        return node;
                    } else {
                        // current node is not a leaf, search failed
                        depth = -1;
                        return nullptr;
                    }
                }
            }
            return node;
        }

        [[nodiscard]] inline std::shared_ptr<const Node>
        Search(const OctreeKey &key, unsigned int &depth) const {
            ERL_ASSERTM(depth <= mk_TreeDepth_, "Depth must be in [0, %u], but got %u.\n", mk_TreeDepth_, depth);

            if (m_root_ == nullptr) { return nullptr; }
            if (depth == 0) { depth = mk_TreeDepth_; }

            // generate appropriate key for the given depth
            OctreeKey key_at_depth = key;
            if (depth != mk_TreeDepth_) { key_at_depth = AdjustKeyToDepth(key_at_depth, depth); }

            // search
            std::shared_ptr<const Node> node = m_root_;
            int diff = mk_TreeDepth_ - depth;
            // follow nodes down to the requested level (for level = 0, it is the leaf level)
            for (int level = mk_TreeDepth_ - 1; level >= diff; --level) {
                unsigned int child_index = OctreeKey::ComputeChildIndex(key_at_depth, level);
                if (node->HasChild(child_index)) {
                    node = GetNodeChild(node, child_index);
                    depth = mk_TreeDepth_ - level;
                } else {
                    // we expect a child but did not get it, is the current node a leaf?
                    if (!node->HasAnyChild()) {
                        // current node is a leaf, so we cannot go deeper
                        depth = mk_TreeDepth_ - level - 1;
                        return node;
                    } else {
                        // current node is not a leaf, search failed
                        depth = -1;
                        return nullptr;
                    }
                }
            }
            return node;
        }

        inline std::shared_ptr<Node>
        InsertNode(double x, double y, double z, unsigned int depth = 0) {
            OctreeKey key;
            if (!CoordToKeyChecked(x, y, z, key)) {
                ERL_WARN("Point (%f, %f, %f) is out of range.\n", x, y, z);
                return nullptr;
            }
            return InsertNode(key, depth);
        }

        inline std::shared_ptr<Node>
        InsertNode(const OctreeKey &key, unsigned int depth = 0) {
            if (depth == 0) { depth = mk_TreeDepth_; }
            ERL_ASSERTM(depth <= mk_TreeDepth_, "Depth must be in [0, %u], but got %u.\n", mk_TreeDepth_, depth);
            if (this->m_root_ == nullptr) {
                this->m_root_ = std::make_shared<Node>();
                this->m_tree_size_++;
            }

            std::shared_ptr<Node> node = this->m_root_;
            int diff = mk_TreeDepth_ - depth;
            for (int level = mk_TreeDepth_ - 1; level >= diff; --level) {
                unsigned int child_index = OctreeKey::ComputeChildIndex(key, level);
                if (node->HasChild(child_index)) {
                    node = GetNodeChild(node, child_index);
                } else {
                    node = CreateNodeChild(node, child_index);
                }
            }

            return node;
        }

        //-- file IO
        /**
         * Read all nodes from the input stream (without file header). For general file IO, use AbstractOctree::Read.
         */
        std::istream &
        ReadData(std::istream &s) override {
            if (!s.good()) {
                ERL_WARN("Input stream is not good.\n");
                return s;
            }

            m_tree_size_ = 0;
            m_size_changed_ = true;

            if (m_root_ != nullptr) {
                ERL_WARN("Octree is not empty, clear it first.\n");
                return s;
            }

            m_root_ = std::make_shared<Node>();
            m_tree_size_++;
            std::vector<std::shared_ptr<Node>> nodes_stack;
            nodes_stack.emplace_back(m_root_);
            while (!nodes_stack.empty()) {
                std::shared_ptr<Node> node = nodes_stack.back();
                nodes_stack.pop_back();

                // load node data
                node->ReadData(s);

                // load children
                char children_char;
                s.read(&children_char, sizeof(char));
                std::bitset<8> children((unsigned long long) children_char);
                node->AllocateChildrenPtr();
                for (int i = 7; i >= 0; --i) {  // the same order as the recursive implementation
                    if (!children[i]) { continue; }
                    auto child_node = std::make_shared<Node>();
                    node->SetChild(child_node, i);
                    nodes_stack.emplace_back(child_node);
                    m_tree_size_++;
                }
            }

            return s;
        }

        std::ostream &
        WriteData(std::ostream &s) const override {
            if (m_root_ == nullptr) { return s; }

            std::vector<std::shared_ptr<const Node>> nodes_stack;
            nodes_stack.emplace_back(m_root_);
            while (!nodes_stack.empty()) {
                auto node = nodes_stack.back();
                nodes_stack.pop_back();

                // write node data
                node->WriteData(s);

                // write children
                std::bitset<8> children;
                for (int i = 7; i >= 0; --i) {  // the same order as the recursive implementation
                    if (node->HasChild(i)) {
                        children[i] = true;
                        nodes_stack.emplace_back(GetNodeChild(node, i));
                    } else {
                        children[i] = false;
                    }
                }
                auto children_char = char(children.to_ulong());
                s.write(&children_char, sizeof(char));
            }

            return s;
        }

    protected:
        /**
         * Constructor to enable derived classes to change tree constants.
         * @param resolution
         * @param tree_depth
         * @param tree_key_offset
         */
        OctreeImpl(double resolution, unsigned int tree_depth, unsigned int tree_key_offset)
            : mk_TreeDepth_(tree_depth),
              mk_TreeKeyOffset_(tree_key_offset),
              m_resolution_(resolution) {
            Init();
        }

        inline void
        Init() {
            SetResolution(m_resolution_);
            m_metric_min_[0] = m_metric_min_[1] = m_metric_min_[2] = std::numeric_limits<double>::infinity();
            m_metric_max_[0] = m_metric_max_[1] = m_metric_max_[2] = -std::numeric_limits<double>::infinity();
            m_size_changed_ = true;
#pragma omp parallel default(none) shared(m_key_rays_)
#pragma omp critical
            {
                // do it on the main thread only
                if (omp_get_thread_num() == 0) { m_key_rays_.resize(omp_get_num_threads()); }
            }
        }

        inline void
        ComputeMinMax() {
            if (!m_size_changed_) { return; }

            // empty tree
            if (m_root_ == nullptr) {
                m_metric_min_[0] = m_metric_min_[1] = m_metric_min_[2] = 0.;
                m_metric_max_[0] = m_metric_max_[1] = m_metric_max_[2] = 0.;
                m_size_changed_ = false;
                return;
            }

            // non-empty tree
            m_metric_min_[0] = m_metric_min_[1] = m_metric_min_[2] = std::numeric_limits<double>::infinity();
            m_metric_max_[0] = m_metric_max_[1] = m_metric_max_[2] = -std::numeric_limits<double>::infinity();

            for (auto it = begin(), end = this->end(); it != end; ++it) {
                double size = it.GetNodeSize();
                double half_size = size / 2.0;
                double x = it.GetX() - half_size;
                double y = it.GetY() - half_size;
                double z = it.GetZ() - half_size;
                if (x < m_metric_min_[0]) { m_metric_min_[0] = x; }
                if (y < m_metric_min_[1]) { m_metric_min_[1] = y; }
                if (z < m_metric_min_[2]) { m_metric_min_[2] = z; }
                x += size;
                y += size;
                z += size;
                if (x > m_metric_max_[0]) { m_metric_max_[0] = x; }
                if (y > m_metric_max_[1]) { m_metric_max_[1] = y; }
                if (z > m_metric_max_[2]) { m_metric_max_[2] = z; }
            }

            m_size_changed_ = false;
        }
    };
}  // namespace erl::geometry
