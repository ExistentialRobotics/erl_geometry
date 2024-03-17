#pragma once

#include "abstract_quadtree.hpp"
#include "quadtree_key.hpp"
#include "utils.hpp"
#include <omp.h>
#include <Eigen/Dense>
#include <stack>
#include <tuple>
#include <bitset>

namespace erl::geometry {

    /**
     * QuadtreeImpl is a template class that implements generic quadtree functionality.
     * @tparam Node
     * @tparam Interface
     */
    template<class Node, class Interface>
    class QuadtreeImpl : public Interface {
        static_assert(std::is_base_of<AbstractQuadtreeNode, Node>::value, "Node must be derived from AbstractQuadtreeNode");
        static_assert(std::is_base_of<AbstractQuadtree, Interface>::value, "Interface must be derived from AbstractQuadtree");

    protected:
        std::shared_ptr<Node> m_root_ = nullptr;  // root node of the quadtree, nullptr if the quadtree is empty
        // constants of the tree
        const uint32_t mk_TreeDepth_;      // depth of the tree
        const uint32_t mk_TreeKeyOffset_;  // offset of the tree key
        // parameters of the tree
        double m_resolution_ = 0.0;                // resolution of the tree
        double m_resolution_inv_ = 0.0;            // inverse resolution of the tree
        std::size_t m_tree_size_ = 0;              // number of nodes in the tree
        bool m_size_changed_ = false;              // flag indicating if the metric size of the tree has changed
        double m_tree_center_[2] = {0, 0};         // metric coordinate of the center
        double m_metric_max_[2] = {};              // max metric coordinate of x and y
        double m_metric_min_[2] = {};              // min metric coordinate of x and y
        std::vector<double> m_size_lookup_table_;  // the size of a quadrant at depth i (0: root node, mk_TreeDepth_: smallest leaf node)
        std::vector<QuadtreeKeyRay> m_key_rays_;   // data structure for parallel ray casting

    public:
        using ImplType = QuadtreeImpl<Node, Interface>;

        //-- constructors
        explicit QuadtreeImpl(double resolution)
            : Interface(),
              mk_TreeDepth_(16),
              mk_TreeKeyOffset_(1 << (mk_TreeDepth_ - 1)),
              m_resolution_(resolution) {
            Init();
        }

        QuadtreeImpl(const ImplType &other)
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
            return "QuadtreeImpl";
        }

        [[nodiscard]] std::size_t
        GetSize() const override {
            return m_tree_size_;
        }

        inline void
        SetResolution(double resolution) override {
            m_resolution_ = resolution;
            m_resolution_inv_ = 1. / resolution;
            m_tree_center_[0] = m_tree_center_[1] = double(mk_TreeKeyOffset_) * m_resolution_;
            // init node size lookup table
            m_size_lookup_table_.resize(mk_TreeDepth_ + 1);
            for (uint32_t i = 0; i <= mk_TreeDepth_; ++i) { m_size_lookup_table_[i] = m_resolution_ * double(1 << (mk_TreeDepth_ - i)); }
            m_size_changed_ = true;
        }

        [[nodiscard]] inline double
        GetResolution() const override {
            return m_resolution_;
        }

        [[nodiscard]] inline Eigen::Vector2d
        GetTreeCenter() const {
            return {m_tree_center_[0], m_tree_center_[1]};
        }

        [[nodiscard]] inline uint32_t
        GetTreeDepth() const {
            return mk_TreeDepth_;
        }

        [[nodiscard]] inline QuadtreeKey::KeyType
        GetTreeKeyOffset() const {
            return mk_TreeKeyOffset_;
        }

        inline void
        GetMetricMin(double &min_x, double &min_y) override {
            ComputeMinMax();
            min_x = m_metric_min_[0];
            min_y = m_metric_min_[1];
        }

        void
        GetMetricMin(double &min_x, double &min_y) const override {
            if (!m_size_changed_) {
                min_x = m_metric_min_[0];
                min_y = m_metric_min_[1];
                return;
            }

            if (m_root_ == nullptr) {
                min_x = min_y = 0;
                return;
            }

            min_x = min_y = std::numeric_limits<double>::infinity();
            for (auto it = begin(), end = this->end(); it != end; ++it) {
                double half_size = it.GetNodeSize() / 2.;
                double node_min_x = it.GetX() - half_size;
                double node_min_y = it.GetY() - half_size;
                if (node_min_x < min_x) { min_x = node_min_x; }
                if (node_min_y < min_y) { min_y = node_min_y; }
            }
        }

        inline void
        GetMetricMax(double &max_x, double &max_y) override {
            ComputeMinMax();
            max_x = m_metric_max_[0];
            max_y = m_metric_max_[1];
        }

        void
        GetMetricMax(double &max_x, double &max_y) const override {
            if (!m_size_changed_) {
                max_x = m_metric_max_[0];
                max_y = m_metric_max_[1];
                return;
            }

            if (m_root_ == nullptr) {
                max_x = max_y = 0;
                return;
            }

            max_x = max_y = -std::numeric_limits<double>::infinity();
            for (auto it = this->begin(), end = this->end(); it != end; ++it) {
                double half_size = it.GetNodeSize() / 2.;
                double node_max_x = it.GetX() + half_size;
                double node_max_y = it.GetY() + half_size;
                if (node_max_x > max_x) { max_x = node_max_x; }
                if (node_max_y > max_y) { max_y = node_max_y; }
            }
        }

        inline void
        GetMetricMinMax(double &min_x, double &min_y, double &max_x, double &max_y) override {
            ComputeMinMax();
            min_x = m_metric_min_[0];
            min_y = m_metric_min_[1];
            max_x = m_metric_max_[0];
            max_y = m_metric_max_[1];
        }

        void
        GetMetricMinMax(double &min_x, double &min_y, double &max_x, double &max_y) const override {
            if (!m_size_changed_) {
                min_x = m_metric_min_[0];
                min_y = m_metric_min_[1];
                max_x = m_metric_max_[0];
                max_y = m_metric_max_[1];
                return;
            }

            if (m_root_ == nullptr) {
                min_x = min_y = max_x = max_y = 0;
                return;
            }

            min_x = min_y = std::numeric_limits<double>::infinity();
            max_x = max_y = -std::numeric_limits<double>::infinity();
            for (auto it = this->begin(), end = this->end(); it != end; ++it) {
                double size = it.GetNodeSize();
                double half_size = size / 2.;
                double node_max_x = it.GetX() + half_size;
                double node_max_y = it.GetY() + half_size;
                double node_min_x = node_max_x - size;
                double node_min_y = node_max_y - size;
                if (node_max_x > max_x) { max_x = node_max_x; }
                if (node_max_y > max_y) { max_y = node_max_y; }
                if (node_min_x < min_x) { min_x = node_min_x; }
                if (node_min_y < min_y) { min_y = node_min_y; }
            }
        }

        inline void
        GetMetricSize(double &x, double &y) override {
            double min_x, min_y, max_x, max_y;
            GetMetricMinMax(min_x, min_y, max_x, max_y);
            x = max_x - min_x;
            y = max_y - min_y;
        }

        void
        GetMetricSize(double &x, double &y) const override {
            if (!m_size_changed_) {
                x = m_metric_max_[0] - m_metric_min_[0];
                y = m_metric_max_[1] - m_metric_min_[1];
                return;
            }

            if (m_root_ == nullptr) {
                x = y = 0;
                return;
            }

            double min_x, min_y, max_x, max_y;
            min_x = min_y = std::numeric_limits<double>::infinity();
            max_x = max_y = -std::numeric_limits<double>::infinity();
            for (auto it = this->begin(), end = this->end(); it != end; ++it) {
                double size = it.GetNodeSize();
                double half_size = size / 2.;
                double node_max_x = it.GetX() + half_size;
                double node_max_y = it.GetY() + half_size;
                double node_min_x = node_max_x - size;
                double node_min_y = node_max_y - size;
                if (node_max_x > max_x) { max_x = node_max_x; }
                if (node_max_y > max_y) { max_y = node_max_y; }
                if (node_min_x < min_x) { min_x = node_min_x; }
                if (node_min_y < min_y) { min_y = node_min_y; }
            }
            x = max_x - min_x;
            y = max_y - min_y;
        }

        [[nodiscard]] inline double
        GetNodeSize(uint32_t depth) const {
            ERL_ASSERTM(depth <= mk_TreeDepth_, "Depth must be in [0, %u], but got %u.\n", mk_TreeDepth_, depth);
            return m_size_lookup_table_[depth];
        }

        [[nodiscard]] inline std::size_t
        ComputeNumberOfLeafNodes() const {
            if (m_root_ == nullptr) { return 0; }

            std::list<const Node *> nodes_stack;
            nodes_stack.push_back(m_root_.get());
            std::size_t num_leaf_nodes = 0;
            while (!nodes_stack.empty()) {
                const Node *node = nodes_stack.back();
                nodes_stack.pop_back();

                if (node->HasAnyChild()) {
                    for (uint32_t i = 0; i < 4; ++i) {
                        const Node *child = GetNodeChild(node, i);
                        if (child != nullptr) { nodes_stack.push_back(child); }
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
            return sizeof(ImplType) + GetMemoryUsagePerNode() * m_tree_size_ + number_of_inner_nodes * sizeof(Node *) * 4;
        }

        [[nodiscard]] inline std::size_t
        GetMemoryUsagePerNode() const override {
            return sizeof(Node);
        }

        [[nodiscard]] inline std::size_t
        ComputeNumberOfNodes() const {
            if (m_root_ == nullptr) { return 0; }

            std::size_t num_nodes = 0;
            std::list<const Node *> nodes_stack;
            nodes_stack.emplace_back(m_root_.get());
            while (!nodes_stack.empty()) {
                const Node *node = nodes_stack.back();
                nodes_stack.pop_back();
                num_nodes++;

                if (node->HasAnyChild()) {  // if the node has any child, push them into the stack
                    for (uint32_t i = 0; i < 4; ++i) {
                        const Node *child = GetNodeChild(node, i);
                        if (child != nullptr) { nodes_stack.push_back(child); }
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
        [[nodiscard]] inline QuadtreeKey::KeyType
        CoordToKey(double coordinate) const {
            return uint32_t(std::floor(coordinate * m_resolution_inv_)) + mk_TreeKeyOffset_;
        }

        /**
         * Convert 1-dim coordinate to key at a given depth.
         * @param coordinate
         * @param depth
         * @return
         */
        [[nodiscard]] inline QuadtreeKey::KeyType
        CoordToKey(double coordinate, uint32_t depth) const {
            ERL_ASSERTM(depth <= mk_TreeDepth_, "Depth must be in [0, %u], but got %u.\n", mk_TreeDepth_, depth);
            uint32_t keyval = std::floor(coordinate * m_resolution_inv_);
            uint32_t diff = mk_TreeDepth_ - depth;
            if (!diff) { return keyval + mk_TreeKeyOffset_; }

            return ((keyval >> diff) << diff) + (1 << (diff - 1)) + mk_TreeKeyOffset_;
        }

        /**
         * Convert 2-dim coordinate to key at depth N.
         * @param x
         * @param y
         * @return
         */
        [[nodiscard]] inline QuadtreeKey
        CoordToKey(double x, double y) const {
            return {CoordToKey(x), CoordToKey(y)};
        }

        /**
         * Convert 2-dim coordinate to key at a given depth.
         * @param x
         * @param y
         * @param depth
         * @return
         */
        [[nodiscard]] inline QuadtreeKey
        CoordToKey(double x, double y, uint32_t depth) const {
            if (depth == mk_TreeDepth_) { return CoordToKey(x, y); }
            return {CoordToKey(x, depth), CoordToKey(y, depth)};
        }

        /**
         * Convert 1-dim coordinate to key at depth N with boundary check.
         * @param coordinate
         * @param key
         * @return
         */
        [[nodiscard]] inline bool
        CoordToKeyChecked(double coordinate, QuadtreeKey::KeyType &key) const {
            int scaled_coord = std::floor(coordinate * m_resolution_inv_) + mk_TreeKeyOffset_;
            if ((scaled_coord >= 0) && ((uint32_t) scaled_coord) < (mk_TreeKeyOffset_ << 1)) {
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
        CoordToKeyChecked(double coordinate, uint32_t depth, QuadtreeKey::KeyType &key) const {
            int scaled_coord = std::floor(coordinate * m_resolution_inv_) + mk_TreeKeyOffset_;
            if ((scaled_coord >= 0) && ((uint32_t) scaled_coord) < (mk_TreeKeyOffset_ << 1)) {
                key = AdjustKeyToDepth((QuadtreeKey::KeyType) scaled_coord, depth);
                return true;
            }
            return false;
        }

        /**
         * Convert 2-dim coordinate to key at depth N with boundary check.
         * @param x
         * @param y
         * @param key
         * @return
         */
        [[nodiscard]] inline bool
        CoordToKeyChecked(double x, double y, QuadtreeKey &key) const {
            if (!CoordToKeyChecked(x, key[0])) { return false; }
            if (!CoordToKeyChecked(y, key[1])) { return false; }
            return true;
        }

        /**
         * Convert 2-dim coordinate to key at a given depth with boundary check.
         * @param x
         * @param y
         * @param depth
         * @param key
         * @return
         */
        [[nodiscard]] inline bool
        CoordToKeyChecked(double x, double y, uint32_t depth, QuadtreeKey &key) const {
            ERL_DEBUG_ASSERT(depth != 0, "When depth = 0, key is 0x0, which is useless!");
            if (depth == mk_TreeDepth_) { return CoordToKeyChecked(x, y, key); }
            if (!CoordToKeyChecked(x, depth, key[0])) { return false; }
            if (!CoordToKeyChecked(y, depth, key[1])) { return false; }
            return true;
        }

        /**
         * Adjust 1-dim key from the lowest level (max depth) to a given depth.
         * @param key the key at the lowest level
         * @param depth the target depth
         * @return
         */
        [[nodiscard]] inline QuadtreeKey::KeyType
        AdjustKeyToDepth(QuadtreeKey::KeyType key, uint32_t depth) const {
            ERL_ASSERTM(depth <= mk_TreeDepth_, "Depth must be in [0, %u], but got %u.\n", mk_TreeDepth_, depth);
            uint32_t diff = mk_TreeDepth_ - depth;
            if (!diff) { return key; }
            return (((key - mk_TreeKeyOffset_) >> diff) << diff) + (1 << (diff - 1)) + mk_TreeKeyOffset_;
        }

        /**
         * Adjust 2-dim key from the lowest level (max depth) to a given depth.
         * @param key the key at the lowest level
         * @param depth the target depth
         * @return
         */
        [[nodiscard]] inline QuadtreeKey
        AdjustKeyToDepth(const QuadtreeKey &key, uint32_t depth) const {
            if (depth == mk_TreeDepth_) { return key; }
            return {AdjustKeyToDepth(key[0], depth), AdjustKeyToDepth(key[1], depth)};
        }

        inline void
        ComputeCommonAncestorKey(const QuadtreeKey &key1, const QuadtreeKey &key2, QuadtreeKey &ancestor_key, uint32_t &ancestor_depth) const {
            QuadtreeKey::KeyType mask = (key1[0] ^ key2[0]) | (key1[1] ^ key2[1]);  // 0: same bit, 1: different bit
            if (!mask) {                                                            // keys are identical
                ancestor_key = key1;
                ancestor_depth = mk_TreeDepth_;
                return;
            }
            // from bit-max_depth to bit-0, find first 1
            uint32_t level = mk_TreeDepth_;
            while (level > 0 && !(mask & (1 << (level - 1)))) { --level; }
            ancestor_depth = mk_TreeDepth_ - level;  // bit[level] = 0, bit[level-1] = 1
            QuadtreeKey::KeyType ancestor_mask = ((1 << mk_TreeDepth_) - 1) << level;
            ancestor_key[0] = key1[0] & ancestor_mask;
            ancestor_key[1] = key1[1] & ancestor_mask;
        }

        /**
         * Compute the key of the west(left) neighbor of a node.
         * @param key
         * @param depth 0 means root
         * @param neighbor_key
         */
        inline bool
        ComputeWestNeighborKey(const QuadtreeKey &key, uint32_t depth, QuadtreeKey &neighbor_key) const {
            QuadtreeKey::KeyType offset = 1 << (mk_TreeDepth_ - depth);
            if (key[0] < offset) { return false; }  // no west neighbor
            neighbor_key[0] = key[0] - offset;
            neighbor_key[1] = key[1];
            return true;
        }

        inline bool
        ComputeEastNeighborKey(const QuadtreeKey &key, uint32_t depth, QuadtreeKey &neighbor_key) const {
            QuadtreeKey::KeyType offset = 1 << (mk_TreeDepth_ - depth);
            if ((1 << mk_TreeDepth_) - key[0] <= offset) { return false; }  // no east neighbor (key[0] + offset >= 2^max_depth)
            neighbor_key[0] = key[0] + offset;
            neighbor_key[1] = key[1];
            return true;
        }

        inline bool
        ComputeNorthNeighborKey(const QuadtreeKey &key, uint32_t depth, QuadtreeKey &neighbor_key) const {
            QuadtreeKey::KeyType offset = 1 << (mk_TreeDepth_ - depth);
            if ((1 << mk_TreeDepth_) - key[1] <= offset) { return false; }  // no north neighbor (key[1] + offset >= 2^max_depth)
            neighbor_key[0] = key[0];
            neighbor_key[1] = key[1] + offset;
            return true;
        }

        inline bool
        ComputeSouthNeighborKey(const QuadtreeKey &key, uint32_t depth, QuadtreeKey &neighbor_key) const {
            QuadtreeKey::KeyType offset = 1 << (mk_TreeDepth_ - depth);
            if (key[1] < offset) { return false; }  // no south neighbor
            neighbor_key[0] = key[0];
            neighbor_key[1] = key[1] - offset;
            return true;
        }

        /**
         * Convert 1-dim key to coordinate.
         * @param key
         * @return
         */
        [[nodiscard]] inline double
        KeyToCoord(QuadtreeKey::KeyType key) const {
            return (double(int(key) - int(mk_TreeKeyOffset_)) + 0.5) * m_resolution_;
        }

        /**
         * Convert 1-dim key to coordinate at a given depth.
         * @param key
         * @param depth
         * @return
         */
        [[nodiscard]] inline double
        KeyToCoord(QuadtreeKey::KeyType key, uint32_t depth) const {
            if (depth == 0) { return 0.0; }
            if (depth == mk_TreeDepth_) { return KeyToCoord(key); }
            return (std::floor((double(key) - double(mk_TreeKeyOffset_)) / double(1 << (mk_TreeDepth_ - depth))) + 0.5) * GetNodeSize(depth);
        }

        /**
         * Convert 2-dim key to coordinate.
         * @param key
         * @param x
         * @param y
         */
        inline void
        KeyToCoord(const QuadtreeKey &key, double &x, double &y) const {
            x = KeyToCoord(key[0]);
            y = KeyToCoord(key[1]);
        }

        /**
         * Convert 2-dim key to coordinate at a given depth.
         * @param key
         * @param depth
         * @param x
         * @param y
         */
        inline void
        KeyToCoord(const QuadtreeKey &key, uint32_t depth, double &x, double &y) const {
            if (depth == 0) {
                x = y = 0.0;
                return;
            }
            if (depth == mk_TreeDepth_) {
                KeyToCoord(key, x, y);
                return;
            }
            x = KeyToCoord(key[0], depth);
            y = KeyToCoord(key[1], depth);
        }

        //-- iterator implementation
        class IteratorBase {
        public:
            struct StackElement {
                Node *node = nullptr;
                QuadtreeKey key = {};
                uint32_t depth = 0;
                std::shared_ptr<void> data = nullptr;  // data pointer for derived classes

                StackElement() = default;

                StackElement(Node *node, QuadtreeKey key, uint32_t depth)
                    : node(node),
                      key(key),
                      depth(depth) {}
            };

        protected:
            const ImplType *m_tree_;                 // the tree this iterator is working on
            uint32_t m_max_depth_;               // the maximum depth to query
            std::deque<StackElement> m_stack_ = {};  // stack for depth first traversal

        public:
            /**
             * Default constructor, only used for the end-iterator.
             */
            IteratorBase()
                : m_tree_(nullptr),
                  m_max_depth_(0) {}

            explicit IteratorBase(const ImplType *tree, uint32_t depth = 0)
                : m_tree_(tree != nullptr && tree->m_root_ != nullptr ? tree : nullptr),
                  m_max_depth_(depth) {
                if (m_tree_ && m_max_depth_ == 0) { m_max_depth_ = m_tree_->GetTreeDepth(); }
                if (m_tree_ && m_tree_->m_root_ != nullptr) {  // tree is not empty
                    m_stack_.emplace_back(m_tree_->m_root_.get(), m_tree_->CoordToKey(0.0, 0.0), 0);
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

            inline Node *
            operator->() {
                return m_stack_.back().node;
            }

            [[nodiscard]] inline const Node *
            operator->() const {
                return m_stack_.back().node;
            }

            inline Node *
            operator*() {
                return m_stack_.back().node;
            }

            [[nodiscard]] inline const Node *
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
            GetNodeSize() const {
                return m_tree_->GetNodeSize(m_stack_.back().depth);
            }

            [[nodiscard]] inline uint32_t
            GetDepth() const {
                return m_stack_.back().depth;
            }

            [[nodiscard]] inline const QuadtreeKey &
            GetKey() const {
                return m_stack_.back().key;
            }

            [[nodiscard]] inline QuadtreeKey
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

                uint32_t next_depth = top.depth + 1;
                ERL_DEBUG_ASSERT(next_depth <= m_max_depth_, "Wrong depth: %u (max: %u).\n", next_depth, m_max_depth_);
                QuadtreeKey next_key;
                QuadtreeKey::KeyType center_offset_key = m_tree_->GetTreeKeyOffset() >> next_depth;
                // push on stack in reverse order
                for (int i = 3; i >= 0; --i) {
                    if (top.node->HasChild(i)) {
                        QuadtreeKey::ComputeChildKey(i, center_offset_key, top.key, next_key);
                        m_stack_.emplace_back(const_cast<Node *>(m_tree_->GetNodeChild(top.node, i)), next_key, next_depth);
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

            explicit TreeIterator(const ImplType *tree, uint32_t depth = 0)
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
            QuadtreeKey m_aabb_min_key_;
            QuadtreeKey m_aabb_max_key_;

        public:
            TreeInAabbIterator() = default;

            TreeInAabbIterator(double aabb_mix_x, double aabb_min_y, double aabb_max_x, double aabb_max_y, const ImplType *tree, uint32_t depth = 0)
                : IteratorBase(tree, depth) {
                if (this->m_stack_.empty()) { return; }
                ERL_ASSERTM(tree != nullptr, "Tree is null.");

                if (this->m_tree_->CoordToKeyChecked(aabb_mix_x, aabb_min_y, m_aabb_min_key_) &&
                    this->m_tree_->CoordToKeyChecked(aabb_max_x, aabb_max_y, m_aabb_max_key_)) {
                    // check if the root node is in the AABB
                    typename IteratorBase::StackElement top = this->m_stack_.back();
                    QuadtreeKey::KeyType center_offset_key = this->m_tree_->GetTreeKeyOffset() >> top.depth;
                    if (!QuadtreeKey::KeyInAabb(top.key, center_offset_key, m_aabb_min_key_, m_aabb_max_key_)) {
                        this->Terminate();
                        return;
                    }
                } else {
                    this->Terminate();
                }
            }

            TreeInAabbIterator(const QuadtreeKey &aabb_min_key, const QuadtreeKey &aabb_max_key, const ImplType *tree, uint32_t depth = 0)
                : IteratorBase(tree, depth),
                  m_aabb_min_key_(aabb_min_key),
                  m_aabb_max_key_(aabb_max_key) {
                if (this->m_stack_.empty()) { return; }
                ERL_ASSERTM(tree != nullptr, "Tree is null.");

                // check if the root node is in the AABB
                typename IteratorBase::StackElement top = this->m_stack_.back();
                QuadtreeKey::KeyType center_offset_key = this->m_tree_->GetTreeKeyOffset() >> top.depth;
                if (!QuadtreeKey::KeyInAabb(top.key, center_offset_key, m_aabb_min_key_, m_aabb_max_key_)) {
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

                uint32_t next_depth = top.depth + 1;
                ERL_DEBUG_ASSERT(next_depth <= this->m_max_depth_, "Wrong depth: %u (max: %u).\n", next_depth, this->m_max_depth_);
                QuadtreeKey next_key;
                QuadtreeKey::KeyType center_offset_key = this->m_tree_->GetTreeKeyOffset() >> next_depth;
                // push on stack in reverse order
                for (int i = 3; i >= 0; --i) {
                    if (!top.node->HasChild(i)) { continue; }
                    QuadtreeKey::ComputeChildKey(i, center_offset_key, top.key, next_key);
                    // check if the child node overlaps with the AABB
                    if (QuadtreeKey::KeyInAabb(next_key, center_offset_key, m_aabb_min_key_, m_aabb_max_key_)) {
                        this->m_stack_.emplace_back(const_cast<Node *>(this->m_tree_->GetNodeChild(top.node, i)), next_key, next_depth);
                    }
                }
            }
        };

        class LeafIterator : public IteratorBase {
        public:
            LeafIterator() = default;

            explicit LeafIterator(const ImplType *tree, uint32_t depth = 0)
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

            LeafOfNodeIterator(QuadtreeKey key, uint32_t cluster_depth, const ImplType *tree, uint32_t depth = 0)
                : IteratorBase(tree, depth) {
                ERL_ASSERTM(cluster_depth <= this->m_max_depth_, "Cluster depth %u is greater than max depth %u.\n", cluster_depth, this->m_max_depth_);

                // modify stack top
                auto &s = this->m_stack_.back();
                uint32_t d = cluster_depth;
                s.node = const_cast<Node *>(this->m_tree_->Search(key, d));
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
            QuadtreeKey m_aabb_min_key_;
            QuadtreeKey m_aabb_max_key_;

        public:
            LeafInAabbIterator() = default;

            LeafInAabbIterator(double aabb_mix_x, double aabb_min_y, double aabb_max_x, double aabb_max_y, const ImplType *tree, uint32_t depth = 0)
                : IteratorBase(tree, depth) {
                if (this->m_stack_.empty()) { return; }
                ERL_ASSERTM(tree != nullptr, "Tree is null.");

                if (this->m_tree_->CoordToKeyChecked(aabb_mix_x, aabb_min_y, m_aabb_min_key_) &&
                    this->m_tree_->CoordToKeyChecked(aabb_max_x, aabb_max_y, m_aabb_max_key_)) {
                    // skip forward to next valid leaf node
                    while (!this->m_stack_.empty() && !this->IsLeaf()) { SingleIncrement(); }
                    if (this->m_stack_.empty()) { this->Terminate(); }
                } else {
                    this->Terminate();
                }
            }

            LeafInAabbIterator(const QuadtreeKey &aabb_min_key, const QuadtreeKey &aabb_max_key, const ImplType *tree, uint32_t depth = 0)
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

                uint32_t next_depth = top.depth + 1;
                ERL_DEBUG_ASSERT(next_depth <= this->m_max_depth_, "Wrong depth: %u (max: %u).\n", next_depth, this->m_max_depth_);
                QuadtreeKey next_key;
                QuadtreeKey::KeyType center_offset_key = this->m_tree_->GetTreeKeyOffset() >> next_depth;
                // push on stack in reverse order
                for (int i = 3; i >= 0; --i) {
                    if (!top.node->HasChild(i)) { continue; }
                    QuadtreeKey::ComputeChildKey(i, center_offset_key, top.key, next_key);
                    // check if the child node overlaps with the AABB
                    if (QuadtreeKey::KeyInAabb(next_key, center_offset_key, m_aabb_min_key_, m_aabb_max_key_)) {
                        this->m_stack_.emplace_back(const_cast<Node *>(this->m_tree_->GetNodeChild(top.node, i)), next_key, next_depth);
                    }
                }
            }
        };

        class LeafNeighborIteratorBase : public IteratorBase {
        public:
            LeafNeighborIteratorBase() = default;

            LeafNeighborIteratorBase(const ImplType *tree, uint32_t max_leaf_depth)
                : IteratorBase(tree, max_leaf_depth) {}

        protected:
            QuadtreeKey m_neighbor_key_ = {};
            QuadtreeKey::KeyType m_max_key_changing_dim_ = 0;

            /**
             *
             * @param key
             * @param key_depth
             * @param changing_dim the dimension that is changing during the search
             * @param unchanged_dim the dimension that is unchanged during the search
             * @param increase
             */
            void
            Init(QuadtreeKey key, uint32_t key_depth, int changing_dim, int unchanged_dim, bool increase) {
                if (this->m_tree_ == nullptr) { return; }
                uint32_t max_depth = this->m_tree_->GetTreeDepth();
                uint32_t level = max_depth - key_depth;
                key = this->m_tree_->AdjustKeyToDepth(key, key_depth);
                QuadtreeKey::KeyType half_offset = (level == 0 ? 0 : 1 << (level - 1));
                int key_unchanged;
                if (increase) {
                    key_unchanged = key[unchanged_dim] + std::max(1, int(half_offset));
                    if (key_unchanged >= (1 << max_depth)) {  // no neighbor on east/north
                        this->Terminate();
                        return;
                    }
                } else {
                    key_unchanged = key[unchanged_dim] - half_offset - 1;
                    if (key_unchanged < 0) {  // no neighbor on west/south
                        this->Terminate();
                        return;
                    }
                }

                // start searching from the smallest neighbor at bottom-left corner
                this->m_neighbor_key_[unchanged_dim] = key_unchanged;
                this->m_neighbor_key_[changing_dim] = key[changing_dim] - half_offset;
                this->m_max_key_changing_dim_ = std::min(key[changing_dim] + (level == 0 ? 1 : half_offset), 1 << max_depth);
                this->m_stack_.resize(1);
                SingleIncrementOf(changing_dim);
            }

            inline void
            SingleIncrementOf(int changing_dim) {
                auto &s = this->m_stack_.back();
                QuadtreeKey::KeyType &key_changing_dim = this->m_neighbor_key_[changing_dim];
                s.node = nullptr;
                while (s.node == nullptr && key_changing_dim < m_max_key_changing_dim_) {
                    s.depth = 0;
                    s.node = const_cast<Node *>(this->m_tree_->Search(m_neighbor_key_, s.depth));
                    if (s.node == nullptr || s.depth > this->m_max_depth_) {
                        s.node = nullptr;
                        ++key_changing_dim;
                        continue;
                    }

                    // found a neighbor
                    s.key = this->m_tree_->AdjustKeyToDepth(m_neighbor_key_, s.depth);
                    uint32_t max_depth = this->m_tree_->GetTreeDepth();
                    key_changing_dim = s.key[changing_dim] + (s.depth == max_depth ? 1 : (1 << (max_depth - s.depth - 1)));
                }
                // check if we have reached the end
                if (s.node == nullptr && key_changing_dim >= m_max_key_changing_dim_) { this->Terminate(); }
            }
        };

        class WestLeafNeighborIterator : public LeafNeighborIteratorBase {
            inline static const int sk_ChangingDim_ = 1;  // changing the y-dim key during the search
            inline static const int sk_UnchangedDim_ = 0;
            inline static const bool sk_Increase_ = false;  // decrease the x-dim key

        public:
            WestLeafNeighborIterator() = default;

            WestLeafNeighborIterator(double x, double y, const ImplType *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {

                QuadtreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, key)) {
                    this->Terminate();
                    return;
                }

                uint32_t key_depth = 0;
                if (this->m_tree_->Search(key, key_depth) == nullptr) {
                    this->Terminate();
                    return;
                }

                this->Init(key, key_depth, sk_ChangingDim_, sk_UnchangedDim_, sk_Increase_);
            }

            WestLeafNeighborIterator(const QuadtreeKey &key, uint32_t key_depth, const ImplType *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                this->Init(key, key_depth, sk_ChangingDim_, sk_UnchangedDim_, sk_Increase_);
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
                this->SingleIncrementOf(sk_ChangingDim_);
                return *this;
            }
        };

        class EastLeafNeighborIterator : public LeafNeighborIteratorBase {
            inline static const int sk_ChangingDim_ = 1;  // changing the y-dim key during the search
            inline static const int sk_UnchangedDim_ = 0;
            inline static const bool sk_Increase_ = true;  // increase the x-dim key

        public:
            EastLeafNeighborIterator() = default;

            EastLeafNeighborIterator(double x, double y, const ImplType *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {

                QuadtreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, key)) {
                    this->Terminate();
                    return;
                }

                uint32_t key_depth = 0;
                if (this->m_tree_->Search(key, key_depth) == nullptr) {
                    this->Terminate();
                    return;
                }
                this->Init(key, key_depth, sk_ChangingDim_, sk_UnchangedDim_, sk_Increase_);
            }

            EastLeafNeighborIterator(const QuadtreeKey &key, uint32_t key_depth, const ImplType *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                this->Init(key, key_depth, sk_ChangingDim_, sk_UnchangedDim_, sk_Increase_);
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
                this->SingleIncrementOf(sk_ChangingDim_);
                return *this;
            }
        };

        class NorthLeafNeighborIterator : public LeafNeighborIteratorBase {
            inline static const int sk_ChangingDim_ = 0;  // changing the x-dim key during the search
            inline static const int sk_UnchangedDim_ = 1;
            inline static const bool sk_Increase_ = true;  // increase the y-dim key

        public:
            NorthLeafNeighborIterator() = default;

            NorthLeafNeighborIterator(double x, double y, const ImplType *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {

                QuadtreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, key)) {
                    this->Terminate();
                    return;
                }

                uint32_t key_depth = 0;
                if (this->m_tree_->Search(key, key_depth) == nullptr) {
                    this->Terminate();
                    return;
                }

                this->Init(key, key_depth, sk_ChangingDim_, sk_UnchangedDim_, sk_Increase_);
            }

            NorthLeafNeighborIterator(const QuadtreeKey &key, uint32_t key_depth, const ImplType *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                this->Init(key, key_depth, sk_ChangingDim_, sk_UnchangedDim_, sk_Increase_);
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
                this->SingleIncrementOf(sk_ChangingDim_);
                return *this;
            }
        };

        class SouthLeafNeighborIterator : public LeafNeighborIteratorBase {
            inline static const int sk_ChangingDim_ = 0;  // changing the x-dim key during the search
            inline static const int sk_UnchangedDim_ = 1;
            inline static const bool sk_Increase_ = false;  // decrease the y-dim key

        public:
            SouthLeafNeighborIterator() = default;

            SouthLeafNeighborIterator(double x, double y, const ImplType *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                QuadtreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, key)) {
                    this->Terminate();
                    return;
                }

                uint32_t key_depth = 0;
                if (this->m_tree_->Search(key, key_depth) == nullptr) {
                    this->Terminate();
                    return;
                }

                this->Init(key, key_depth, sk_ChangingDim_, sk_UnchangedDim_, sk_Increase_);
            }

            SouthLeafNeighborIterator(const QuadtreeKey &key, uint32_t key_depth, const ImplType *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                this->Init(key, key_depth, sk_ChangingDim_, sk_UnchangedDim_, sk_Increase_);
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
                this->SingleIncrementOf(sk_ChangingDim_);
                return *this;
            }
        };

        class LeafOnRayIterator : public IteratorBase {
            Eigen::Vector2d m_origin_ = {};
            Eigen::Vector2d m_dir_ = {};
            Eigen::Vector2d m_dir_inv_ = {};
            double m_max_range_ = 0.;
            bool m_bidirectional_ = false;

        public:
            LeafOnRayIterator() = default;

            LeafOnRayIterator(
                double px,  //
                double py,
                double vx,
                double vy,
                double max_range,
                bool bidirectional,
                const ImplType *tree,
                uint32_t max_leaf_depth)
                : IteratorBase(tree, max_leaf_depth),
                  m_origin_(px, py),
                  m_dir_(vx, vy),
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
                    QuadtreeKey::KeyType center_offset_key = this->m_tree_->GetTreeKeyOffset() >> s_child.depth;
                    for (int i = 3; i >= 0; --i) {
                        if (s_child.depth > this->m_max_depth_) { continue; }
                        if (!s.node->HasChild(i)) { continue; }
                        double dist = 0.0, dist2 = 0.0;
                        bool intersected = false;
                        const Node *child = this->m_tree_->GetNodeChild(s.node, i);
                        QuadtreeKey::ComputeChildKey(i, center_offset_key, s.key, s_child.key);

                        double center_x = this->m_tree_->KeyToCoord(s_child.key[0], s_child.depth);
                        double center_y = this->m_tree_->KeyToCoord(s_child.key[1], s_child.depth);
                        double half_size = this->m_tree_->GetNodeSize(s_child.depth) / 2.0;
                        Eigen::Vector2d box_min(center_x - half_size, center_y - half_size);
                        Eigen::Vector2d box_max(center_x + half_size, center_y + half_size);
                        ComputeIntersectionBetweenRayAndAabb2D(m_origin_, m_dir_inv_, box_min, box_max, dist, dist2, intersected);
                        if (!intersected) { continue; }
                        if (!child->HasAnyChild()) {
                            if (!m_bidirectional_ && dist < 0.) { continue; }
                            if (m_max_range_ > 0. && std::abs(dist) > m_max_range_) { continue; }
                        }
                        s_child.node = const_cast<Node *>(child);
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
        begin(uint32_t max_depth = 0) const {
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
        BeginLeaf(uint32_t max_depth = 0) const {
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
        BeginLeafOfNode(QuadtreeKey key, uint32_t node_depth, uint32_t max_depth = 0) const {
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
            double aabb_max_x,
            double aabb_max_y,
            uint32_t max_depth = 0) const {
            return LeafInAabbIterator(aabb_mix_x, aabb_min_y, aabb_max_x, aabb_max_y, this, max_depth);
        }

        [[nodiscard]] inline LeafInAabbIterator
        BeginLeafInAabb(const QuadtreeKey &aabb_min_key, const QuadtreeKey &aabb_max_key, uint32_t max_depth = 0) const {
            return LeafInAabbIterator(aabb_min_key, aabb_max_key, this, max_depth);
        }

        [[nodiscard]] inline LeafInAabbIterator
        EndLeafInAabb() const {
            return LeafInAabbIterator();
        }

        [[nodiscard]] inline TreeIterator
        BeginTree(uint32_t max_depth = 0) const {
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
            double aabb_max_x,
            double aabb_max_y,
            uint32_t max_depth = 0) const {
            return TreeInAabbIterator(aabb_mix_x, aabb_min_y, aabb_max_x, aabb_max_y, this, max_depth);
        }

        [[nodiscard]] inline TreeInAabbIterator
        BeginTreeInAabb(const QuadtreeKey &aabb_min_key, const QuadtreeKey &aabb_max_key, uint32_t max_depth = 0) const {
            return TreeInAabbIterator(aabb_min_key, aabb_max_key, this, max_depth);
        }

        [[nodiscard]] inline TreeInAabbIterator
        EndTreeInAabb() const {
            return TreeInAabbIterator();
        }

        [[nodiscard]] inline WestLeafNeighborIterator
        BeginWestLeafNeighbor(double x, double y, uint32_t max_leaf_depth = 0) const {
            return WestLeafNeighborIterator(x, y, this, max_leaf_depth);
        }

        [[nodiscard]] inline WestLeafNeighborIterator
        BeginWestLeafNeighbor(const QuadtreeKey &key, uint32_t key_depth, uint32_t max_leaf_depth = 0) const {
            return WestLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[nodiscard]] inline WestLeafNeighborIterator
        EndWestLeafNeighbor() const {
            return WestLeafNeighborIterator();
        }

        [[nodiscard]] inline EastLeafNeighborIterator
        BeginEastLeafNeighbor(double x, double y, uint32_t max_leaf_depth = 0) const {
            return EastLeafNeighborIterator(x, y, this, max_leaf_depth);
        }

        [[nodiscard]] inline EastLeafNeighborIterator
        BeginEastLeafNeighbor(const QuadtreeKey &key, uint32_t key_depth, uint32_t max_leaf_depth = 0) const {
            return EastLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[nodiscard]] inline EastLeafNeighborIterator
        EndEastLeafNeighbor() const {
            return EastLeafNeighborIterator();
        }

        [[nodiscard]] inline NorthLeafNeighborIterator
        BeginNorthLeafNeighbor(double x, double y, uint32_t max_leaf_depth = 0) const {
            return NorthLeafNeighborIterator(x, y, this, max_leaf_depth);
        }

        [[nodiscard]] inline NorthLeafNeighborIterator
        BeginNorthLeafNeighbor(const QuadtreeKey &key, uint32_t key_depth, uint32_t max_leaf_depth = 0) const {
            return NorthLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[nodiscard]] inline NorthLeafNeighborIterator
        EndNorthLeafNeighbor() const {
            return NorthLeafNeighborIterator();
        }

        [[nodiscard]] inline SouthLeafNeighborIterator
        BeginSouthLeafNeighbor(double x, double y, uint32_t max_leaf_depth = 0) const {
            return SouthLeafNeighborIterator(x, y, this, max_leaf_depth);
        }

        [[nodiscard]] inline SouthLeafNeighborIterator
        BeginSouthLeafNeighbor(const QuadtreeKey &key, uint32_t key_depth, uint32_t max_leaf_depth = 0) const {
            return SouthLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[nodiscard]] inline SouthLeafNeighborIterator
        EndSouthLeafNeighbor() const {
            return SouthLeafNeighborIterator();
        }

        [[nodiscard]] inline LeafOnRayIterator
        BeginLeafOnRay(
            double px,  //
            double py,
            double vx,
            double vy,
            double max_range = -1,
            bool bidirectional = false,
            uint32_t max_leaf_depth = 0) const {
            return LeafOnRayIterator(px, py, vx, vy, max_range, bidirectional, this, max_leaf_depth);
        }

        [[nodiscard]] inline LeafOnRayIterator
        EndLeafOnRay() const {
            return LeafOnRayIterator();
        }

        //-- ray tracing
        /**
         * Trace a ray from origin to end (excluded), returning a QuadtreeKeyRay of all nodes' QuadtreeKey traversed by the ray. For each key, the
         * corresponding node may not exist. You can use Search() to check if the node exists.
         * @param sx x coordinate of the origin
         * @param sy y coordinate of the origin
         * @param ex x coordinate of the end (excluded)
         * @param ey y coordinate of the end (excluded)
         * @param ray
         * @return
         */
        [[nodiscard]] bool
        ComputeRayKeys(double sx, double sy, double ex, double ey, QuadtreeKeyRay &ray) const {
            // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo Digital Difference Analyzer (DDA) algorithm
            // Note that we cannot use Bresenham's line algorithm because it may miss some voxels when the ray is not axis-aligned.
            // For example, if the ray is from (0, 0) to (1, 1), Bresenham's algorithm will miss (1, 0) and (0, 1) but the ray should traverse them.
            // Also look at https://en.wikipedia.org/wiki/File:Bresenham.svg for another example of Bresenham's algorithm.

            ray.Reset();
            QuadtreeKey key_start, key_end;
            if (!CoordToKeyChecked(sx, sy, key_start) || !CoordToKeyChecked(ex, ey, key_end)) {
                ERL_WARN("Ray (%f, %f) -> (%f, %f) is out of range.\n", sx, sy, ex, ey);
                return false;
            }
            if (key_start == key_end) { return true; }

            // initialization phase
            double direction[2];
            double &dx = direction[0];
            double &dy = direction[1];
            dx = ex - sx;
            dy = ey - sy;
            double length = std::sqrt(dx * dx + dy * dy);
            dx /= length;
            dy /= length;

            // compute step direction
            int step[2];
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
            if (step[0] == 0 && step[1] == 0) {
                ERL_WARN("Ray casting in direction (0, 0) is impossible!");
                return false;
            }

            // compute t_max and t_delta
            double t_max[2];
            double t_delta[2];
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

            // incremental phase
            ray.AddKey(key_start);
            QuadtreeKey current_key = key_start;
            while (true) {
                if (t_max[0] < t_max[1]) {
                    t_max[0] += t_delta[0];
                    current_key[0] += step[0];
                    ERL_DEBUG_ASSERT(
                        current_key[0] < (mk_TreeKeyOffset_ << 1),
                        "current_key[0] = %d exceeds limit %d.\n",
                        current_key[0],
                        (mk_TreeKeyOffset_ << 1));
                } else {
                    t_max[1] += t_delta[1];
                    current_key[1] += step[1];
                    ERL_DEBUG_ASSERT(
                        current_key[1] < (mk_TreeKeyOffset_ << 1),
                        "current_key[1] = %d exceeds limit %d.\n",
                        current_key[1],
                        (mk_TreeKeyOffset_ << 1));
                }

                if (current_key == key_end) { break; }

                // this seems to be unlikely to happen
                double dist_from_origin = std::min(t_max[0], t_max[1]);
                if (dist_from_origin > length) { break; }  // this happens due to numerical error

                ray.AddKey(current_key);
                ERL_DEBUG_ASSERT(ray.size() < ray.capacity() - 1, "Ray capacity is not enough.");
            }

            return true;
        }

        /**
         * Trace a ray from origin to end (excluded), returning a list of all nodes' coordinates traversed by the ray. For each coordinate, the corresponding
         * node may not exist. You can use Search() to check if the node exists.
         * @param sx x coordinate of the origin
         * @param sy y coordinate of the origin
         * @param ex x coordinate of the end (excluded)
         * @param ey y coordinate of the end (excluded)
         * @param ray
         * @return
         */
        [[nodiscard]] bool
        ComputeRayCoords(double sx, double sy, double ex, double ey, std::vector<std::array<double, 2>> &ray) const {
            ray.clear();
            QuadtreeKeyRay key_ray;
            if (!ComputeRayKeys(sx, sy, ex, ey, key_ray)) { return false; }
            ray.reserve(key_ray.size());
            for (auto &key: key_ray) { ray.emplace_back(std::array<double, 2>{KeyToCoord(key[0]), KeyToCoord(key[1])}); }
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
        inline Node *
        CreateNodeChild(Node *node, uint32_t child_idx) {
            node->AllocateChildrenPtr();                                            // allocate children if necessary
            ERL_DEBUG_ASSERT(!node->HasChild(child_idx), "Child already exists.");  // child must not exist
            Node *new_child = node->template CreateChild<Node>(child_idx);          // create child
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
        DeleteNodeChild(Node *node, uint32_t child_idx) {
            node->RemoveChild(child_idx);  // remove child
            m_tree_size_--;                // decrease tree size
            m_size_changed_ = true;        // size of the tree has changed
        }

        /**
         * Get a child node of the given node. Before calling this function, make sure node->HasChildrenPtr() or node->HasAnyChild() returns true.
         * @param node
         * @param child_idx
         * @return
         */
        inline Node *
        GetNodeChild(Node *node, uint32_t child_idx) {
            return node->template GetChild<Node>(child_idx);
        }

        /**
         * Get a child node of the given node. Before calling this function, make sure node->HasChildrenPtr() or node->HasAnyChild() returns true.
         * @param node
         * @param child_idx
         * @return
         */
        inline const Node *
        GetNodeChild(const Node *node, uint32_t child_idx) const {
            return node->template GetChild<Node>(child_idx);
        }

        /**
         * Check if a node is collapsible. For example, for occupancy quadtree, a node is collapsible if all its children exist, none of them have its own
         * children, and they all have the same occupancy value.
         * @param node
         * @return
         */
        virtual bool
        IsNodeCollapsible(const Node *node) const {
            // all children must exist
            if (node->GetNumChildren() != 4) { return false; }

            auto first_child = this->GetNodeChild(node, 0);
            if (first_child->HasAnyChild()) { return false; }

            for (uint32_t i = 1; i < 4; ++i) {
                auto child = this->GetNodeChild(node, i);
                // child should be a leaf node
                if (child->HasAnyChild() || *child != *first_child) { return false; }
            }

            return true;
        }

        /**
         * Expand a node: all children are created and their data is copied from the parent.
         * @param node
         * @return
         */
        void
        ExpandNode(Node *node) {
            ERL_DEBUG_ASSERT(!node->HasAnyChild(), "Node already has children.");
            for (uint32_t i = 0; i < 4; ++i) {
                auto child = CreateNodeChild(node, i);
                OnExpandNode(node, child);
            }
        }

        virtual void
        OnExpandNode(Node *node, Node *child) {
            child->CopyData(*node);
        }

        /**
         * Prune a node: delete all children if the node is collapsible.
         * @param node
         * @return
         */
        bool
        PruneNode(Node *node) {
            if (!IsNodeCollapsible(node)) { return false; }

            OnPruneNode(node);                          // call prune node callback
            node->template DeleteChildrenPtr2<Node>();  // delete children
            m_tree_size_ -= 4;
            m_size_changed_ = true;

            return true;
        }

        virtual void
        OnPruneNode(Node *node) {
            node->CopyData(*this->GetNodeChild(node, 0));  // copy log odds from child to parent
        }

        /**
         * Delete a node at the given depth given a point if it exists. Will always delete at the lowest level unless depth != 0, and expand pruned inner nodes
         * as needed. Pruned nodes at depth "depth" will directly be deleted as a whole.
         * @param x
         * @param y
         * @param depth
         * @return
         */
        inline bool
        DeleteNode(double x, double y, uint32_t depth = 0) {
            QuadtreeKey key;
            if (!CoordToKeyChecked(x, y, key)) {
                ERL_WARN("Point (%f, %f) is out of range.", x, y);
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
        DeleteNode(const QuadtreeKey &key, uint32_t depth = 0) {
            if (m_root_ == nullptr) { return true; }
            if (depth == 0) { depth = mk_TreeDepth_; }
            return DeleteNodeRecurs(m_root_.get(), key, 0, depth);
        }

        virtual void
        OnDeleteNodeChild(Node *node, uint32_t child_idx) = 0;

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
        DeleteNode(std::shared_ptr<Node> node, const QuadtreeKey &key, uint32_t depth, uint32_t max_depth) {
            if (depth >= max_depth) { return true; }
            ERL_DEBUG_ASSERT(node != nullptr, "node should not be nullptr.");

            uint32_t pos = QuadtreeKey::ComputeChildIndex(key, this->mk_TreeDepth_ - depth - 1);
            if (!node->HasChild(pos)) {                                 // child does not exist, but maybe node is pruned
                if (!node->HasAnyChild() && (node != m_root_.get())) {  // this node is pruned
                    ExpandNode(node);                                   // expand it, tree size is updated in ExpandNode
                } else {                                                // node is not pruned, we are done
                    return false;                                       // nothing to delete
                }
            }

            bool delete_child = DeleteNodeRecurs(this->GetNodeChild(node, pos), key, depth + 1, max_depth);
            if (delete_child) {
                this->DeleteNodeChild(node, pos);
                if (!node->HasAnyChild()) { return true; }
                this->OnDeleteNodeChild(node, pos);
            }
            return false;
        }

        /**
         * Delete all children of a node.
         * @param node
         * @return
         */
        inline void
        DeleteNodeRecurs(Node *node) {
            ERL_DEBUG_ASSERT(node != nullptr, "node should not be nullptr.");
            if (!node->HasAnyChild()) { return; }
            for (int i = 0; i < 4; ++i) {
                Node *child = this->GetNodeChild(node, i);
                if (child == nullptr) { continue; }
                this->DeleteNodeRecurs(child);
            }
            m_tree_size_ -= node->GetNumChildren();
            m_size_changed_ = true;
            node->DeleteChildrenPtr();  // delete node's children
        }

        bool
        DeleteNodeRecurs(Node *node, const QuadtreeKey &key, uint32_t depth, uint32_t max_depth) {
            if (depth >= max_depth) { return true; }
            ERL_DEBUG_ASSERT(node != nullptr, "node should not be nullptr.");

            uint32_t pos = QuadtreeKey::ComputeChildIndex(key, this->mk_TreeDepth_ - depth - 1);
            if (!node->HasChild(pos)) {                                 // child does not exist, but maybe node is pruned
                if (!node->HasAnyChild() && (node != m_root_.get())) {  // this node is pruned
                    ExpandNode(node);                                   // expand it, tree size is updated in ExpandNode
                } else {                                                // node is not pruned, we are done
                    return false;                                       // nothing to delete
                }
            }

            bool delete_child = DeleteNodeRecurs(this->GetNodeChild(node, pos), key, depth + 1, max_depth);
            if (delete_child) {
                this->DeleteNodeChild(node, pos);
                if (!node->HasAnyChild()) { return true; }
                this->OnDeleteNodeChild(node, pos);
            }
            return false;
        }

    public:
        /**
         * Delete the whole tree.
         * @return
         */
        void
        Clear() override {
            if (m_root_ == nullptr) { return; }

            DeleteNodeRecurs(m_root_.get());
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

            for (uint32_t depth = this->mk_TreeDepth_ - 1; depth > 0; --depth) {
                uint32_t num_pruned = 0;
                PruneRecurs(this->m_root_.get(), 0, depth, num_pruned);
                if (num_pruned == 0) { break; }
            }
        }

    protected:
        void
        PruneRecurs(Node *node, uint32_t depth, uint32_t max_depth, uint32_t &num_pruned) {
            if (depth < max_depth) {
                if (!node->HasAnyChild()) { return; }
                for (int i = 0; i < 4; ++i) {
                    Node *child = this->GetNodeChild(node, i);
                    if (child == nullptr) { continue; }
                    PruneRecurs(child, depth + 1, max_depth, num_pruned);
                }
            } else {
                if (PruneNode(node)) { num_pruned++; }
            }
        }

    public:
        /**
         * Expand all pruned nodes (reverse operation of Prune).
         * @attention This is an expensive operation, especially when the tree is nearly empty!
         */
        virtual void
        Expand() {
            if (m_root_ == nullptr) { return; }
            ExpandRecurs(m_root_.get(), 0, this->mk_TreeDepth_);
        }

    protected:
        void
        ExpandRecurs(Node *node, uint32_t depth, uint32_t max_depth) {
            if (depth >= max_depth) { return; }
            ERL_DEBUG_ASSERT(node != nullptr, "node is nullptr");

            if (!node->HasAnyChild()) { ExpandNode(node); }  // node has no children, expand it
            for (int i = 0; i < 4; ++i) {
                Node *child = this->GetNodeChild(node, i);
                if (child == nullptr) { continue; }
                ExpandRecurs(child, depth + 1, max_depth);
            }
        }

    public:
        inline const std::shared_ptr<Node> &
        GetRoot() const {
            return m_root_;
        }

        //-- Search functions

        inline Node *
        Search(double x, double y) {
            uint32_t depth = 0;
            return Search(x, y, depth);
        }

        [[nodiscard]] inline const Node *
        Search(double x, double y) const {
            uint32_t depth = 0;
            return Search(x, y, depth);
        }

        /**
         * Search node at specified depth given a point.
         * @param x
         * @param y
         * @param depth specially, depth=0 means searching from root. The output value indicates the depth of the returned node. If -1, search is failed.
         * @return
         */
        [[nodiscard]] inline Node *
        Search(double x, double y, uint32_t &depth) {
            QuadtreeKey key;
            if (!CoordToKeyChecked(x, y, key)) {
                ERL_WARN("Point (%f, %f) is out of range.\n", x, y);
                return nullptr;
            }

            return Search(key, depth);
        }

        [[nodiscard]] inline const Node *
        Search(double x, double y, uint32_t &depth) const {
            QuadtreeKey key;
            if (!CoordToKeyChecked(x, y, key)) {
                ERL_WARN("Point (%f, %f) is out of range.\n", x, y);
                return nullptr;
            }

            return Search(key, depth);
        }

        inline Node *
        Search(const QuadtreeKey &key) {
            uint32_t depth = 0;
            return Search(key, depth);
        }

        [[nodiscard]] inline const Node *
        Search(const QuadtreeKey &key) const {
            uint32_t depth = 0;
            return Search(key, depth);
        }

        /**
         * Search node at specified depth given a key.
         * @param key
         * @param depth specially, depth=0 means searching from root. The output value indicates the depth of the returned node. If -1, search is failed.
         * @return
         */
        Node *
        Search(const QuadtreeKey &key, uint32_t &depth) {
            ERL_DEBUG_ASSERT(depth <= mk_TreeDepth_, "Depth must be in [0, %u], but got %u.\n", mk_TreeDepth_, depth);

            if (m_root_ == nullptr) { return nullptr; }
            if (depth == 0) { depth = mk_TreeDepth_; }

            // generate appropriate key for the given depth
            QuadtreeKey key_at_depth = key;
            if (depth != mk_TreeDepth_) { key_at_depth = AdjustKeyToDepth(key_at_depth, depth); }

            // search
            Node *node = m_root_.get();
            int diff = mk_TreeDepth_ - depth;
            // follow nodes down to the requested level (for level = 0, it is the leaf level)
            for (int level = mk_TreeDepth_ - 1; level >= diff; --level) {
                uint32_t child_index = QuadtreeKey::ComputeChildIndex(key_at_depth, level);
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

        [[nodiscard]] inline const Node *
        Search(const QuadtreeKey &key, uint32_t &depth) const {
            ERL_DEBUG_ASSERT(depth <= mk_TreeDepth_, "Depth must be in [0, %u], but got %u.\n", mk_TreeDepth_, depth);

            if (m_root_ == nullptr) { return nullptr; }
            if (depth == 0) { depth = mk_TreeDepth_; }

            // generate appropriate key for the given depth
            QuadtreeKey key_at_depth = key;
            if (depth != mk_TreeDepth_) { key_at_depth = AdjustKeyToDepth(key_at_depth, depth); }

            // search
            const Node *node = m_root_.get();
            int diff = mk_TreeDepth_ - depth;
            // follow nodes down to the requested level (for level = 0, it is the leaf level)
            for (int level = mk_TreeDepth_ - 1; level >= diff; --level) {
                uint32_t child_index = QuadtreeKey::ComputeChildIndex(key_at_depth, level);
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

        inline Node *
        InsertNode(double x, double y, uint32_t depth = 0) {
            QuadtreeKey key;
            if (!CoordToKeyChecked(x, y, key)) {
                ERL_WARN("Point (%f, %f) is out of range.\n", x, y);
                return nullptr;
            }
            return InsertNode(key, depth);
        }

        inline Node *
        InsertNode(const QuadtreeKey &key, uint32_t depth = 0) {
            if (depth == 0) { depth = mk_TreeDepth_; }
            ERL_ASSERTM(depth <= mk_TreeDepth_, "Depth must be in [0, %u], but got %u.\n", mk_TreeDepth_, depth);
            if (this->m_root_ == nullptr) {
                this->m_root_ = std::make_shared<Node>();
                this->m_tree_size_++;
            }

            Node *node = this->m_root_.get();
            int diff = mk_TreeDepth_ - depth;
            for (int level = mk_TreeDepth_ - 1; level >= diff; --level) {
                uint32_t child_index = QuadtreeKey::ComputeChildIndex(key, level);
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
         * Read all nodes from the input stream (without file header). For general file IO, use AbstractQuadtree::Read.
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
                ERL_WARN("Quadtree is not empty, clear it first.\n");
                return s;
            }

            m_root_ = std::make_shared<Node>();
            m_tree_size_++;
            std::list<Node *> nodes_stack;
            nodes_stack.push_back(m_root_.get());
            while (!nodes_stack.empty()) {
                Node *node = nodes_stack.back();
                nodes_stack.pop_back();

                // load node data
                node->ReadData(s);

                // load children
                char children_char;
                s.read(&children_char, sizeof(char));
                std::bitset<4> children((unsigned long long) children_char);
                node->AllocateChildrenPtr();
                for (int i = 3; i >= 0; --i) {  // the same order as the recursive implementation
                    if (!children[i]) { continue; }
                    Node *child_node = node->template CreateChild<Node>(i);
                    nodes_stack.push_back(child_node);
                    m_tree_size_++;
                }
            }

            return s;
        }

        std::ostream &
        WriteData(std::ostream &s) const override {
            if (m_root_ == nullptr) { return s; }

            std::list<const Node *> nodes_stack;
            nodes_stack.push_back(m_root_.get());
            while (!nodes_stack.empty()) {
                auto node = nodes_stack.back();
                nodes_stack.pop_back();

                // write node data
                node->WriteData(s);

                // write children
                std::bitset<4> children;
                for (int i = 3; i >= 0; --i) {  // the same order as the recursive implementation
                    if (node->HasChild(i)) {
                        children[i] = true;
                        nodes_stack.push_back(GetNodeChild(node, i));
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
        QuadtreeImpl(double resolution, uint32_t tree_depth, uint32_t tree_key_offset)
            : mk_TreeDepth_(tree_depth),
              mk_TreeKeyOffset_(tree_key_offset),
              m_resolution_(resolution) {
            Init();
        }

        inline void
        Init() {
            SetResolution(m_resolution_);
            m_metric_min_[0] = m_metric_min_[1] = std::numeric_limits<double>::infinity();
            m_metric_max_[0] = m_metric_max_[1] = -std::numeric_limits<double>::infinity();
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
                m_metric_min_[0] = m_metric_min_[1] = 0.;
                m_metric_max_[0] = m_metric_max_[1] = 0.;
                m_size_changed_ = false;
                return;
            }

            // non-empty tree
            m_metric_min_[0] = m_metric_min_[1] = std::numeric_limits<double>::infinity();
            m_metric_max_[0] = m_metric_max_[1] = -std::numeric_limits<double>::infinity();

            for (auto it = begin(), end = this->end(); it != end; ++it) {
                double size = it.GetNodeSize();
                double half_size = size / 2.0;
                double x = it.GetX() - half_size;
                double y = it.GetY() - half_size;
                if (x < m_metric_min_[0]) { m_metric_min_[0] = x; }
                if (y < m_metric_min_[1]) { m_metric_min_[1] = y; }
                x += size;
                y += size;
                if (x > m_metric_max_[0]) { m_metric_max_[0] = x; }
                if (y > m_metric_max_[1]) { m_metric_max_[1] = y; }
            }

            m_size_changed_ = false;
        }
    };
}  // namespace erl::geometry
