#pragma once

#include "erl_common/eigen.hpp"
#include "abstract_octree.hpp"
#include "octree_key.hpp"
#include "utils.hpp"
#include "aabb.hpp"
#include <omp.h>
#include <stack>
#include <tuple>
#include <bitset>
#include <utility>

namespace erl::geometry {

    /**
     * OctreeImpl is a template class that implements generic octree functionality. The tree is centered at the origin.
     * @tparam Node
     * @tparam Interface
     */
    template<class Node, class Interface, class InterfaceSetting>
    class OctreeImpl : public Interface {
        static_assert(std::is_base_of<AbstractOctreeNode, Node>::value, "Node must be derived from AbstractOctreeNode");
        static_assert(std::is_base_of<AbstractOctree, Interface>::value, "Interface must be derived from AbstractOctree");

    protected:
        std::shared_ptr<Node> m_root_ = nullptr;  // root node of the quadtree, nullptr if the quadtree is empty
        double m_resolution_inv_ = 0;             // inverse of the resolution
        uint32_t m_tree_key_offset_ = 0;          // offset of the key, = 1 << (tree_depth - 1)
        std::size_t m_tree_size_ = 0;             // number of nodes in the tree
        bool m_size_changed_ = false;             // flag indicating if the metric size of the tree has changed
        double m_metric_max_[3] = {               // max metric coordinate of x, y and z
                                   std::numeric_limits<double>::lowest(),
                                   std::numeric_limits<double>::lowest(),
                                   std::numeric_limits<double>::lowest()};
        double m_metric_min_[3] = {// min metric coordinate of x, y and z
                                   std::numeric_limits<double>::max(),
                                   std::numeric_limits<double>::max(),
                                   std::numeric_limits<double>::max()};
        std::vector<double> m_size_lookup_table_;  // the size of a quadrant at depth i (0: root node, m_setting_->tree_depth: smallest leaf node)
        std::vector<OctreeKeyRay> m_key_rays_;     // data structure for parallel ray casting

    public:
        OctreeImpl() = delete;  // no default constructor

        explicit OctreeImpl(const std::shared_ptr<InterfaceSetting> &setting)
            : Interface(setting) {
            this->ApplySetting();
        }

        OctreeImpl(const OctreeImpl &) = delete;  // no copy constructor

        //-- comparison operators
        [[nodiscard]] bool
        operator==(const AbstractOctree &other) const override {
            if (typeid(*this) != typeid(other)) { return false; }  // compare type
            const auto &other_impl = dynamic_cast<const OctreeImpl &>(other);
            if (*this->m_setting_ != *other_impl.m_setting_) { return false; }
            if (m_tree_size_ != other_impl.m_tree_size_) { return false; }
            if (m_root_ == nullptr && other_impl.m_root_ == nullptr) { return true; }
            if (m_root_ == nullptr || other_impl.m_root_ == nullptr) { return false; }
            return *m_root_ == *other_impl.m_root_;
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

        [[maybe_unused]] [[nodiscard]] inline Eigen::Vector3d
        GetTreeCenter() const {
            double x = this->KeyToCoord(m_tree_key_offset_);
            return {x, x, x};
        }

        [[nodiscard]] OctreeKey
        GetTreeCenterKey() const {
            OctreeKey::KeyType key = m_tree_key_offset_;
            return {key, key, key};
        }

        [[maybe_unused]] [[nodiscard]] inline Eigen::Vector3d
        GetTreeMaxHalfSize() const {
            double size = -this->KeyToCoord(0);
            return {size, size, size};
        }

        void
        ApplySetting() override {
            const double &kResolution = this->m_setting_->resolution;
            const uint32_t &kTreeDepth = this->m_setting_->tree_depth;
            m_resolution_inv_ = 1.0 / kResolution;
            m_tree_key_offset_ = 1 << (kTreeDepth - 1);

            // init node size lookup table
            m_size_lookup_table_.resize(kTreeDepth + 1);
            for (uint32_t i = 0; i <= kTreeDepth; ++i) { m_size_lookup_table_[i] = kResolution * double(1 << (kTreeDepth - i)); }
            m_size_changed_ = true;

            // do it on the main thread only
#pragma omp parallel default(none) shared(m_key_rays_)
#pragma omp critical
            {
                if (omp_get_thread_num() == 0) { m_key_rays_.resize(omp_get_num_threads()); }
            }
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
            this->ComputeMinMax();
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
            this->ComputeMinMax();
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
        GetNodeSize(uint32_t depth) const {
            ERL_DEBUG_ASSERT(depth <= this->m_setting_->tree_depth, "Depth must be in [0, %u], but got %u.\n", this->m_setting_->tree_depth, depth);
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
                    for (uint32_t i = 0; i < 8; ++i) {
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
            std::size_t number_of_leaf_nodes = this->ComputeNumberOfLeafNodes();
            std::size_t number_of_inner_nodes = m_tree_size_ - number_of_leaf_nodes;
            // TODO: update the statistics
            return sizeof(OctreeImpl) + this->GetMemoryUsagePerNode() * m_tree_size_ + number_of_inner_nodes * sizeof(Node *) * 8;
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
                    for (uint32_t i = 0; i < 8; ++i) {
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
        [[nodiscard]] inline OctreeKey::KeyType
        CoordToKey(double coordinate) const {
            return uint32_t(std::floor(coordinate * m_resolution_inv_)) + m_tree_key_offset_;
        }

        /**
         * Convert 1-dim coordinate to key at a given depth.
         * @param coordinate
         * @param depth
         * @return
         */
        [[nodiscard]] inline OctreeKey::KeyType
        CoordToKey(double coordinate, uint32_t depth) const {
            auto &tree_depth = this->m_setting_->tree_depth;
            ERL_DEBUG_ASSERT(depth <= tree_depth, "Depth must be in [0, %u], but got %u.\n", tree_depth, depth);
            uint32_t keyval = std::floor(coordinate * m_resolution_inv_);
            uint32_t diff = tree_depth - depth;
            if (!diff) { return keyval + m_tree_key_offset_; }
            return ((keyval >> diff) << diff) + (1 << (diff - 1)) + m_tree_key_offset_;
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
        CoordToKey(double x, double y, double z, uint32_t depth) const {
            if (depth == this->m_setting_->tree_depth) { return CoordToKey(x, y, z); }
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
            int scaled_coord = std::floor(coordinate * m_resolution_inv_) + m_tree_key_offset_;
            if ((scaled_coord >= 0) && ((uint32_t) scaled_coord) < (m_tree_key_offset_ << 1)) {
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
        CoordToKeyChecked(double coordinate, uint32_t depth, OctreeKey::KeyType &key) const {
            int scaled_coord = std::floor(coordinate * m_resolution_inv_) + m_tree_key_offset_;
            if ((scaled_coord >= 0) && ((uint32_t) scaled_coord) < (m_tree_key_offset_ << 1)) {
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
        CoordToKeyChecked(double x, double y, double z, uint32_t depth, OctreeKey &key) const {
            ERL_DEBUG_ASSERT(depth != 0, "When depth = 0, key is 0x0, which is useless!");
            if (depth == this->m_setting_->tree_depth) { return CoordToKeyChecked(x, y, z, key); }
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
        AdjustKeyToDepth(OctreeKey::KeyType key, uint32_t depth) const {
            const uint32_t &kTreeDepth = this->m_setting_->tree_depth;
            ERL_DEBUG_ASSERT(depth <= kTreeDepth, "Depth must be in [0, %u], but got %u.\n", kTreeDepth, depth);
            uint32_t diff = kTreeDepth - depth;
            if (!diff) { return key; }
            return (((key - m_tree_key_offset_) >> diff) << diff) + (1 << (diff - 1)) + m_tree_key_offset_;
        }

        /**
         * Adjust 3-dim key from the lowest level (max depth) to a given depth.
         * @param key the key at the lowest level
         * @param depth the target depth
         * @return
         */
        [[nodiscard]] inline OctreeKey
        AdjustKeyToDepth(const OctreeKey &key, uint32_t depth) const {
            if (depth == this->m_setting_->tree_depth) { return key; }
            return {AdjustKeyToDepth(key[0], depth), AdjustKeyToDepth(key[1], depth), AdjustKeyToDepth(key[2], depth)};
        }

        inline void
        ComputeCommonAncestorKey(const OctreeKey &key1, const OctreeKey &key2, OctreeKey &ancestor_key, uint32_t &ancestor_depth) const {
            OctreeKey::KeyType mask = (key1[0] ^ key2[0]) | (key1[1] ^ key2[1]) | (key1[2] ^ key2[2]);  // 0: same bit, 1: different bit
            const auto &kTreeDepth = this->m_setting_->tree_depth;
            if (!mask) {  // keys are identical
                ancestor_key = key1;
                ancestor_depth = kTreeDepth;
                return;
            }
            // from bit-max_depth to bit-0, find first 1
            uint32_t level = kTreeDepth;
            while (level > 0 && !(mask & (1 << (level - 1)))) { --level; }
            ancestor_depth = kTreeDepth - level;  // bit[level] = 0, bit[level-1] = 1
            OctreeKey::KeyType ancestor_mask = ((1l << kTreeDepth) - 1) << level;
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
        ComputeWestNeighborKey(const OctreeKey &key, uint32_t depth, OctreeKey &neighbor_key) const {
            OctreeKey::KeyType offset = 1 << (this->m_setting_->tree_depth - depth);
            if (key[0] < offset) { return false; }  // no west neighbor
            neighbor_key[0] = key[0] - offset;
            neighbor_key[1] = key[1];
            neighbor_key[2] = key[2];
            return true;
        }

        inline bool
        ComputeEastNeighborKey(const OctreeKey &key, uint32_t depth, OctreeKey &neighbor_key) const {
            const uint32_t &kTreeDepth = this->m_setting_->tree_depth;
            OctreeKey::KeyType offset = 1 << (kTreeDepth - depth);
            if ((1 << kTreeDepth) - key[0] <= offset) { return false; }  // no east neighbor (key[0] + offset >= 2^max_depth)
            neighbor_key[0] = key[0] + offset;
            neighbor_key[1] = key[1];
            neighbor_key[2] = key[2];
            return true;
        }

        inline bool
        ComputeNorthNeighborKey(const OctreeKey &key, uint32_t depth, OctreeKey &neighbor_key) const {
            const uint32_t &kTreeDepth = this->m_setting_->tree_depth;
            OctreeKey::KeyType offset = 1 << (kTreeDepth - depth);
            if ((1 << kTreeDepth) - key[1] <= offset) { return false; }  // no north neighbor (key[1] + offset >= 2^max_depth)
            neighbor_key[0] = key[0];
            neighbor_key[1] = key[1] + offset;
            neighbor_key[2] = key[2];
            return true;
        }

        inline bool
        ComputeSouthNeighborKey(const OctreeKey &key, uint32_t depth, OctreeKey &neighbor_key) const {
            OctreeKey::KeyType offset = 1 << (this->m_setting_->tree_depth - depth);
            if (key[1] < offset) { return false; }  // no south neighbor
            neighbor_key[0] = key[0];
            neighbor_key[1] = key[1] - offset;
            neighbor_key[2] = key[2];
            return true;
        }

        inline bool
        ComputeBottomNeighborKey(const OctreeKey &key, uint32_t depth, OctreeKey &neighbor_key) const {
            OctreeKey::KeyType offset = 1 << (this->m_setting_->tree_depth - depth);
            if (key[2] < offset) { return false; }  // no bottom neighbor
            neighbor_key[0] = key[0];
            neighbor_key[1] = key[1];
            neighbor_key[2] = key[2] - offset;
            return true;
        }

        inline bool
        ComputeTopNeighborKey(const OctreeKey &key, uint32_t depth, OctreeKey &neighbor_key) const {
            const uint32_t &kTreeDepth = this->m_setting_->tree_depth;
            OctreeKey::KeyType offset = 1 << (kTreeDepth - depth);
            if ((1 << kTreeDepth) - key[2] <= offset) { return false; }  // no top neighbor (key[2] + offset >= 2^max_depth)
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
            return (double(int(key) - int(m_tree_key_offset_)) + 0.5) * this->m_setting_->resolution;
        }

        /**
         * Convert 1-dim key to coordinate at a given depth.
         * @param key
         * @param depth
         * @return
         */
        [[nodiscard]] inline double
        KeyToCoord(OctreeKey::KeyType key, uint32_t depth) const {
            const uint32_t &kTreeDepth = this->m_setting_->tree_depth;
            if (depth == 0) { return 0.0; }
            if (depth == kTreeDepth) { return KeyToCoord(key); }
            return (std::floor((double(key) - double(m_tree_key_offset_)) / double(1 << (kTreeDepth - depth))) + 0.5) * GetNodeSize(depth);
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
        KeyToCoord(const OctreeKey &key, uint32_t depth, double &x, double &y, double &z) const {
            if (depth == 0) {
                x = y = z = 0.0;
                return;
            }
            if (depth == this->m_setting_->tree_depth) {
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
                Node *node = nullptr;
                OctreeKey key = {};
                std::shared_ptr<void> data = nullptr;  // data pointer for derived classes

                StackElement() = default;

                StackElement(Node *node, OctreeKey key)
                    : node(node),
                      key(std::move(key)) {}

                template<typename T>
                inline const T &
                GetData() const {
                    return *reinterpret_cast<T *>(data.get());
                }
            };

        protected:
            const OctreeImpl *m_tree_;               // the tree this iterator is working on
            uint32_t m_max_node_depth_;              // the maximum depth to query
            std::deque<StackElement> m_stack_ = {};  // stack for depth first traversal

        public:
            /**
             * Default constructor, only used for the end-iterator.
             */
            IteratorBase()
                : m_tree_(nullptr),
                  m_max_node_depth_(0) {}

            explicit IteratorBase(const OctreeImpl *tree, uint32_t max_node_depth)
                : m_tree_(tree),
                  m_max_node_depth_(max_node_depth) {
                if (m_tree_ == nullptr) { return; }
                if (m_max_node_depth_ == 0) { m_max_node_depth_ = m_tree_->GetTreeDepth(); }
                if (m_tree_->m_root_ != nullptr) {  // tree is not empty
                    m_stack_.emplace_back(m_tree_->m_root_.get(), m_tree_->CoordToKey(0.0, 0.0, 0.0));
                } else {
                    m_tree_ = nullptr;
                    m_max_node_depth_ = 0;
                }
            }

            [[nodiscard]] bool
            operator==(const IteratorBase &other) const {
                // we do not need to compare m_max_node_depth_ here, since it is always the same for the same tree
                if (m_tree_ != other.m_tree_) { return false; }
                if (m_stack_.size() != other.m_stack_.size()) { return false; }
                if (m_stack_.empty()) { return true; }

                const StackElement &kTop = m_stack_.back();
                auto &other_top = other.m_stack_.back();
                if (kTop.node != other_top.node) { return false; }
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
                return m_tree_->KeyToCoord(kTop.key[0], kTop.node->GetDepth());
            }

            [[nodiscard]] inline double
            GetY() const {
                const StackElement &kTop = m_stack_.back();
                return m_tree_->KeyToCoord(kTop.key[1], kTop.node->GetDepth());
            }

            [[nodiscard]] inline double
            GetZ() const {
                const StackElement &kTop = m_stack_.back();
                return m_tree_->KeyToCoord(kTop.key[2], kTop.node->GetDepth());
            }

            [[nodiscard]] inline double
            GetNodeSize() const {
                return m_tree_->GetNodeSize(m_stack_.back().node->GetDepth());
            }

            [[nodiscard]] inline Aabb3D
            GetNodeAabb() const {
                Eigen::Vector3d center(this->GetX(), this->GetY(), this->GetZ());
                return {center, this->GetNodeSize() * 0.5};
            }

            [[maybe_unused]] [[nodiscard]] inline const OctreeKey &
            GetKey() const {
                return m_stack_.back().key;
            }

            [[maybe_unused]] [[nodiscard]] inline OctreeKey
            GetIndexKey() const {
                const StackElement &kTop = m_stack_.back();
                return m_tree_->AdjustKeyToDepth(kTop.key, kTop.node->GetDepth());
            }

        protected:
            /**
             * One step of depth-first tree traversal.
             */
            void
            SingleIncrement1() {
                StackElement top = m_stack_.back();
                m_stack_.pop_back();
                const uint32_t &kNodeDepth = top.node->GetDepth();
                if (kNodeDepth == m_max_node_depth_) { return; }

                uint32_t next_depth = kNodeDepth + 1;
                ERL_DEBUG_ASSERT(next_depth <= m_max_node_depth_, "Wrong depth: %u (max: %u).\n", next_depth, m_max_node_depth_);
                OctreeKey next_key;
                OctreeKey::KeyType center_offset_key = m_tree_->m_tree_key_offset_ >> next_depth;
                // push on stack in reverse order
                for (int i = 7; i >= 0; --i) {
                    if (top.node->HasChild(i)) {
                        OctreeKey::ComputeChildKey(i, center_offset_key, top.key, next_key);
                        m_stack_.emplace_back(const_cast<Node *>(m_tree_->GetNodeChild(top.node, i)), next_key);
                    }
                }
            }

            [[nodiscard]] inline bool
            IsLeaf() const {
                ERL_DEBUG_ASSERT(!m_stack_.empty(), "Stack is empty.");
                return !m_stack_.back().node->HasAnyChild();
            }

            inline void
            Terminate() {
                this->m_stack_.clear();
                this->m_tree_ = nullptr;
                this->m_max_node_depth_ = 0;
            }

            /**
             * @brief Get the biggest in-tree axis-aligned bounding box (AABB) that is inside the given AABB.
             *
             * @param aabb_min_x
             * @param aabb_min_y
             * @param aabb_min_z
             * @param aabb_max_x
             * @param aabb_max_y
             * @param aabb_max_z
             * @return true if such an AABB exists, false otherwise.
             */
            inline bool
            GetInTreeAabb(double &aabb_min_x, double &aabb_min_y, double &aabb_min_z, double &aabb_max_x, double &aabb_max_y, double &aabb_max_z) const {
                if (m_tree_ == nullptr) { return false; }

                double center = m_tree_->KeyToCoord(m_tree_->m_tree_key_offset_);
                double half_size = m_tree_->GetNodeSize(0) / 2.0;
                double aabb_min = center - half_size;
                double aabb_max = center + half_size;

                aabb_min_x = std::max(aabb_min_x, aabb_min);
                aabb_max_x = std::min(aabb_max_x, aabb_max);
                if (aabb_min_x >= aabb_max_x) { return false; }
                aabb_min_y = std::max(aabb_min_y, aabb_min);
                aabb_max_y = std::min(aabb_max_y, aabb_max);
                if (aabb_min_y >= aabb_max_y) { return false; }
                aabb_min_z = std::max(aabb_min_z, aabb_min);
                aabb_max_z = std::min(aabb_max_z, aabb_max);
                if (aabb_min_z >= aabb_max_z) { return false; }
                return true;
            }
        };

        class TreeIterator : public IteratorBase {
        public:
            TreeIterator() = default;

            explicit TreeIterator(const OctreeImpl *tree, uint32_t max_node_depth = 0)
                : IteratorBase(tree, max_node_depth) {}

            // post-increment
            inline TreeIterator
            operator++(int) {
                const TreeIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            inline TreeIterator &
            operator++() {
                if (!this->m_stack_.empty()) { this->SingleIncrement1(); }
                if (this->m_stack_.empty()) { this->Terminate(); }
                return *this;
            }
        };

        class InAabbIteratorBase : public IteratorBase {
        private:
            OctreeKey m_aabb_min_key_;
            OctreeKey m_aabb_max_key_;

        public:
            InAabbIteratorBase() = default;

            InAabbIteratorBase(
                double aabb_min_x,
                double aabb_min_y,
                double aabb_min_z,
                double aabb_max_x,
                double aabb_max_y,
                double aabb_max_z,
                const OctreeImpl *tree,
                uint32_t max_node_depth = 0)
                : IteratorBase(tree, max_node_depth) {
                if (this->m_stack_.empty()) { return; }
                ERL_DEBUG_ASSERT(tree != nullptr, "Tree is null.");

                // If the provided AABB is too large, CoordsToKeyChecked will return false. We should avoid this case.
                if (!this->GetInTreeAabb(aabb_min_x, aabb_min_y, aabb_min_z, aabb_max_x, aabb_max_y, aabb_max_z)) {
                    this->Terminate();  // the tree is not in the AABB at all
                    return;
                }

                if (this->m_tree_->CoordToKeyChecked(aabb_min_x, aabb_min_y, aabb_min_z, m_aabb_min_key_) &&
                    this->m_tree_->CoordToKeyChecked(aabb_max_x, aabb_max_y, aabb_max_z, m_aabb_max_key_)) {
                    // check if the root node is in the AABB
                    typename IteratorBase::StackElement top = this->m_stack_.back();
                    OctreeKey::KeyType center_offset_key = this->m_tree_->m_tree_key_offset_ >> top.node->GetDepth();
                    if (!OctreeKey::KeyInAabb(top.key, center_offset_key, m_aabb_min_key_, m_aabb_max_key_)) {
                        this->Terminate();
                        return;
                    }
                } else {
                    this->Terminate();  // the AABB is out of the tree, but unlikely to happen here. We still check it for safety.
                }
            }

            InAabbIteratorBase(OctreeKey aabb_min_key, OctreeKey aabb_max_key, const OctreeImpl *tree, uint32_t max_node_depth = 0)
                : IteratorBase(tree, max_node_depth),
                  m_aabb_min_key_(std::move(aabb_min_key)),
                  m_aabb_max_key_(std::move(aabb_max_key)) {
                if (this->m_stack_.empty()) { return; }
                ERL_DEBUG_ASSERT(tree != nullptr, "Tree is null.");

                // check if the root node is in the AABB
                typename IteratorBase::StackElement top = this->m_stack_.back();
                OctreeKey::KeyType center_offset_key = this->m_tree_->m_tree_key_offset_ >> top.node->GetDepth();
                if (!OctreeKey::KeyInAabb(top.key, center_offset_key, m_aabb_min_key_, m_aabb_max_key_)) {
                    this->Terminate();
                    return;
                }
            }

        protected:
            void
            SingleIncrement2() {
                typename IteratorBase::StackElement top = this->m_stack_.back();
                this->m_stack_.pop_back();
                const uint32_t &kNodeDepth = top.node->GetDepth();
                if (kNodeDepth == this->m_max_node_depth_) { return; }

                uint32_t next_depth = kNodeDepth + 1;
                ERL_DEBUG_ASSERT(next_depth <= this->m_max_node_depth_, "Wrong depth: %u (max: %u).\n", next_depth, this->m_max_node_depth_);
                OctreeKey next_key;
                OctreeKey::KeyType center_offset_key = this->m_tree_->m_tree_key_offset_ >> next_depth;
                // push on stack in reverse order
                for (int i = 7; i >= 0; --i) {
                    if (!top.node->HasChild(i)) { continue; }
                    OctreeKey::ComputeChildKey(i, center_offset_key, top.key, next_key);
                    // check if the child node overlaps with the AABB
                    if (OctreeKey::KeyInAabb(next_key, center_offset_key, m_aabb_min_key_, m_aabb_max_key_)) {
                        this->m_stack_.emplace_back(const_cast<Node *>(this->m_tree_->GetNodeChild(top.node, i)), next_key);
                    }
                }
            }
        };

        class TreeInAabbIterator : public InAabbIteratorBase {
        public:
            TreeInAabbIterator() = default;

            TreeInAabbIterator(
                double aabb_min_x,
                double aabb_min_y,
                double aabb_min_z,
                double aabb_max_x,
                double aabb_max_y,
                double aabb_max_z,
                const OctreeImpl *tree,
                uint32_t max_node_depth = 0)
                : InAabbIteratorBase(aabb_min_x, aabb_min_y, aabb_min_z, aabb_max_x, aabb_max_y, aabb_max_z, tree, max_node_depth) {}

            TreeInAabbIterator(OctreeKey aabb_min_key, OctreeKey aabb_max_key, const OctreeImpl *tree, uint32_t max_node_depth = 0)
                : InAabbIteratorBase(std::move(aabb_min_key), std::move(aabb_max_key), tree, max_node_depth) {}

            // post-increment
            inline TreeInAabbIterator
            operator++(int) {
                const TreeInAabbIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            inline TreeInAabbIterator &
            operator++() {
                if (!this->m_stack_.empty()) { this->SingleIncrement2(); }
                if (this->m_stack_.empty()) { this->Terminate(); }
                return *this;
            }
        };

        /**
         * Iterate over all leaf nodes that intersect with the given axis-aligned bounding box.
         */
        class LeafInAabbIterator : public InAabbIteratorBase {
        public:
            LeafInAabbIterator() = default;

            LeafInAabbIterator(
                double aabb_min_x,
                double aabb_min_y,
                double aabb_min_z,
                double aabb_max_x,
                double aabb_max_y,
                double aabb_max_z,
                const OctreeImpl *tree,
                uint32_t max_leaf_depth = 0)
                : InAabbIteratorBase(aabb_min_x, aabb_min_y, aabb_min_z, aabb_max_x, aabb_max_y, aabb_max_z, tree, max_leaf_depth) {
                while (!this->m_stack_.empty() && !this->IsLeaf()) { this->SingleIncrement2(); }
                if (this->m_stack_.empty()) { this->Terminate(); }
            }

            LeafInAabbIterator(OctreeKey aabb_min_key, OctreeKey aabb_max_key, const OctreeImpl *tree, uint32_t max_leaf_depth = 0)
                : InAabbIteratorBase(std::move(aabb_min_key), std::move(aabb_max_key), tree, max_leaf_depth) {
                while (!this->m_stack_.empty() && !this->IsLeaf()) { this->SingleIncrement2(); }
                if (this->m_stack_.empty()) { this->Terminate(); }
            }

            // post-increment
            LeafInAabbIterator
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

                do { this->SingleIncrement2(); } while (!this->m_stack_.empty() && !this->IsLeaf());
                if (this->m_stack_.empty()) { this->Terminate(); }
                return *this;
            }
        };

        class LeafIterator : public IteratorBase {
        public:
            LeafIterator() = default;

            explicit LeafIterator(const OctreeImpl *tree, uint32_t depth = 0)
                : IteratorBase(tree, depth) {
                if (this->m_stack_.empty()) { return; }
                // skip forward to next valid leaf node
                while (!this->m_stack_.empty() && !this->IsLeaf()) { this->SingleIncrement1(); }
                if (this->m_stack_.empty()) { this->Terminate(); }
            }

            // post-increment
            LeafIterator
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

                do { this->SingleIncrement1(); } while (!this->m_stack_.empty() && !this->IsLeaf());
                if (this->m_stack_.empty()) { this->Terminate(); }
                return *this;
            }
        };

        class LeafOfNodeIterator : public IteratorBase {
        public:
            LeafOfNodeIterator() = default;

            LeafOfNodeIterator(OctreeKey key, uint32_t cluster_depth, const OctreeImpl *tree, uint32_t max_node_depth = 0)
                : IteratorBase(tree, max_node_depth) {
                ERL_ASSERTM(
                    cluster_depth <= this->m_max_node_depth_,
                    "Cluster max_node_depth %u is greater than max max_node_depth %u.\n",
                    cluster_depth,
                    this->m_max_node_depth_);

                // modify stack top
                auto &s = this->m_stack_.back();
                s.node = const_cast<Node *>(this->m_tree_->Search(key, cluster_depth));
                if (s.node == nullptr) {
                    this->Terminate();
                    return;
                }
                s.key = key;

                // skip forward to next valid leaf node
                while (!this->m_stack_.empty() && !this->IsLeaf()) { this->SingleIncrement1(); }
                if (this->m_stack_.empty()) { this->Terminate(); }
            }

            // post-increment
            LeafOfNodeIterator
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

                do { this->SingleIncrement1(); } while (!this->m_stack_.empty() && !this->IsLeaf());
                if (this->m_stack_.empty()) { this->Terminate(); }
                return *this;
            }
        };

        class LeafNeighborIteratorBase : public IteratorBase {
        public:
            LeafNeighborIteratorBase() = default;

            LeafNeighborIteratorBase(const OctreeImpl *tree, uint32_t max_leaf_depth)
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
            Init(OctreeKey key, uint32_t key_depth, int changing_dim1, int changing_dim2, int unchanged_dim, bool increase) {
                if (this->m_tree_ == nullptr) { return; }
                uint32_t max_depth = this->m_tree_->GetTreeDepth();
                uint32_t level = max_depth - key_depth;
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
                this->SingleIncrementOf(changing_dim1, changing_dim2);
            }

            void
            SingleIncrementOf(int changing_dim1, int changing_dim2) {
                auto &s = this->m_stack_.back();

                OctreeKey::KeyType &key_changing_dim1 = this->m_neighbor_key_[changing_dim1];
                OctreeKey::KeyType &key_changing_dim2 = this->m_neighbor_key_[changing_dim2];
                s.node = nullptr;
                while (s.node == nullptr && key_changing_dim1 < m_max_key_changing_dim1_ && key_changing_dim2 < m_max_key_changing_dim2_) {
                    s.node = const_cast<Node *>(this->m_tree_->Search(m_neighbor_key_));
                    const uint32_t &kNodeDepth = s.node->GetDepth();
                    if (s.node == nullptr || kNodeDepth > this->m_max_node_depth_) {  // not found
                        s.node = nullptr;
                        ++key_changing_dim2;
                        if (key_changing_dim2 >= m_max_key_changing_dim2_) {  // go to next row
                            key_changing_dim2 = m_min_key_changing_dim2_;
                            ++key_changing_dim1;
                        }
                        continue;
                    }

                    // found a neighbor
                    s.key = this->m_tree_->AdjustKeyToDepth(m_neighbor_key_, kNodeDepth);
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
                    uint32_t max_depth = this->m_tree_->GetTreeDepth();
                    key_changing_dim2 = s.key[changing_dim2] + (kNodeDepth == max_depth ? 1 : (1 << (max_depth - kNodeDepth - 1)));
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

            WestLeafNeighborIterator(double x, double y, double z, const OctreeImpl *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {

                OctreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, z, key)) {
                    this->Terminate();
                    return;
                }

                const Node *node = this->m_tree_->Search(key);
                if (node == nullptr) {
                    this->Terminate();
                    return;
                }

                this->Init(key, node->GetDepth(), sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            WestLeafNeighborIterator(const OctreeKey &key, uint32_t key_depth, const OctreeImpl *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            // post-increment
            inline WestLeafNeighborIterator
            operator++(int) {
                const WestLeafNeighborIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            inline WestLeafNeighborIterator &
            operator++() {
                if (!this->m_stack_.empty()) { this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_); }
                if (this->m_stack_.empty()) { this->Terminate(); }
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

            EastLeafNeighborIterator(double x, double y, double z, const OctreeImpl *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {

                OctreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, z, key)) {
                    this->Terminate();
                    return;
                }

                const Node *node = this->m_tree_->Search(key);
                if (node == nullptr) {
                    this->Terminate();
                    return;
                }
                this->Init(key, node->GetDepth(), sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            EastLeafNeighborIterator(const OctreeKey &key, uint32_t key_depth, const OctreeImpl *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            // post-increment
            inline EastLeafNeighborIterator
            operator++(int) {
                const EastLeafNeighborIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            inline EastLeafNeighborIterator &
            operator++() {
                if (!this->m_stack_.empty()) { this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_); }
                if (this->m_stack_.empty()) { this->Terminate(); }
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

            NorthLeafNeighborIterator(double x, double y, double z, const OctreeImpl *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {

                OctreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, z, key)) {
                    this->Terminate();
                    return;
                }

                const Node *node = this->m_tree_->Search(key);
                if (node == nullptr) {
                    this->Terminate();
                    return;
                }

                this->Init(key, node->GetDepth(), sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            NorthLeafNeighborIterator(const OctreeKey &key, uint32_t key_depth, const OctreeImpl *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            // post-increment
            inline NorthLeafNeighborIterator
            operator++(int) {
                const NorthLeafNeighborIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            inline NorthLeafNeighborIterator &
            operator++() {
                if (!this->m_stack_.empty()) { this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_); }
                if (this->m_stack_.empty()) { this->Terminate(); }
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

            SouthLeafNeighborIterator(double x, double y, double z, const OctreeImpl *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                OctreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, z, key)) {
                    this->Terminate();
                    return;
                }

                const Node *node = this->m_tree_->Search(key);
                if (node == nullptr) {
                    this->Terminate();
                    return;
                }

                this->Init(key, node->GetDepth(), sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            SouthLeafNeighborIterator(const OctreeKey &key, uint32_t key_depth, const OctreeImpl *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {
                this->Init(key, key_depth, sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            // post-increment
            inline SouthLeafNeighborIterator
            operator++(int) {
                const SouthLeafNeighborIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            inline SouthLeafNeighborIterator &
            operator++() {
                if (!this->m_stack_.empty()) { this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_); }
                if (this->m_stack_.empty()) { this->Terminate(); }
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

            TopLeafNeighborIterator(double x, double y, double z, const OctreeImpl *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {

                OctreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, z, key)) {
                    this->Terminate();
                    return;
                }

                const Node *node = this->m_tree_->Search(key);
                if (node == nullptr) {
                    this->Terminate();
                    return;
                }

                this->Init(key, node->GetDepth(), sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            TopLeafNeighborIterator(const OctreeKey &key, uint32_t key_depth, const OctreeImpl *tree, uint32_t max_leaf_depth)
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
                if (!this->m_stack_.empty()) { this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_); }
                if (this->m_stack_.empty()) { this->Terminate(); }
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

            BottomLeafNeighborIterator(double x, double y, double z, const OctreeImpl *tree, uint32_t max_leaf_depth)
                : LeafNeighborIteratorBase(tree, max_leaf_depth) {

                OctreeKey key;
                if (!this->m_tree_->CoordToKeyChecked(x, y, z, key)) {
                    this->Terminate();
                    return;
                }

                const Node *node = this->m_tree_->Search(key);
                if (node == nullptr) {
                    this->Terminate();
                    return;
                }

                this->Init(key, node->GetDepth(), sk_ChangingDim1_, sk_ChangingDim2_, sk_UnchangedDim_, sk_Increase_);
            }

            BottomLeafNeighborIterator(const OctreeKey &key, uint32_t key_depth, const OctreeImpl *tree, uint32_t max_leaf_depth)
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
                if (!this->m_stack_.empty()) { this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_); }
                if (this->m_stack_.empty()) { this->Terminate(); }
                return *this;
            }
        };

        class NodeOnRayIterator : public IteratorBase {
            Eigen::Vector3d m_origin_ = {};
            Eigen::Vector3d m_dir_ = {};
            Eigen::Vector3d m_dir_inv_ = {};
            double m_max_range_ = 0.;
            bool m_bidirectional_ = false;
            bool m_leaf_only_ = false;
            uint32_t m_min_node_depth_ = 0;

        public:
            NodeOnRayIterator() = default;

            /**
             *
             * @param px
             * @param py
             * @param pz
             * @param vx
             * @param vy
             * @param vz
             * @param max_range
             * @param bidirectional
             * @param tree
             * @param leaf_only
             * @param min_node_depth
             * @param max_node_depth 0 means using the tree's max depth
             */
            NodeOnRayIterator(
                double px,
                double py,
                double pz,
                double vx,
                double vy,
                double vz,
                double max_range,
                bool bidirectional,
                const OctreeImpl *tree,
                bool leaf_only = false,
                uint32_t min_node_depth = 0,
                uint32_t max_node_depth = 0)
                : IteratorBase(tree, max_node_depth),
                  m_origin_(px, py, pz),
                  m_dir_(vx, vy, vz),
                  m_max_range_(max_range),
                  m_bidirectional_(bidirectional),
                  m_leaf_only_(leaf_only),
                  m_min_node_depth_(min_node_depth) {
                m_dir_inv_ = m_dir_.cwiseInverse();
                if (this->m_stack_.empty()) { return; }
                this->m_stack_.back().data = std::make_shared<double>(0.);
                this->SingleIncrement2();
                if (this->m_stack_.empty()) { this->Terminate(); }
            }

            [[maybe_unused]] [[nodiscard]] inline double
            GetDistance() const {
                if (this->m_stack_.empty()) { return 0.; }
                return *reinterpret_cast<double *>(this->m_stack_.back().data.get());
            }

            // post-increment
            NodeOnRayIterator
            operator++(int) {
                const NodeOnRayIterator kResult = *this;
                ++(*this);
                return kResult;
            }

            // pre-increment
            NodeOnRayIterator &
            operator++() {
                if (!this->m_stack_.empty()) { this->SingleIncrement2(); }
                if (this->m_stack_.empty()) { this->Terminate(); }
                return *this;
            }

        protected:
            struct StackElementCompare {  // for min-heap

                inline bool
                operator()(const typename IteratorBase::StackElement &lhs, const typename IteratorBase::StackElement &rhs) const {
                    const double &kLhsDist = lhs.template GetData<double>();
                    const double &kRhsDist = rhs.template GetData<double>();
                    if (kLhsDist >= 0) {
                        if (kRhsDist >= 0) { return kLhsDist > kRhsDist; }  // both are positive
                        return false;
                    } else {
                        if (kRhsDist >= 0) { return true; }
                        return kLhsDist < kRhsDist;  // both are negative
                    }
                }
            };

            void
            SingleIncrement2() {
                // make_heap: make the front element the smallest
                // pop_heap: move the smallest element to the back
                // push_heap: rectify the heap after adding a new element to the back
                // stack pop is the last element

                StackElementCompare cmp;
                typename IteratorBase::StackElement s = this->m_stack_.back();  // s.node is leaf node unless it is root node
                this->m_stack_.pop_back();

                // s.node may be the root node, m_min_node_depth_ may be 0
                if (!s.node->HasAnyChild()) {      // leaf node, we need to find the next internal node
                    if (this->m_stack_.empty()) {  // end of iteration
                        this->Terminate();
                        return;
                    } else {                                                                // check the next stack top
                        std::pop_heap(this->m_stack_.begin(), this->m_stack_.end(), cmp);   // now the stack top is at the back
                        if (this->m_stack_.back().node->GetDepth() >= m_min_node_depth_) {  // current stack top is deep enough
                            if (!this->m_leaf_only_) { return; }                            // stack top is not required to be leaf node
                            if (this->IsLeaf()) { return; }                                 // current stack top is leaf node
                        }
                        s = this->m_stack_.back();  // current stack top is internal node
                        this->m_stack_.pop_back();
                    }
                }

                // now s.node is internal node
                while (true) {
                    typename IteratorBase::StackElement s_child;
                    uint32_t child_depth = s.node->GetDepth() + 1;
                    if (child_depth <= this->m_max_node_depth_) {  // do not go beyond the specified max depth
                        OctreeKey::KeyType center_offset_key = this->m_tree_->m_tree_key_offset_ >> child_depth;
                        for (int i = 7; i >= 0; --i) {
                            if (!s.node->HasChild(i)) { continue; }

                            const Node *child = this->m_tree_->GetNodeChild(s.node, i);
                            OctreeKey::ComputeChildKey(i, center_offset_key, s.key, s_child.key);

                            double center_x = this->m_tree_->KeyToCoord(s_child.key[0], child_depth);
                            double center_y = this->m_tree_->KeyToCoord(s_child.key[1], child_depth);
                            double center_z = this->m_tree_->KeyToCoord(s_child.key[2], child_depth);
                            double half_size = this->m_tree_->GetNodeSize(child_depth) / 2.0;
                            Eigen::Vector3d box_min(center_x - half_size, center_y - half_size, center_z - half_size);
                            Eigen::Vector3d box_max(center_x + half_size, center_y + half_size, center_z + half_size);
                            double dist1 = 0.0, dist2 = 0.0;
                            bool intersected = false;
                            ComputeIntersectionBetweenRayAndAabb3D(m_origin_, m_dir_inv_, box_min, box_max, dist1, dist2, intersected);
                            if (!intersected) { continue; }
                            if (!child->HasAnyChild()) {  // leaf node
                                if (!m_bidirectional_ && dist1 < 0.) { continue; }
                                if (m_max_range_ > 0. && std::abs(dist1) > m_max_range_) { continue; }
                            }
                            s_child.node = const_cast<Node *>(child);
                            s_child.data = std::make_shared<double>(dist1);
                            this->m_stack_.push_back(s_child);
                            std::push_heap(this->m_stack_.begin(), this->m_stack_.end(), cmp);
                        }
                    }
                    if (this->m_stack_.empty()) {  // end of iteration
                        this->Terminate();
                        return;
                    }

                    std::pop_heap(this->m_stack_.begin(), this->m_stack_.end(), cmp);   // now the stack top is at the back
                    if (this->m_stack_.back().node->GetDepth() >= m_min_node_depth_) {  // current stack top is deep enough
                        if (!this->m_leaf_only_) { return; }                            // stack top is not required to be leaf node
                        if (this->IsLeaf()) { return; }                                 // current stack top is leaf node
                    }
                    s = this->m_stack_.back();  // current stack top is internal node
                    this->m_stack_.pop_back();
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

        [[maybe_unused]] [[nodiscard]] inline LeafOfNodeIterator
        BeginLeafOfNode(const OctreeKey &key, uint32_t node_depth, uint32_t max_depth = 0) const {
            return LeafOfNodeIterator(key, node_depth, this, max_depth);
        }

        [[maybe_unused]] [[nodiscard]] inline LeafOfNodeIterator
        EndLeafOfNode() const {
            return LeafOfNodeIterator();
        }

        [[nodiscard]] inline LeafInAabbIterator
        BeginLeafInAabb(
            double aabb_min_x,  //
            double aabb_min_y,
            double aabb_min_z,
            double aabb_max_x,
            double aabb_max_y,
            double aabb_max_z,
            uint32_t max_depth = 0) const {
            return LeafInAabbIterator(aabb_min_x, aabb_min_y, aabb_min_z, aabb_max_x, aabb_max_y, aabb_max_z, this, max_depth);
        }

        [[nodiscard]] inline LeafInAabbIterator
        BeginLeafInAabb(const OctreeKey &aabb_min_key, const OctreeKey &aabb_max_key, uint32_t max_depth = 0) const {
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
            double aabb_min_x,  //
            double aabb_min_y,
            double aabb_min_z,
            double aabb_max_x,
            double aabb_max_y,
            double aabb_max_z,
            uint32_t max_depth = 0) const {
            return TreeInAabbIterator(aabb_min_x, aabb_min_y, aabb_min_z, aabb_max_x, aabb_max_y, aabb_max_z, this, max_depth);
        }

        [[nodiscard]] inline TreeInAabbIterator
        BeginTreeInAabb(const OctreeKey &aabb_min_key, const OctreeKey &aabb_max_key, uint32_t max_depth = 0) const {
            return TreeInAabbIterator(aabb_min_key, aabb_max_key, this, max_depth);
        }

        [[nodiscard]] inline TreeInAabbIterator
        EndTreeInAabb() const {
            return TreeInAabbIterator();
        }

        [[maybe_unused]] [[nodiscard]] inline WestLeafNeighborIterator
        BeginWestLeafNeighbor(double x, double y, double z, uint32_t max_leaf_depth = 0) const {
            return WestLeafNeighborIterator(x, y, z, this, max_leaf_depth);
        }

        [[maybe_unused]] [[nodiscard]] inline WestLeafNeighborIterator
        BeginWestLeafNeighbor(const OctreeKey &key, uint32_t key_depth, uint32_t max_leaf_depth = 0) const {
            return WestLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[maybe_unused]] [[nodiscard]] inline WestLeafNeighborIterator
        EndWestLeafNeighbor() const {
            return WestLeafNeighborIterator();
        }

        [[maybe_unused]] [[nodiscard]] inline EastLeafNeighborIterator
        BeginEastLeafNeighbor(double x, double y, double z, uint32_t max_leaf_depth = 0) const {
            return EastLeafNeighborIterator(x, y, z, this, max_leaf_depth);
        }

        [[maybe_unused]] [[nodiscard]] inline EastLeafNeighborIterator
        BeginEastLeafNeighbor(const OctreeKey &key, uint32_t key_depth, uint32_t max_leaf_depth = 0) const {
            return EastLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[maybe_unused]] [[nodiscard]] inline EastLeafNeighborIterator
        EndEastLeafNeighbor() const {
            return EastLeafNeighborIterator();
        }

        [[maybe_unused]] [[nodiscard]] inline NorthLeafNeighborIterator
        BeginNorthLeafNeighbor(double x, double y, double z, uint32_t max_leaf_depth = 0) const {
            return NorthLeafNeighborIterator(x, y, z, this, max_leaf_depth);
        }

        [[maybe_unused]] [[nodiscard]] inline NorthLeafNeighborIterator
        BeginNorthLeafNeighbor(const OctreeKey &key, uint32_t key_depth, uint32_t max_leaf_depth = 0) const {
            return NorthLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[maybe_unused]] [[nodiscard]] inline NorthLeafNeighborIterator
        EndNorthLeafNeighbor() const {
            return NorthLeafNeighborIterator();
        }

        [[maybe_unused]] [[nodiscard]] inline SouthLeafNeighborIterator
        BeginSouthLeafNeighbor(double x, double y, double z, uint32_t max_leaf_depth = 0) const {
            return SouthLeafNeighborIterator(x, y, z, this, max_leaf_depth);
        }

        [[maybe_unused]] [[nodiscard]] inline SouthLeafNeighborIterator
        BeginSouthLeafNeighbor(const OctreeKey &key, uint32_t key_depth, uint32_t max_leaf_depth = 0) const {
            return SouthLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[maybe_unused]] [[nodiscard]] inline SouthLeafNeighborIterator
        EndSouthLeafNeighbor() const {
            return SouthLeafNeighborIterator();
        }

        [[maybe_unused]] [[nodiscard]] inline TopLeafNeighborIterator
        BeginTopLeafNeighbor(double x, double y, double z, uint32_t max_leaf_depth = 0) const {
            return TopLeafNeighborIterator(x, y, z, this, max_leaf_depth);
        }

        [[maybe_unused]] [[nodiscard]] inline TopLeafNeighborIterator
        BeginTopLeafNeighbor(const OctreeKey &key, uint32_t key_depth, uint32_t max_leaf_depth = 0) const {
            return TopLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[maybe_unused]] [[nodiscard]] inline TopLeafNeighborIterator
        EndTopLeafNeighbor() const {
            return TopLeafNeighborIterator();
        }

        [[maybe_unused]] [[nodiscard]] inline BottomLeafNeighborIterator
        BeginBottomLeafNeighbor(double x, double y, double z, uint32_t max_leaf_depth = 0) const {
            return BottomLeafNeighborIterator(x, y, z, this, max_leaf_depth);
        }

        [[maybe_unused]] [[nodiscard]] inline BottomLeafNeighborIterator
        BeginBottomLeafNeighbor(const OctreeKey &key, uint32_t key_depth, uint32_t max_leaf_depth = 0) const {
            return BottomLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
        }

        [[maybe_unused]] [[nodiscard]] inline BottomLeafNeighborIterator
        EndBottomLeafNeighbor() const {
            return BottomLeafNeighborIterator();
        }

        [[maybe_unused]] [[nodiscard]] inline NodeOnRayIterator
        BeginNodeOnRay(
            double px,
            double py,
            double pz,
            double vx,
            double vy,
            double vz,
            double max_range = -1,
            bool bidirectional = false,
            bool leaf_only = false,
            uint32_t min_node_depth = 0,
            uint32_t max_node_depth = 0) const {
            return NodeOnRayIterator(px, py, pz, vx, vy, vz, max_range, bidirectional, /*tree*/ this, leaf_only, min_node_depth, max_node_depth);
        }

        [[maybe_unused]] [[nodiscard]] inline NodeOnRayIterator
        EndNodeOnRay() const {
            return NodeOnRayIterator();
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
            const double &kResolution = this->m_setting_->resolution;
            double t_max[3];
            double t_delta[3];
            if (step[0] == 0) {
                t_max[0] = std::numeric_limits<double>::infinity();
                t_delta[0] = std::numeric_limits<double>::infinity();
            } else {
                double voxel_border = KeyToCoord(key_start[0]) + double(step[0]) * 0.5 * kResolution;
                t_max[0] = (voxel_border - sx) / dx;
                t_delta[0] = kResolution / std::abs(dx);
            }
            if (step[1] == 0) {
                t_max[1] = std::numeric_limits<double>::infinity();
                t_delta[1] = std::numeric_limits<double>::infinity();
            } else {
                double voxel_border = KeyToCoord(key_start[1]) + double(step[1]) * 0.5 * kResolution;
                t_max[1] = (voxel_border - sy) / dy;
                t_delta[1] = kResolution / std::abs(dy);
            }
            if (step[2] == 0) {
                t_max[2] = std::numeric_limits<double>::infinity();
                t_delta[2] = std::numeric_limits<double>::infinity();
            } else {
                double voxel_border = KeyToCoord(key_start[2]) + double(step[2]) * 0.5 * kResolution;
                t_max[2] = (voxel_border - sz) / dz;
                t_delta[2] = kResolution / std::abs(dz);
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
                    current_key[idx] < (m_tree_key_offset_ << 1),
                    "current_key[%d] = %d exceeds limit %d.\n",
                    idx,
                    current_key[idx],
                    (m_tree_key_offset_ << 1));

                if (current_key == key_end) { break; }

                // this seems to be unlikely to happen
                double dist_from_origin = std::min(t_max[0], std::min(t_max[1], t_max[2]));
                if (dist_from_origin > length) { break; }  // this happens due to numerical error

                ray.AddKey(current_key);
                ERL_DEBUG_ASSERT(ray.size() < ray.capacity() - 1, "Ray capacity is not enough.");
            }

            return true;
        }

        /**
         * Trace a ray from origin to end (excluded), returning a list of all nodes' coordinates traversed by the ray. For each coordinate, the
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
        ComputeRayCoords(double sx, double sy, double sz, double ex, double ey, double ez, std::vector<Eigen::Vector3d> &ray) const {
            ray.clear();
            OctreeKeyRay key_ray;
            if (!ComputeRayKeys(sx, sy, sz, ex, ey, ez, key_ray)) { return false; }
            ray.reserve(key_ray.size());
            std::transform(key_ray.begin(), key_ray.end(), std::back_inserter(ray), [this](const OctreeKey &key) -> Eigen::Vector3d {
                return {this->KeyToCoord(key[0]), this->KeyToCoord(key[1]), this->KeyToCoord(key[2])};
            });
            return true;
        }

        /**
         * Clear KeyRay vector to minimize unneeded memory. This is only useful for the StaticMemberInitializer classes, don't call it for a quadtree that
         * is actually used.
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

        inline uint32_t
        DeleteNodeChild(Node *node, uint32_t child_idx, const OctreeKey &key) {
            uint32_t old_tree_size = m_tree_size_;
            Node *child = node->template GetChild<Node>(child_idx);
            this->DeleteNodeDescendants(child, key);
            this->OnDeleteNodeChild(node, child, key);
            node->RemoveChild(child_idx);
            m_tree_size_--;
            m_size_changed_ = true;
            return old_tree_size - m_tree_size_;
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
            if (node->GetNumChildren() != 8) { return false; }
            // child should be a leaf node
            // if child is a leaf node, its data should be equal to the first child
            // we don't need to check if their depth etc. are the same, because they are all children of the same node
            auto child_0 = this->GetNodeChild(node, 0);
            if (!child_0->AllowMerge(this->GetNodeChild(node, 1))) { return false; }
            if (!child_0->AllowMerge(this->GetNodeChild(node, 2))) { return false; }
            if (!child_0->AllowMerge(this->GetNodeChild(node, 3))) { return false; }
            if (!child_0->AllowMerge(this->GetNodeChild(node, 4))) { return false; }
            if (!child_0->AllowMerge(this->GetNodeChild(node, 5))) { return false; }
            if (!child_0->AllowMerge(this->GetNodeChild(node, 6))) { return false; }
            if (!child_0->AllowMerge(this->GetNodeChild(node, 7))) { return false; }
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
            node->Expand();
            ERL_DEBUG_ASSERT(node->GetNumChildren() == 8, "Node should have 8 children.");
            m_tree_size_ += 8;
            m_size_changed_ = true;
        }

        /**
         * Prune a node: delete all children if the node is collapsible.
         * @param node
         * @return
         */
        bool
        PruneNode(Node *node) {
            if (!IsNodeCollapsible(node)) { return false; }
            ERL_DEBUG_ASSERT(node->GetNumChildren() == 8, "Node should have 8 children.");
            node->Prune();
            ERL_DEBUG_ASSERT(node->GetNumChildren() == 0, "Node should have no children.");
            m_tree_size_ -= 8;
            m_size_changed_ = true;
            return true;
        }

        /**
         * Delete nodes that contain the given point, at or deeper than the given depth.
         * @param x
         * @param y
         * @param z
         * @param delete_depth depth to start deleting nodes, nodes at depth >= delete_depth will be deleted. If delete_depth == 0, delete at the lowest level.
         * @return
         */
        inline uint32_t
        DeleteNode(double x, double y, double z, uint32_t delete_depth = 0) {
            OctreeKey key;
            if (!this->CoordToKeyChecked(x, y, z, key)) {
                ERL_WARN("Point (%f, %f, %f) is out of range.", x, y, z);
                return 0;
            } else {
                uint32_t old_tree_size = m_tree_size_;
                this->DeleteNode(key, delete_depth);
                return old_tree_size - m_tree_size_;
            }
        }

        /**
         * Delete nodes that contain the given key, at or deeper than the given depth.
         * @param key
         * @param delete_depth depth to start deleting nodes, nodes at depth >= delete_depth will be deleted. If delete_depth == 0, delete at the lowest level.
         * @return
         */
        inline void
        DeleteNode(const OctreeKey &key, uint32_t delete_depth = 0) {
            if (m_root_ == nullptr) { return; }
            if (delete_depth == 0) { delete_depth = this->m_setting_->tree_depth; }
            if (this->DeleteNodeRecurs(m_root_.get(), key, delete_depth)) {  // delete the root node
                this->OnDeleteNodeChild(nullptr, m_root_.get(), key);
                m_root_ = nullptr;
                m_tree_size_ = 0;
                m_size_changed_ = true;
            }
        }

    protected:
        /**
         * Callback before deleting a child node.
         * @param node parent node, may be nullptr if the child is the root node
         * @param child child node to be deleted
         * @param key key of the child node
         */
        inline virtual void
        OnDeleteNodeChild(Node * /*node*/, Node * /*child*/, const geometry::OctreeKey & /*key*/) {}

        /**
         * Delete child nodes down to max_depth matching the given key of the given node that is at the given depth.
         * @param node node at depth, must not be nullptr and it will not be deleted
         * @param key
         * @param depth
         * @param max_depth
         * @return
         */
        bool
        DeleteNodeRecurs(Node *node, const OctreeKey &key, uint32_t max_depth) {
            const uint32_t &kDepth = node->GetDepth();
            if (kDepth >= max_depth) { return true; }  // return true to delete this node
            ERL_DEBUG_ASSERT(node != nullptr, "node should not be nullptr.");

            uint32_t child_idx = OctreeKey::ComputeChildIndex(key, this->m_setting_->tree_depth - kDepth - 1);
            if (!node->HasChild(child_idx)) {                           // child does not exist, but maybe node is pruned
                if (!node->HasAnyChild() && (node != m_root_.get())) {  // this node is pruned
                    ExpandNode(node);                                   // expand it, tree size is updated in ExpandNode
                } else {                                                // node is not pruned, we are done
                    return false;                                       // nothing to delete
                }
            }

            auto child = this->GetNodeChild(node, child_idx);
            bool delete_child = this->DeleteNodeRecurs(child, key, max_depth);
            if (delete_child) {
                this->DeleteNodeDescendants(child, key);    // delete the child's children recursively
                this->OnDeleteNodeChild(node, child, key);  // callback before deleting the child
                node->RemoveChild(child_idx);               // remove child
                m_tree_size_--;                             // decrease tree size
                m_size_changed_ = true;                     // size of the tree has changed
                if (!node->HasAnyChild()) { return true; }  // node is pruned, can be deleted
            }
            return false;
        }

        /**
         * Delete all descendants of the given node.
         * @param node
         * @return
         */
        inline void
        DeleteNodeDescendants(Node *node, const OctreeKey &key) {
            ERL_DEBUG_ASSERT(node != nullptr, "node should not be nullptr.");
            if (!node->HasAnyChild()) { return; }
            for (int i = 0; i < 8; ++i) {
                Node *child = this->GetNodeChild(node, i);
                if (child == nullptr) { continue; }
                this->DeleteNodeDescendants(child, key);
                this->OnDeleteNodeChild(node, child, key);  // callback before deleting the child
                node->RemoveChild(i);                       // remove child
                m_tree_size_--;                             // decrease tree size
                m_size_changed_ = true;                     // size of the tree has changed
            }
        }

    public:
        /**
         * Delete the whole tree.
         * @return
         */
        void
        Clear() override {
            m_root_ = nullptr;  // delete the root node to trigger the destructor of the nodes
            m_tree_size_ = 0;
            m_size_changed_ = true;
        }

        /**
         * Lossless compression of the tree: a node will replace all of its children if the node is collapsible.
         */
        void
        Prune() override {
            if (m_root_ == nullptr) { return; }
            long tree_depth = this->m_setting_->tree_depth;
            for (long depth = tree_depth - 1; depth > 0; --depth) {
                uint32_t old_tree_size = m_tree_size_;
                this->PruneRecurs(this->m_root_.get(), depth);
                uint32_t num_pruned = old_tree_size - m_tree_size_;
                if (num_pruned == 0) { break; }
            }
        }

    protected:
        void
        PruneRecurs(Node *node, uint32_t max_depth) {
            const uint32_t &kDepth = node->GetDepth();
            if (kDepth < max_depth) {
                if (!node->HasAnyChild()) { return; }
                for (int i = 0; i < 8; ++i) {
                    Node *child = this->GetNodeChild(node, i);
                    if (child == nullptr) { continue; }
                    this->PruneRecurs(child, max_depth);
                }
            } else {
                this->PruneNode(node);
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
            this->ExpandRecurs(m_root_.get(), 0, this->m_setting_->tree_depth);
        }

    protected:
        void
        ExpandRecurs(Node *node, uint32_t depth, uint32_t max_depth) {
            if (depth >= max_depth) { return; }
            ERL_DEBUG_ASSERT(node != nullptr, "node is nullptr");

            if (!node->HasAnyChild()) { this->ExpandNode(node); }  // node has no children, expand it
            for (int i = 0; i < 8; ++i) {
                Node *child = this->GetNodeChild(node, i);
                if (child == nullptr) { continue; }
                this->ExpandRecurs(child, depth + 1, max_depth);
            }
        }

    public:
        inline const std::shared_ptr<Node> &
        GetRoot() const {
            return m_root_;
        }

        //-- Search functions
        /**
         * Search node given a point.
         * @param x
         * @param y
         * @param z
         * @param max_depth max depth to search. However, max_depth=0 means searching from root.
         * @return
         */
        [[nodiscard]] inline const Node *
        Search(double x, double y, double z, uint32_t max_depth = 0) const {
            OctreeKey key;
            if (!CoordToKeyChecked(x, y, z, key)) {
                ERL_WARN("Point (%f, %f, %f) is out of range.\n", x, y, z);
                return nullptr;
            }

            return Search(key, max_depth);
        }

        /**
         * Search node at specified depth given a key.
         * @param key
         * @param max_depth max depth to search. However, max_depth=0 means searching from root.
         * @return
         */
        [[nodiscard]] const Node *
        Search(const OctreeKey &key, uint32_t max_depth = 0) const {
            auto &tree_depth = this->m_setting_->tree_depth;
            ERL_DEBUG_ASSERT(max_depth <= tree_depth, "Depth must be in [0, %u], but got %u.", tree_depth, max_depth);

            if (m_root_ == nullptr) { return nullptr; }
            if (max_depth == 0) { max_depth = tree_depth; }

            // generate appropriate key for the given depth
            OctreeKey key_at_depth = key;
            if (max_depth != tree_depth) { key_at_depth = AdjustKeyToDepth(key_at_depth, max_depth); }

            // search
            const Node *node = m_root_.get();
            int min_level = tree_depth - max_depth;
            // follow nodes down to the requested level (for level = 0, it is the leaf level)
            for (int level = tree_depth - 1; level >= min_level; --level) {
                uint32_t child_index = OctreeKey::ComputeChildIndex(key_at_depth, level);
                if (node->HasChild(child_index)) {
                    node = GetNodeChild(node, child_index);
                } else {
                    // we expect a child but did not get it, is the current node a leaf?
                    if (!node->HasAnyChild()) {
                        return node;  // current node is a leaf, so we cannot go deeper
                    } else {
                        return nullptr;  // current node is not a leaf, search failed
                    }
                }
            }
            return node;
        }

        inline Node *
        InsertNode(double x, double y, double z, uint32_t depth = 0) {
            OctreeKey key;
            if (!CoordToKeyChecked(x, y, z, key)) {
                ERL_WARN("Point (%f, %f, %f) is out of range.\n", x, y, z);
                return nullptr;
            }
            return InsertNode(key, depth);
        }

        inline Node *
        InsertNode(const OctreeKey &key, uint32_t depth = 0) {
            auto &tree_depth = this->m_setting_->tree_depth;
            if (depth == 0) { depth = tree_depth; }
            ERL_DEBUG_ASSERT(depth <= tree_depth, "Depth must be in [0, %u], but got %u.", tree_depth, depth);
            if (this->m_root_ == nullptr) {
                this->m_root_ = std::make_shared<Node>();
                this->m_tree_size_++;
            }

            Node *node = this->m_root_.get();
            int diff = tree_depth - depth;
            for (int level = tree_depth - 1; level >= diff; --level) {
                uint32_t child_index = OctreeKey::ComputeChildIndex(key, level);
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
                std::bitset<8> children((unsigned long long) children_char);
                node->AllocateChildrenPtr();
                for (int i = 7; i >= 0; --i) {  // the same order as the recursive implementation
                    if (!children[i]) { continue; }
                    Node *child_node = node->template CreateChild<Node>(i);
                    nodes_stack.push_back(child_node);
                    m_tree_size_++;
                }
            }

            return s;
        }

    protected:
        std::ostream &
        WriteData(std::ostream &s) const override {
            if (m_root_ == nullptr) { return s; }

            std::vector<const Node *> nodes_stack;
            nodes_stack.push_back(m_root_.get());
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
