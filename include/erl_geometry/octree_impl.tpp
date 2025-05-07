// ReSharper disable CppRedundantParentheses
#pragma once

#include "intersection.hpp"

#include <omp.h>

#include <bitset>
#include <utility>

namespace erl::geometry {

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::OctreeImpl(
        std::shared_ptr<InterfaceSetting> setting)
        : Interface(setting),
          m_setting_(std::move(setting)) {
        this->ApplySettingToOctreeImpl();
    }

    template<class Node, class Interface, class InterfaceSetting>
    std::shared_ptr<AbstractOctree<typename OctreeImpl<Node, Interface, InterfaceSetting>::Dtype>>
    OctreeImpl<Node, Interface, InterfaceSetting>::Clone() const {
        // we don't know the exact setting type, so we can't clone it immediately
        std::shared_ptr<AbstractOctree<Dtype>> tree = this->Create(nullptr);  // create a new tree
        std::shared_ptr<OctreeImpl> tree_impl = std::dynamic_pointer_cast<OctreeImpl>(tree);
        // now the setting is created, we can copy the setting
        *tree_impl->m_setting_ = *m_setting_;
        tree_impl->m_resolution_inv_ = m_resolution_inv_;
        tree_impl->m_tree_key_offset_ = m_tree_key_offset_;
        tree_impl->m_tree_size_ = m_tree_size_;
        tree_impl->m_size_changed_ = m_size_changed_;
        tree_impl->m_metric_max_[0] = m_metric_max_[0];
        tree_impl->m_metric_max_[1] = m_metric_max_[1];
        tree_impl->m_metric_max_[2] = m_metric_max_[2];
        tree_impl->m_metric_min_[0] = m_metric_min_[0];
        tree_impl->m_metric_min_[1] = m_metric_min_[1];
        tree_impl->m_metric_min_[2] = m_metric_min_[2];
        tree_impl->m_size_lookup_table_ = m_size_lookup_table_;
        tree_impl->m_key_rays_ = m_key_rays_;
        if (m_root_ == nullptr) {
            tree_impl->m_root_ = nullptr;
        } else {
            tree_impl->m_root_ = std::make_shared<Node>(*m_root_);
        }
        return tree;
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::operator==(
        const AbstractOctree<Dtype> &other) const {
        if (typeid(*this) != typeid(other)) { return false; }  // compare type
        const auto &other_impl = dynamic_cast<const OctreeImpl &>(other);
        if (*m_setting_ != *other_impl.m_setting_) { return false; }
        if (m_tree_size_ != other_impl.m_tree_size_) { return false; }
        if (m_root_ == nullptr && other_impl.m_root_ == nullptr) { return true; }
        if (m_root_ == nullptr || other_impl.m_root_ == nullptr) { return false; }
        return *m_root_ == *other_impl.m_root_;
    }

    template<class Node, class Interface, class InterfaceSetting>
    std::size_t
    OctreeImpl<Node, Interface, InterfaceSetting>::GetSize() const {
        return m_tree_size_;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::Vector3
    OctreeImpl<Node, Interface, InterfaceSetting>::GetTreeCenter() const {
        Dtype x = this->KeyToCoord(m_tree_key_offset_);
        return {x, x, x};
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeKey
    OctreeImpl<Node, Interface, InterfaceSetting>::GetTreeCenterKey() const {
        OctreeKey::KeyType key = m_tree_key_offset_;
        return {key, key, key};
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::Vector3
    OctreeImpl<Node, Interface, InterfaceSetting>::GetTreeMaxHalfSize() const {
        Dtype size = -this->KeyToCoord(0);
        return {size, size, size};
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::ApplySetting() {
        this->ApplySettingToOctreeImpl();
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::ApplySettingToOctreeImpl() {
        const Dtype resolution = m_setting_->resolution;
        const uint32_t tree_depth = m_setting_->tree_depth;
        m_resolution_inv_ = 1.0 / resolution;
        m_tree_key_offset_ = 1 << (tree_depth - 1);

        // init node size lookup table
        m_size_lookup_table_.resize(tree_depth + 1);
        for (uint32_t i = 0; i <= tree_depth; ++i) {
            m_size_lookup_table_[i] = resolution * static_cast<Dtype>(1 << (tree_depth - i));
        }
        m_size_changed_ = true;

        // do it on the main thread only
#pragma omp parallel default(none) shared(m_key_rays_)
#pragma omp critical
        {
            if (omp_get_thread_num() == 0) {
                m_key_rays_.resize(omp_get_num_threads());
                for (auto &key_ray: m_key_rays_) { key_ray.reserve(100000); }
            }
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::GetMetricMin(
        Dtype &min_x,
        Dtype &min_y,
        Dtype &min_z) {
        ComputeMinMax();
        min_x = m_metric_min_[0];
        min_y = m_metric_min_[1];
        min_z = m_metric_min_[2];
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::GetMetricMin(
        Dtype &min_x,
        Dtype &min_y,
        Dtype &min_z) const {
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

        min_x = min_y = min_z = std::numeric_limits<Dtype>::infinity();
        for (auto it = this->BeginLeaf(), end = this->EndLeaf(); it != end; ++it) {
            const Dtype half_size = it.GetNodeSize() / 2.;
            const Dtype node_min_x = it.GetX() - half_size;
            const Dtype node_min_y = it.GetY() - half_size;
            const Dtype node_min_z = it.GetZ() - half_size;
            if (node_min_x < min_x) { min_x = node_min_x; }
            if (node_min_y < min_y) { min_y = node_min_y; }
            if (node_min_z < min_z) { min_z = node_min_z; }
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::GetMetricMax(
        Dtype &max_x,
        Dtype &max_y,
        Dtype &max_z) {
        this->ComputeMinMax();
        max_x = m_metric_max_[0];
        max_y = m_metric_max_[1];
        max_z = m_metric_max_[2];
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::GetMetricMax(
        Dtype &max_x,
        Dtype &max_y,
        Dtype &max_z) const {
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

        max_x = max_y = max_z = -std::numeric_limits<Dtype>::infinity();
        for (auto it = this->BeginLeaf(), end = this->EndLeaf(); it != end; ++it) {
            const Dtype half_size = it.GetNodeSize() / 2.;
            const Dtype node_max_x = it.GetX() + half_size;
            const Dtype node_max_y = it.GetY() + half_size;
            const Dtype node_max_z = it.GetZ() + half_size;
            if (node_max_x > max_x) { max_x = node_max_x; }
            if (node_max_y > max_y) { max_y = node_max_y; }
            if (node_max_z > max_z) { max_z = node_max_z; }
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::GetMetricMinMax(
        Dtype &min_x,
        Dtype &min_y,
        Dtype &min_z,
        Dtype &max_x,
        Dtype &max_y,
        Dtype &max_z) {
        this->ComputeMinMax();
        min_x = m_metric_min_[0];
        min_y = m_metric_min_[1];
        min_z = m_metric_min_[2];
        max_x = m_metric_max_[0];
        max_y = m_metric_max_[1];
        max_z = m_metric_max_[2];
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::GetMetricMinMax(
        Dtype &min_x,
        Dtype &min_y,
        Dtype &min_z,
        Dtype &max_x,
        Dtype &max_y,
        Dtype &max_z) const {
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

        min_x = min_y = min_z = std::numeric_limits<Dtype>::infinity();
        max_x = max_y = max_z = -std::numeric_limits<Dtype>::infinity();
        for (auto it = this->BeginLeaf(), end = this->EndLeaf(); it != end; ++it) {
            const Dtype size = it.GetNodeSize();
            const Dtype half_size = size / 2.;
            const Dtype node_max_x = it.GetX() + half_size;
            const Dtype node_max_y = it.GetY() + half_size;
            const Dtype node_max_z = it.GetZ() + half_size;
            const Dtype node_min_x = node_max_x - size;
            const Dtype node_min_y = node_max_y - size;
            const Dtype node_min_z = node_max_z - size;
            if (node_max_x > max_x) { max_x = node_max_x; }
            if (node_max_y > max_y) { max_y = node_max_y; }
            if (node_max_z > max_z) { max_z = node_max_z; }
            if (node_min_x < min_x) { min_x = node_min_x; }
            if (node_min_y < min_y) { min_y = node_min_y; }
            if (node_min_z < min_z) { min_z = node_min_z; }
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::GetMetricSize(Dtype &x, Dtype &y, Dtype &z) {
        Dtype min_x, min_y, min_z, max_x, max_y, max_z;
        GetMetricMinMax(min_x, min_y, min_z, max_x, max_y, max_z);
        x = max_x - min_x;
        y = max_y - min_y;
        z = max_z - min_z;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::GetMetricSize(Dtype &x, Dtype &y, Dtype &z)
        const {
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

        Dtype min_x = std::numeric_limits<Dtype>::infinity(), min_y = min_x, min_z = min_x;
        Dtype max_x = -std::numeric_limits<Dtype>::infinity(), max_y = max_x, max_z = max_x;
        for (auto it = this->BeginLeaf(), end = this->EndLeaf(); it != end; ++it) {
            const Dtype size = it.GetNodeSize();
            const Dtype half_size = size / 2.;
            const Dtype node_max_x = it.GetX() + half_size;
            const Dtype node_max_y = it.GetY() + half_size;
            const Dtype node_max_z = it.GetZ() + half_size;
            const Dtype node_min_x = node_max_x - size;
            const Dtype node_min_y = node_max_y - size;
            const Dtype node_min_z = node_max_z - size;
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

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::Dtype
    OctreeImpl<Node, Interface, InterfaceSetting>::GetNodeSize(const uint32_t depth) const {
        ERL_DEBUG_ASSERT(
            depth <= m_setting_->tree_depth,
            "Depth must be in [0, %u], but got %u.\n",
            m_setting_->tree_depth,
            depth);
        return m_size_lookup_table_[depth];
    }

    template<class Node, class Interface, class InterfaceSetting>
    Aabb<typename OctreeImpl<Node, Interface, InterfaceSetting>::Dtype, 3>
    OctreeImpl<Node, Interface, InterfaceSetting>::GetNodeAabb(
        const OctreeKey &key,
        const uint32_t depth) const {
        Vector3 center;
        this->KeyToCoord(key, depth, center.x(), center.y(), center.z());
        const Dtype half_size = GetNodeSize(depth) * 0.5;
        return {center, half_size};
    }

    template<class Node, class Interface, class InterfaceSetting>
    std::size_t
    OctreeImpl<Node, Interface, InterfaceSetting>::ComputeNumberOfLeafNodes() const {
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

    template<class Node, class Interface, class InterfaceSetting>
    std::size_t
    OctreeImpl<Node, Interface, InterfaceSetting>::GetMemoryUsage() const {
        const std::size_t number_of_leaf_nodes = this->ComputeNumberOfLeafNodes();
        const std::size_t number_of_inner_nodes = m_tree_size_ - number_of_leaf_nodes;
        // TODO: update the statistics
        return sizeof(OctreeImpl) + this->GetMemoryUsagePerNode() * m_tree_size_ +
               number_of_inner_nodes * sizeof(Node *) * 8;
    }

    template<class Node, class Interface, class InterfaceSetting>
    std::size_t
    OctreeImpl<Node, Interface, InterfaceSetting>::GetMemoryUsagePerNode() const {
        return sizeof(Node);
    }

    template<class Node, class Interface, class InterfaceSetting>
    std::size_t
    OctreeImpl<Node, Interface, InterfaceSetting>::ComputeNumberOfNodes() const {
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

    template<class Node, class Interface, class InterfaceSetting>
    OctreeKey::KeyType
    OctreeImpl<Node, Interface, InterfaceSetting>::CoordToKey(const Dtype coordinate) const {
        return static_cast<uint32_t>(std::floor(coordinate * m_resolution_inv_)) +
               m_tree_key_offset_;
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeKey::KeyType
    OctreeImpl<Node, Interface, InterfaceSetting>::CoordToKey(
        const Dtype coordinate,
        const uint32_t depth) const {
        const uint32_t tree_depth = m_setting_->tree_depth;
        ERL_DEBUG_ASSERT(
            depth <= tree_depth,
            "Depth must be in [0, %u], but got %u.\n",
            tree_depth,
            depth);
        const uint32_t keyval = std::floor(coordinate * m_resolution_inv_);
        const uint32_t diff = tree_depth - depth;
        if (!diff) { return keyval + m_tree_key_offset_; }
        return ((keyval >> diff) << diff) + (1 << (diff - 1)) + m_tree_key_offset_;
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeKey
    OctreeImpl<Node, Interface, InterfaceSetting>::CoordToKey(
        const Eigen::Ref<const Vector3> &coord) const {
        return {CoordToKey(coord[0]), CoordToKey(coord[1]), CoordToKey(coord[2])};
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeKey
    OctreeImpl<Node, Interface, InterfaceSetting>::CoordToKey(
        const Eigen::Ref<const Vector3> &coord,
        const uint32_t depth) const {
        return {
            CoordToKey(coord[0], depth),
            CoordToKey(coord[1], depth),
            CoordToKey(coord[2], depth)};
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeKey
    OctreeImpl<Node, Interface, InterfaceSetting>::CoordToKey(
        const Dtype x,
        const Dtype y,
        const Dtype z) const {
        return {CoordToKey(x), CoordToKey(y), CoordToKey(z)};
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeKey
    OctreeImpl<Node, Interface, InterfaceSetting>::CoordToKey(
        const Dtype x,
        const Dtype y,
        const Dtype z,
        uint32_t depth) const {
        if (depth == m_setting_->tree_depth) { return CoordToKey(x, y, z); }
        return {CoordToKey(x, depth), CoordToKey(y, depth), CoordToKey(z, depth)};
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::CoordToKeyChecked(
        const Dtype coordinate,
        OctreeKey::KeyType &key) const {
        if (const int scaled_coord =
                std::floor(coordinate * m_resolution_inv_) + m_tree_key_offset_;
            (scaled_coord >= 0) &&
            static_cast<uint32_t>(scaled_coord) < (m_tree_key_offset_ << 1)) {
            key = scaled_coord;
            return true;
        }
        return false;
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::CoordToKeyChecked(
        const Dtype coordinate,
        const uint32_t depth,
        OctreeKey::KeyType &key) const {
        if (const int scaled_coord =
                std::floor(coordinate * m_resolution_inv_) + m_tree_key_offset_;
            scaled_coord >= 0 && static_cast<uint32_t>(scaled_coord) < (m_tree_key_offset_ << 1)) {
            key = AdjustKeyToDepth(static_cast<OctreeKey::KeyType>(scaled_coord), depth);
            return true;
        }
        return false;
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::CoordToKeyChecked(
        const Eigen::Ref<const Vector3> &coord,
        OctreeKey &key) const {
        return CoordToKeyChecked(coord[0], coord[1], coord[2], key);
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::CoordToKeyChecked(
        const Eigen::Ref<const Vector3> &coord,
        const uint32_t depth,
        OctreeKey &key) const {
        return CoordToKeyChecked(coord[0], coord[1], coord[2], depth, key);
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::CoordToKeyChecked(
        const Dtype x,
        const Dtype y,
        const Dtype z,
        OctreeKey &key) const {
        if (!CoordToKeyChecked(x, key[0])) { return false; }
        if (!CoordToKeyChecked(y, key[1])) { return false; }
        if (!CoordToKeyChecked(z, key[2])) { return false; }
        return true;
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::CoordToKeyChecked(
        const Dtype x,
        const Dtype y,
        const Dtype z,
        uint32_t depth,
        OctreeKey &key) const {
        ERL_DEBUG_ASSERT(depth != 0, "When depth = 0, key is 0x0, which is useless!");
        if (depth == m_setting_->tree_depth) { return CoordToKeyChecked(x, y, z, key); }
        if (!CoordToKeyChecked(x, depth, key[0])) { return false; }
        if (!CoordToKeyChecked(y, depth, key[1])) { return false; }
        if (!CoordToKeyChecked(z, depth, key[2])) { return false; }
        return true;
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeKey::KeyType
    OctreeImpl<Node, Interface, InterfaceSetting>::AdjustKeyToDepth(
        const OctreeKey::KeyType key,
        const uint32_t depth) const {
        const uint32_t tree_depth = m_setting_->tree_depth;
        ERL_DEBUG_ASSERT(
            depth <= tree_depth,
            "Depth must be in [0, %u], but got %u.\n",
            tree_depth,
            depth);
        const uint32_t diff = tree_depth - depth;
        if (!diff) { return key; }
        // (((key - m_tree_key_offset_) >> diff) << diff) + (1 << (diff - 1)) + m_tree_key_offset_;
        return ((key >> diff) << diff) + (1 << (diff - 1));  // quick version
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeKey
    OctreeImpl<Node, Interface, InterfaceSetting>::AdjustKeyToDepth(
        const OctreeKey &key,
        uint32_t depth) const {
        if (depth == m_setting_->tree_depth) { return key; }
        return {
            AdjustKeyToDepth(key[0], depth),
            AdjustKeyToDepth(key[1], depth),
            AdjustKeyToDepth(key[2], depth)};
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::ComputeCommonAncestorKey(
        const OctreeKey &key1,
        const OctreeKey &key2,
        OctreeKey &ancestor_key,
        uint32_t &ancestor_depth) const {
        // 0: same bit, 1: different bit
        const OctreeKey::KeyType mask =
            (key1[0] ^ key2[0]) | (key1[1] ^ key2[1]) | (key1[2] ^ key2[2]);
        const uint32_t tree_depth = m_setting_->tree_depth;
        if (!mask) {  // keys are identical
            ancestor_key = key1;
            ancestor_depth = tree_depth;
            return;
        }
        // from bit-max_depth to bit-0, find first 1
        uint32_t level = tree_depth;
        while (level > 0 && !(mask & (1 << (level - 1)))) { --level; }
        ancestor_depth = tree_depth - level;  // bit[level] = 0, bit[level-1] = 1
        const OctreeKey::KeyType ancestor_mask = ((1l << tree_depth) - 1) << level;
        ancestor_key[0] = key1[0] & ancestor_mask;
        ancestor_key[1] = key1[1] & ancestor_mask;
        ancestor_key[2] = key1[2] & ancestor_mask;
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::ComputeWestNeighborKey(
        const OctreeKey &key,
        uint32_t depth,
        OctreeKey &neighbor_key) const {
        const OctreeKey::KeyType offset = 1 << (m_setting_->tree_depth - depth);
        if (key[0] < offset) { return false; }  // no west neighbor
        neighbor_key[0] = key[0] - offset;
        neighbor_key[1] = key[1];
        neighbor_key[2] = key[2];
        return true;
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::ComputeEastNeighborKey(
        const OctreeKey &key,
        const uint32_t depth,
        OctreeKey &neighbor_key) const {
        const uint32_t tree_depth = m_setting_->tree_depth;
        const OctreeKey::KeyType offset = 1 << (tree_depth - depth);
        // no east neighbor (key[0] + offset >= 2^max_depth)
        if ((1 << tree_depth) - key[0] <= offset) { return false; }
        neighbor_key[0] = key[0] + offset;
        neighbor_key[1] = key[1];
        neighbor_key[2] = key[2];
        return true;
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::ComputeNorthNeighborKey(
        const OctreeKey &key,
        const uint32_t depth,
        OctreeKey &neighbor_key) const {
        const uint32_t tree_depth = m_setting_->tree_depth;
        const OctreeKey::KeyType offset = 1 << (tree_depth - depth);
        // no north neighbor (key[1] + offset >= 2^max_depth)
        if ((1 << tree_depth) - key[1] <= offset) { return false; }
        neighbor_key[0] = key[0];
        neighbor_key[1] = key[1] + offset;
        neighbor_key[2] = key[2];
        return true;
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::ComputeSouthNeighborKey(
        const OctreeKey &key,
        uint32_t depth,
        OctreeKey &neighbor_key) const {
        const OctreeKey::KeyType offset = 1 << (m_setting_->tree_depth - depth);
        if (key[1] < offset) { return false; }  // no southern neighbor
        neighbor_key[0] = key[0];
        neighbor_key[1] = key[1] - offset;
        neighbor_key[2] = key[2];
        return true;
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::ComputeBottomNeighborKey(
        const OctreeKey &key,
        uint32_t depth,
        OctreeKey &neighbor_key) const {
        const OctreeKey::KeyType offset = 1 << (m_setting_->tree_depth - depth);
        if (key[2] < offset) { return false; }  // no bottom neighbor
        neighbor_key[0] = key[0];
        neighbor_key[1] = key[1];
        neighbor_key[2] = key[2] - offset;
        return true;
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::ComputeTopNeighborKey(
        const OctreeKey &key,
        const uint32_t depth,
        OctreeKey &neighbor_key) const {
        const uint32_t tree_depth = m_setting_->tree_depth;
        const OctreeKey::KeyType offset = 1 << (tree_depth - depth);
        // no top neighbor (key[2] + offset >= 2^max_depth)
        if ((1 << tree_depth) - key[2] <= offset) { return false; }
        neighbor_key[0] = key[0];
        neighbor_key[1] = key[1];
        neighbor_key[2] = key[2] + offset;
        return true;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::Dtype
    OctreeImpl<Node, Interface, InterfaceSetting>::KeyToCoord(const OctreeKey::KeyType key) const {
        return (static_cast<Dtype>(static_cast<int>(key) - static_cast<int>(m_tree_key_offset_)) +
                0.5) *
               m_setting_->resolution;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::Dtype
    OctreeImpl<Node, Interface, InterfaceSetting>::KeyToCoord(
        const OctreeKey::KeyType key,
        const uint32_t depth) const {
        const uint32_t tree_depth = m_setting_->tree_depth;
        if (depth == 0) { return 0.0; }
        if (depth == tree_depth) { return KeyToCoord(key); }
        uint32_t &&diff = tree_depth - depth;
        Dtype &&r = this->GetNodeSize(depth);
        return (std::floor(
                    (static_cast<Dtype>(key) - static_cast<Dtype>(m_tree_key_offset_)) /
                    static_cast<Dtype>(1 << diff)) +
                0.5) *
               r;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::KeyToCoord(
        const OctreeKey &key,
        Dtype &x,
        Dtype &y,
        Dtype &z) const {
        x = KeyToCoord(key[0]);
        y = KeyToCoord(key[1]);
        z = KeyToCoord(key[2]);
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::KeyToCoord(
        const OctreeKey &key,
        uint32_t depth,
        Dtype &x,
        Dtype &y,
        Dtype &z) const {
        if (depth == 0) {
            x = y = z = 0.0;
            return;
        }
        if (depth == m_setting_->tree_depth) {
            KeyToCoord(key, x, y, z);
            return;
        }
        x = KeyToCoord(key[0], depth);
        y = KeyToCoord(key[1], depth);
        z = KeyToCoord(key[2], depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::KeyToCoord(
        const OctreeKey &key,
        const uint32_t depth,
        Vector3 &coord) const {
        KeyToCoord(key, depth, coord.x(), coord.y(), coord.z());
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::Vector3
    OctreeImpl<Node, Interface, InterfaceSetting>::KeyToCoord(
        const OctreeKey &key,
        const uint32_t depth) const {
        Vector3 coord;
        KeyToCoord(key, depth, coord.x(), coord.y(), coord.z());
        return coord;
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::StackElement::StackElement(
        const Node *node,
        OctreeKey key)
        : node(node),
          key(std::move(key)) {}

    template<class Node, class Interface, class InterfaceSetting>
    template<typename T>
    const T &
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::StackElement::GetData() const {
        return *static_cast<T *>(data.get());
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::IteratorBase()
        : m_tree_(nullptr),
          m_max_node_depth_(0) {}

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::IteratorBase(
        const OctreeImpl *tree,
        const uint32_t max_node_depth)
        : m_tree_(tree),
          m_max_node_depth_(max_node_depth) {
        if (m_tree_ == nullptr) { return; }
        if (m_max_node_depth_ == 0) { m_max_node_depth_ = m_tree_->GetTreeDepth(); }
        if (m_tree_->m_root_ != nullptr) {  // the tree is not empty
            m_stack_.emplace_back(m_tree_->m_root_.get(), m_tree_->CoordToKey(0.0, 0.0, 0.0));
        } else {
            m_tree_ = nullptr;
            m_max_node_depth_ = 0;
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::operator==(
        const IteratorBase &other) const {
        // we do not need to compare m_max_node_depth_ here, since it is always the same for the
        // same tree
        if (m_tree_ != other.m_tree_) { return false; }
        if (m_stack_.size() != other.m_stack_.size()) { return false; }
        if (m_stack_.empty()) { return true; }

        const StackElement &top = m_stack_.back();
        auto &other_top = other.m_stack_.back();
        if (top.node != other_top.node) { return false; }
        if (top.key != other_top.key) { return false; }
        return true;
    }

    template<class Node, class Interface, class InterfaceSetting>
    Node *
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::operator->() {
        return const_cast<Node *>(m_stack_.back().node);
    }

    template<class Node, class Interface, class InterfaceSetting>
    const Node *
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::operator->() const {
        return m_stack_.back().node;
    }

    template<class Node, class Interface, class InterfaceSetting>
    Node *
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::operator*() {
        return const_cast<Node *>(m_stack_.back().node);
    }

    template<class Node, class Interface, class InterfaceSetting>
    const Node *
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::operator*() const {
        return m_stack_.back().node;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::Dtype
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::GetX() const {
        const StackElement &top = m_stack_.back();
        return m_tree_->KeyToCoord(top.key[0], top.node->GetDepth());
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::Dtype
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::GetY() const {
        const StackElement &top = m_stack_.back();
        return m_tree_->KeyToCoord(top.key[1], top.node->GetDepth());
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::Dtype
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::GetZ() const {
        const StackElement &top = m_stack_.back();
        return m_tree_->KeyToCoord(top.key[2], top.node->GetDepth());
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::Dtype
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::GetNodeSize() const {
        return m_tree_->GetNodeSize(m_stack_.back().node->GetDepth());
    }

    template<class Node, class Interface, class InterfaceSetting>
    uint32_t
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::GetDepth() const {
        return m_stack_.back().node->GetDepth();
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::IsValid() const {
        return !m_stack_.empty();
    }

    template<class Node, class Interface, class InterfaceSetting>
    const Node *
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::GetNode() const {
        return m_stack_.back().node;
    }

    template<class Node, class Interface, class InterfaceSetting>
    const Node *
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::GetNode() {
        return m_stack_.back().node;
    }

    template<class Node, class Interface, class InterfaceSetting>
    Aabb<typename OctreeImpl<Node, Interface, InterfaceSetting>::Dtype, 3>
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::GetNodeAabb() const {
        const StackElement &top = m_stack_.back();
        return m_tree_->GetNodeAabb(top.key, top.node->GetDepth());
    }

    template<class Node, class Interface, class InterfaceSetting>
    const OctreeKey &
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::GetKey() const {
        return m_stack_.back().key;
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeKey
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::GetIndexKey() const {
        const StackElement &top = m_stack_.back();
        return m_tree_->AdjustKeyToDepth(top.key, top.node->GetDepth());
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::SingleIncrement1() {
        StackElement top = m_stack_.back();
        m_stack_.pop_back();
        const uint32_t node_depth = top.node->GetDepth();
        if (node_depth == m_max_node_depth_) { return; }

        uint32_t next_depth = node_depth + 1;
        ERL_DEBUG_ASSERT(
            next_depth <= m_max_node_depth_,
            "Wrong depth: %u (max: %u).\n",
            next_depth,
            m_max_node_depth_);
        OctreeKey next_key;
        const OctreeKey::KeyType center_offset_key = m_tree_->m_tree_key_offset_ >> next_depth;
        // push on stack in reverse order
        for (int i = 7; i >= 0; --i) {
            if (top.node->HasChild(i)) {
                OctreeKey::ComputeChildKey(i, center_offset_key, top.key, next_key);
                m_stack_.emplace_back(m_tree_->GetNodeChild(top.node, i), next_key);
            }
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::IsLeaf() const {
        ERL_DEBUG_ASSERT(!m_stack_.empty(), "Stack is empty.");
        return !m_stack_.back().node->HasAnyChild();
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::Terminate() {
        this->m_stack_.clear();
        this->m_tree_ = nullptr;
        this->m_max_node_depth_ = 0;
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::IteratorBase::GetInTreeAabb(
        Dtype &aabb_min_x,
        Dtype &aabb_min_y,
        Dtype &aabb_min_z,
        Dtype &aabb_max_x,
        Dtype &aabb_max_y,
        Dtype &aabb_max_z) const {
        if (m_tree_ == nullptr) { return false; }

        const Dtype center = m_tree_->KeyToCoord(m_tree_->m_tree_key_offset_);
        const Dtype half_size = m_tree_->GetNodeSize(0) / 2.0;
        const Dtype aabb_min = center - half_size;
        const Dtype aabb_max = center + half_size;

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

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::TreeIterator::TreeIterator(
        const OctreeImpl *tree,
        uint32_t max_node_depth)
        : IteratorBase(tree, max_node_depth) {}

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::TreeIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::TreeIterator::operator++(int) {
        const TreeIterator result = *this;
        ++(*this);
        return result;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::TreeIterator &
    OctreeImpl<Node, Interface, InterfaceSetting>::TreeIterator::operator++() {
        if (!this->m_stack_.empty()) { this->SingleIncrement1(); }
        if (this->m_stack_.empty()) { this->Terminate(); }
        return *this;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::TreeIterator::Next() {
        ++(*this);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::InAabbIteratorBase::InAabbIteratorBase(
        Dtype aabb_min_x,
        Dtype aabb_min_y,
        Dtype aabb_min_z,
        Dtype aabb_max_x,
        Dtype aabb_max_y,
        Dtype aabb_max_z,
        const OctreeImpl *tree,
        uint32_t max_node_depth)
        : IteratorBase(tree, max_node_depth) {
        if (this->m_stack_.empty()) { return; }
        ERL_DEBUG_ASSERT(tree != nullptr, "Tree is null.");

        // If the provided AABB is too large, CoordsToKeyChecked will return false. We should avoid
        // this case.
        if (!this->GetInTreeAabb(
                aabb_min_x,
                aabb_min_y,
                aabb_min_z,
                aabb_max_x,
                aabb_max_y,
                aabb_max_z)) {
            this->Terminate();  // the tree is not in the AABB at all
            return;
        }

        if (this->m_tree_->CoordToKeyChecked(aabb_min_x, aabb_min_y, aabb_min_z, m_aabb_min_key_) &&
            this->m_tree_->CoordToKeyChecked(aabb_max_x, aabb_max_y, aabb_max_z, m_aabb_max_key_)) {
            // check if the root node is in the AABB
            if (typename IteratorBase::StackElement top = this->m_stack_.back();
                !OctreeKey::KeyInAabb(
                    top.key,
                    this->m_tree_->m_tree_key_offset_ >> top.node->GetDepth(),
                    m_aabb_min_key_,
                    m_aabb_max_key_)) {
                this->Terminate();
                return;
            }
        } else {
            // The AABB is out of the tree, but unlikely to happen here.
            // We still check it for safety.
            this->Terminate();
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::InAabbIteratorBase::InAabbIteratorBase(
        OctreeKey aabb_min_key,
        OctreeKey aabb_max_key,
        const OctreeImpl *tree,
        uint32_t max_node_depth)
        : IteratorBase(tree, max_node_depth),
          m_aabb_min_key_(std::move(aabb_min_key)),
          m_aabb_max_key_(std::move(aabb_max_key)) {
        if (this->m_stack_.empty()) { return; }
        ERL_DEBUG_ASSERT(tree != nullptr, "Tree is null.");

        // check if the root node is in the AABB
        if (typename IteratorBase::StackElement top = this->m_stack_.back(); !OctreeKey::KeyInAabb(
                top.key,
                this->m_tree_->m_tree_key_offset_ >> top.node->GetDepth(),
                m_aabb_min_key_,
                m_aabb_max_key_)) {
            this->Terminate();
            return;
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::InAabbIteratorBase::SingleIncrement2() {
        typename IteratorBase::StackElement top = this->m_stack_.back();
        this->m_stack_.pop_back();
        const uint32_t node_depth = top.node->GetDepth();
        if (node_depth == this->m_max_node_depth_) { return; }

        uint32_t next_depth = node_depth + 1;
        ERL_DEBUG_ASSERT(
            next_depth <= this->m_max_node_depth_,
            "Wrong depth: %u (max: %u).\n",
            next_depth,
            this->m_max_node_depth_);
        OctreeKey next_key;
        const OctreeKey::KeyType center_offset_key =
            this->m_tree_->m_tree_key_offset_ >> next_depth;
        // push on stack in reverse order
        for (int i = 7; i >= 0; --i) {
            if (!top.node->HasChild(i)) { continue; }
            OctreeKey::ComputeChildKey(i, center_offset_key, top.key, next_key);
            // check if the child node overlaps with the AABB
            if (OctreeKey::KeyInAabb(
                    next_key,
                    center_offset_key,
                    m_aabb_min_key_,
                    m_aabb_max_key_)) {
                this->m_stack_.emplace_back(this->m_tree_->GetNodeChild(top.node, i), next_key);
            }
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::TreeInAabbIterator::TreeInAabbIterator(
        Dtype aabb_min_x,
        Dtype aabb_min_y,
        Dtype aabb_min_z,
        Dtype aabb_max_x,
        Dtype aabb_max_y,
        Dtype aabb_max_z,
        const OctreeImpl *tree,
        uint32_t max_node_depth)
        : InAabbIteratorBase(
              aabb_min_x,
              aabb_min_y,
              aabb_min_z,
              aabb_max_x,
              aabb_max_y,
              aabb_max_z,
              tree,
              max_node_depth) {}

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::TreeInAabbIterator::TreeInAabbIterator(
        OctreeKey aabb_min_key,
        OctreeKey aabb_max_key,
        const OctreeImpl *tree,
        uint32_t max_node_depth)
        : InAabbIteratorBase(
              std::move(aabb_min_key),
              std::move(aabb_max_key),
              tree,
              max_node_depth) {}

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::TreeInAabbIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::TreeInAabbIterator::operator++(int) {
        const TreeInAabbIterator result = *this;
        ++(*this);
        return result;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::TreeInAabbIterator &
    OctreeImpl<Node, Interface, InterfaceSetting>::TreeInAabbIterator::operator++() {
        if (!this->m_stack_.empty()) { this->SingleIncrement2(); }
        if (this->m_stack_.empty()) { this->Terminate(); }
        return *this;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::TreeInAabbIterator::Next() {
        ++(*this);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafInAabbIterator::LeafInAabbIterator(
        Dtype aabb_min_x,
        Dtype aabb_min_y,
        Dtype aabb_min_z,
        Dtype aabb_max_x,
        Dtype aabb_max_y,
        Dtype aabb_max_z,
        const OctreeImpl *tree,
        uint32_t max_leaf_depth)
        : InAabbIteratorBase(
              aabb_min_x,
              aabb_min_y,
              aabb_min_z,
              aabb_max_x,
              aabb_max_y,
              aabb_max_z,
              tree,
              max_leaf_depth) {
        while (!this->m_stack_.empty() && !this->IsLeaf()) { this->SingleIncrement2(); }
        if (this->m_stack_.empty()) { this->Terminate(); }
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafInAabbIterator::LeafInAabbIterator(
        OctreeKey aabb_min_key,
        OctreeKey aabb_max_key,
        const OctreeImpl *tree,
        uint32_t max_leaf_depth)
        : InAabbIteratorBase(
              std::move(aabb_min_key),
              std::move(aabb_max_key),
              tree,
              max_leaf_depth) {
        while (!this->m_stack_.empty() && !this->IsLeaf()) { this->SingleIncrement2(); }
        if (this->m_stack_.empty()) { this->Terminate(); }
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::LeafInAabbIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafInAabbIterator::operator++(int) {
        const LeafInAabbIterator result = *this;
        ++(*this);
        return result;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::LeafInAabbIterator &
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafInAabbIterator::operator++() {
        if (this->m_stack_.empty()) {
            this->Terminate();
            return *this;
        }

        do { this->SingleIncrement2(); } while (!this->m_stack_.empty() && !this->IsLeaf());
        if (this->m_stack_.empty()) { this->Terminate(); }
        return *this;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafInAabbIterator::Next() {
        ++(*this);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafIterator::LeafIterator(
        const OctreeImpl *tree,
        uint32_t depth)
        : IteratorBase(tree, depth) {
        if (this->m_stack_.empty()) { return; }
        // skip forward to next valid leaf node
        while (!this->m_stack_.empty() && !this->IsLeaf()) { this->SingleIncrement1(); }
        if (this->m_stack_.empty()) { this->Terminate(); }
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::LeafIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafIterator::operator++(int) {
        const LeafIterator result = *this;
        ++(*this);
        return result;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::LeafIterator &
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafIterator::operator++() {
        if (this->m_stack_.empty()) {
            this->Terminate();
            return *this;
        }

        do { this->SingleIncrement1(); } while (!this->m_stack_.empty() && !this->IsLeaf());
        if (this->m_stack_.empty()) { this->Terminate(); }
        return *this;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafIterator::Next() {
        ++(*this);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafOfNodeIterator::LeafOfNodeIterator(
        OctreeKey key,
        uint32_t cluster_depth,
        const OctreeImpl *tree,
        uint32_t max_node_depth)
        : IteratorBase(tree, max_node_depth) {
        ERL_ASSERTM(
            cluster_depth <= this->m_max_node_depth_,
            "Cluster max_node_depth %u is greater than max max_node_depth %u.\n",
            cluster_depth,
            this->m_max_node_depth_);

        // modify stack top
        auto &s = this->m_stack_.back();
        s.node = this->m_tree_->Search(key, cluster_depth);
        if (s.node == nullptr) {
            this->Terminate();
            return;
        }
        s.key = key;

        // skip forward to next valid leaf node
        while (!this->m_stack_.empty() && !this->IsLeaf()) { this->SingleIncrement1(); }
        if (this->m_stack_.empty()) { this->Terminate(); }
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::LeafOfNodeIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafOfNodeIterator::operator++(int) {
        const LeafOfNodeIterator result = *this;
        ++(*this);
        return result;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::LeafOfNodeIterator &
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafOfNodeIterator::operator++() {
        if (this->m_stack_.empty()) {
            this->Terminate();
            return *this;
        }

        do { this->SingleIncrement1(); } while (!this->m_stack_.empty() && !this->IsLeaf());
        if (this->m_stack_.empty()) { this->Terminate(); }
        return *this;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafOfNodeIterator::Next() {
        ++(*this);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafNeighborIteratorBase::
        LeafNeighborIteratorBase(const OctreeImpl *tree, uint32_t max_leaf_depth)
        : IteratorBase(tree, max_leaf_depth) {}

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafNeighborIteratorBase::Init(
        OctreeKey key,
        uint32_t key_depth,
        int changing_dim1,
        int changing_dim2,
        int unchanged_dim,
        const bool increase) {
        if (this->m_tree_ == nullptr) { return; }
        const uint32_t max_depth = this->m_tree_->GetTreeDepth();
        const uint32_t level = max_depth - key_depth;
        key = this->m_tree_->AdjustKeyToDepth(key, key_depth);
        const OctreeKey::KeyType half_offset = (level == 0 ? 0 : 1 << (level - 1));
        int key_unchanged;
        if (increase) {
            key_unchanged = key[unchanged_dim] + std::max(1, static_cast<int>(half_offset));
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

        // start searching from the smallest neighbor in the bottom-left corner
        this->m_neighbor_key_[unchanged_dim] = key_unchanged;
        this->m_neighbor_key_[changing_dim1] = key[changing_dim1] - half_offset;
        this->m_neighbor_key_[changing_dim2] = key[changing_dim2] - half_offset;
        this->m_max_key_changing_dim1_ =
            std::min(key[changing_dim1] + (level == 0 ? 1 : half_offset), 1 << max_depth);
        this->m_min_key_changing_dim2_ = this->m_neighbor_key_[changing_dim2];
        this->m_max_key_changing_dim2_ =
            std::min(key[changing_dim2] + (level == 0 ? 1 : half_offset), 1 << max_depth);
        this->m_stack_.resize(1);
        this->SingleIncrementOf(changing_dim1, changing_dim2);
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::LeafNeighborIteratorBase::SingleIncrementOf(
        int changing_dim1,
        int changing_dim2) {
        auto &s = this->m_stack_.back();

        OctreeKey::KeyType &key_changing_dim1 = this->m_neighbor_key_[changing_dim1];
        OctreeKey::KeyType &key_changing_dim2 = this->m_neighbor_key_[changing_dim2];
        s.node = nullptr;
        while (s.node == nullptr && key_changing_dim1 < m_max_key_changing_dim1_ &&
               key_changing_dim2 < m_max_key_changing_dim2_) {
            s.node = this->m_tree_->Search(m_neighbor_key_);
            const uint32_t node_depth = s.node->GetDepth();
            if (s.node == nullptr || node_depth > this->m_max_node_depth_) {  // not found
                s.node = nullptr;
                ++key_changing_dim2;
                if (key_changing_dim2 >= m_max_key_changing_dim2_) {  // go to the next row
                    key_changing_dim2 = m_min_key_changing_dim2_;
                    ++key_changing_dim1;
                }
                continue;
            }

            // found a neighbor
            s.key = this->m_tree_->AdjustKeyToDepth(m_neighbor_key_, node_depth);
            // avoid duplicate
            if (s.key[changing_dim1] != key_changing_dim1) {
                // the node's center is not on the current row
                s.node = nullptr;
                ++key_changing_dim2;
                if (key_changing_dim2 >= m_max_key_changing_dim2_) {  // go to the next row
                    key_changing_dim2 = m_min_key_changing_dim2_;
                    ++key_changing_dim1;
                }
                continue;
            }
            // while the loop ends here, update key_changing_dim2 to the next value
            const uint32_t max_depth = this->m_tree_->GetTreeDepth();
            key_changing_dim2 = s.key[changing_dim2] +
                                (node_depth == max_depth ? 1 : (1 << (max_depth - node_depth - 1)));
        }
        // check if we have reached the end
        if (s.node == nullptr && key_changing_dim1 >= m_max_key_changing_dim1_ &&
            key_changing_dim2 >= m_max_key_changing_dim2_) {
            this->Terminate();
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::WestLeafNeighborIterator::
        WestLeafNeighborIterator(
            Dtype x,
            Dtype y,
            Dtype z,
            const OctreeImpl *tree,
            uint32_t max_leaf_depth)
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

        this->Init(
            key,
            node->GetDepth(),
            sk_ChangingDim1_,
            sk_ChangingDim2_,
            sk_UnchangedDim_,
            sk_Increase_);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::WestLeafNeighborIterator::
        WestLeafNeighborIterator(
            const OctreeKey &key,
            uint32_t key_depth,
            const OctreeImpl *tree,
            uint32_t max_leaf_depth)
        : LeafNeighborIteratorBase(tree, max_leaf_depth) {
        this->Init(
            key,
            key_depth,
            sk_ChangingDim1_,
            sk_ChangingDim2_,
            sk_UnchangedDim_,
            sk_Increase_);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::WestLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::WestLeafNeighborIterator::operator++(int) {
        const WestLeafNeighborIterator result = *this;
        ++(*this);
        return result;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::WestLeafNeighborIterator &
    OctreeImpl<Node, Interface, InterfaceSetting>::WestLeafNeighborIterator::operator++() {
        if (!this->m_stack_.empty()) {
            this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_);
        }
        if (this->m_stack_.empty()) { this->Terminate(); }
        return *this;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::WestLeafNeighborIterator::Next() {
        ++(*this);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::EastLeafNeighborIterator::
        EastLeafNeighborIterator(
            Dtype x,
            Dtype y,
            Dtype z,
            const OctreeImpl *tree,
            uint32_t max_leaf_depth)
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
        this->Init(
            key,
            node->GetDepth(),
            sk_ChangingDim1_,
            sk_ChangingDim2_,
            sk_UnchangedDim_,
            sk_Increase_);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::EastLeafNeighborIterator::
        EastLeafNeighborIterator(
            const OctreeKey &key,
            uint32_t key_depth,
            const OctreeImpl *tree,
            uint32_t max_leaf_depth)
        : LeafNeighborIteratorBase(tree, max_leaf_depth) {
        this->Init(
            key,
            key_depth,
            sk_ChangingDim1_,
            sk_ChangingDim2_,
            sk_UnchangedDim_,
            sk_Increase_);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::EastLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::EastLeafNeighborIterator::operator++(int) {
        const EastLeafNeighborIterator result = *this;
        ++(*this);
        return result;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::EastLeafNeighborIterator &
    OctreeImpl<Node, Interface, InterfaceSetting>::EastLeafNeighborIterator::operator++() {
        if (!this->m_stack_.empty()) {
            this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_);
        }
        if (this->m_stack_.empty()) { this->Terminate(); }
        return *this;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::EastLeafNeighborIterator::Next() {
        ++(*this);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::NorthLeafNeighborIterator::
        NorthLeafNeighborIterator(
            Dtype x,
            Dtype y,
            Dtype z,
            const OctreeImpl *tree,
            uint32_t max_leaf_depth)
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

        this->Init(
            key,
            node->GetDepth(),
            sk_ChangingDim1_,
            sk_ChangingDim2_,
            sk_UnchangedDim_,
            sk_Increase_);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::NorthLeafNeighborIterator::
        NorthLeafNeighborIterator(
            const OctreeKey &key,
            uint32_t key_depth,
            const OctreeImpl *tree,
            uint32_t max_leaf_depth)
        : LeafNeighborIteratorBase(tree, max_leaf_depth) {
        this->Init(
            key,
            key_depth,
            sk_ChangingDim1_,
            sk_ChangingDim2_,
            sk_UnchangedDim_,
            sk_Increase_);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::NorthLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::NorthLeafNeighborIterator::operator++(int) {
        const NorthLeafNeighborIterator result = *this;
        ++(*this);
        return result;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::NorthLeafNeighborIterator &
    OctreeImpl<Node, Interface, InterfaceSetting>::NorthLeafNeighborIterator::operator++() {
        if (!this->m_stack_.empty()) {
            this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_);
        }
        if (this->m_stack_.empty()) { this->Terminate(); }
        return *this;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::NorthLeafNeighborIterator::Next() {
        ++(*this);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::SouthLeafNeighborIterator::
        SouthLeafNeighborIterator(
            Dtype x,
            Dtype y,
            Dtype z,
            const OctreeImpl *tree,
            uint32_t max_leaf_depth)
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

        this->Init(
            key,
            node->GetDepth(),
            sk_ChangingDim1_,
            sk_ChangingDim2_,
            sk_UnchangedDim_,
            sk_Increase_);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::SouthLeafNeighborIterator::
        SouthLeafNeighborIterator(
            const OctreeKey &key,
            uint32_t key_depth,
            const OctreeImpl *tree,
            uint32_t max_leaf_depth)
        : LeafNeighborIteratorBase(tree, max_leaf_depth) {
        this->Init(
            key,
            key_depth,
            sk_ChangingDim1_,
            sk_ChangingDim2_,
            sk_UnchangedDim_,
            sk_Increase_);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::SouthLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::SouthLeafNeighborIterator::operator++(int) {
        const SouthLeafNeighborIterator result = *this;
        ++(*this);
        return result;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::SouthLeafNeighborIterator &
    OctreeImpl<Node, Interface, InterfaceSetting>::SouthLeafNeighborIterator::operator++() {
        if (!this->m_stack_.empty()) {
            this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_);
        }
        if (this->m_stack_.empty()) { this->Terminate(); }
        return *this;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::SouthLeafNeighborIterator::Next() {
        ++(*this);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::TopLeafNeighborIterator::TopLeafNeighborIterator(
        Dtype x,
        Dtype y,
        Dtype z,
        const OctreeImpl *tree,
        uint32_t max_leaf_depth)
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

        this->Init(
            key,
            node->GetDepth(),
            sk_ChangingDim1_,
            sk_ChangingDim2_,
            sk_UnchangedDim_,
            sk_Increase_);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::TopLeafNeighborIterator::TopLeafNeighborIterator(
        const OctreeKey &key,
        uint32_t key_depth,
        const OctreeImpl *tree,
        uint32_t max_leaf_depth)
        : LeafNeighborIteratorBase(tree, max_leaf_depth) {
        this->Init(
            key,
            key_depth,
            sk_ChangingDim1_,
            sk_ChangingDim2_,
            sk_UnchangedDim_,
            sk_Increase_);
    }

    template<class Node, class Interface, class InterfaceSetting>
    auto
    OctreeImpl<Node, Interface, InterfaceSetting>::TopLeafNeighborIterator::operator++(int) {
        const TopLeafNeighborIterator result = *this;
        ++(*this);
        return result;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::TopLeafNeighborIterator &
    OctreeImpl<Node, Interface, InterfaceSetting>::TopLeafNeighborIterator::operator++() {
        if (!this->m_stack_.empty()) {
            this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_);
        }
        if (this->m_stack_.empty()) { this->Terminate(); }
        return *this;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::TopLeafNeighborIterator::Next() {
        ++(*this);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::BottomLeafNeighborIterator::
        BottomLeafNeighborIterator(
            Dtype x,
            Dtype y,
            Dtype z,
            const OctreeImpl *tree,
            uint32_t max_leaf_depth)
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

        this->Init(
            key,
            node->GetDepth(),
            sk_ChangingDim1_,
            sk_ChangingDim2_,
            sk_UnchangedDim_,
            sk_Increase_);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::BottomLeafNeighborIterator::
        BottomLeafNeighborIterator(
            const OctreeKey &key,
            uint32_t key_depth,
            const OctreeImpl *tree,
            uint32_t max_leaf_depth)
        : LeafNeighborIteratorBase(tree, max_leaf_depth) {
        this->Init(
            key,
            key_depth,
            sk_ChangingDim1_,
            sk_ChangingDim2_,
            sk_UnchangedDim_,
            sk_Increase_);
    }

    template<class Node, class Interface, class InterfaceSetting>
    auto
    OctreeImpl<Node, Interface, InterfaceSetting>::BottomLeafNeighborIterator::operator++(int) {
        const BottomLeafNeighborIterator result = *this;
        ++(*this);
        return result;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::BottomLeafNeighborIterator &
    OctreeImpl<Node, Interface, InterfaceSetting>::BottomLeafNeighborIterator::operator++() {
        if (!this->m_stack_.empty()) {
            this->SingleIncrementOf(sk_ChangingDim1_, sk_ChangingDim2_);
        }
        if (this->m_stack_.empty()) { this->Terminate(); }
        return *this;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::BottomLeafNeighborIterator::Next() {
        ++(*this);
    }

    template<class Node, class Interface, class InterfaceSetting>
    OctreeImpl<Node, Interface, InterfaceSetting>::NodeOnRayIterator::NodeOnRayIterator(
        const Dtype px,
        const Dtype py,
        const Dtype pz,
        const Dtype vx,
        const Dtype vy,
        const Dtype vz,
        const Dtype max_range,
        const Dtype node_padding,
        const bool &bidirectional,
        const OctreeImpl *tree,
        const bool leaf_only,
        const uint32_t min_node_depth,
        uint32_t max_node_depth)
        : IteratorBase(tree, max_node_depth),
          m_origin_(px, py, pz),
          m_dir_(vx, vy, vz),
          m_max_range_(max_range),
          m_node_padding_(node_padding),
          m_bidirectional_(bidirectional),
          m_leaf_only_(leaf_only),
          m_min_node_depth_(min_node_depth) {
        m_dir_inv_ = m_dir_.cwiseInverse();
        if (this->m_stack_.empty()) { return; }
        this->m_stack_.back().data = std::make_shared<Dtype>(0.);
        this->SingleIncrement2();
        if (this->m_stack_.empty()) { this->Terminate(); }
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::Dtype
    OctreeImpl<Node, Interface, InterfaceSetting>::NodeOnRayIterator::GetDistance() const {
        if (this->m_stack_.empty()) { return 0.; }
        return *reinterpret_cast<Dtype *>(this->m_stack_.back().data.get());
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::NodeOnRayIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::NodeOnRayIterator::operator++(int) {
        const NodeOnRayIterator result = *this;
        ++(*this);
        return result;
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::NodeOnRayIterator &
    OctreeImpl<Node, Interface, InterfaceSetting>::NodeOnRayIterator::operator++() {
        if (!this->m_stack_.empty()) { this->SingleIncrement2(); }
        if (this->m_stack_.empty()) { this->Terminate(); }
        return *this;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::NodeOnRayIterator::Next() {
        ++(*this);
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::NodeOnRayIterator::StackElementCompare::
    operator()(
        const typename IteratorBase::StackElement &lhs,
        const typename IteratorBase::StackElement &rhs) const {
        const Dtype lhs_dist = lhs.template GetData<Dtype>();
        const Dtype rhs_dist = rhs.template GetData<Dtype>();
        if (lhs_dist >= 0) {
            if (rhs_dist >= 0) { return lhs_dist > rhs_dist; }  // both are positive
            return false;
        }
        if (rhs_dist >= 0) { return true; }
        return lhs_dist < rhs_dist;  // both are negative
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::NodeOnRayIterator::SingleIncrement2() {
        // make_heap: make the front element the smallest
        // pop_heap: move the smallest element to the back
        // push_heap: rectify the heap after adding a new element to the back
        // stack pop is the last element
        StackElementCompare cmp;
        // s.node is the leaf node unless it is the root node
        typename IteratorBase::StackElement s = this->m_stack_.back();
        this->m_stack_.pop_back();

        // s.node may be the root node, m_min_node_depth_ may be 0
        if (!s.node->HasAnyChild()) {      // leaf node, we need to find the next internal node
            if (this->m_stack_.empty()) {  // end of iteration
                this->Terminate();
                return;
            }
            // check the next stack top
            std::pop_heap(this->m_stack_.begin(), this->m_stack_.end(), cmp);
            // once called std::pop_heap, the stack top is at the back
            if (this->m_stack_.back().node->GetDepth() >= m_min_node_depth_) {
                // the current stack top is deep enough
                if (!this->m_leaf_only_) { return; }  // not required to be a leaf node
                if (this->IsLeaf()) { return; }       // current stack top is leaf node
            }
            s = this->m_stack_.back();  // current stack top is internal node
            this->m_stack_.pop_back();
        }

        // now s.node is an internal node
        while (true) {
            uint32_t child_depth = s.node->GetDepth() + 1;
            if (child_depth <= this->m_max_node_depth_) {
                // do not go beyond the specified max depth
                OctreeKey::KeyType center_offset_key =
                    this->m_tree_->m_tree_key_offset_ >> child_depth;
                for (int i = 7; i >= 0; --i) {
                    if (!s.node->HasChild(i)) { continue; }

                    const Node *child = this->m_tree_->GetNodeChild(s.node, i);
                    typename IteratorBase::StackElement s_child;
                    OctreeKey::ComputeChildKey(i, center_offset_key, s.key, s_child.key);

                    const Dtype center_x = this->m_tree_->KeyToCoord(s_child.key[0], child_depth);
                    const Dtype center_y = this->m_tree_->KeyToCoord(s_child.key[1], child_depth);
                    const Dtype center_z = this->m_tree_->KeyToCoord(s_child.key[2], child_depth);
                    const Dtype half_size =
                        this->m_tree_->GetNodeSize(child_depth) / 2.0 + m_node_padding_;
                    Vector3 box_min(
                        center_x - half_size,
                        center_y - half_size,
                        center_z - half_size);
                    Vector3 box_max(
                        center_x + half_size,
                        center_y + half_size,
                        center_z + half_size);
                    Dtype dist1 = 0.0, dist2 = 0.0;
                    bool intersected = false, is_inside = false;
                    ComputeIntersectionBetweenRayAndAabb3D(
                        m_origin_,
                        m_dir_inv_,
                        box_min,
                        box_max,
                        dist1,
                        dist2,
                        intersected,
                        is_inside);
                    if (!intersected) { continue; }
                    if (!child->HasAnyChild()) {  // leaf node
                        if (!m_bidirectional_ && dist1 < 0.) { continue; }
                        if (m_max_range_ > 0. && std::abs(dist1) > m_max_range_) { continue; }
                    }
                    s_child.node = child;
                    s_child.data = std::make_shared<Dtype>(dist1);
                    this->m_stack_.push_back(s_child);
                    std::push_heap(this->m_stack_.begin(), this->m_stack_.end(), cmp);
                }
            }
            if (this->m_stack_.empty()) {  // end of iteration
                this->Terminate();
                return;
            }

            std::pop_heap(this->m_stack_.begin(), this->m_stack_.end(), cmp);
            // now the stack top is at the back
            if (this->m_stack_.back().node->GetDepth() >= m_min_node_depth_) {
                // the current stack top is deep enough
                if (!this->m_leaf_only_) { return; }  // not required to be a leaf node
                if (this->IsLeaf()) { return; }       // current stack top is leaf node
            }
            s = this->m_stack_.back();  // current stack top is internal node
            this->m_stack_.pop_back();
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::LeafIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginLeaf(uint32_t max_depth) const {
        return LeafIterator(this, max_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::LeafIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::EndLeaf() const {
        return LeafIterator();
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::LeafOfNodeIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginLeafOfNode(
        const OctreeKey &key,
        uint32_t node_depth,
        uint32_t max_depth) const {
        return LeafOfNodeIterator(key, node_depth, this, max_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::LeafOfNodeIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::EndLeafOfNode() const {
        return LeafOfNodeIterator();
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::LeafInAabbIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginLeafInAabb(
        const Aabb<Dtype, 3> &aabb,
        const uint32_t max_depth) const {
        return LeafInAabbIterator(
            aabb.min().x(),
            aabb.min().y(),
            aabb.min().z(),
            aabb.max().x(),
            aabb.max().y(),
            aabb.max().z(),
            this,
            max_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::LeafInAabbIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginLeafInAabb(
        Dtype aabb_min_x,
        Dtype aabb_min_y,
        Dtype aabb_min_z,
        Dtype aabb_max_x,
        Dtype aabb_max_y,
        Dtype aabb_max_z,
        const uint32_t max_depth) const {
        return LeafInAabbIterator(
            aabb_min_x,
            aabb_min_y,
            aabb_min_z,
            aabb_max_x,
            aabb_max_y,
            aabb_max_z,
            this,
            max_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::LeafInAabbIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginLeafInAabb(
        const OctreeKey &aabb_min_key,
        const OctreeKey &aabb_max_key,
        uint32_t max_depth) const {
        return LeafInAabbIterator(aabb_min_key, aabb_max_key, this, max_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::LeafInAabbIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::EndLeafInAabb() const {
        return LeafInAabbIterator();
    }

    template<class Node, class Interface, class InterfaceSetting>
    std::shared_ptr<typename AbstractOctree<
        typename OctreeImpl<Node, Interface, InterfaceSetting>::Dtype>::OctreeNodeIterator>
    OctreeImpl<Node, Interface, InterfaceSetting>::GetLeafInAabbIterator(
        const Aabb<Dtype, 3> &aabb,
        uint32_t max_depth) const {
        return std::make_shared<LeafInAabbIterator>(
            aabb.min().x(),
            aabb.min().y(),
            aabb.min().z(),
            aabb.max().x(),
            aabb.max().y(),
            aabb.max().z(),
            this,
            max_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::TreeIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginTree(uint32_t max_depth) const {
        return TreeIterator(this, max_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::TreeIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::EndTree() const {
        return TreeIterator();
    }

    template<class Node, class Interface, class InterfaceSetting>
    std::shared_ptr<typename AbstractOctree<
        typename OctreeImpl<Node, Interface, InterfaceSetting>::Dtype>::OctreeNodeIterator>
    OctreeImpl<Node, Interface, InterfaceSetting>::GetTreeIterator(uint32_t max_depth) const {
        return std::make_shared<TreeIterator>(this, max_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::TreeInAabbIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginTreeInAabb(
        const Aabb<Dtype, 3> &aabb,
        const uint32_t max_depth) const {
        return TreeInAabbIterator(
            aabb.min().x(),
            aabb.min().y(),
            aabb.min().z(),
            aabb.max().x(),
            aabb.max().y(),
            aabb.max().z(),
            this,
            max_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::TreeInAabbIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginTreeInAabb(
        Dtype aabb_min_x,
        Dtype aabb_min_y,
        Dtype aabb_min_z,
        Dtype aabb_max_x,
        Dtype aabb_max_y,
        Dtype aabb_max_z,
        uint32_t max_depth) const {
        return TreeInAabbIterator(
            aabb_min_x,
            aabb_min_y,
            aabb_min_z,
            aabb_max_x,
            aabb_max_y,
            aabb_max_z,
            this,
            max_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::TreeInAabbIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginTreeInAabb(
        const OctreeKey &aabb_min_key,
        const OctreeKey &aabb_max_key,
        uint32_t max_depth) const {
        return TreeInAabbIterator(aabb_min_key, aabb_max_key, this, max_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::TreeInAabbIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::EndTreeInAabb() const {
        return TreeInAabbIterator();
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::WestLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginWestLeafNeighbor(
        Dtype x,
        Dtype y,
        Dtype z,
        uint32_t max_leaf_depth) const {
        return WestLeafNeighborIterator(x, y, z, this, max_leaf_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::WestLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginWestLeafNeighbor(
        const OctreeKey &key,
        uint32_t key_depth,
        uint32_t max_leaf_depth) const {
        return WestLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::WestLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::EndWestLeafNeighbor() const {
        return WestLeafNeighborIterator();
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::EastLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginEastLeafNeighbor(
        Dtype x,
        Dtype y,
        Dtype z,
        uint32_t max_leaf_depth) const {
        return EastLeafNeighborIterator(x, y, z, this, max_leaf_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::EastLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginEastLeafNeighbor(
        const OctreeKey &key,
        uint32_t key_depth,
        uint32_t max_leaf_depth) const {
        return EastLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::EastLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::EndEastLeafNeighbor() const {
        return EastLeafNeighborIterator();
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::NorthLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginNorthLeafNeighbor(
        Dtype x,
        Dtype y,
        Dtype z,
        uint32_t max_leaf_depth) const {
        return NorthLeafNeighborIterator(x, y, z, this, max_leaf_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::NorthLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginNorthLeafNeighbor(
        const OctreeKey &key,
        uint32_t key_depth,
        uint32_t max_leaf_depth) const {
        return NorthLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::NorthLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::EndNorthLeafNeighbor() const {
        return NorthLeafNeighborIterator();
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::SouthLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginSouthLeafNeighbor(
        Dtype x,
        Dtype y,
        Dtype z,
        uint32_t max_leaf_depth) const {
        return SouthLeafNeighborIterator(x, y, z, this, max_leaf_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::SouthLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginSouthLeafNeighbor(
        const OctreeKey &key,
        uint32_t key_depth,
        uint32_t max_leaf_depth) const {
        return SouthLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::SouthLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::EndSouthLeafNeighbor() const {
        return SouthLeafNeighborIterator();
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::TopLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginTopLeafNeighbor(
        Dtype x,
        Dtype y,
        Dtype z,
        uint32_t max_leaf_depth) const {
        return TopLeafNeighborIterator(x, y, z, this, max_leaf_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::TopLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginTopLeafNeighbor(
        const OctreeKey &key,
        uint32_t key_depth,
        uint32_t max_leaf_depth) const {
        return TopLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::TopLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::EndTopLeafNeighbor() const {
        return TopLeafNeighborIterator();
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::BottomLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginBottomLeafNeighbor(
        Dtype x,
        Dtype y,
        Dtype z,
        uint32_t max_leaf_depth) const {
        return BottomLeafNeighborIterator(x, y, z, this, max_leaf_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::BottomLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginBottomLeafNeighbor(
        const OctreeKey &key,
        uint32_t key_depth,
        uint32_t max_leaf_depth) const {
        return BottomLeafNeighborIterator(key, key_depth, this, max_leaf_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::BottomLeafNeighborIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::EndBottomLeafNeighbor() const {
        return BottomLeafNeighborIterator();
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::NodeOnRayIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginNodeOnRay(
        Dtype px,
        Dtype py,
        Dtype pz,
        Dtype vx,
        Dtype vy,
        Dtype vz,
        Dtype max_range,
        Dtype node_padding,
        bool bidirectional,
        bool leaf_only,
        uint32_t min_node_depth,
        uint32_t max_node_depth) const {
        return {
            px,
            py,
            pz,
            vx,
            vy,
            vz,
            max_range,
            node_padding,
            bidirectional,
            this,
            leaf_only,
            min_node_depth,
            max_node_depth};
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::NodeOnRayIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::BeginNodeOnRay(
        const Eigen::Ref<const Vector3> &origin,
        const Eigen::Ref<const Vector3> &direction,
        Dtype max_range,
        Dtype node_padding,
        bool bidirectional,
        bool leaf_only,
        uint32_t min_node_depth,
        uint32_t max_node_depth) const {
        return NodeOnRayIterator(
            origin.x(),
            origin.y(),
            origin.z(),
            direction.x(),
            direction.y(),
            direction.z(),
            max_range,
            node_padding,
            bidirectional,
            this,
            leaf_only,
            min_node_depth,
            max_node_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    typename OctreeImpl<Node, Interface, InterfaceSetting>::NodeOnRayIterator
    OctreeImpl<Node, Interface, InterfaceSetting>::EndNodeOnRay() const {
        return {};
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::ComputeRayKeys(
        Dtype sx,
        Dtype sy,
        Dtype sz,
        Dtype ex,
        Dtype ey,
        Dtype ez,
        OctreeKeyRay &ray) const {
        // See "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo Digital
        // Difference Analyzer (DDA) algorithm. Note that we cannot use Bresenham's line algorithm
        // because it may miss some voxels when the ray is not axis-aligned. For example, if the ray
        // is from (0, 0) to (1, 1), Bresenham's algorithm will miss (1, 0) and (0, 1), but the ray
        // should traverse them. Also look at https://en.wikipedia.org/wiki/File:Bresenham.svg for
        // another example of Bresenham's algorithm.

        ray.clear();
        OctreeKey key_start, key_end;
        if (!CoordToKeyChecked(sx, sy, sz, key_start) || !CoordToKeyChecked(ex, ey, ez, key_end)) {
            ERL_WARN("Ray ({}, {}, {}) -> ({}, {}, {}) is out of range.\n", sx, sy, sz, ex, ey, ez);
            return false;
        }
        if (key_start == key_end) { return true; }

        // initialization phase
        Dtype direction[3];
        Dtype &dx = direction[0];
        Dtype &dy = direction[1];
        Dtype &dz = direction[2];
        dx = ex - sx;
        dy = ey - sy;
        dz = ez - sz;
        const Dtype length = std::sqrt(dx * dx + dy * dy + dz * dz);
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
        const auto resolution = static_cast<Dtype>(m_setting_->resolution);
        Dtype t_max[3];
        Dtype t_delta[3];
        if (step[0] == 0) {
            t_max[0] = std::numeric_limits<Dtype>::infinity();
            t_delta[0] = std::numeric_limits<Dtype>::infinity();
        } else {
            t_max[0] =
                (KeyToCoord(key_start[0]) + static_cast<Dtype>(step[0]) * 0.5 * resolution - sx) /
                dx;
            t_delta[0] = resolution / std::abs(dx);
        }
        if (step[1] == 0) {
            t_max[1] = std::numeric_limits<Dtype>::infinity();
            t_delta[1] = std::numeric_limits<Dtype>::infinity();
        } else {
            t_max[1] =
                (KeyToCoord(key_start[1]) + static_cast<Dtype>(step[1]) * 0.5 * resolution - sy) /
                dy;
            t_delta[1] = resolution / std::abs(dy);
        }
        if (step[2] == 0) {
            t_max[2] = std::numeric_limits<Dtype>::infinity();
            t_delta[2] = std::numeric_limits<Dtype>::infinity();
        } else {
            t_max[2] =
                (KeyToCoord(key_start[2]) + static_cast<Dtype>(step[2]) * 0.5 * resolution - sz) /
                dz;
            t_delta[2] = resolution / std::abs(dz);
        }

        // incremental phase
        // for each step, find the next voxel, by the smallest travel distance, t_max, and
        // update t_max, and push the key to ray
        ray.push_back(key_start);
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

            // this happens due to the numerical error
            if (std::min(t_max[0], std::min(t_max[1], t_max[2])) > length) { break; }

            ray.push_back(current_key);
        }

        return true;
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::ComputeRayCoords(
        const Dtype sx,
        const Dtype sy,
        const Dtype sz,
        const Dtype ex,
        const Dtype ey,
        const Dtype ez,
        std::vector<Vector3> &ray) const {
        ray.clear();
        OctreeKeyRay key_ray;
        if (!ComputeRayKeys(sx, sy, sz, ex, ey, ez, key_ray)) { return false; }
        ray.reserve(key_ray.size());
        std::transform(
            key_ray.begin(),
            key_ray.end(),
            std::back_inserter(ray),
            [this](const OctreeKey &key) -> Vector3 {
                return {
                    this->KeyToCoord(key[0]),
                    this->KeyToCoord(key[1]),
                    this->KeyToCoord(key[2])};
            });
        return true;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::ClearKeyRays() {
        m_key_rays_.clear();
    }

    template<class Node, class Interface, class InterfaceSetting>
    Node *
    OctreeImpl<Node, Interface, InterfaceSetting>::CreateNodeChild(Node *node, uint32_t child_idx) {
        node->AllocateChildrenPtr();  // allocate children if necessary
        ERL_DEBUG_ASSERT(!node->HasChild(child_idx), "Child already exists.");
        Node *new_child = reinterpret_cast<Node *>(node->CreateChild(child_idx));  // create child
        m_tree_size_++;          // increase tree size
        m_size_changed_ = true;  // the size of the tree has changed
        return new_child;
    }

    template<class Node, class Interface, class InterfaceSetting>
    uint32_t
    OctreeImpl<Node, Interface, InterfaceSetting>::DeleteNodeChild(
        Node *node,
        uint32_t child_idx,
        const OctreeKey &key) {
        const uint32_t old_tree_size = m_tree_size_;
        Node *child = node->template GetChild<Node>(child_idx);
        this->DeleteNodeDescendants(child, key);
        this->OnDeleteNodeChild(node, child, key);
        node->RemoveChild(child_idx);
        m_tree_size_--;
        m_size_changed_ = true;
        return old_tree_size - m_tree_size_;
    }

    template<class Node, class Interface, class InterfaceSetting>
    Node *
    OctreeImpl<Node, Interface, InterfaceSetting>::GetNodeChild(Node *node, uint32_t child_idx) {
        return node->template GetChild<Node>(child_idx);
    }

    template<class Node, class Interface, class InterfaceSetting>
    const Node *
    OctreeImpl<Node, Interface, InterfaceSetting>::GetNodeChild(
        const Node *node,
        uint32_t child_idx) const {
        return node->template GetChild<Node>(child_idx);
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::IsNodeCollapsible(const Node *node) const {
        // all children must exist
        if (node->GetNumChildren() != 8) { return false; }
        // the children should be leaf nodes
        // if a child is a leaf node, its data should be equal to the first child
        // we don't need to check if their depth etc. is the same, because they are all children of
        // the same node
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

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::ExpandNode(Node *node) {
        ERL_DEBUG_ASSERT(!node->HasAnyChild(), "Node already has children.");
        node->Expand();
        ERL_DEBUG_ASSERT(node->GetNumChildren() == 8, "Node should have 8 children.");
        m_tree_size_ += 8;
        m_size_changed_ = true;
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::PruneNode(Node *node) {
        if (!IsNodeCollapsible(node)) { return false; }
        ERL_DEBUG_ASSERT(node->GetNumChildren() == 8, "Node should have 8 children.");
        node->Prune();
        ERL_DEBUG_ASSERT(node->GetNumChildren() == 0, "Node should have no children.");
        m_tree_size_ -= 8;
        m_size_changed_ = true;
        return true;
    }

    template<class Node, class Interface, class InterfaceSetting>
    uint32_t
    OctreeImpl<Node, Interface, InterfaceSetting>::DeleteNode(
        Dtype x,
        Dtype y,
        Dtype z,
        const uint32_t delete_depth) {
        if (OctreeKey key; !this->CoordToKeyChecked(x, y, z, key)) {
            ERL_WARN("Point ({}, {}, {}) is out of range.", x, y, z);
            return 0;
        } else {
            const uint32_t old_tree_size = m_tree_size_;
            this->DeleteNode(key, delete_depth);
            return old_tree_size - m_tree_size_;
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::DeleteNode(
        const OctreeKey &key,
        uint32_t delete_depth) {
        if (m_root_ == nullptr) { return; }
        if (delete_depth == 0) { delete_depth = m_setting_->tree_depth; }
        if (this->DeleteNodeRecurs(m_root_.get(), key, delete_depth)) {  // delete the root node
            this->OnDeleteNodeChild(nullptr, m_root_.get(), key);
            m_root_ = nullptr;
            m_tree_size_ = 0;
            m_size_changed_ = true;
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::OnDeleteNodeChild(
        Node *node,
        Node *child,
        const OctreeKey &key) {
        (void) node;
        (void) child;
        (void) key;
    }

    template<class Node, class Interface, class InterfaceSetting>
    bool
    OctreeImpl<Node, Interface, InterfaceSetting>::DeleteNodeRecurs(
        Node *node,
        const OctreeKey &key,
        uint32_t max_depth) {
        const uint32_t depth = node->GetDepth();
        if (depth >= max_depth) { return true; }  // return true to delete this node
        ERL_DEBUG_ASSERT(node != nullptr, "node should not be nullptr.");

        uint32_t child_idx = OctreeKey::ComputeChildIndex(key, m_setting_->tree_depth - depth - 1);
        if (!node->HasChild(child_idx)) {  // child does not exist, but maybe the node is pruned
            if (!node->HasAnyChild() && (node != m_root_.get())) {  // this node is pruned
                ExpandNode(node);  // expand it, the tree size is updated in ExpandNode
            } else {               // node is not pruned, we are done
                return false;      // nothing to delete
            }
        }

        if (auto child = this->GetNodeChild(node, child_idx);
            this->DeleteNodeRecurs(child, key, max_depth)) {
            this->DeleteNodeDescendants(child, key);    // delete the child's children recursively
            this->OnDeleteNodeChild(node, child, key);  // callback before deleting the child
            node->RemoveChild(child_idx);               // remove child
            m_tree_size_--;                             // decrease tree size
            m_size_changed_ = true;                     // the size of the tree has changed
            if (!node->HasAnyChild()) { return true; }  // node is pruned, can be deleted
        }
        return false;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::DeleteNodeDescendants(
        Node *node,
        const OctreeKey &key) {
        ERL_DEBUG_ASSERT(node != nullptr, "node should not be nullptr.");
        if (!node->HasAnyChild()) { return; }
        for (int i = 0; i < 8; ++i) {
            Node *child = this->GetNodeChild(node, i);
            if (child == nullptr) { continue; }
            this->DeleteNodeDescendants(child, key);
            this->OnDeleteNodeChild(node, child, key);  // callback before deleting the child
            node->RemoveChild(i);                       // remove child
            m_tree_size_--;                             // decrease tree size
            m_size_changed_ = true;                     // the size of the tree has changed
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::Clear() {
        m_root_ = nullptr;  // delete the root node to trigger the destructor of the nodes
        m_tree_size_ = 0;
        m_size_changed_ = true;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::Prune() {
        if (m_root_ == nullptr) { return; }
        for (long depth = m_setting_->tree_depth - 1; depth > 0; --depth) {
            const uint32_t old_tree_size = m_tree_size_;
            this->PruneRecurs(this->m_root_.get(), depth);
            if (old_tree_size - m_tree_size_ == 0) { break; }
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::PruneRecurs(
        Node *node,
        const uint32_t max_depth) {
        if (node->GetDepth() < max_depth) {
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

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::Expand() {
        if (m_root_ == nullptr) { return; }
        this->ExpandRecurs(m_root_.get(), 0, m_setting_->tree_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::ExpandRecurs(
        Node *node,
        const uint32_t depth,
        const uint32_t max_depth) {
        if (depth >= max_depth) { return; }
        ERL_DEBUG_ASSERT(node != nullptr, "node is nullptr");

        if (!node->HasAnyChild()) { this->ExpandNode(node); }  // node has no children, expand it
        for (int i = 0; i < 8; ++i) {
            Node *child = this->GetNodeChild(node, i);
            if (child == nullptr) { continue; }
            this->ExpandRecurs(child, depth + 1, max_depth);
        }
    }

    template<class Node, class Interface, class InterfaceSetting>
    const std::shared_ptr<Node> &
    OctreeImpl<Node, Interface, InterfaceSetting>::GetRoot() const {
        return m_root_;
    }

    template<class Node, class Interface, class InterfaceSetting>
    const AbstractOctreeNode *
    OctreeImpl<Node, Interface, InterfaceSetting>::SearchNode(
        const Dtype x,
        const Dtype y,
        const Dtype z,
        const uint32_t max_depth) const {
        return static_cast<const AbstractOctreeNode *>(Search(x, y, z, max_depth));
    }

    template<class Node, class Interface, class InterfaceSetting>
    const Node *
    OctreeImpl<Node, Interface, InterfaceSetting>::Search(
        const Eigen::Ref<const Vector3> &position,
        const uint32_t max_depth) const {
        return Search(position.x(), position.y(), position.z(), max_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    const Node *
    OctreeImpl<Node, Interface, InterfaceSetting>::Search(
        Dtype x,
        Dtype y,
        Dtype z,
        const uint32_t max_depth) const {
        OctreeKey key;
        if (!CoordToKeyChecked(x, y, z, key)) {
            ERL_WARN("Point ({}, {}, {}) is out of range.\n", x, y, z);
            return nullptr;
        }

        return Search(key, max_depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    const Node *
    OctreeImpl<Node, Interface, InterfaceSetting>::Search(const OctreeKey &key, uint32_t max_depth)
        const {
        auto &tree_depth = m_setting_->tree_depth;
        ERL_DEBUG_ASSERT(
            max_depth <= tree_depth,
            "Depth must be in [0, %u], but got %u.",
            tree_depth,
            max_depth);

        if (m_root_ == nullptr) { return nullptr; }
        if (max_depth == 0) { max_depth = tree_depth; }

        // generate the appropriate key for the given depth
        OctreeKey key_at_depth = key;
        if (max_depth != tree_depth) {
            key_at_depth = this->AdjustKeyToDepth(key_at_depth, max_depth);
        }

        // search
        const Node *node = m_root_.get();
        const int min_level = tree_depth - max_depth;
        // follow nodes down to the requested level (for level = 0, it is the leaf level)
        for (int level = tree_depth - 1; level >= min_level; --level) {
            if (const uint32_t child_index = OctreeKey::ComputeChildIndex(key_at_depth, level);
                node->HasChild(child_index)) {
                node = this->GetNodeChild(node, child_index);
            } else {
                // we expect a child but did not get it, is the current node a leaf?
                if (!node->HasAnyChild()) { return node; }  // a leaf, so we cannot go deeper
                return nullptr;                             // not a leaf, search failed
            }
        }
        return node;
    }

    template<class Node, class Interface, class InterfaceSetting>
    Node *
    OctreeImpl<Node, Interface, InterfaceSetting>::InsertNode(
        Dtype x,
        Dtype y,
        Dtype z,
        const uint32_t depth) {
        OctreeKey key;
        if (!CoordToKeyChecked(x, y, z, key)) {
            ERL_WARN("Point ({}, {}, {}) is out of range.\n", x, y, z);
            return nullptr;
        }
        return InsertNode(key, depth);
    }

    template<class Node, class Interface, class InterfaceSetting>
    Node *
    OctreeImpl<Node, Interface, InterfaceSetting>::InsertNode(
        const OctreeKey &key,
        uint32_t depth) {
        const uint32_t tree_depth = m_setting_->tree_depth;
        if (depth == 0) { depth = tree_depth; }
        ERL_DEBUG_ASSERT(
            depth <= tree_depth,
            "Depth must be in [0, %u], but got %u.",
            tree_depth,
            depth);
        if (this->m_root_ == nullptr) {
            this->m_root_ = std::make_shared<Node>();
            ++this->m_tree_size_;
        }

        Node *node = this->m_root_.get();
        const int diff = tree_depth - depth;
        for (int level = tree_depth - 1; level >= diff; --level) {
            if (const uint32_t child_index = OctreeKey::ComputeChildIndex(key, level);
                node->HasChild(child_index)) {
                node = GetNodeChild(node, child_index);
            } else {
                node = CreateNodeChild(node, child_index);
            }
        }

        return node;
    }

    template<class Node, class Interface, class InterfaceSetting>
    std::istream &
    OctreeImpl<Node, Interface, InterfaceSetting>::ReadData(std::istream &s) {
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
            std::bitset<8> children(static_cast<unsigned long long>(children_char));
            node->AllocateChildrenPtr();
            for (int i = 7; i >= 0; --i) {  // the same order as the recursive implementation
                if (!children[i]) { continue; }
                Node *child_node = reinterpret_cast<Node *>(node->CreateChild(i));
                nodes_stack.push_back(child_node);
                m_tree_size_++;
            }
        }

        return s;
    }

    template<class Node, class Interface, class InterfaceSetting>
    std::ostream &
    OctreeImpl<Node, Interface, InterfaceSetting>::WriteData(std::ostream &s) const {
        if (m_root_ == nullptr) { return s; }

        std::list<const Node *> nodes_stack;
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
            auto children_char = static_cast<char>(children.to_ulong());
            s.write(&children_char, sizeof(char));
        }

        return s;
    }

    template<class Node, class Interface, class InterfaceSetting>
    void
    OctreeImpl<Node, Interface, InterfaceSetting>::ComputeMinMax() {
        if (!m_size_changed_) { return; }

        // empty tree
        if (m_root_ == nullptr) {
            m_metric_min_[0] = m_metric_min_[1] = m_metric_min_[2] = 0.;
            m_metric_max_[0] = m_metric_max_[1] = m_metric_max_[2] = 0.;
            m_size_changed_ = false;
            return;
        }

        // non-empty tree
        m_metric_min_[0] = m_metric_min_[1] = m_metric_min_[2] =
            std::numeric_limits<Dtype>::infinity();
        m_metric_max_[0] = m_metric_max_[1] = m_metric_max_[2] =
            -std::numeric_limits<Dtype>::infinity();

        for (auto it = this->BeginLeaf(), end = this->EndLeaf(); it != end; ++it) {
            const Dtype size = it.GetNodeSize();
            const Dtype half_size = size / 2.0;
            Dtype x = it.GetX() - half_size;
            Dtype y = it.GetY() - half_size;
            Dtype z = it.GetZ() - half_size;
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
}  // namespace erl::geometry
