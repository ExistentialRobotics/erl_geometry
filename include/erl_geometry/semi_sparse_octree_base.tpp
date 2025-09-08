#pragma once

#include "semi_sparse_octree_base.hpp"

namespace erl::geometry {

    template<typename Dtype, class Node, class Setting>
    SemiSparseOctreeBase<Dtype, Node, Setting>::SemiSparseOctreeBase(
        const std::shared_ptr<Setting> &setting)
        : Super(setting),
          m_setting_(setting) {
        m_parents_.reserve(m_setting_->init_voxel_num);
        m_children_.reserve(m_setting_->init_voxel_num);
        m_vertices_.reserve(m_setting_->init_voxel_num);
        m_voxels_.reserve(m_setting_->init_voxel_num);
        m_key_to_vertex_map_.reserve(m_setting_->init_voxel_num << 3);
        m_vertex_keys_.reserve(m_setting_->init_voxel_num << 3);

        // create the root node
        const uint32_t &x = this->m_tree_key_offset_;
        const OctreeKey root_key(x, x, x);
        this->m_root_.reset(CreateNode(root_key, m_setting_->tree_depth).first);
    }

    template<typename Dtype, class Node, class Setting>
    std::shared_ptr<AbstractOctree<Dtype>>
    SemiSparseOctreeBase<Dtype, Node, Setting>::Clone() const {
        std::shared_ptr<AbstractOctree<Dtype>> tree = Super::Clone();
        std::shared_ptr<SemiSparseOctreeBase> semi_sparse_tree =
            std::dynamic_pointer_cast<SemiSparseOctreeBase>(tree);
        semi_sparse_tree->m_children_ = m_children_;
        semi_sparse_tree->m_voxels_ = m_voxels_;
        semi_sparse_tree->m_vertices_ = m_vertices_;
        semi_sparse_tree->m_key_to_vertex_map_ = m_key_to_vertex_map_;
        semi_sparse_tree->m_vertex_keys_ = m_vertex_keys_;
        return tree;
    }

    template<typename Dtype, class Node, class Setting>
    const std::vector<typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex> &
    SemiSparseOctreeBase<Dtype, Node, Setting>::GetParents() const {
        return m_parents_;
    }

    template<typename Dtype, class Node, class Setting>
    const std::vector<
        std::array<typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex, 8>> &
    SemiSparseOctreeBase<Dtype, Node, Setting>::GetChildren() const {
        return m_children_;
    }

    template<typename Dtype, class Node, class Setting>
    const std::vector<std::array<uint16_t, 4>> &
    SemiSparseOctreeBase<Dtype, Node, Setting>::GetVoxels() const {
        return m_voxels_;
    }

    template<typename Dtype, class Node, class Setting>
    const std::vector<
        std::array<typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex, 8>> &
    SemiSparseOctreeBase<Dtype, Node, Setting>::GetVertices() const {
        return m_vertices_;
    }

    template<typename Dtype, class Node, class Setting>
    std::size_t
    SemiSparseOctreeBase<Dtype, Node, Setting>::GetVertexCount() const {
        return m_vertex_keys_.size();
    }

    template<typename Dtype, class Node, class Setting>
    const OctreeKeyVector &
    SemiSparseOctreeBase<Dtype, Node, Setting>::GetVertexKeys() const {
        return m_vertex_keys_;
    }

    template<typename Dtype, class Node, class Setting>
    std::vector<typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex>
    SemiSparseOctreeBase<Dtype, Node, Setting>::InsertPoints(
        const Dtype *points,
        const std::size_t num_points) {
        std::vector<NodeIndex> voxel_indices;
        voxel_indices.reserve(num_points);

        auto p = points;
        for (std::size_t i = 0; i < num_points; ++i, p += 3) {
            if (OctreeKey key; !this->CoordToKeyChecked(p[0], p[1], p[2], key)) {
                ERL_WARN("Point ({}, {}, {}) is out of range.", p[0], p[1], p[2]);
                voxel_indices.push_back(-1);
            } else {
                voxel_indices.push_back(InsertPoint(key, 0));
            }
        }

        return voxel_indices;
    }

    template<typename Dtype, class Node, class Setting>
    std::vector<typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex>
    SemiSparseOctreeBase<Dtype, Node, Setting>::InsertPoints(
        const OctreeKey *keys,
        const std::size_t num_points) {
        std::vector<NodeIndex> voxel_indices;
        voxel_indices.reserve(num_points);

        auto k = keys;
        for (std::size_t i = 0; i < num_points; ++i) {
            voxel_indices.push_back(InsertPoint(*k++, 0));
        }

        return voxel_indices;
    }

    template<typename Dtype, class Node, class Setting>
    typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex
    SemiSparseOctreeBase<Dtype, Node, Setting>::InsertPoint(
        const OctreeKey &key,
        uint32_t max_depth) {
        Node *node = this->m_root_.get();
        NodeIndex node_index = 0;

        const uint32_t tree_depth = m_setting_->tree_depth;
        if (max_depth == 0) { max_depth = tree_depth; }
        const int min_level = tree_depth - max_depth;

        int child_level = tree_depth - 1;
        const uint64_t code = key.ToMortonCode();
        uint64_t shift = (child_level << 1) + child_level;  // child_level * 3
        uint64_t mask = 0b111 << shift;
        while (child_level >= min_level) {
            if (const auto child_index = static_cast<int>((code & mask) >> shift);
                node->HasChild(child_index)) {
                node = this->GetNodeChild(node, child_index);
                node_index = m_children_[node_index][child_index];
            } else {
                std::tie(node, node_index) = CreateNode(key, child_level, node, child_index);
            }
            --child_level;
            shift -= 3;
            mask >>= 3;
        }

        return node_index;
    }

    template<typename Dtype, class Node, class Setting>
    std::vector<typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex>
    SemiSparseOctreeBase<Dtype, Node, Setting>::FindVoxelIndices(
        const Dtype *points,
        const std::size_t num_points,
        bool parallel) const {
        std::vector<NodeIndex> voxel_indices;
        voxel_indices.resize(num_points);

#pragma omp parallel if (parallel) default(none) shared(num_points, points, voxel_indices)
        for (std::size_t i = 0; i < num_points; ++i) {
            std::size_t j = (i << 1) + i;  // i * 3
            if (OctreeKey key;
                !this->CoordToKeyChecked(points[j], points[j + 1], points[j + 2], key)) {
                voxel_indices[i] = -1;
            } else {
                voxel_indices[i] = FindVoxelIndex(key);
            }
        }

        return voxel_indices;
    }

    template<typename Dtype, class Node, class Setting>
    std::vector<typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex>
    SemiSparseOctreeBase<Dtype, Node, Setting>::FindVoxelIndices(
        const OctreeKey *keys,
        const std::size_t num_points,
        bool parallel) const {
        std::vector<NodeIndex> voxel_indices;
        voxel_indices.resize(num_points);
#pragma omp parallel if (parallel) default(none) shared(num_points, keys, voxel_indices)
        for (std::size_t i = 0; i < num_points; ++i) { voxel_indices[i] = FindVoxelIndex(keys[i]); }
        return voxel_indices;
    }

    template<typename Dtype, class Node, class Setting>
    typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex
    SemiSparseOctreeBase<Dtype, Node, Setting>::FindVoxelIndex(const OctreeKey &key) const {
        int child_level = m_setting_->tree_depth - 1;
        const uint64_t code = key.ToMortonCode();
        uint64_t shift = 3 * child_level;
        uint64_t mask = 0b111 << shift;
        NodeIndex node_index = 0;

        while (child_level >= 0) {
            if (const auto child_index = static_cast<int>((code & mask) >> shift);
                m_children_[node_index][child_index] >= 0) {
                node_index = m_children_[node_index][child_index];
            } else {
                return node_index;
            }
            --child_level;
            shift -= 3;
            mask >>= 3;
        }

        return node_index;
    }

    template<typename Dtype, class Node, class Setting>
    typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex
    SemiSparseOctreeBase<Dtype, Node, Setting>::AllocateVoxelEntry(
        const OctreeKey &key,
        const OctreeKey::KeyType level,
        const NodeIndex parent_node_index,
        const NodeIndex child_index) {

        if (m_recycled_node_indices_.empty()) {
            const auto node_index = static_cast<NodeIndex>(m_children_.size());
            m_parents_.push_back(parent_node_index);
            m_children_.push_back({-1, -1, -1, -1, -1, -1, -1, -1});
            m_voxels_.push_back({key[0], key[1], key[2], level});
            m_vertices_.push_back({-1, -1, -1, -1, -1, -1, -1, -1});
            if (parent_node_index >= 0) {
                m_children_[parent_node_index][child_index] = node_index;
            }
            return node_index;
        }
        const auto it = m_recycled_node_indices_.begin();
        const NodeIndex node_index = *it;
        m_recycled_node_indices_.erase(it);
        m_parents_[node_index] = parent_node_index;
        m_voxels_[node_index] = {key[0], key[1], key[2], level};

        // already reset in OnDeleteNodeChild
        // m_children_[node_index] = {-1, -1, -1, -1, -1, -1, -1, -1};

        // will be set in RecordVertices
        // m_vertices_[node_index] = {-1, -1, -1, -1, -1, -1, -1, -1};

        if (parent_node_index >= 0) { m_children_[parent_node_index][child_index] = node_index; }

        return node_index;
    }

    template<typename Dtype, class Node, class Setting>
    std::pair<Node *, typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex>
    SemiSparseOctreeBase<Dtype, Node, Setting>::CreateNode(
        OctreeKey node_key,
        uint32_t level,
        Node *parent,
        int child_index) {

        NodeIndex node_index;
        Node *node = nullptr;

        const uint32_t depth = m_setting_->tree_depth - level;
        node_key = this->AdjustKeyToDepth(node_key, depth);

        if (parent != nullptr) {
            // We are going to create the child node. But we need to make sure its siblings are
            // also created if the depth is still smaller than full_depth.
            const NodeIndex parent_node_idx = parent->GetNodeIndex();
            if (BuildFullTree(this->AdjustKeyToDepth(node_key, depth - 1), parent)) {
                node = this->GetNodeChild(parent, child_index);
                node_index = m_children_[parent_node_idx][child_index];
            } else {
                // the child is not created in BuildFullTree, create it now.
                node = this->CreateNodeChild(parent, child_index);
                node_index = AllocateVoxelEntry(node_key, level, parent_node_idx, child_index);
                RecordVertices(node_key, node_index, level);
                node->SetNodeIndex(node_index);
            }
        } else {  // root
            node_index = AllocateVoxelEntry(node_key, level, -1, -1);
            node = new Node;
            node->SetNodeIndex(node_index);
            ++this->m_tree_size_;
            RecordVertices(node_key, node_index, level);
        }

        return {node, node_index};
    }

    template<typename Dtype, class Node, class Setting>
    void
    SemiSparseOctreeBase<Dtype, Node, Setting>::RecordVertices(
        const OctreeKey &node_key,
        const NodeIndex node_idx,
        uint32_t level) {
        OctreeKey vertex_key;
        for (int i = 0; i < 8; ++i) {
            OctreeKey::ComputeVertexKey(i, level, node_key, vertex_key);

            auto [it, inserted] =
                m_key_to_vertex_map_.try_emplace(vertex_key, m_vertex_keys_.size());
            if (inserted) { m_vertex_keys_.push_back(vertex_key); }
            m_vertices_[node_idx][i] = it->second;
        }
    }

    template<typename Dtype, class Node, class Setting>
    bool
    SemiSparseOctreeBase<Dtype, Node, Setting>::BuildFullTree(
        const OctreeKey &node_key,
        Node *node) {

        const uint32_t depth = node->GetDepth();
        if (depth >= m_setting_->full_depth) { return false; }
        if (node->GetNumChildren() > 0) { return false; }

        this->ExpandNode(node);
        const OctreeKey::KeyType child_level = m_setting_->tree_depth - depth - 1;
        const OctreeKey::KeyType offset = (1 << child_level) >> 1;
        const NodeIndex parent_node_idx = node->GetNodeIndex();
        for (int i = 0; i < 8; ++i) {
            OctreeKey child_key;
            OctreeKey::ComputeChildKey(i, offset, node_key, child_key);
            auto child_node_index = AllocateVoxelEntry(child_key, child_level, parent_node_idx, i);
            this->GetNodeChild(node, i)->SetNodeIndex(child_node_index);
            RecordVertices(child_key, child_node_index, child_level);
        }
        return true;
    }

    template<typename Dtype, class Node, class Setting>
    void
    SemiSparseOctreeBase<Dtype, Node, Setting>::OnDeleteNodeChild(
        Node *node,
        Node *child,
        const OctreeKey &key) {
        Super::OnDeleteNodeChild(node, child, key);

        const NodeIndex node_index = node->GetNodeIndex();
        const NodeIndex child_node_index = child->GetNodeIndex();
        const int child_pos = child->GetChildIndex();
        m_children_[node_index][child_pos] = -1;
        m_recycled_node_indices_.emplace(child_node_index);

        m_parents_[child_node_index] = -1;
        m_children_[child_node_index] = {-1, -1, -1, -1, -1, -1, -1, -1};
        m_voxels_[child_node_index] = {0, 0, 0, 0};
        m_vertices_[child_node_index] = {-1, -1, -1, -1, -1, -1, -1, -1};
    }
}  // namespace erl::geometry
