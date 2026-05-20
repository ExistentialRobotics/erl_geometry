#pragma once

#include "find_voxel_indices.hpp"

#include "erl_common/serialization.hpp"

namespace erl::geometry {

    template<typename Dtype, class Node, class Setting>
    SemiSparseOctreeBase<Dtype, Node, Setting>::SemiSparseOctreeBase(
        const std::shared_ptr<Setting> &setting)
        : Super(setting), m_setting_(setting) {

        ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        ERL_ASSERTM(m_setting_->tree_depth > 0, "tree_depth should be > 0.");
        ERL_ASSERTM(m_setting_->init_voxel_num > 0, "init_voxel_num should be > 0.");

        m_parents_.resize(m_setting_->init_voxel_num);
        m_children_.resize(Eigen::NoChange, m_setting_->init_voxel_num);
        m_voxels_.resize(Eigen::NoChange, m_setting_->init_voxel_num);
        m_vertices_.resize(Eigen::NoChange, m_setting_->init_voxel_num);

        // Set the last column (size) to 1 as the default value because in the Python
        // implementation, index=-1 is used to indicate the absence of a node, which will
        // pick the last column of the voxel.
        // The size of a non-existent node should be >0 to avoid zero-division.
        m_voxels_.rightCols<1>()[3] = 1;

        if (m_setting_->cache_voxel_centers) {
            m_voxel_centers_.resize(Eigen::NoChange, m_setting_->init_voxel_num);
        }

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
    const typename SemiSparseOctreeBase<Dtype, Node, Setting>::BufferParents &
    SemiSparseOctreeBase<Dtype, Node, Setting>::GetParents() const {
        return m_parents_;
    }

    template<typename Dtype, class Node, class Setting>
    const typename SemiSparseOctreeBase<Dtype, Node, Setting>::BufferChildren &
    SemiSparseOctreeBase<Dtype, Node, Setting>::GetChildren() const {
        return m_children_;
    }

    template<typename Dtype, class Node, class Setting>
    const typename SemiSparseOctreeBase<Dtype, Node, Setting>::BufferVoxels &
    SemiSparseOctreeBase<Dtype, Node, Setting>::GetVoxels() const {
        return m_voxels_;
    }

    template<typename Dtype, class Node, class Setting>
    const typename SemiSparseOctreeBase<Dtype, Node, Setting>::Matrix3X &
    SemiSparseOctreeBase<Dtype, Node, Setting>::GetVoxelCenters() const {
        return m_voxel_centers_;
    }

    template<typename Dtype, class Node, class Setting>
    const typename SemiSparseOctreeBase<Dtype, Node, Setting>::BufferVertices &
    SemiSparseOctreeBase<Dtype, Node, Setting>::GetVertices() const {
        return m_vertices_;
    }

    template<typename Dtype, class Node, class Setting>
    std::size_t
    SemiSparseOctreeBase<Dtype, Node, Setting>::GetVertexCount() const {
        return m_vertex_keys_.size();
    }

    template<typename Dtype, class Node, class Setting>
    std::size_t
    SemiSparseOctreeBase<Dtype, Node, Setting>::GetIndependentLeafVertexCount() const {
        return m_key_to_vertex_map_leaf_.size();
    }

    template<typename Dtype, class Node, class Setting>
    const OctreeKeyVector &
    SemiSparseOctreeBase<Dtype, Node, Setting>::GetVertexKeys() const {
        return m_vertex_keys_;
    }

    template<typename Dtype, class Node, class Setting>
    typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex
    SemiSparseOctreeBase<Dtype, Node, Setting>::GetBufHead() const {
        return m_buf_head_;
    }

    template<typename Dtype, class Node, class Setting>
    typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndices
    SemiSparseOctreeBase<Dtype, Node, Setting>::InsertPoints(const Matrix3X &points) {
        return InsertPoints(points.data(), points.cols());
    }

    template<typename Dtype, class Node, class Setting>
    typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndices
    SemiSparseOctreeBase<Dtype, Node, Setting>::InsertPoints(
        const Dtype *points,
        const long num_points) {

        NodeIndices voxel_indices(num_points);

        auto p = points;
        for (long i = 0; i < num_points; ++i, p += 3) {
            if (OctreeKey key; !this->CoordToKeyChecked(p[0], p[1], p[2], key)) {
                ERL_WARN("Point ({}, {}, {}) is out of range.", p[0], p[1], p[2]);
                voxel_indices[i] = -1;
            } else {
                voxel_indices[i] = InsertKey(key, 0);
            }
        }

        return voxel_indices;
    }

    template<typename Dtype, class Node, class Setting>
    typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndices
    SemiSparseOctreeBase<Dtype, Node, Setting>::InsertKeys(
        const OctreeKey *keys,
        const long num_keys) {

        NodeIndices voxel_indices(num_keys);

        auto k = keys;
        for (long i = 0; i < num_keys; ++i) { voxel_indices[i] = InsertKey(*k++, 0); }

        return voxel_indices;
    }

    template<typename Dtype, class Node, class Setting>
    typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex
    SemiSparseOctreeBase<Dtype, Node, Setting>::InsertKey(
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
        uint64_t mask = 0b111ul << shift;
        while (child_level >= min_level) {
            if (const auto child_index = static_cast<int>((code & mask) >> shift);
                node->HasChild(child_index)) {
                node = this->GetNodeChild(node, child_index);
                node_index = m_children_(child_index, node_index);
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
    typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndices
    SemiSparseOctreeBase<Dtype, Node, Setting>::FindVoxelIndices(
        const Matrix3X &points,
        bool parallel) const {
        return FindVoxelIndices(points.data(), points.cols(), parallel);
    }

    template<typename Dtype, class Node, class Setting>
    typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndices
    SemiSparseOctreeBase<Dtype, Node, Setting>::FindVoxelIndices(
        const Dtype *points,
        const long num_points,
        bool parallel) const {

        NodeIndices voxel_indices(num_points);

#pragma omp parallel if (parallel) default(none) shared(num_points, points, voxel_indices)
        for (long i = 0; i < num_points; ++i) {
            long j = (i << 1) + i;  // i * 3
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
    typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndices
    SemiSparseOctreeBase<Dtype, Node, Setting>::FindVoxelIndices(
        const OctreeKey *keys,
        const long num_points,
        bool parallel) const {

        NodeIndices voxel_indices(num_points);

#pragma omp parallel if (parallel) default(none) shared(num_points, keys, voxel_indices)
        for (long i = 0; i < num_points; ++i) { voxel_indices[i] = FindVoxelIndex(keys[i]); }
        return voxel_indices;
    }

    template<typename Dtype, class Node, class Setting>
    typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex
    SemiSparseOctreeBase<Dtype, Node, Setting>::FindVoxelIndex(const OctreeKey &key) const {
        return geometry::FindVoxelIndex<NodeIndex, uint64_t, 3>(
            key.ToMortonCode(),
            m_setting_->tree_depth - 1,
            m_children_.data());
    }

    template<typename Dtype, class Node, class Setting>
    std::ostream &
    SemiSparseOctreeBase<Dtype, Node, Setting>::WriteData(std::ostream &s) const {
        using Self = SemiSparseOctreeBase;
        using namespace common::serialization;

        // OctreeKey is a packed struct of 3 KeyType values; assert layout before doing
        // contiguous bulk writes of vectors-of-keys.
        static_assert(
            sizeof(OctreeKey) == 3 * sizeof(OctreeKey::KeyType),
            "OctreeKey must be packed (3 * KeyType) for binary IO.");

        static const TokenWriteFunctionPairs<Self> token_function_pairs = {
            {
                // Tree topology + each node's m_node_index_, delegated to the parent.
                "topology",
                [](const Self *self, std::ostream &stream) {
                    self->Super::WriteData(stream);
                    return stream.good();
                },
            },
            {
                // buf_size (allocated column count) + buf_head (live prefix length) + the live
                // prefix of each side buffer. m_voxel_centers_ is included only when the
                // setting cached them; a leading flag records this.
                //
                // We save the *minimum* allocation needed to round-trip: buf_head columns of live
                // data plus one sentinel column. The reader's max(buf_size, buf_head + 1) bound
                // then lands on buf_head + 1 — a compact buffer with no trailing unused slots.
                "buffers",
                [](const Self *self, std::ostream &stream) {
                    const NodeIndex buf_head = self->m_buf_head_;
                    const NodeIndex buf_size = buf_head + 1;
                    stream.write(reinterpret_cast<const char *>(&buf_size), sizeof(NodeIndex));
                    stream.write(reinterpret_cast<const char *>(&buf_head), sizeof(NodeIndex));
                    if (buf_head > 0) {
                        // Eigen matrices are column-major: the first buf_head columns occupy a
                        // contiguous prefix of data(), regardless of total allocation.
                        stream.write(
                            reinterpret_cast<const char *>(self->m_parents_.data()),
                            sizeof(NodeIndex) * buf_head);
                        stream.write(
                            reinterpret_cast<const char *>(self->m_children_.data()),
                            sizeof(NodeIndex) * 8 * buf_head);
                        stream.write(
                            reinterpret_cast<const char *>(self->m_voxels_.data()),
                            sizeof(OctreeKey::KeyType) * 4 * buf_head);
                        stream.write(
                            reinterpret_cast<const char *>(self->m_vertices_.data()),
                            sizeof(NodeIndex) * 8 * buf_head);
                    }
                    const char cache_centers = self->m_setting_->cache_voxel_centers ? 1 : 0;
                    stream.write(&cache_centers, sizeof(char));
                    if (cache_centers && buf_head > 0) {
                        stream.write(
                            reinterpret_cast<const char *>(self->m_voxel_centers_.data()),
                            sizeof(Dtype) * 3 * buf_head);
                    }
                    return stream.good();
                },
            },
            {
                // m_vertex_keys_ as a packed blob.
                "vertex_keys",
                [](const Self *self, std::ostream &stream) {
                    const uint64_t n = self->m_vertex_keys_.size();
                    stream.write(reinterpret_cast<const char *>(&n), sizeof(uint64_t));
                    if (n > 0) {
                        stream.write(
                            reinterpret_cast<const char *>(self->m_vertex_keys_.data()),
                            sizeof(OctreeKey) * n);
                    }
                    return stream.good();
                },
            },
            {
                // m_key_to_vertex_map_ serialized as (count, [(key, NodeIndex)] * count).
                "vertex_map",
                [](const Self *self, std::ostream &stream) {
                    const uint64_t n = self->m_key_to_vertex_map_.size();
                    stream.write(reinterpret_cast<const char *>(&n), sizeof(uint64_t));
                    for (const auto &kv: self->m_key_to_vertex_map_) {
                        stream.write(reinterpret_cast<const char *>(&kv.first), sizeof(OctreeKey));
                        stream.write(reinterpret_cast<const char *>(&kv.second), sizeof(NodeIndex));
                    }
                    return stream.good();
                },
            },
            {
                // m_key_to_vertex_map_leaf_, same layout as vertex_map. Only populated when
                // independent_smallest_leaf_vertex is on; otherwise the count is 0.
                "vertex_map_leaf",
                [](const Self *self, std::ostream &stream) {
                    const uint64_t n = self->m_key_to_vertex_map_leaf_.size();
                    stream.write(reinterpret_cast<const char *>(&n), sizeof(uint64_t));
                    for (const auto &kv: self->m_key_to_vertex_map_leaf_) {
                        stream.write(reinterpret_cast<const char *>(&kv.first), sizeof(OctreeKey));
                        stream.write(reinterpret_cast<const char *>(&kv.second), sizeof(NodeIndex));
                    }
                    return stream.good();
                },
            },
            {
                "recycled",
                [](const Self *self, std::ostream &stream) {
                    const uint64_t n = self->m_recycled_node_indices_.size();
                    stream.write(reinterpret_cast<const char *>(&n), sizeof(uint64_t));
                    for (const NodeIndex idx: self->m_recycled_node_indices_) {
                        stream.write(reinterpret_cast<const char *>(&idx), sizeof(NodeIndex));
                    }
                    return stream.good();
                },
            },
        };
        WriteTokens(s, this, token_function_pairs);
        return s;
    }

    template<typename Dtype, class Node, class Setting>
    std::istream &
    SemiSparseOctreeBase<Dtype, Node, Setting>::ReadData(std::istream &s) {
        using Self = SemiSparseOctreeBase;
        using namespace common::serialization;

        static_assert(
            sizeof(OctreeKey) == 3 * sizeof(OctreeKey::KeyType),
            "OctreeKey must be packed (3 * KeyType) for binary IO.");

        // AbstractOctree::Read calls Super::Clear() before reaching this point, which resets
        // m_root_ / m_tree_size_ but leaves our side buffers in their constructor-seeded state.
        // Wipe them so the load is not contaminated.
        m_buf_head_ = 0;
        m_parents_.resize(0);
        m_children_.resize(Eigen::NoChange, 0);
        m_voxels_.resize(Eigen::NoChange, 0);
        m_voxel_centers_.resize(Eigen::NoChange, 0);
        m_vertices_.resize(Eigen::NoChange, 0);
        m_key_to_vertex_map_.clear();
        m_key_to_vertex_map_leaf_.clear();
        m_vertex_keys_.clear();
        m_recycled_node_indices_.clear();

        static const TokenReadFunctionPairs<Self> token_function_pairs = {
            {
                "topology",
                [](Self *self, std::istream &stream) {
                    self->Super::ReadData(stream);
                    return stream.good();
                },
            },
            {
                "buffers",
                [](Self *self, std::istream &stream) {
                    NodeIndex buf_size = 0;
                    stream.read(reinterpret_cast<char *>(&buf_size), sizeof(NodeIndex));
                    stream.read(reinterpret_cast<char *>(&self->m_buf_head_), sizeof(NodeIndex));
                    if (!stream.good()) { return false; }

                    // Match the saved allocation so a round-trip preserves the buffer state
                    // exactly. The lower bound m_buf_head_ + 1 reserves the last column for the
                    // m_voxels_ size-sentinel (set to 1) that other code relies on when an
                    // index of -1 is passed in.
                    const NodeIndex alloc = std::max<NodeIndex>(buf_size, self->m_buf_head_ + 1);
                    self->m_parents_.resize(alloc);
                    self->m_children_.resize(Eigen::NoChange, alloc);
                    self->m_voxels_.resize(Eigen::NoChange, alloc);
                    self->m_vertices_.resize(Eigen::NoChange, alloc);

                    if (self->m_buf_head_ > 0) {
                        stream.read(
                            reinterpret_cast<char *>(self->m_parents_.data()),
                            sizeof(NodeIndex) * self->m_buf_head_);
                        stream.read(
                            reinterpret_cast<char *>(self->m_children_.data()),
                            sizeof(NodeIndex) * 8 * self->m_buf_head_);
                        stream.read(
                            reinterpret_cast<char *>(self->m_voxels_.data()),
                            sizeof(OctreeKey::KeyType) * 4 * self->m_buf_head_);
                        stream.read(
                            reinterpret_cast<char *>(self->m_vertices_.data()),
                            sizeof(NodeIndex) * 8 * self->m_buf_head_);
                    }
                    self->m_voxels_.rightCols<1>()[3] = 1;

                    char saved_cache_centers = 0;
                    stream.read(&saved_cache_centers, sizeof(char));
                    if (saved_cache_centers && self->m_buf_head_ > 0) {
                        self->m_voxel_centers_.resize(Eigen::NoChange, alloc);
                        stream.read(
                            reinterpret_cast<char *>(self->m_voxel_centers_.data()),
                            sizeof(Dtype) * 3 * self->m_buf_head_);
                    } else if (self->m_setting_->cache_voxel_centers && self->m_buf_head_ > 0) {
                        // The file does not carry centers but the current setting wants them
                        // cached; rebuild from m_voxels_ using AllocateVoxelEntry's convention.
                        self->m_voxel_centers_.resize(Eigen::NoChange, alloc);
                        const auto r = static_cast<Dtype>(self->m_setting_->resolution);
                        const auto key_offset = static_cast<Dtype>(self->m_tree_key_offset_);
                        for (NodeIndex i = 0; i < self->m_buf_head_; ++i) {
                            const auto kx = static_cast<Dtype>(self->m_voxels_(0, i));
                            const auto ky = static_cast<Dtype>(self->m_voxels_(1, i));
                            const auto kz = static_cast<Dtype>(self->m_voxels_(2, i));
                            const auto size = self->m_voxels_(3, i);
                            if (size == 1) {  // level 0: cell-center offset of +0.5
                                // clang-format off
                                self->m_voxel_centers_.col(i) <<
                                    (kx - key_offset + 0.5f) * r,
                                    (ky - key_offset + 0.5f) * r,
                                    (kz - key_offset + 0.5f) * r;
                                // clang-format on
                            } else {
                                // clang-format off
                                self->m_voxel_centers_.col(i) <<
                                    (kx - key_offset) * r,
                                    (ky - key_offset) * r,
                                    (kz - key_offset) * r;
                                // clang-format on
                            }
                        }
                    }
                    return stream.good();
                },
            },
            {
                "vertex_keys",
                [](Self *self, std::istream &stream) {
                    uint64_t n = 0;
                    stream.read(reinterpret_cast<char *>(&n), sizeof(uint64_t));
                    self->m_vertex_keys_.resize(n);
                    if (n > 0) {
                        stream.read(
                            reinterpret_cast<char *>(self->m_vertex_keys_.data()),
                            sizeof(OctreeKey) * n);
                    }
                    return stream.good();
                },
            },
            {
                "vertex_map",
                [](Self *self, std::istream &stream) {
                    uint64_t n = 0;
                    stream.read(reinterpret_cast<char *>(&n), sizeof(uint64_t));
                    self->m_key_to_vertex_map_.reserve(n);
                    for (uint64_t i = 0; i < n; ++i) {
                        OctreeKey key;
                        NodeIndex idx = -1;
                        stream.read(reinterpret_cast<char *>(&key), sizeof(OctreeKey));
                        stream.read(reinterpret_cast<char *>(&idx), sizeof(NodeIndex));
                        self->m_key_to_vertex_map_.emplace(key, idx);
                    }
                    return stream.good();
                },
            },
            {
                "vertex_map_leaf",
                [](Self *self, std::istream &stream) {
                    uint64_t n = 0;
                    stream.read(reinterpret_cast<char *>(&n), sizeof(uint64_t));
                    self->m_key_to_vertex_map_leaf_.reserve(n);
                    for (uint64_t i = 0; i < n; ++i) {
                        OctreeKey key;
                        NodeIndex idx = -1;
                        stream.read(reinterpret_cast<char *>(&key), sizeof(OctreeKey));
                        stream.read(reinterpret_cast<char *>(&idx), sizeof(NodeIndex));
                        self->m_key_to_vertex_map_leaf_.emplace(key, idx);
                    }
                    return stream.good();
                },
            },
            {
                "recycled",
                [](Self *self, std::istream &stream) {
                    uint64_t n = 0;
                    stream.read(reinterpret_cast<char *>(&n), sizeof(uint64_t));
                    self->m_recycled_node_indices_.reserve(n);
                    for (uint64_t i = 0; i < n; ++i) {
                        NodeIndex idx = -1;
                        stream.read(reinterpret_cast<char *>(&idx), sizeof(NodeIndex));
                        self->m_recycled_node_indices_.emplace(idx);
                    }
                    return stream.good();
                },
            },
        };
        ReadTokens(s, this, token_function_pairs);
        return s;
    }

    template<typename Dtype, class Node, class Setting>
    typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex
    SemiSparseOctreeBase<Dtype, Node, Setting>::AllocateVoxelEntry(
        const OctreeKey &key,
        const OctreeKey::KeyType level,
        const NodeIndex parent_node_index,
        const NodeIndex child_index) {

        if (m_recycled_node_indices_.empty()) {
            const NodeIndex node_index = m_buf_head_;
            if (m_buf_head_ >= m_parents_.size()) {  // need to expand the buffers
                const long new_size = 2 * m_buf_head_ + 1;
                m_parents_.conservativeResize(new_size);
                m_children_.conservativeResize(Eigen::NoChange, new_size);
                m_voxels_.conservativeResize(Eigen::NoChange, new_size);
                m_vertices_.conservativeResize(Eigen::NoChange, new_size);
                if (m_setting_->cache_voxel_centers) {
                    m_voxel_centers_.conservativeResize(Eigen::NoChange, new_size);
                }
                m_voxels_.rightCols<1>()[3] = 1;
            }
            m_parents_[m_buf_head_] = parent_node_index;
            m_children_.col(m_buf_head_).setConstant(-1);
            m_voxels_.col(m_buf_head_) << key[0], key[1], key[2], (1ul << level);
            m_vertices_.col(m_buf_head_).setConstant(-1);
            if (m_setting_->cache_voxel_centers) {
                const auto r = static_cast<Dtype>(m_setting_->resolution);
                const auto key_offset = this->m_tree_key_offset_;
                if (level == 0) {
                    m_voxel_centers_.col(m_buf_head_)
                        << (static_cast<Dtype>(key[0]) - static_cast<Dtype>(key_offset) + 0.5f) * r,
                        (static_cast<Dtype>(key[1]) - static_cast<Dtype>(key_offset) + 0.5f) * r,
                        (static_cast<Dtype>(key[2]) - static_cast<Dtype>(key_offset) + 0.5f) * r;
                } else {
                    m_voxel_centers_.col(m_buf_head_)
                        << (static_cast<Dtype>(key[0]) - static_cast<Dtype>(key_offset)) * r,
                        (static_cast<Dtype>(key[1]) - static_cast<Dtype>(key_offset)) * r,
                        (static_cast<Dtype>(key[2]) - static_cast<Dtype>(key_offset)) * r;
                }
            }
            ++m_buf_head_;

            if (parent_node_index >= 0) {
                m_children_(child_index, parent_node_index) = node_index;
            }
            return node_index;
        }
        const auto it = m_recycled_node_indices_.begin();
        const NodeIndex node_index = *it;
        m_recycled_node_indices_.erase(it);
        m_parents_[node_index] = parent_node_index;
        m_voxels_.col(node_index) << key[0], key[1], key[2], (1ul << level);
        if (m_setting_->cache_voxel_centers) {
            const auto r = static_cast<Dtype>(m_setting_->resolution);
            const auto key_offset = this->m_tree_key_offset_;
            if (level == 0) {
                m_voxel_centers_.col(node_index)
                    << (static_cast<Dtype>(key[0]) - static_cast<Dtype>(key_offset) + 0.5f) * r,
                    (static_cast<Dtype>(key[1]) - static_cast<Dtype>(key_offset) + 0.5f) * r,
                    (static_cast<Dtype>(key[2]) - static_cast<Dtype>(key_offset) + 0.5f) * r;
            } else {
                m_voxel_centers_.col(node_index)
                    << (static_cast<Dtype>(key[0]) - static_cast<Dtype>(key_offset)) * r,
                    (static_cast<Dtype>(key[1]) - static_cast<Dtype>(key_offset)) * r,
                    (static_cast<Dtype>(key[2]) - static_cast<Dtype>(key_offset)) * r;
            }
        }

        // already reset in OnDeleteNodeChild
        // m_children_[node_index] = {-1, -1, -1, -1, -1, -1, -1, -1};

        // will be set in RecordVertices
        // m_vertices_[node_index] = {-1, -1, -1, -1, -1, -1, -1, -1};

        if (parent_node_index >= 0) { m_children_(child_index, parent_node_index) = node_index; }

        return node_index;
    }

    template<typename Dtype, class Node, class Setting>
    std::pair<Node *, typename SemiSparseOctreeBase<Dtype, Node, Setting>::NodeIndex>
    SemiSparseOctreeBase<Dtype, Node, Setting>::CreateNode(
        OctreeKey node_key,
        uint32_t level,
        Node *parent,
        int child_index) {

        NodeIndex node_index = 0;
        Node *node = nullptr;

        const uint32_t depth = m_setting_->tree_depth - level;
        node_key = this->AdjustKeyToDepth(node_key, depth);

        if (parent != nullptr) {
            // We are going to create the child node. But we need to make sure its siblings are
            // also created if the depth is still smaller than full_depth.
            const NodeIndex parent_node_idx = parent->GetNodeIndex();
            if (BuildFullTree(this->AdjustKeyToDepth(node_key, depth - 1), parent)) {
                node = this->GetNodeChild(parent, child_index);
                node_index = m_children_(child_index, parent_node_idx);
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
        const uint32_t level) {

        if (m_setting_->independent_smallest_leaf_vertex && level == 0) {
            // we need to create 8 independent vertices for this leaf node.
            // it will not share vertices with parent, but with siblings.
            OctreeKey vertex_key;
            for (int i = 0; i < 8; ++i) {
                OctreeKey::ComputeVertexKey(i, level, node_key, vertex_key);
                auto [it, inserted] =
                    m_key_to_vertex_map_leaf_.try_emplace(vertex_key, m_vertex_keys_.size());
                if (inserted) { m_vertex_keys_.push_back(vertex_key); }
                m_vertices_(i, node_idx) = it->second;
            }
            return;
        }

        OctreeKey vertex_key;
        for (int i = 0; i < 8; ++i) {
            OctreeKey::ComputeVertexKey(i, level, node_key, vertex_key);

            auto [it, inserted] =
                m_key_to_vertex_map_.try_emplace(vertex_key, m_vertex_keys_.size());
            if (inserted) { m_vertex_keys_.push_back(vertex_key); }
            m_vertices_(i, node_idx) = it->second;
        }
    }

    template<typename Dtype, class Node, class Setting>
    bool
    SemiSparseOctreeBase<Dtype, Node, Setting>::BuildFullTree(
        const OctreeKey &node_key,
        Node *node) {

        const uint32_t depth = node->GetDepth();
        if (depth >= m_setting_->semi_sparse_depth) { return false; }
        if (node->GetNumChildren() > 0) { return false; }

        this->ExpandNode(node);
        const OctreeKey::KeyType child_level = m_setting_->tree_depth - depth - 1;
        const OctreeKey::KeyType offset = (1ul << child_level) >> 1;
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
        m_children_(child_pos, node_index) = -1;
        m_recycled_node_indices_.emplace(child_node_index);

        m_parents_[child_node_index] = -1;
        m_children_.col(child_node_index).setConstant(-1);
        m_voxels_.col(child_node_index).setZero();
        m_vertices_.col(child_node_index).setConstant(-1);
    }
}  // namespace erl::geometry
