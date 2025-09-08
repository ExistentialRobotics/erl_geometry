#pragma once

#include "octree_impl.hpp"
#include "semi_sparse_octree_node.hpp"

namespace erl::geometry {

    /**
     * Base class for semi-sparse octree, where all child nodes of an inner node at shallow depths
     * are always allocated but nodes at deeper depths are allocated only when needed. Also, a
     * continuous storage is used to keep track of all nodes for fast access.
     *
     * @tparam Dtype data precision, double or float
     * @tparam Node type of the octree node
     * @tparam Setting type of the octree setting
     */
    template<typename Dtype, class Node, class Setting>
    class SemiSparseOctreeBase : public OctreeImpl<Node, AbstractOctree<Dtype>, Setting> {
        static_assert(std::is_base_of_v<SemiSparseOctreeNode, Node>);
        static_assert(std::is_base_of_v<SemiSparseNdTreeSetting, Setting>);

    protected:
        using Super = OctreeImpl<Node, AbstractOctree<Dtype>, Setting>;
        using NodeIndex = int64_t;

        std::shared_ptr<Setting> m_setting_ = nullptr;

        std::vector<NodeIndex> m_parents_ = {};                  // node index -> parent node index
        std::vector<std::array<NodeIndex, 8>> m_children_ = {};  // node index -> child indices
        std::vector<std::array<uint16_t, 4>> m_voxels_ = {};     // buffer for voxels (x,y,z,level)

        std::vector<std::array<NodeIndex, 8>> m_vertices_ = {};  // node index -> vertex indices
        OctreeKeyLongMap m_key_to_vertex_map_ = {};              // map from key to vertex index
        OctreeKeyVector m_vertex_keys_ = {};

        absl::flat_hash_set<NodeIndex> m_recycled_node_indices_ = {};  // indices of recycled nodes

    public:
        SemiSparseOctreeBase() = delete;  // no default constructor

        explicit SemiSparseOctreeBase(const std::shared_ptr<Setting> &setting);

        SemiSparseOctreeBase(const SemiSparseOctreeBase &other) = default;
        SemiSparseOctreeBase &
        operator=(const SemiSparseOctreeBase &other) = default;
        SemiSparseOctreeBase(SemiSparseOctreeBase &&other) noexcept = default;
        SemiSparseOctreeBase &
        operator=(SemiSparseOctreeBase &&other) noexcept = default;

        [[nodiscard]] std::shared_ptr<AbstractOctree<Dtype>>
        Clone() const override;

        [[nodiscard]] const std::vector<NodeIndex> &
        GetParents() const;

        [[nodiscard]] const std::vector<std::array<NodeIndex, 8>> &
        GetChildren() const;

        [[nodiscard]] const std::vector<std::array<uint16_t, 4>> &
        GetVoxels() const;

        [[nodiscard]] const std::vector<std::array<NodeIndex, 8>> &
        GetVertices() const;

        [[nodiscard]] std::size_t
        GetVertexCount() const;

        [[nodiscard]] const OctreeKeyVector &
        GetVertexKeys() const;

        /**
         * Insert multiple points into the octree and return the indices of the voxels containing
         * the points. The octree will be expanded if necessary.
         * @param points
         * @param num_points
         */
        std::vector<NodeIndex>
        InsertPoints(const Dtype *points, std::size_t num_points);

        std::vector<NodeIndex>
        InsertPoints(const OctreeKey *keys, std::size_t num_points);

        NodeIndex
        InsertPoint(const OctreeKey &key, uint32_t max_depth);

        [[nodiscard]] std::vector<NodeIndex>
        FindVoxelIndices(const Dtype *points, std::size_t num_points, bool parallel) const;

        [[nodiscard]] std::vector<NodeIndex>
        FindVoxelIndices(const OctreeKey *keys, std::size_t num_points, bool parallel) const;

        [[nodiscard]] NodeIndex
        FindVoxelIndex(const OctreeKey &key) const;

    private:
        NodeIndex
        AllocateVoxelEntry(
            const OctreeKey &key,
            OctreeKey::KeyType level,
            NodeIndex parent_node_index = -1,
            NodeIndex child_index = -1);

        /**
         *
         * @param node_key key of the node to create
         * @param level level of the node to create
         * @param parent parent of the node to create, if nullptr, create the root node
         * @param child_index child index of the node to create in the parent node
         * @return pointer and index of the created node
         */
        std::pair<Node *, NodeIndex>
        CreateNode(
            OctreeKey node_key,
            uint32_t level,
            Node *parent = nullptr,
            int child_index = -1);

        void
        RecordVertices(const OctreeKey &node_key, NodeIndex node_idx, uint32_t level);

        /**
         * If the node depth is less than full_depth:
         * 1. expand all its children;
         * 2. update the buffers.
         * @param node_key
         * @param node
         * @return true if the tree is expanded, false otherwise.
         */
        bool
        BuildFullTree(const OctreeKey &node_key, Node *node);

        void
        OnDeleteNodeChild(Node *node, Node *child, const OctreeKey &key) override;
    };
}  // namespace erl::geometry

#include "semi_sparse_octree_base.tpp"
