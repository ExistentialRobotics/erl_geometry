#pragma once

#include "quadtree_impl.hpp"
#include "semi_sparse_quadtree_node.hpp"

namespace erl::geometry {

    /**
     * Base class for semi-sparse quadtree, where all child nodes of an inner node at shallow depths
     * are always allocated but nodes at deeper depths are allocated only when needed. Also, a
     * continuous storage is used to keep track of all nodes for fast access.
     *
     * @tparam Dtype data precision, double or float
     * @tparam Node type of the quadtree node
     * @tparam Setting type of the quadtree setting
     */
    template<typename Dtype, class Node, class Setting>
    class SemiSparseQuadtreeBase : public QuadtreeImpl<Node, AbstractQuadtree<Dtype>, Setting> {
        static_assert(std::is_base_of_v<SemiSparseQuadtreeNode, Node>);
        static_assert(std::is_base_of_v<SemiSparseNdTreeSetting, Setting>);

    protected:
        using Super = QuadtreeImpl<Node, AbstractQuadtree<Dtype>, Setting>;
        using NodeIndex = int64_t;

        std::shared_ptr<Setting> m_setting_ = nullptr;

        std::vector<NodeIndex> m_parents_ = {};                  // node index -> parent node index
        std::vector<std::array<NodeIndex, 4>> m_children_ = {};  // node index -> child indices
        std::vector<std::array<uint16_t, 3>> m_voxels_ = {};     // buffer for voxels (x,y,level)

        std::vector<std::array<NodeIndex, 4>> m_vertices_ = {};  // node index -> vertex indices
        QuadtreeKeyLongMap m_key_to_vertex_map_ = {};            // map from key to vertex index
        QuadtreeKeyVector m_vertex_keys_ = {};

        absl::flat_hash_set<NodeIndex> m_recycled_node_indices_ = {};  // indices of recycled nodes

    public:
        SemiSparseQuadtreeBase() = delete;  // no default constructor

        explicit SemiSparseQuadtreeBase(const std::shared_ptr<Setting> &setting);

        SemiSparseQuadtreeBase(const SemiSparseQuadtreeBase &other) = default;
        SemiSparseQuadtreeBase &
        operator=(const SemiSparseQuadtreeBase &other) = default;
        SemiSparseQuadtreeBase(SemiSparseQuadtreeBase &&other) noexcept = default;
        SemiSparseQuadtreeBase &
        operator=(SemiSparseQuadtreeBase &&other) noexcept = default;

        [[nodiscard]] std::shared_ptr<AbstractQuadtree<Dtype>>
        Clone() const override;

        [[nodiscard]] const std::vector<NodeIndex> &
        GetParents() const;

        [[nodiscard]] const std::vector<std::array<NodeIndex, 4>> &
        GetChildren() const;

        [[nodiscard]] const std::vector<std::array<uint16_t, 3>> &
        GetVoxels() const;

        [[nodiscard]] const std::vector<std::array<NodeIndex, 4>> &
        GetVertices() const;

        [[nodiscard]] std::size_t
        GetVertexCount() const;

        [[nodiscard]] const QuadtreeKeyVector &
        GetVertexKeys() const;

        /**
         * Insert multiple points into the quadtree and return the indices of the voxels containing
         * the points. The quadtree will be expanded if necessary.
         * @param points
         * @param num_points
         */
        std::vector<NodeIndex>
        InsertPoints(const Dtype *points, std::size_t num_points);

        std::vector<NodeIndex>
        InsertPoints(const QuadtreeKey *keys, std::size_t num_points);

        NodeIndex
        InsertPoint(const QuadtreeKey &key, uint32_t max_depth);

        [[nodiscard]] std::vector<NodeIndex>
        FindVoxelIndices(const Dtype *points, std::size_t num_points, bool parallel) const;

        [[nodiscard]] std::vector<NodeIndex>
        FindVoxelIndices(const QuadtreeKey *keys, std::size_t num_points, bool parallel) const;

        [[nodiscard]] NodeIndex
        FindVoxelIndex(const QuadtreeKey &key) const;

    private:
        NodeIndex
        AllocateVoxelEntry(
            const QuadtreeKey &key,
            QuadtreeKey::KeyType level,
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
            QuadtreeKey node_key,
            uint32_t level,
            Node *parent = nullptr,
            int child_index = -1);

        void
        RecordVertices(const QuadtreeKey &node_key, NodeIndex node_idx, uint32_t level);

        /**
         * If the node depth is less than full_depth:
         * 1. expand all its children;
         * 2. update the buffers.
         * @param node_key
         * @param node
         * @return true if the tree is expanded, false otherwise.
         */
        bool
        BuildFullTree(const QuadtreeKey &node_key, Node *node);

        void
        OnDeleteNodeChild(Node *node, Node *child, const QuadtreeKey &key) override;
    };
}  // namespace erl::geometry

#include "semi_sparse_quadtree_base.tpp"
