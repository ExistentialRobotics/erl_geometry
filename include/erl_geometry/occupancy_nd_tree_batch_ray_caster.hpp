#pragma once

#include "erl_common/eigen.hpp"

#include <vector>

namespace erl::geometry {

    template<typename Tree, int Dim>
    class OccupancyNdTreeBatchRayCaster {
    public:
        using Dtype = typename Tree::DataType;
        using VectorD = Eigen::Vector<Dtype, Dim>;
        using VectorX = Eigen::VectorX<Dtype>;
        using MatrixDX = Eigen::Matrix<Dtype, Dim, Eigen::Dynamic>;
        using NodeOnRayIterator = typename Tree::NodeOnRayIterator;

    private:
        const Tree *m_tree_ = nullptr;
        MatrixDX m_origins_{};
        MatrixDX m_directions_{};
        std::vector<NodeOnRayIterator> m_node_iterators_{};

        Eigen::VectorXb m_hit_flags_{};
        Eigen::VectorXb m_ever_hit_flags_{};

        VectorX m_hit_distances_{};
        std::vector<typename Tree::NodeType *> m_hit_nodes_{};
        std::vector<VectorD> m_hit_positions_{};

        std::vector<typename Tree::NodeType *> m_frontier_nodes_;
        std::vector<typename Tree::KeyType> m_frontier_keys_;
        std::vector<std::vector<long>> m_frontier_ray_indices_;

    public:
        OccupancyNdTreeBatchRayCaster(
            const Tree *tree,
            MatrixDX origins,
            MatrixDX directions,
            const VectorX &max_ranges,
            const VectorX &node_paddings,
            const Eigen::VectorXb &bidirectional_flags,
            const Eigen::VectorXb &leaf_only_flags,
            const Eigen::VectorXi &min_node_depths,
            const Eigen::VectorXi &max_node_depths);

        [[nodiscard]] long
        GetNumRays() const;

        [[nodiscard]] const MatrixDX &
        GetRayOrigins() const;

        [[nodiscard]] const MatrixDX &
        GetRayDirections() const;

        [[nodiscard]] const Eigen::VectorXb &
        GetHitFlags() const;

        [[nodiscard]] const Eigen::VectorXb &
        GetEverHitFlags() const;

        [[nodiscard]] const VectorX &
        GetHitDistances() const;

        [[nodiscard]] const std::vector<typename Tree::NodeType *> &
        GetHitNodes() const;

        [[nodiscard]] const std::vector<VectorD> &
        GetHitPositions() const;

        OccupancyNdTreeBatchRayCaster
        operator++(int) = delete;  // Disable post-increment, i.e. caster++.

        OccupancyNdTreeBatchRayCaster &
        Step(const Eigen::VectorXb &mask);

        OccupancyNdTreeBatchRayCaster &
        operator++();

        [[nodiscard]] const std::vector<typename Tree::NodeType *> &
        GetFrontierNodes() const;

        [[nodiscard]] const std::vector<typename Tree::KeyType> &
        GetFrontierKeys() const;

        [[nodiscard]] const std::vector<std::vector<long>> &
        GetFrontierRayIndices() const;

    private:
        void
        UpdateFrontier();
    };

}  // namespace erl::geometry

#include "occupancy_nd_tree_batch_ray_caster.tpp"
