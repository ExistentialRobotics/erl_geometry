#pragma once

#include "occupancy_nd_tree_batch_ray_caster.hpp"

#include <absl/container/flat_hash_map.h>

namespace erl::geometry {
    template<typename Tree, int Dim>
    OccupancyNdTreeBatchRayCaster<Tree, Dim>::OccupancyNdTreeBatchRayCaster(
        const Tree *tree,
        MatrixDX origins,
        MatrixDX directions,
        const VectorX &max_ranges,
        const VectorX &node_paddings,
        const Eigen::VectorXb &bidirectional_flags,
        const Eigen::VectorXb &leaf_only_flags,
        const Eigen::VectorXi &min_node_depths,
        const Eigen::VectorXi &max_node_depths)
        : m_tree_(tree),
          m_origins_(std::move(origins)),
          m_directions_(std::move(directions)) {

        ERL_ASSERTM(
            m_origins_.cols() == m_directions_.cols(),
            "Origins and directions must have the same number of rays.");
        ERL_ASSERTM(m_origins_.cols() > 0, "At least one ray must be provided.");

        m_node_iterators_.resize(m_origins_.cols());
        m_hit_flags_.resize(m_origins_.cols());
        m_ever_hit_flags_.resize(m_origins_.cols());
        m_hit_distances_.resize(m_origins_.cols());
        m_hit_nodes_.resize(m_origins_.cols());
        m_hit_positions_.resize(m_origins_.cols());
#pragma omp parallel for default(none) \
    shared(max_ranges,                 \
               node_paddings,          \
               bidirectional_flags,    \
               leaf_only_flags,        \
               min_node_depths,        \
               max_node_depths)
        for (int i = 0; i < m_origins_.cols(); ++i) {
            m_node_iterators_[i] = m_tree_->BeginNodeOnRay(
                m_origins_.col(i),
                m_directions_.col(i),
                max_ranges.size() > 0 ? max_ranges[i] : -1,
                node_paddings.size() > 0 ? node_paddings[i] : 0,
                bidirectional_flags.size() > 0 ? bidirectional_flags[i] : false,
                leaf_only_flags.size() > 0 ? leaf_only_flags[i] : false,
                min_node_depths.size() > 0 ? min_node_depths[i] : 0,
                max_node_depths.size() > 0 ? max_node_depths[i] : 0);
            NodeOnRayIterator &itr = m_node_iterators_[i];

            m_hit_flags_[i] = false;
            while (itr.IsValid()) {
                if (m_tree_->IsNodeOccupied(*itr)) {
                    m_hit_flags_[i] = true;
                    break;
                }
                ++itr;
            }
            if (m_hit_flags_[i]) {
                m_hit_distances_[i] = itr.GetDistance();
                m_hit_nodes_[i] = *itr;
                m_hit_positions_[i] =
                    m_origins_.col(i) + m_directions_.col(i) * m_hit_distances_[i];
            }
            m_ever_hit_flags_[i] |= m_hit_flags_[i];
        }
        UpdateFrontier();
    }

    template<typename Tree, int Dim>
    long
    OccupancyNdTreeBatchRayCaster<Tree, Dim>::GetNumRays() const {
        return m_origins_.cols();
    }

    template<typename Tree, int Dim>
    const typename OccupancyNdTreeBatchRayCaster<Tree, Dim>::MatrixDX &
    OccupancyNdTreeBatchRayCaster<Tree, Dim>::GetRayOrigins() const {
        return m_origins_;
    }

    template<typename Tree, int Dim>
    const typename OccupancyNdTreeBatchRayCaster<Tree, Dim>::MatrixDX &
    OccupancyNdTreeBatchRayCaster<Tree, Dim>::GetRayDirections() const {
        return m_directions_;
    }

    template<typename Tree, int Dim>
    const Eigen::VectorXb &
    OccupancyNdTreeBatchRayCaster<Tree, Dim>::GetHitFlags() const {
        return m_hit_flags_;
    }

    template<typename Tree, int Dim>
    const Eigen::VectorXb &
    OccupancyNdTreeBatchRayCaster<Tree, Dim>::GetEverHitFlags() const {
        return m_ever_hit_flags_;
    }

    template<typename Tree, int Dim>
    const typename OccupancyNdTreeBatchRayCaster<Tree, Dim>::VectorX &
    OccupancyNdTreeBatchRayCaster<Tree, Dim>::GetHitDistances() const {
        return m_hit_distances_;
    }

    template<typename Tree, int Dim>
    const std::vector<typename Tree::NodeType *> &
    OccupancyNdTreeBatchRayCaster<Tree, Dim>::GetHitNodes() const {
        return m_hit_nodes_;
    }

    template<typename Tree, int Dim>
    const std::vector<typename OccupancyNdTreeBatchRayCaster<Tree, Dim>::VectorD> &
    OccupancyNdTreeBatchRayCaster<Tree, Dim>::GetHitPositions() const {
        return m_hit_positions_;
    }

    template<typename Tree, int Dim>
    OccupancyNdTreeBatchRayCaster<Tree, Dim> &
    OccupancyNdTreeBatchRayCaster<Tree, Dim>::Step(const Eigen::VectorXb &mask) {
        if (mask.size() == 0) {
            ++(*this);
            return *this;
        }

        ERL_ASSERTM(mask.size() == m_origins_.cols(), "Mask size must match the number of rays.");
#pragma omp parallel for default(none) shared(mask)
        for (int i = 0; i < m_origins_.cols(); ++i) {
            m_hit_flags_[i] = false;
            if (!mask[i]) { continue; }  // Skip rays that are not in the mask.

            NodeOnRayIterator &itr = m_node_iterators_[i];
            ++itr;

            while (itr.IsValid()) {
                if (m_tree_->IsNodeOccupied(*itr)) {
                    m_hit_flags_[i] = true;
                    break;
                }
                ++itr;
            }
            if (m_hit_flags_[i]) {
                m_hit_distances_[i] = itr.GetDistance();
                m_hit_nodes_[i] = *itr;
                m_hit_positions_[i] =
                    m_origins_.col(i) + m_directions_.col(i) * m_hit_distances_[i];
            }
            m_ever_hit_flags_[i] |= m_hit_flags_[i];
        }
        UpdateFrontier();
        return *this;
    }

    template<typename Tree, int Dim>
    OccupancyNdTreeBatchRayCaster<Tree, Dim> &
    OccupancyNdTreeBatchRayCaster<Tree, Dim>::operator++() {
#pragma omp parallel for default(none)
        for (int i = 0; i < m_origins_.cols(); ++i) {
            NodeOnRayIterator &itr = m_node_iterators_[i];
            ++itr;

            m_hit_flags_[i] = false;
            while (itr.IsValid()) {
                if (m_tree_->IsNodeOccupied(*itr)) {
                    m_hit_flags_[i] = true;
                    break;
                }
                ++itr;
            }
            if (m_hit_flags_[i]) {
                m_hit_distances_[i] = itr.GetDistance();
                m_hit_nodes_[i] = *itr;
                m_hit_positions_[i] =
                    m_origins_.col(i) + m_directions_.col(i) * m_hit_distances_[i];
            }
            m_ever_hit_flags_[i] |= m_hit_flags_[i];
        }
        UpdateFrontier();
        return *this;
    }

    template<typename Tree, int Dim>
    const std::vector<typename Tree::NodeType *> &
    OccupancyNdTreeBatchRayCaster<Tree, Dim>::GetFrontierNodes() const {
        return m_frontier_nodes_;
    }

    template<typename Tree, int Dim>
    const std::vector<typename Tree::KeyType> &
    OccupancyNdTreeBatchRayCaster<Tree, Dim>::GetFrontierKeys() const {
        return m_frontier_keys_;
    }

    template<typename Tree, int Dim>
    const std::vector<std::vector<long>> &
    OccupancyNdTreeBatchRayCaster<Tree, Dim>::GetFrontierRayIndices() const {
        return m_frontier_ray_indices_;
    }

    template<typename Tree, int Dim>
    void
    OccupancyNdTreeBatchRayCaster<Tree, Dim>::UpdateFrontier() {
        m_frontier_nodes_.clear();
        m_frontier_keys_.clear();
        m_frontier_ray_indices_.clear();
        absl::flat_hash_map<uint64_t, std::size_t> indices;
        for (long i = 0; i < m_origins_.cols(); ++i) {
            if (!m_hit_flags_[i]) { continue; }
            const auto &key = m_node_iterators_[i].GetIndexKey();
            const auto &node = m_hit_nodes_[i];
            if (indices.try_emplace(reinterpret_cast<uint64_t>(node), m_frontier_keys_.size())
                    .second) {
                m_frontier_nodes_.push_back(node);
                m_frontier_keys_.push_back(key);
                m_frontier_ray_indices_.emplace_back();
            }
            const std::size_t index = indices[reinterpret_cast<uint64_t>(node)];
            m_frontier_ray_indices_[index].push_back(i);
        }
    }
}  // namespace erl::geometry
