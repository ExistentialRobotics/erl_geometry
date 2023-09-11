#pragma once

#include "erl_common/eigen.hpp"
#include <nanoflann.hpp>
#include <memory>

namespace erl::geometry {

    template<typename T, int Dim, typename Metric = nanoflann::metric_L2_Simple, typename IndexType = int>
    class KdTreeEigenAdaptor {

        using EigenMatrix = Eigen::Matrix<T, Dim, Eigen::Dynamic>;
        using Self = KdTreeEigenAdaptor<T, Dim, Metric, IndexType>;
        using NumType = typename EigenMatrix::Scalar;
        using MetricType = typename Metric::template traits<NumType, Self>::distance_t;
        using TreeType = nanoflann::KDTreeSingleIndexAdaptor<MetricType, Self, Dim, IndexType>;

        std::shared_ptr<TreeType> m_tree_ = nullptr;
        EigenMatrix m_data_matrix_;
        const int mk_MLeafMaxSize_;

    public:
        explicit KdTreeEigenAdaptor(EigenMatrix mat, bool build = true, int leaf_max_size = 10)
            : m_data_matrix_(std::move(mat)),
              mk_MLeafMaxSize_(leaf_max_size) {

            if (build) { Rebuild(); }
        }

        [[nodiscard]] const EigenMatrix &
        GetDataMatrix() const {
            return m_data_matrix_;
        }

        // Rebuild the KD tree from scratch
        void
        Rebuild() {
            m_tree_ = std::make_shared<TreeType>(Dim, *this, nanoflann::KDTreeSingleIndexAdaptorParams(mk_MLeafMaxSize_));
            m_tree_->buildIndex();
        }

        inline void
        Knn(size_t k, const Eigen::Vector<T, Dim> &point, IndexType &indices_out, NumType &metric_out) {
            nanoflann::KNNResultSet<NumType, IndexType> result_set(k);
            result_set.init(&indices_out, &metric_out);
            m_tree_->findNeighbors(result_set, point.data(), nanoflann::SearchParameters());
        }

        // Returns the number of points: used by TreeType
        [[nodiscard]] inline size_t
        kdtree_get_point_count() const {
            return m_data_matrix_.cols();
        }

        // Returns the dim-th component of the idx-th point in the class, used by TreeType
        [[nodiscard]] inline NumType
        kdtree_get_pt(const size_t idx, int dim) const {
            return m_data_matrix_.coeff(dim, idx);
        }

        // Optional bounding-box computation: return false to default to a standard bbox computation loop.
        template<class BBOX>
        bool
        kdtree_get_bbox(BBOX &) const {
            return false;
        }
    };
}  // namespace erl::geometry
