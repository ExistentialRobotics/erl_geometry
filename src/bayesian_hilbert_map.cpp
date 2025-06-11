#include "erl_geometry/bayesian_hilbert_map.hpp"

#include "erl_common/block_timer.hpp"
#include "erl_common/serialization.hpp"
#include "erl_geometry/intersection.hpp"

#include <unordered_set>

namespace erl::geometry {

    template<typename Dtype, int Dim>
    BayesianHilbertMap<Dtype, Dim>::BayesianHilbertMap(
        std::shared_ptr<BayesianHilbertMapSetting> setting,
        std::shared_ptr<Covariance> kernel,
        MatrixDX hinged_points,
        Aabb<Dtype, Dim> map_boundary,
        const uint64_t seed)
        : m_setting_(std::move(setting)),
          m_kernel_(std::move(kernel)),
          m_hinged_points_(std::move(hinged_points)),
          m_map_boundary_(std::move(map_boundary)),
          m_generator_(seed) {

        ERL_DEBUG_ASSERT(m_setting_ != nullptr, "Setting is null.");
        ERL_DEBUG_ASSERT(m_kernel_ != nullptr, "Kernel is null.");

        const long m = m_hinged_points_.cols();
        const Dtype sigma = m_setting_->init_sigma;
        const Dtype sigma_inv = 1.0f / sigma;
        if (m_setting_->diagonal_sigma) {
            m_sigma_ = VectorX::Zero(m);
            m_sigma_inv_ = VectorX::Zero(m);
            for (long i = 0; i < m; ++i) {
                m_sigma_(i, 0) = sigma;          // initialize the diagonal of the covariance matrix
                m_sigma_inv_(i, 0) = sigma_inv;  // initialize the inverse covariance matrix
            }
        } else {
            m_sigma_ = MatrixX::Zero(m, m);
            m_sigma_inv_ = MatrixX::Zero(m, m);
            for (long i = 0; i < m; ++i) {
                m_sigma_(i, i) = sigma;          // initialize the diagonal of the covariance matrix
                m_sigma_inv_(i, i) = sigma_inv;  // initialize the inverse covariance matrix
            }
        }
        m_mu_ = VectorX::Constant(m, m_setting_->init_mu);
        m_alpha_ = VectorX::Zero(m);
    }

    template<typename Dtype, int Dim>
    std::shared_ptr<const BayesianHilbertMapSetting>
    BayesianHilbertMap<Dtype, Dim>::GetSetting() const {
        return m_setting_;
    }

    template<typename Dtype, int Dim>
    const typename BayesianHilbertMap<Dtype, Dim>::MatrixDX &
    BayesianHilbertMap<Dtype, Dim>::GetHingedPoints() const {
        return m_hinged_points_;
    }

    template<typename Dtype, int Dim>
    const typename BayesianHilbertMap<Dtype, Dim>::VectorX &
    BayesianHilbertMap<Dtype, Dim>::GetWeights() const {
        return m_mu_;
    }

    template<typename Dtype, int Dim>
    const typename BayesianHilbertMap<Dtype, Dim>::MatrixX &
    BayesianHilbertMap<Dtype, Dim>::GetWeightsCovariance() const {
        return m_sigma_;
    }

    template<typename Dtype, int Dim>
    const Aabb<Dtype, Dim> &
    BayesianHilbertMap<Dtype, Dim>::GetMapBoundary() const {
        return m_map_boundary_;
    }

    template<typename Dtype, int Dim>
    uint64_t
    BayesianHilbertMap<Dtype, Dim>::GetIterationCount() const {
        return m_iteration_cnt_;
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::GenerateDataset(
        const Eigen::Ref<const VectorD> &sensor_position,
        const Eigen::Ref<const MatrixDX> &points,
        const std::vector<long> &point_indices,
        const long max_dataset_size,
        long &num_samples,
        MatrixDX &dataset_points,
        VectorX &dataset_labels,
        std::vector<long> &hit_indices) {

        // 1. check if the ray intersects with the map boundary.
        // 2. compute the range to sample free points and the number of points to sample.
        // 3. sample the free points uniformly within the range.
        // 4. return the result.

        // tuple of (point_index, hit_flag, num_free_points, d1, d2)
        std::vector<std::tuple<long, bool, long, Dtype, Dtype>> infos;
        long max_num_free_points = 0, max_num_hit_points = 0;
        GenerateRayInfos(
            sensor_position,
            points,
            point_indices,
            infos,
            max_num_free_points,
            max_num_hit_points);

        // check if the dataset size limit is exceeded.
        // if exceeded, adjust the number of points to sample.
        const long max_num_points = max_num_free_points + max_num_hit_points;
        const bool limit_exceeded = max_dataset_size > 0 && max_num_points > max_dataset_size;
        long num_hit_to_sample, num_free_to_sample;
        if (limit_exceeded) {
            num_hit_to_sample = max_dataset_size * max_num_hit_points / max_num_points;
            num_free_to_sample = max_dataset_size * max_num_free_points / max_num_points;
        } else {
            num_hit_to_sample = max_num_hit_points;
            num_free_to_sample = max_num_free_points;
        }

        GenerateSamples(
            sensor_position,
            points,
            infos,
            limit_exceeded,
            num_hit_to_sample,
            num_free_to_sample,
            num_samples,
            dataset_points,
            dataset_labels,
            hit_indices);
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::RunExpectationMaximization(
        const MatrixDX &points,
        const VectorX &labels,
        const long num_points) {
        ERL_DEBUG_ASSERT(
            points.cols() == labels.size(),
            "points.cols() = {}, labels.size() = {}.",
            points.cols(),
            labels.size());
        PrepareExpectationMaximization(points, labels, num_points);
        if (m_setting_->use_sparse) {
            for (int itr = 0; itr < m_setting_->num_em_iterations; ++itr) {
                RunExpectationMaximizationIterationSparse(num_points);
            }
        } else {
            for (int itr = 0; itr < m_setting_->num_em_iterations; ++itr) {
                RunExpectationMaximizationIteration(num_points);
            }
        }
        // release the memory
        if (m_setting_->use_sparse) {
            m_phi_transpose_sparse_ = {};
            m_phi_sq_sparse_ = {};
            m_phi_sq_transpose_sparse_ = {};
        } else {
            m_phi_transpose_ = {};
            m_phi_sq_ = {};
            m_phi_sq_transpose_ = {};
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::PrepareExpectationMaximization(
        const MatrixDX &points,
        const VectorX &labels,
        const long num_points) {

        ERL_DEBUG_ASSERT(
            points.cols() >= num_points,
            "points.cols() = {}, num_points = {}.",
            points.cols(),
            num_points);

        // prepare memory
        if (m_xi_.rows() < num_points) {
            if (!m_setting_->use_sparse) { m_phi_.resize(num_points, m_hinged_points_.cols()); }
            m_labels_.resize(num_points);
            m_xi_.resize(num_points);
            m_lambda_.resize(num_points);
        }

        // prepare xi
        m_xi_.head(num_points).setOnes();
        // prepare the labels
        const Dtype *labels_in_ptr = labels.data();
        Dtype *labels_ptr = m_labels_.data();  // labels - 0.5
        for (long j = 0; j < num_points; ++j) { labels_ptr[j] = labels_in_ptr[j] - 0.5f; }

        // prepare phi
        if (m_setting_->use_sparse) {
            m_kernel_->ComputeKtestSparse(
                points,
                num_points,
                m_hinged_points_,
                m_hinged_points_.cols(),
                m_setting_->sparse_zero_threshold,
                m_phi_sparse_);
            ERL_DEBUG(
                "sparse: {}, dense: {}, sparsity score: {:.3f}",
                m_phi_sparse_.nonZeros(),
                m_phi_sparse_.size(),
                static_cast<Dtype>(m_phi_sparse_.size() - m_phi_sparse_.nonZeros()) /
                    static_cast<Dtype>(m_phi_sparse_.size()));
            m_phi_transpose_sparse_ = m_phi_sparse_.transpose();
            if (m_setting_->diagonal_sigma) {
                m_phi_sq_sparse_ = m_phi_sparse_.cwiseAbs2();
                m_phi_sq_transpose_sparse_ = m_phi_sq_sparse_.transpose();
            }
        } else {
            m_kernel_->ComputeKtest(
                points,
                num_points,
                m_hinged_points_,
                m_hinged_points_.cols(),
                m_phi_);
            m_phi_transpose_ = m_phi_.transpose();
            if (m_setting_->diagonal_sigma) {
                m_phi_sq_ = m_phi_.cwiseAbs2();
                m_phi_sq_transpose_ = m_phi_sq_.transpose();
            }
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::RunExpectationMaximizationIteration(const long n_points) {
        ++m_iteration_cnt_;

        const long num_feat = m_hinged_points_.cols();

        Dtype *xi_ptr = m_xi_.data();
        Dtype *lam_ptr = m_lambda_.data();
        Dtype *alpha_ptr = m_alpha_.data();

        // calculate 2 * lambda(xi) = (sigmoid(xi) - 0.5) / xi
        for (long j = 0; j < n_points; ++j) {
            lam_ptr[j] = (1.0f / (1.0f + std::exp(-xi_ptr[j])) - 0.5f) / xi_ptr[j];
        }

        if (m_setting_->diagonal_sigma) {  // diagonal sigma
            // E-Step: calculate the posterior
            Dtype *mu_ptr = m_mu_.data();
            Dtype *sigma_inv_ptr = m_sigma_inv_.data();
            Dtype *sigma_ptr = m_sigma_.data();
#pragma omp parallel for default(none) \
    shared(num_feat, n_points, sigma_inv_ptr, alpha_ptr, sigma_ptr, mu_ptr)
            for (long i = 0; i < num_feat; ++i) {  // loop over the features
                sigma_inv_ptr[i] += m_lambda_.head(n_points).dot(m_phi_sq_.col(i).head(n_points));
                alpha_ptr[i] += m_labels_.head(n_points).dot(m_phi_.col(i).head(n_points));
                // sigma_inv' = sigma_inv + 2 * ((phi ** 2).T * lams).sum(dim=1)
                sigma_ptr[i] = 1.0f / sigma_inv_ptr[i];
                // mu' = sigma' * (mu / sigma + phi.T @ (labels - 0.5))
                mu_ptr[i] = sigma_ptr[i] * alpha_ptr[i];
            }
            // M-Step: update xi
#pragma omp parallel for default(none) shared(n_points, xi_ptr)
            for (long j = 0; j < n_points; ++j) {
                Dtype a = m_mu_.dot(m_phi_transpose_.col(j));
                xi_ptr[j] = m_sigma_.col(0).dot(m_phi_sq_transpose_.col(j)) + a * a;
                // xi = sqrt(phi.T @ sigma @ phi + (phi.T @ mu) ** 2)
                xi_ptr[j] = std::sqrt(xi_ptr[j]);
            }
        } else {
            // non-diagonal sigma
            // E-Step: calculate the posterior
            // sigma_inv' = sigma_inv + 2 * (phi.T * lams) @ phi
            // mu' = sigma' @ (sigma_inv @ mu + phi.T @ (labels - 0.5))
            // alpha = sigma_inv @ mu
#pragma omp parallel for default(none) shared(num_feat, n_points, alpha_ptr)
            for (long c = 0; c < num_feat; ++c) {  // loop over cols
                auto phi_c = m_phi_.col(c).head(n_points);
                VectorX lam_phi = m_lambda_.head(n_points).cwiseProduct(phi_c);
                alpha_ptr[c] += m_labels_.head(n_points).dot(phi_c);
                Dtype *sigma_inv_ptr = m_sigma_inv_.col(c).data();
                sigma_inv_ptr[c] += lam_phi.dot(phi_c);
                for (long r = c + 1; r < num_feat; ++r) {
                    sigma_inv_ptr[r] += lam_phi.dot(m_phi_.col(r).head(n_points));
                    // copy the lower triangular part to the upper triangular part
                    m_sigma_inv_(c, r) = sigma_inv_ptr[r];
                }
            }
            m_sigma_inv_mat_l_ = m_sigma_inv_.llt().matrixL();  // Cholesky decomposition
            m_mu_ = m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solve(m_alpha_);
            m_sigma_inv_mat_l_.transpose()
                .template triangularView<Eigen::Upper>()  //
                .solveInPlace(m_mu_);
            // M-Step: update xi
#pragma omp parallel for default(none) shared(n_points, num_feat, xi_ptr)
            for (long j = 0; j < n_points; ++j) {
                auto phi_j = m_phi_transpose_.col(j).head(num_feat);
                const Dtype a = phi_j.dot(m_mu_);
                VectorX phi = phi_j;
                m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solveInPlace(phi);
                xi_ptr[j] = std::sqrt(phi.squaredNorm() + a * a);
            }
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::RunExpectationMaximizationIterationSparse(long num_points) {
        ++m_iteration_cnt_;
        const long num_feat = m_hinged_points_.cols();
        Dtype *xi_ptr = m_xi_.data();
        Dtype *lam_ptr = m_lambda_.data();
        Dtype *alpha_ptr = m_alpha_.data();

        // calculate 2 * lambda(xi) = (sigmoid(xi) - 0.5) / xi
        for (long j = 0; j < num_points; ++j) {
            lam_ptr[j] = (1.0f / (1.0f + std::exp(-xi_ptr[j])) - 0.5f) / xi_ptr[j];
        }

        if (m_setting_->diagonal_sigma) {
            // E-Step: calculate the posterior
            Dtype *mu_ptr = m_mu_.data();
            Dtype *sigma_inv_ptr = m_sigma_inv_.data();
            Dtype *sigma_ptr = m_sigma_.data();
#pragma omp parallel for default(none) \
    shared(num_feat, num_points, sigma_inv_ptr, alpha_ptr, sigma_ptr, mu_ptr)
            for (long i = 0; i < num_feat; ++i) {
                sigma_inv_ptr[i] += m_phi_sq_sparse_.col(i)
                                        .head(num_points)  //
                                        .dot(m_lambda_.head(num_points));
                alpha_ptr[i] += m_phi_sparse_.col(i)
                                    .head(num_points)  //
                                    .dot(m_labels_.head(num_points));
                // sigma_inv' = sigma_inv + 2 * ((phi ** 2).T * lams).sum(dim=1)
                sigma_ptr[i] = 1.0f / sigma_inv_ptr[i];
                // mu' = sigma' * (mu / sigma + phi.T @ (labels - 0.5))
                mu_ptr[i] = sigma_ptr[i] * alpha_ptr[i];
            }
            // M-Step: update xi
#pragma omp parallel for default(none) shared(num_points, xi_ptr)
            for (long j = 0; j < num_points; ++j) {
                Dtype a = m_phi_transpose_sparse_.col(j).dot(m_mu_);
                xi_ptr[j] =
                    std::sqrt(m_phi_sq_transpose_sparse_.col(j).dot(m_sigma_.col(0)) + a * a);
            }
        } else {
            // non-diagonal sigma
            // E-Step: calculate the posterior
#pragma omp parallel for default(none) shared(num_feat, num_points, alpha_ptr)
            for (long c = 0; c < num_feat; ++c) {  // loop over cols
                auto phi_c = m_phi_sparse_.col(c).head(num_points);
                VectorX lam_phi = phi_c.cwiseProduct(m_lambda_.head(num_points));
                alpha_ptr[c] += phi_c.dot(m_labels_.head(num_points));
                Dtype *sigma_inv_ptr = m_sigma_inv_.col(c).data();
                sigma_inv_ptr[c] += phi_c.dot(lam_phi);
                for (long r = c + 1; r < num_feat; ++r) {
                    sigma_inv_ptr[r] += m_phi_sparse_.col(r).head(num_points).dot(lam_phi);
                    // copy the upper triangular part to the lower triangular part
                    m_sigma_inv_(c, r) = sigma_inv_ptr[r];
                }
            }
            m_sigma_inv_mat_l_ = m_sigma_inv_.llt().matrixL();  // Cholesky decomposition
            m_mu_ = m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solve(m_alpha_);
            m_sigma_inv_mat_l_.transpose()
                .template triangularView<Eigen::Upper>()  //
                .solveInPlace(m_mu_);
            // M-Step: update xi
#pragma omp parallel for default(none) shared(num_points, num_feat, xi_ptr)
            for (long j = 0; j < num_points; ++j) {
                auto phi_j = m_phi_transpose_sparse_.col(j).head(num_feat);
                const Dtype a = phi_j.dot(m_mu_);
                VectorX phi = phi_j;
                m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solveInPlace(phi);
                xi_ptr[j] = std::sqrt(phi.squaredNorm() + a * a);
            }
        }
    }

    template<typename Dtype, int Dim>
    bool
    BayesianHilbertMap<Dtype, Dim>::Update(
        const Eigen::Ref<const VectorD> &sensor_position,
        const Eigen::Ref<const MatrixDX> &points,
        const std::vector<long> &point_indices,
        const long max_dataset_size,
        long &num_points,
        MatrixDX &dataset_points,
        VectorX &dataset_labels,
        std::vector<long> &hit_indices) {
        GenerateDataset(
            sensor_position,
            points,
            point_indices,
            max_dataset_size,
            num_points,
            dataset_points,
            dataset_labels,
            hit_indices);
        if (num_points == 0) {
            ERL_WARN("No valid points generated for update. Skipping update.");
            return false;
        }
        RunExpectationMaximization(dataset_points, dataset_labels, num_points);
        return true;
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::Predict(
        const Eigen::Ref<const MatrixDX> &points,
        const bool logodd,
        const bool faster,
        const bool compute_gradient,
        const bool gradient_with_sigmoid,
        const bool parallel,
        VectorX &prob_occupied,
        MatrixDX &gradient) const {

        // sparsity does not improve the performance of the prediction
        // if (m_setting_->use_sparse) {
        //     PredictSparse(points, logodd, faster, compute_gradient, gradient_with_sigmoid,
        //     parallel, prob_occupied, gradient); return;
        // }

        constexpr auto kPI = static_cast<Dtype>(M_PI);
        (void) parallel;
        const long n_hinged = m_hinged_points_.cols();
        const long n_points = points.cols();
        const long phi_cols = compute_gradient ? n_points * (Dim + 1) : n_points;

        const Eigen::VectorXl grad_flags = Eigen::VectorXl::Constant(n_hinged, 0);
        MatrixX phi(n_hinged, phi_cols);
        if (compute_gradient) {
            m_kernel_->ComputeKtestWithGradient(
                m_hinged_points_,
                n_hinged,
                grad_flags,
                points,
                n_points,
                true,
                phi);
            if (gradient.cols() < n_points) { gradient.resize(Dim, n_points); }
        } else {
            m_kernel_->ComputeKtest(m_hinged_points_, n_hinged, points, n_points, phi);
        }

        prob_occupied.resize(n_points);
        Dtype *prob_occupied_ptr = prob_occupied.data();
        if (faster) {  // assume sigma is very small; we can use the mean directly
#pragma omp parallel for if (parallel) default(none) \
    shared(logodd,                                   \
               n_points,                             \
               phi,                                  \
               prob_occupied_ptr,                    \
               compute_gradient,                     \
               gradient,                             \
               gradient_with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                const Dtype t1 = phi.col(i).dot(m_mu_);
                prob_occupied_ptr[i] = logodd ? t1 : 1.0f / (1.0f + std::exp(-t1));

                if (!compute_gradient) { continue; }

                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    grad_ptr[d] = phi.col(j).dot(m_mu_);
                    if (gradient_with_sigmoid) {
                        Dtype p = logodd ? 1.0f / (1.0f + std::exp(-t1)) : prob_occupied_ptr[i];
                        grad_ptr[d] *= p * (1.0f - p);
                    }
                }
            }
            return;
        }

        const Dtype *mu_ptr = m_mu_.data();
        if (m_setting_->diagonal_sigma) {
            // assume sigma is diagonal, i.e. element of mu is independent
#pragma omp parallel for if (parallel) default(none) \
    shared(logodd,                                   \
               n_points,                             \
               phi,                                  \
               prob_occupied_ptr,                    \
               compute_gradient,                     \
               gradient,                             \
               n_hinged,                             \
               mu_ptr,                               \
               gradient_with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                Dtype t1 = phi.col(i).dot(m_mu_);
                Dtype t2 = 1.0f + phi.col(i).cwiseAbs2().dot(m_sigma_.col(0)) * (kPI / 8.0f);
                Dtype t3 = std::sqrt(t2);
                Dtype h = t1 / t3;
                prob_occupied_ptr[i] = logodd ? h : 1.0f / (1.0f + std::exp(-h));  // sigmoid

                if (!compute_gradient) { continue; }

                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    const Dtype *grad_phi_x = phi.col(j).data();
                    const Dtype *phi_ptr = phi.col(i).data();
                    const Dtype *sigma_ptr = m_sigma_.data();
                    grad_ptr[d] = 0;
                    for (long k = 0; k < n_hinged; ++k) {
                        Dtype tmp = mu_ptr[k] * t2 - kPI * t1 * sigma_ptr[k] * phi_ptr[k];
                        grad_ptr[d] += tmp * grad_phi_x[k];
                    }
                    if (gradient_with_sigmoid) {
                        Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied_ptr[i];
                        grad_ptr[d] *= p * (1.0f - p) / (8.0f * t2 * t3);
                    } else {
                        grad_ptr[d] /= 8.0f * t2 * t3;
                    }
                }
            }
            return;
        }

#pragma omp parallel for if (parallel) default(none) \
    shared(logodd,                                   \
               n_points,                             \
               phi,                                  \
               prob_occupied_ptr,                    \
               compute_gradient,                     \
               gradient,                             \
               n_hinged,                             \
               mu_ptr,                               \
               gradient_with_sigmoid)
        for (long i = 0; i < n_points; ++i) {
            const Dtype t1 = phi.col(i).dot(m_mu_);
            VectorX beta = m_sigma_inv_mat_l_  //
                               .template triangularView<Eigen::Lower>()
                               .solve(phi.col(i));
            // 1 + phi^T @ sigma @ phi * (kPI / 8.0)
            const Dtype t2 = 1.0f + beta.squaredNorm() * (kPI / 8.0f);
            const Dtype t3 = std::sqrt(t2);
            const Dtype h = t1 / t3;
            prob_occupied_ptr[i] = logodd ? h : 1.0f / (1.0f + std::exp(-h));  // sigmoid function

            if (!compute_gradient) { continue; }

            // beta = sigma @ phi
            m_sigma_inv_mat_l_.template triangularView<Eigen::Upper>().solveInPlace(beta);
            Dtype *grad_ptr = gradient.col(i).data();
            const Dtype *beta_ptr = beta.data();
            for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                // grad = grad_phi_x @ grad_phi_h @ grad_prob_h
                // phi: n x (n * (Dim + 1))
                grad_ptr[d] = 0;
                Dtype *grad_phi_x = phi.col(j).data();
                for (long k = 0; k < n_hinged; ++k) {
                    grad_ptr[d] += (mu_ptr[k] * t2 - kPI * t1 * beta_ptr[k]) * grad_phi_x[k];
                }

                if (gradient_with_sigmoid) {
                    const Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied_ptr[i];
                    grad_ptr[d] *= p * (1.0f - p) / (8.0f * t2 * t3);
                } else {
                    grad_ptr[d] /= 8.0f * t2 * t3;
                }
            }
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::Predict(
        const VectorD &point,
        const bool logodd,
        const bool faster,
        const bool compute_gradient,
        const bool gradient_with_sigmoid,
        Dtype &prob_occupied,
        VectorD &gradient) const {

        // sparsity does not improve the performance of the prediction
        // if (m_setting_->use_sparse) {
        //     PredictSparse(points, logodd, faster, compute_gradient, gradient_with_sigmoid,
        //     parallel, prob_occupied, gradient); return;
        // }

        constexpr auto kPI = static_cast<Dtype>(M_PI);
        const long n_hinged = m_hinged_points_.cols();
        const long phi_cols = compute_gradient ? (Dim + 1) : 1;

        const Eigen::VectorXl grad_flags = Eigen::VectorXl::Constant(n_hinged, 0);
        MatrixX phi(n_hinged, phi_cols);
        if (compute_gradient) {
            m_kernel_->ComputeKtestWithGradient(
                m_hinged_points_,
                n_hinged,
                grad_flags,
                point,
                1,
                true,
                phi);
        } else {
            m_kernel_->ComputeKtest(m_hinged_points_, n_hinged, point, 1, phi);
        }

        if (faster) {  // assume sigma is very small; we can use the mean directly
            const Dtype t1 = phi.col(0).dot(m_mu_);
            prob_occupied = logodd ? t1 : 1.0f / (1.0f + std::exp(-t1));

            if (!compute_gradient) { return; }

            for (long d = 0, j = 1; d < Dim; ++d, ++j) {
                gradient[d] = phi.col(j).dot(m_mu_);
                if (gradient_with_sigmoid) {
                    Dtype p = logodd ? 1.0f / (1.0f + std::exp(-t1)) : prob_occupied;
                    gradient[d] *= p * (1.0f - p);
                }
            }
            return;
        }

        const Dtype *mu_ptr = m_mu_.data();
        if (m_setting_->diagonal_sigma) {
            // assume sigma is diagonal, i.e. element of mu is independent
            Dtype t1 = phi.col(0).dot(m_mu_);
            Dtype t2 = 1.0f + phi.col(0).cwiseAbs2().dot(m_sigma_.col(0)) * (kPI / 8.0f);
            Dtype t3 = std::sqrt(t2);
            Dtype h = t1 / t3;
            prob_occupied = logodd ? h : 1.0f / (1.0f + std::exp(-h));  // sigmoid

            if (!compute_gradient) { return; }

            for (long d = 0, j = 1; d < Dim; ++d, ++j) {
                const Dtype *grad_phi_x = phi.col(j).data();
                const Dtype *phi_ptr = phi.col(0).data();
                const Dtype *sigma_ptr = m_sigma_.data();
                gradient[d] = 0;
                for (long k = 0; k < n_hinged; ++k) {
                    Dtype tmp = mu_ptr[k] * t2 - kPI * t1 * sigma_ptr[k] * phi_ptr[k];
                    gradient[d] += tmp * grad_phi_x[k];
                }
                if (gradient_with_sigmoid) {
                    Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied;
                    gradient[d] *= p * (1.0f - p) / (8.0f * t2 * t3);
                } else {
                    gradient[d] /= 8.0f * t2 * t3;
                }
            }
            return;
        }

        const Dtype t1 = phi.col(0).dot(m_mu_);
        VectorX beta = m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solve(phi.col(0));
        // 1 + phi^T @ sigma @ phi * (kPI / 8.0)
        const Dtype t2 = 1.0f + beta.squaredNorm() * (kPI / 8.0f);
        const Dtype t3 = std::sqrt(t2);
        const Dtype h = t1 / t3;
        prob_occupied = logodd ? h : 1.0f / (1.0f + std::exp(-h));  // sigmoid function

        if (!compute_gradient) { return; }

        // beta = sigma @ phi
        m_sigma_inv_mat_l_.template triangularView<Eigen::Upper>().solveInPlace(beta);
        const Dtype *beta_ptr = beta.data();
        for (long d = 0, j = 1; d < Dim; ++d, ++j) {
            // grad = grad_phi_x @ grad_phi_h @ grad_prob_h
            // phi: n x (n * (Dim + 1))
            gradient[d] = 0;
            Dtype *grad_phi_x = phi.col(j).data();
            for (long k = 0; k < n_hinged; ++k) {
                gradient[d] += (mu_ptr[k] * t2 - kPI * t1 * beta_ptr[k]) * grad_phi_x[k];
            }

            if (gradient_with_sigmoid) {
                const Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied;
                gradient[d] *= p * (1.0f - p) / (8.0f * t2 * t3);
            } else {
                gradient[d] /= 8.0f * t2 * t3;
            }
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::PredictSparse(
        const Eigen::Ref<const MatrixDX> &points,
        const bool logodd,
        const bool faster,
        const bool compute_gradient,
        const bool gradient_with_sigmoid,
        const bool parallel,
        VectorX &prob_occupied,
        MatrixDX &gradient) const {

        constexpr auto kPI = static_cast<Dtype>(M_PI);
        (void) parallel;
        const long n_hinged = m_hinged_points_.cols();
        const long n_points = points.cols();
        const long phi_cols = compute_gradient ? n_points * (Dim + 1) : n_points;

        const Eigen::VectorXl grad_flags = Eigen::VectorXl::Constant(n_hinged, 0);
        SparseMatrix phi(n_hinged, phi_cols);
        if (compute_gradient) {
            m_kernel_->ComputeKtestWithGradientSparse(
                m_hinged_points_,
                n_hinged,
                grad_flags,
                points,
                n_points,
                compute_gradient,
                m_setting_->sparse_zero_threshold,
                phi);
            if (gradient.cols() < n_points) { gradient.resize(Dim, n_points); }
        } else {
            m_kernel_->ComputeKtestSparse(
                m_hinged_points_,
                n_hinged,
                points,
                n_points,
                m_setting_->sparse_zero_threshold,
                phi);
        }

        prob_occupied.resize(n_points);
        Dtype *prob_occupied_ptr = prob_occupied.data();
        if (faster) {  // assume sigma is very small; we can use the mean directly
#pragma omp parallel for if (parallel) default(none) \
    shared(logodd,                                   \
               n_points,                             \
               phi,                                  \
               prob_occupied_ptr,                    \
               compute_gradient,                     \
               gradient,                             \
               gradient_with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                const Dtype t1 = phi.col(i).dot(m_mu_);
                prob_occupied_ptr[i] = logodd ? t1 : 1.0f / (1.0f + std::exp(-t1));

                if (!compute_gradient) { continue; }

                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    grad_ptr[d] = phi.col(j).dot(m_mu_);
                    if (gradient_with_sigmoid) {
                        Dtype p = logodd ? 1.0f / (1.0f + std::exp(-t1)) : prob_occupied_ptr[i];
                        grad_ptr[d] *= p * (1.0f - p);
                    }
                }
            }
            return;
        }

        const Dtype *mu_ptr = m_mu_.data();
        if (m_setting_->diagonal_sigma) {
            // assume sigma is diagonal, i.e. element of mu is independent
#pragma omp parallel for if (parallel) default(none) \
    shared(logodd,                                   \
               n_points,                             \
               phi,                                  \
               prob_occupied_ptr,                    \
               compute_gradient,                     \
               gradient,                             \
               n_hinged,                             \
               mu_ptr,                               \
               gradient_with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                Dtype t1 = phi.col(i).dot(m_mu_);
                Dtype t2 = 1.0f + phi.col(i).cwiseAbs2().dot(m_sigma_.col(0)) * (kPI / 8.0f);
                Dtype t3 = std::sqrt(t2);
                Dtype h = t1 / t3;
                prob_occupied_ptr[i] = logodd ? h : 1.0f / (1.0f + std::exp(-h));

                if (!compute_gradient) { continue; }

                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    const Dtype *sigma_ptr = m_sigma_.data();
                    grad_ptr[d] = 0;
                    for (typename SparseMatrix::InnerIterator it(phi, j); it; ++it) {
                        const long k = it.row();
                        Dtype tmp = mu_ptr[k] * t2 - kPI * t1 * sigma_ptr[k] * phi.coeff(k, i);
                        grad_ptr[d] += tmp * it.value();
                    }
                    if (gradient_with_sigmoid) {
                        Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied_ptr[i];
                        grad_ptr[d] *= p * (1.0f - p) / (8.0f * t2 * t3);
                    } else {
                        grad_ptr[d] /= 8.0f * t2 * t3;
                    }
                }
            }
            return;
        }

#pragma omp parallel for if (parallel) default(none) \
    shared(logodd,                                   \
               n_points,                             \
               phi,                                  \
               prob_occupied_ptr,                    \
               compute_gradient,                     \
               gradient,                             \
               n_hinged,                             \
               mu_ptr,                               \
               gradient_with_sigmoid)
        for (long i = 0; i < n_points; ++i) {
            Dtype t1 = phi.col(i).dot(m_mu_);
            VectorX beta = m_sigma_inv_mat_l_  //
                               .template triangularView<Eigen::Lower>()
                               .solve(phi.col(i).toDense());
            // 1 + phi^T @ sigma @ phi * (kPI / 8.0)
            Dtype t2 = 1.0f + beta.squaredNorm() * (kPI / 8.0f);
            Dtype t3 = std::sqrt(t2);
            Dtype h = t1 / t3;
            prob_occupied_ptr[i] = logodd ? h : 1.0f / (1.0f + std::exp(-h));  // sigmoid function

            if (!compute_gradient) { continue; }

            // beta = sigma @ phi
            m_sigma_inv_mat_l_.template triangularView<Eigen::Upper>().solveInPlace(beta);
            Dtype *grad_ptr = gradient.col(i).data();
            const Dtype *beta_ptr = beta.data();
            for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                // grad = grad_phi_x @ grad_phi_h @ grad_prob_h
                // phi: n x (n * (Dim + 1))
                grad_ptr[d] = 0;
                for (typename SparseMatrix::InnerIterator it(phi, j); it; ++it) {
                    const long k = it.row();
                    grad_ptr[d] += (mu_ptr[k] * t2 - kPI * t1 * beta_ptr[k]) * it.value();
                }
                if (gradient_with_sigmoid) {
                    const Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied_ptr[i];
                    grad_ptr[d] *= p * (1.0f - p) / (8.0f * t2 * t3);
                } else {
                    grad_ptr[d] /= 8.0f * t2 * t3;
                }
            }
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::PredictGradient(
        const Eigen::Ref<const MatrixDX> &points,
        const bool faster,
        const bool with_sigmoid,
        const bool parallel,
        MatrixDX &gradient) const {

        constexpr auto kPI = static_cast<Dtype>(M_PI);
        (void) parallel;
        const long n_hinged = m_hinged_points_.cols();
        const long n_points = points.cols();
        const long phi_cols = n_points * (Dim + 1);

        const Eigen::VectorXl grad_flags = Eigen::VectorXl::Constant(n_hinged, 0);
        MatrixX phi(n_hinged, phi_cols);
        m_kernel_->ComputeKtestWithGradient(
            m_hinged_points_,
            n_hinged,
            grad_flags,
            points,
            n_points,
            true,
            phi);
        if (gradient.cols() < n_points) { gradient.resize(Dim, n_points); }

        if (faster) {  // assume sigma is very small, we can use the mean directly
#pragma omp parallel for if (parallel) default(none) shared(n_points, phi, gradient, with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    grad_ptr[d] = phi.col(j).dot(m_mu_);
                    if (with_sigmoid) {
                        Dtype t1 = phi.col(i).dot(m_mu_);
                        Dtype y = 1.0f / (1.0f + std::exp(-t1));
                        grad_ptr[d] *= y * (1.0f - y);
                    }
                }
            }
            return;
        }

        const Dtype *mu_ptr = m_mu_.data();
        if (m_setting_->diagonal_sigma) {
            // assume sigma is diagonal, i.e. element of mu is independent
#pragma omp parallel for if (parallel) default(none) \
    shared(n_points, phi, gradient, n_hinged, mu_ptr, with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                Dtype t1 = phi.col(i).dot(m_mu_);
                Dtype t2 = 1.0f + phi.col(i).cwiseAbs2().dot(m_sigma_.col(0)) * (kPI / 8.0f);
                Dtype t3 = std::sqrt(t2);

                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    const Dtype *grad_phi_x = phi.col(j).data();
                    const Dtype *phi_ptr = phi.col(i).data();
                    const Dtype *sigma_ptr = m_sigma_.data();
                    grad_ptr[d] = 0;
                    for (long k = 0; k < n_hinged; ++k) {
                        Dtype tmp = mu_ptr[k] * t3 - kPI * t1 * sigma_ptr[k] * phi_ptr[k];
                        grad_ptr[d] += tmp * grad_phi_x[k];
                    }
                    if (with_sigmoid) {
                        Dtype h = t1 / t3;
                        Dtype y = 1.0f / (1.0f + std::exp(-h));  // sigmoid function
                        grad_ptr[d] *= y * (1.0f - y) / (8.0f * t2 * t3);
                    } else {
                        grad_ptr[d] /= 8.0f * t2 * t3;
                    }
                }
            }
            return;
        }

#pragma omp parallel for if (parallel) default(none) \
    shared(n_points, phi, gradient, n_hinged, mu_ptr, with_sigmoid)
        for (long i = 0; i < n_points; ++i) {
            Dtype t1 = phi.col(i).dot(m_mu_);
            VectorX beta =
                m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solve(phi.col(i));
            // 1 + phi^T @ sigma @ phi * (M_PI / 8.0)
            Dtype t2 = 1.0f + beta.squaredNorm() * (kPI / 8.0f);
            Dtype t3 = std::sqrt(t2);

            // beta = sigma @ phi
            m_sigma_inv_mat_l_.template triangularView<Eigen::Upper>().solveInPlace(beta);
            Dtype *grad_ptr = gradient.col(i).data();
            const Dtype *beta_ptr = beta.data();
            for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                // grad = grad_phi_x @ grad_phi_h @ grad_prob_h
                // phi: n x (n * (Dim + 1))
                grad_ptr[d] = 0;
                Dtype *grad_phi_x = phi.col(j).data();
                for (long k = 0; k < n_hinged; ++k) {
                    grad_ptr[d] += (mu_ptr[k] * t3 - M_PI * t1 * beta_ptr[k]) * grad_phi_x[k];
                }
                if (with_sigmoid) {
                    Dtype h = t1 / t3;
                    Dtype y = 1.0f / (1.0f + std::exp(-h));  // sigmoid function
                    grad_ptr[d] *= y * (1.0f - y) / (8.0f * t2 * t3);
                } else {
                    grad_ptr[d] /= 8.0f * t2 * t3;
                }
            }
        }
    }

    template<typename Dtype, int Dim>
    bool
    BayesianHilbertMap<Dtype, Dim>::Write(std::ostream &s) const {
        using namespace common;
        static const TokenWriteFunctionPairs<BayesianHilbertMap> token_function_pairs = {
            {
                "setting",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return self->m_setting_->Write(stream) && stream.good();
                },
            },
            {
                "kernel",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return self->m_kernel_->Write(stream) && stream.good();
                },
            },
            {
                "hinged_points",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_hinged_points_) &&
                           stream.good();
                },
            },
            {
                "map_boundary",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return stream.write(
                               reinterpret_cast<const char *>(self->m_map_boundary_.center.data()),
                               sizeof(Dtype) * Dim) &&
                           stream.write(
                               reinterpret_cast<const char *>(
                                   self->m_map_boundary_.half_sizes.data()),
                               sizeof(Dtype) * Dim) &&
                           stream.good();
                },
            },
            {
                "generator",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    stream << self->m_generator_ << '\n';
                    return stream.good();
                },
            },
            {
                "iteration_cnt",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    stream.write(
                        reinterpret_cast<const char *>(&self->m_iteration_cnt_),
                        sizeof(uint64_t));
                    return stream.good();
                },
            },
            {
                "sigma_inv",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_sigma_inv_) &&
                           stream.good();
                },
            },
            {
                "sigma",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_sigma_) && stream.good();
                },
            },
            {
                "sigma_inv_mat_l",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_sigma_inv_mat_l_) &&
                           stream.good();
                },
            },
            {
                "alpha",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_alpha_) && stream.good();
                },
            },
            {
                "labels",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_labels_) && stream.good();
                },
            },
            {
                "mu",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_mu_) && stream.good();
                },
            },
            {
                "phi",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_phi_) && stream.good();
                },
            },
            {
                "phi_sq",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_phi_sq_) && stream.good();
                },
            },
            {
                "phi_transpose",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_phi_transpose_) &&
                           stream.good();
                },
            },
            {
                "phi_sq_transpose",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_phi_sq_transpose_) &&
                           stream.good();
                },
            },
            {
                "phi_sparse",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveSparseEigenMatrixToBinaryStream(stream, self->m_phi_sparse_) &&
                           stream.good();
                },
            },
            {
                "phi_sq_sparse",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveSparseEigenMatrixToBinaryStream(stream, self->m_phi_sq_sparse_) &&
                           stream.good();
                },
            },
            {
                "phi_transpose_sparse",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveSparseEigenMatrixToBinaryStream(
                               stream,
                               self->m_phi_transpose_sparse_) &&
                           stream.good();
                },
            },
            {
                "phi_sq_transpose_sparse",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveSparseEigenMatrixToBinaryStream(
                               stream,
                               self->m_phi_sq_transpose_sparse_) &&
                           stream.good();
                },
            },
            {
                "xi",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_xi_) && stream.good();
                },
            },
            {
                "lambda",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_lambda_) && stream.good();
                },
            },
        };
        return WriteTokens(s, this, token_function_pairs);
    }

    template<typename Dtype, int Dim>
    bool
    BayesianHilbertMap<Dtype, Dim>::Read(std::istream &s) {
        using namespace common;
        static const TokenReadFunctionPairs<BayesianHilbertMap> token_function_pairs = {
            {
                "setting",
                [](const BayesianHilbertMap *self, std::istream &stream) {
                    return self->m_setting_->Read(stream) && stream.good();
                },
            },
            {
                "kernel",
                [](const BayesianHilbertMap *self, std::istream &stream) {
                    return self->m_kernel_->Read(stream) && stream.good();
                },
            },
            {
                "hinged_points",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_hinged_points_) &&
                           stream.good();
                },
            },
            {
                "map_boundary",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return stream.read(
                               reinterpret_cast<char *>(self->m_map_boundary_.center.data()),
                               sizeof(Dtype) * Dim) &&
                           stream.read(
                               reinterpret_cast<char *>(self->m_map_boundary_.half_sizes.data()),
                               sizeof(Dtype) * Dim) &&
                           stream.good();
                },
            },
            {
                "generator",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    stream >> self->m_generator_;
                    SkipLine(stream);
                    return stream.good();
                },
            },
            {
                "iteration_cnt",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    stream.read(
                        reinterpret_cast<char *>(&self->m_iteration_cnt_),
                        sizeof(uint64_t));
                    return stream.good();
                },
            },
            {
                "sigma_inv",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_sigma_inv_) &&
                           stream.good();
                },
            },
            {
                "sigma",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_sigma_) && stream.good();
                },
            },
            {
                "sigma_inv_mat_l",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_sigma_inv_mat_l_) &&
                           stream.good();
                },
            },
            {
                "alpha",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_alpha_) && stream.good();
                },
            },
            {
                "labels",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_labels_) &&
                           stream.good();
                },
            },
            {
                "mu",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_mu_) && stream.good();
                },
            },
            {
                "phi",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_phi_) && stream.good();
                },
            },
            {
                "phi_sq",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_phi_sq_) &&
                           stream.good();
                },
            },
            {
                "phi_transpose",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_phi_transpose_) &&
                           stream.good();
                },
            },
            {
                "phi_sq_transpose",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_phi_sq_transpose_) &&
                           stream.good();
                },
            },
            {
                "phi_sparse",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadSparseEigenMatrixFromBinaryStream(stream, self->m_phi_sparse_) &&
                           stream.good();
                },
            },
            {
                "phi_sq_sparse",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadSparseEigenMatrixFromBinaryStream(stream, self->m_phi_sq_sparse_) &&
                           stream.good();
                },
            },
            {
                "phi_transpose_sparse",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadSparseEigenMatrixFromBinaryStream(
                               stream,
                               self->m_phi_transpose_sparse_) &&
                           stream.good();
                },
            },
            {
                "phi_sq_transpose_sparse",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadSparseEigenMatrixFromBinaryStream(
                               stream,
                               self->m_phi_sq_transpose_sparse_) &&
                           stream.good();
                },
            },
            {
                "xi",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_xi_) && stream.good();
                },
            },
            {
                "lambda",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_lambda_) &&
                           stream.good();
                },
            },
        };
        return ReadTokens(s, this, token_function_pairs);
    }

    template<typename Dtype, int Dim>
    bool
    BayesianHilbertMap<Dtype, Dim>::operator==(const BayesianHilbertMap &other) const {
        if (m_setting_ == nullptr && other.m_setting_ != nullptr) { return false; }
        if (m_setting_ != nullptr &&
            (other.m_setting_ == nullptr || *m_setting_ != *other.m_setting_)) {
            return false;
        }
        if (m_kernel_ == nullptr && other.m_kernel_ != nullptr) { return false; }
        if (m_kernel_ != nullptr &&
            (other.m_kernel_ == nullptr || *m_kernel_ != *other.m_kernel_)) {
            return false;
        }
        using namespace common;
        if (!SafeEigenMatrixEqual(m_hinged_points_, other.m_hinged_points_)) { return false; }
        if (m_map_boundary_ != other.m_map_boundary_) { return false; }
        // if (m_generator_ != other.m_generator_) { return false; }  // cannot compare
        if (m_iteration_cnt_ != other.m_iteration_cnt_) { return false; }
        if (!SafeEigenMatrixEqual(m_sigma_inv_, other.m_sigma_inv_)) { return false; }
        if (!SafeEigenMatrixEqual(m_sigma_, other.m_sigma_)) { return false; }
        if (!SafeEigenMatrixEqual(m_sigma_inv_mat_l_, other.m_sigma_inv_mat_l_)) { return false; }
        if (!SafeEigenMatrixEqual(m_alpha_, other.m_alpha_)) { return false; }
        if (!SafeEigenMatrixEqual(m_labels_, other.m_labels_)) { return false; }
        if (!SafeEigenMatrixEqual(m_mu_, other.m_mu_)) { return false; }
        if (!SafeEigenMatrixEqual(m_phi_, other.m_phi_)) { return false; }
        if (!SafeEigenMatrixEqual(m_phi_sq_, other.m_phi_sq_)) { return false; }
        if (!SafeEigenMatrixEqual(m_phi_transpose_, other.m_phi_transpose_)) { return false; }
        if (!SafeEigenMatrixEqual(m_phi_sq_transpose_, other.m_phi_sq_transpose_)) { return false; }
        for (auto &[left, right]:
             std::vector<std::pair<const SparseMatrix &, const SparseMatrix &>>{
                 {m_phi_sparse_, other.m_phi_sparse_},
                 {m_phi_sq_sparse_, other.m_phi_sq_sparse_},
                 {m_phi_transpose_sparse_, other.m_phi_transpose_sparse_},
                 {m_phi_sq_transpose_sparse_, other.m_phi_sq_transpose_sparse_},
             }) {
            if (!SafeSparseEigenMatrixEqual(left, right)) { return false; }
        }
        if (!SafeEigenMatrixEqual(m_xi_, other.m_xi_)) { return false; }
        if (!SafeEigenMatrixEqual(m_lambda_, other.m_lambda_)) { return false; }
        return true;
    }

    template<typename Dtype, int Dim>
    bool
    BayesianHilbertMap<Dtype, Dim>::operator!=(const BayesianHilbertMap &other) const {
        return !(*this == other);
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::GenerateRayInfos(
        const Eigen::Ref<const VectorD> &sensor_position,
        const Eigen::Ref<const MatrixDX> &points,
        const std::vector<long> &point_indices,
        std::vector<std::tuple<long, bool, long, Dtype, Dtype>> &infos,
        long &max_num_free_points,
        long &max_num_hit_points) {

        const Dtype max_distance = m_setting_->max_distance;
        const Dtype free_sampling_margin = m_setting_->free_sampling_margin;
        const Dtype free_points_per_meter = m_setting_->free_points_per_meter;

        infos.reserve(points.cols() / 10);  // reserve space for the infos
        max_num_free_points = 0;
        max_num_hit_points = 0;

        auto npts = point_indices.empty() ? points.cols() : static_cast<long>(point_indices.size());

        for (long i = 0; i < npts; ++i) {
            long idx = point_indices.empty() ? i : point_indices[i];
            VectorD point = points.col(idx);
            VectorD v = point - sensor_position;
            Dtype v_norm = v.norm();
            v /= v_norm;  // normalize the vector
            Dtype d1 = 0;
            Dtype d2 = 0;
            bool hit_flag = false;
            bool intersected = false;
            bool is_inside = false;
            // compute intersection between the ray (point -> sensor_position) and the map boundary
            geometry::ComputeIntersectionBetweenRayAndAabb<Dtype, Dim>(
                sensor_position,
                v.cwiseInverse(),
                m_map_boundary_.min(),
                m_map_boundary_.max(),
                d1,
                d2,
                intersected,
                is_inside);
            // the ray does not intersect with the map boundary, or
            // hits a point outside the map, and v points away from the map; or
            // the ray hits a point outside the map, v points toward the map.
            if (!intersected || (d1 < 0 && d2 < 0) || (v_norm <= d1 && d1 <= d2)) { continue; }
            // check if the point is inside the map
            hit_flag = m_map_boundary_.contains(point) && (v_norm < max_distance);
            if (is_inside) {  // the ray hits a point inside the map, d2 < 0 is useless
                d2 = std::min((1.0f - free_sampling_margin) * v_norm, d1);
                d1 = free_sampling_margin * v_norm;
            } else {
                d1 = std::max(free_sampling_margin * v_norm, d1);
                d2 = std::min((1.0f - free_sampling_margin) * v_norm, d2);
            }
            // number of free points to sample
            auto n = std::max(0l, static_cast<long>(std::ceil((d2 - d1) * free_points_per_meter)));
            if (n == 0 && !hit_flag) { continue; }  // no free points and the point is not hit
            max_num_free_points += n;               // count the number of free points to sample
            max_num_hit_points += static_cast<long>(hit_flag);  // count the number of hit points
            d1 /= v_norm;
            d2 /= v_norm;
            infos.emplace_back(idx, hit_flag, n, d1, d2);
        }

        // ERL_DEBUG_ASSERT(
        //     (max_num_hit_points == 0 && max_num_free_points == 0) ||
        //         (max_num_hit_points > 0 && max_num_free_points > 0),
        //     "max_num_hit_points = {}, max_num_free_points = {}.",
        //     max_num_hit_points,
        //     max_num_free_points);
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::GenerateSamples(
        const Eigen::Ref<const VectorD> &sensor_position,
        const Eigen::Ref<const MatrixDX> &points,
        const std::vector<std::tuple<long, bool, long, Dtype, Dtype>> &infos,
        const bool random_infos,
        const long num_hit_to_sample,
        const long num_free_to_sample,
        long &num_samples,
        MatrixDX &dataset_points,
        VectorX &dataset_labels,
        std::vector<long> &hit_indices) {

        ERL_DEBUG_ASSERT(!infos.empty(), "infos is empty.");

        const long n_to_sample = num_hit_to_sample + num_free_to_sample;
        if (dataset_points.cols() < n_to_sample) { dataset_points.resize(Dim, n_to_sample); }
        if (dataset_labels.size() < n_to_sample) { dataset_labels.resize(n_to_sample); }

        std::vector<std::size_t> info_indices(infos.size());
        std::iota(info_indices.begin(), info_indices.end(), 0);
        if (random_infos) { std::shuffle(info_indices.begin(), info_indices.end(), m_generator_); }

        Dtype *points_ptr = dataset_points.data();
        Dtype *labels_ptr = dataset_labels.data();
        num_samples = 0;
        long n_hit = 0, n_free = 0;
        hit_indices.clear();

        for (std::size_t i = 0; i < infos.size(); ++i) {
            if (num_samples >= n_to_sample) { break; }  // already sampled enough points

            std::size_t idx = i;
            if (random_infos) { idx = info_indices[i]; }
            const auto &[point_index, hit_flag, num_free_points, d1, d2] = infos[idx];
            const Dtype *point_ptr = points.col(point_index).data();

            if (hit_flag && n_hit < num_hit_to_sample) {
                std::memcpy(points_ptr, point_ptr, sizeof(Dtype) * Dim);  // save the hit point
                *labels_ptr++ = 1.0f;                                     // label as occupied
                points_ptr += Dim;  // move to the next position
                ++n_hit;
                ++num_samples;
                hit_indices.push_back(point_index);  // add the index to the list
            }

            const long n = std::min(num_free_points, num_free_to_sample - n_free);
            if (n <= 0) { continue; }  // no free points to sample

            n_free += n;
            num_samples += n;
            std::uniform_real_distribution<Dtype> distribution(d1, d2);
            for (long j = 0; j < n; ++j) {
                // sample a random distance within the range [d1, d2]
                Dtype r = distribution(m_generator_);
                Dtype s = 1 - r;
                for (long k = 0; k < Dim; ++k) {  // compute the free point position
                    *points_ptr++ = sensor_position[k] * s + point_ptr[k] * r;
                }
                *labels_ptr++ = 0.0f;  // label as free
            }
        }

        ERL_DEBUG("Sampled {} points, {} hit points, {} free points.", num_samples, n_hit, n_free);
    }

    template class BayesianHilbertMap<float, 2>;
    template class BayesianHilbertMap<double, 2>;
    template class BayesianHilbertMap<float, 3>;
    template class BayesianHilbertMap<double, 3>;
}  // namespace erl::geometry

YAML::Node
YAML::convert<erl::geometry::BayesianHilbertMapSetting>::encode(
    const erl::geometry::BayesianHilbertMapSetting &setting) {
    Node node;
    ERL_YAML_SAVE_ATTR(node, setting, diagonal_sigma);
    ERL_YAML_SAVE_ATTR(node, setting, max_distance);
    ERL_YAML_SAVE_ATTR(node, setting, free_points_per_meter);
    ERL_YAML_SAVE_ATTR(node, setting, free_sampling_margin);
    ERL_YAML_SAVE_ATTR(node, setting, init_mu);
    ERL_YAML_SAVE_ATTR(node, setting, init_sigma);
    ERL_YAML_SAVE_ATTR(node, setting, num_em_iterations);
    ERL_YAML_SAVE_ATTR(node, setting, sparse_zero_threshold);
    ERL_YAML_SAVE_ATTR(node, setting, use_sparse);
    return node;
}

bool
YAML::convert<erl::geometry::BayesianHilbertMapSetting>::decode(
    const Node &node,
    erl::geometry::BayesianHilbertMapSetting &setting) {
    if (!node.IsMap()) { return false; }
    ERL_YAML_LOAD_ATTR(node, setting, diagonal_sigma);
    ERL_YAML_LOAD_ATTR(node, setting, max_distance);
    ERL_YAML_LOAD_ATTR(node, setting, free_points_per_meter);
    ERL_YAML_LOAD_ATTR(node, setting, free_sampling_margin);
    ERL_YAML_LOAD_ATTR(node, setting, init_mu);
    ERL_YAML_LOAD_ATTR(node, setting, init_sigma);
    ERL_YAML_LOAD_ATTR(node, setting, num_em_iterations);
    ERL_YAML_LOAD_ATTR(node, setting, sparse_zero_threshold);
    ERL_YAML_LOAD_ATTR(node, setting, use_sparse);
    return true;
}
